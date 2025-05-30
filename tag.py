import asyncio
import cv2
import numpy as np
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
import picamera2 from Picamera2

class Shared:
    def __init__(self):
        self.lock = asyncio.Lock()
        self.detection = None
        self.frame_size = (4608, 2592)
        self.should_land = asyncio.Event()

async def self_landing(drone):
    print("aruco tag finding and landing")
    picam2 = Picamera2()
    picam2.configure(picam.create_video_configuration())
    picam2.start()

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # my honest reaction when i just copy everything from inferno in which they copied from chatgpt

    pid_x = PID(kp=0.001, ki=0.0000, kd=0.000, dt=0.1)
    pid_y = PID(kp=0.001, ki=0.0000, kd=0.000, dt=0.1)

   # Threshold in pixels under which we consider the drone to be aligned
    threshold = 15

    aligned_counter = 0
    required_alignments = 20  # Require several consecutive frames to confirm alignment
    failed_detection_counter = 0
    iteration = 0

    # Parameters for drone descent
    camFOV = math.radians(75)
    safety_factor = 1.5
    tag_size = 0.183 # change tag size in m to maximum!
    tag_circumradius = tag_size * math.sqrt(2)/2
    max_descent_rate = 0.5 #m/s
    while True:
        # Read frame in a non-blocking way
        frame = picam2.capture_array()
        if frame is None:
            print("Failed to capture frame")
            continue
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Determine frame center (assume camera is calibrated so that frame center aligns with desired landing point)
        frame_height, frame_width = frame.shape[:2]
        frame_center = (frame_width // 2, frame_height // 2)

        # Detect ArUco markers in the frame
        corners, ids, _ = detector.detectMarkers(frame)
        if corners and len(corners) > 0:
            failed_detection_counter = 0
            # Assume the first detected marker is our target
            marker_corners = corners[0]
            # Compute the center of the marker (average of its corner points)
            marker_center = np.mean(marker_corners[0], axis=0)
            marker_center = tuple(np.int32(marker_center))

            # Calculate errors in X and Y (in pixels)
            error_x = marker_center[0] - frame_center[0]
            error_y = marker_center[1] - frame_center[1]
            
            # PID correction outputs (mapping pixel error to m/s command, adjust gains as needed)
            control_x = pid_x.update(error_x)  # Negative sign if image x error is opposite to drone's right movement
            control_y = pid_y.update(error_y)  # Adjust sign based on camera mounting and coordinate frame
            print(control_x, control_y)
            # Draw marker and error information on frame (for debugging)
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, marker_center, 5, (0, 255, 0), -1)
            cv2.putText(frame, f"ErrX: {error_x:.1f}", (marker_center[0]+10, marker_center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"ErrY: {error_y:.1f}", (marker_center[0]+10, marker_center[1]+15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Command the drone with the computed corrections.
            # Here, control_x adjusts forward/backward and control_y adjusts left/right.
            # We keep vertical velocity zero as we want to continue descending by landing command.
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(control_x, control_y, 0.0, 0.0)
            )

            # Check if errors are within the threshold.
            if abs(error_x) < threshold and abs(error_y) < threshold:
                aligned_counter += 1
                print(f'Aligned Counter: {aligned_counter}')
            elif aligned_counter != 0:
                aligned_counter = 0
                print('Aligned Counter Reset')

            # If aligned for several consecutive frames, exit loop and initiate landing.
            if aligned_counter >= required_alignments:
                print("Marker aligned! Initiating landing...")
                break

        else:
            # If no marker is detected, hover in place before landing
            print("-- No Marker, Hovering")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            if (failed_detection_counter == 10):
                print("-- No Marker, Landing")
                await drone.action.land()
            # Optionally show frame as is
            else:
                failed_detection_counter += 1  
                print("Failed Detection Counter: " + str(failed_detection_counter))
        if(iteration % 10 ==0):
            # Show the frame with detected markers
            cv2.imwrite("~/Rebirdth/Frames/", frame)
        iteration +=1

    cv2.destroyAllWindows()
    return

    

async def controller(shared, drone):
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            break

    print("arming")
    await drone.action.arm()
    await drone.action.takeoff() # @TODO: change this later to match a certain altitude up
    await asyncio.sleep(5)
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()

    try:
        while not shared.should_land.is_set():
            async with shared.lock:
                detection = shared.detection
            if detection and detection['detections']:
                tag = detection['detections'][0]
                w, h = detection['frame_size']
                cx, cy = tag.center
                error_x = cx - (w / 2)
                error_y = cy - (h / 2)

                position = await drone.telemetry.position().__anext__()
                altitude = position.relative_altitude_m

                fov_x = 66
                fov_y = 41

                scale_x = (2 * altitude * np.tan(np.radians(fov_x / 2))) / w
                scale_y = (2 * altitude * np.tan(np.radians(fov_y / 2))) / h
                
                # random deepseek pid
                Kp = 0.5
                vx = -Kp * error_y * scale_y  
                vy = Kp * error_x * scale_x   
                
                max_speed = 1.0
                vx = np.clip(vx, -max_speed, max_speed)
                vy = np.clip(vy, -max_speed, max_speed)

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(vx, vy, 0.0, 0.0)
                )

                if abs(error_x) < 20 and abs(error_y) < 20:
                    print("Centered - Landing!")
                    shared_state.should_land.set()
            else:
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                )

            await asyncio.sleep(0.5)

    finally:
        await drone.action.land()
        await drone.offboard.stop()

async def main():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyS0:57600")

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("connect")
            break
    
        '''async for health in drone.telemetry.health():
            if health.is_global_position_ok:
                print("Global position estimate OK")
                break'''
    shared = SharedState()

    detector_task = asyncio.create_task(detect_tag(shared))
    controller_task = asyncio.create_task(controller(shared, drone))
    
    '''
    print("arming")
    await drone.action.arm()

    print("fly up")
    await drone.action.takeoff()
    await asyncio.sleep(10)
    '''
    await shared_state.should_land.wait()
    
    detector_task.cancel()
    controller_task.cancel()
    await asyncio.gather(detector_task, controller_task, return_exceptions=True)

if __name__ == "__main__":
    asyncio.run(main())
