import asyncio 
import numpy as np
import cv2 # unused
from pupil_apriltags import Detector # unused
from mavsdk import System

# async function for camera detection

async def run():
    # start of do not touch 
    drone = System()
    await drone.connect(system_address="udp://:14540")
   
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("drone connected")
            break
    
    
        if health.is_global_position_ok:
            print("drone skib")
            break
    # end of do not touch

    print("arming")
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(5) 

    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        current_alt = position.absolute_altitude_m
        print(f"pos: {current_lat}, {current_lon}, {current_alt}")
        break

    # set up
    square = 20
    speed = 5

    dlat = square / 111111
    dlon = square / (111111 * np.cos(np.radians(current_lat))) # sidenote i have no idea what this is

    corners = [
            (current_lat, current_lon),
            (current_lat + dlat, current_lon), 
            (current_lat + dlat, current_lon + dlon),
            (current_lat, current_lon + dlon), 
            (current_lat, current_lon)
        ]
    
    # start of rectangular search code
    tagfound = asyncio.Event()
    detect_task = asyncio.create_task(tagfound)

    try:
        for i, (target_lat, target_lon) in enumerate(corners):
            print(f"target: {target_lat}, {target_lon}")
            await drone.action.goto_location(target_lat, target_lon, current_alt, speed)
            # while loop with async sleep for short duration and constantly checking?
            await asyncio.sleep(15)
     

       

    await drone.action.return_to_launch()

    # await drone.action.land()
    # i didnt realize something like return to launch existed lmao

    async for is_landed in drone.telemetry.in_air():
        if not is_landed:
            print("all landed")
   
    await drone.action.disarm()

if __name__ == "__main__":
    asyncio.run(run())
