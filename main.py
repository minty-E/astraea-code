import cv2
import numpy as np
from pupil_apriltags import Detector

at_detector = Detector(
   families="tag36h11",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)


fx = 189  # Focal length in x-axis (in pixels)
fy = 91   # Focal length in y-axis (in pixels)

frame_width = 1280
frame_height = 720
cx = frame_width / 2
cy = frame_height / 2

camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]])
dist_coeffs = np.zeros((4, 1))  


tag_size = 0.165

capt = cv2.VideoCapture(0)
capt.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
capt.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

if not capt.isOpened():
    print("Error: Could not open video capture")
    exit()

try:
    while True:
        ret, frame = capt.read()
        if not ret:
            print("Error: Failed to capture image")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray)

        for tag in tags:
            image_points = tag.corners

            cv2.rectangle(frame, tuple(tag.corners[0].astype(int)), tuple(tag.corners[2].astype(int)), (0, 255, 0), 2)

        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break

except KeyboardInterrupt:
    print("Exiting...")

finally:
    capt.release()
    cv2.destroyAllWindows()
