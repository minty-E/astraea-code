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

capt = cv2.VideoCapture(0)

capt.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
capt.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

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
            cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)
            
        cv2.imshow('Frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break

except KeyboardInterrupt:
    print("Exiting...")

finally:
    capt.release()
    cv2.destroyAllWindows()
