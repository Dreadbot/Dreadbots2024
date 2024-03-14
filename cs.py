from cscore import CameraServer
import numpy as np
import time
import cv2

def main():
    CameraServer.enableLogging()
    cstream = CameraServer.putVideo("backcam", 640, 480)

    cap = cv2.VideoCapture(0)
    prevframe = time.process_time()

    while True:
        if time.process_time() - prevframe > 1/18:
            _, frame = cap.read()
            cstream.putFrame(frame)
            prevframe = time.process_time()

if __name__ == "__main__":
    main()
