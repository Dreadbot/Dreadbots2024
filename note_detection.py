import cv2
import numpy as np

def main():
    cap = cv2.VideoCapture(4)
    
    while True:
        ret, frame = cap.read()
        if not ret: break 

        hsvim = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        upper_bound = np.array([25, 255, 255])
        lower_bound = np.array([0, 75, 125])
        ring_imrange = cv2.inRange(hsvim, lower_bound, upper_bound)

        red_upper_bound = np.array([5, 255, 255])
        red_lower_bound = np.array([0, 75, 0])
        red_imrange = cv2.inRange(hsvim, red_lower_bound, red_upper_bound)

        not_imrange = cv2.bitwise_not(red_imrange)

        imrange = cv2.bitwise_and(not_imrange, ring_imrange)

        kernel = np.ones((5, 5), np.uint8)

        blurim = cv2.GaussianBlur(imrange, (1, 1), 0)
        erodeim = cv2.erode(blurim, kernel)
        dilateim = cv2.dilate(erodeim, kernel)
        
        cv2.imshow("frame", dilateim)
                 
        if cv2.waitKey(1) == ord('q') & 0xff:
            break

if __name__ == "__main__":
    main()
