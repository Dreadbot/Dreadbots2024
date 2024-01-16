import cv2
import numpy as np

def main():
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret: break

        # rgb_hoop_img = cv2.cvtColor()
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 2, 100, minRadius=40, maxRadius=600)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            mean_center = circles.mean(axis=1)[0]
            center = (int(mean_center[0]), int(mean_center[1]))
            cv2.circle(gray_img, center, 1, (0, 100, 100), 3)
            radius = int(mean_center[2])
            cv2.circle(gray_img, center, radius, (255, 0, 255), 3)

        cv2.imshow("frame", gray_img)

        if cv2.waitKey(1) == ord('q') & 0xff:
            break

if __name__ == "__main__":
    main()
