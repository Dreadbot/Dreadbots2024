import cv2
import numpy as np
import math

def main():
    cap = cv2.VideoCapture(4)
    
    while True:
        ret, frame = cap.read()
        if not ret: break 

        bndreclist = []

        hsvim = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        upper_bound = np.array([25, 255, 255])
        lower_bound = np.array([0, 75, 125])
        ring_imrange = cv2.inRange(hsvim, lower_bound, upper_bound)

        red_upper_bound = np.array([7, 255, 255])
        red_lower_bound = np.array([0, 75, 0])
        red_imrange = cv2.inRange(hsvim, red_lower_bound, red_upper_bound)

        not_imrange = cv2.bitwise_not(red_imrange)

        imrange = cv2.bitwise_and(not_imrange, ring_imrange)

        kernel = np.ones((3, 3), np.uint8)

        blurim = cv2.GaussianBlur(imrange, (1, 1), 0)
        erodeim = cv2.erode(blurim, kernel)
        dilateim = cv2.dilate(erodeim, kernel)

        contours, hierarchy = cv2.findContours(dilateim, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        flen = 652
        cam_pitch = 25
        cam_height = 0.5461
        
        for c in contours:
            if cv2.contourArea(c) < 1200:
                continue
            
            x, y, w, h = cv2.boundingRect(c)

            a = x + (w//2)
            b = y + (h//2)
            
            bndreclist.append([x,y,w,h,a,b])
            
        for b in bndreclist:
            cv2.rectangle(frame, (b[0], b[1]), (b[0]+b[2], b[1]+b[3]), (0, 255, 0), 2)

            pixel_y = b[5] - 240

            center_coordinates = b[4], b[5]

            color = (255, 0, 0)

            cv2.circle(frame, center_coordinates, 5, color, 2)

            a = math.atan(pixel_y / flen)
            b = math.radians(90 - cam_pitch)
            h = flen / math.cos(a)

            x = math.sin(b-a) * h
            y = math.cos(b-a) * h

            distance_to_note = x * (cam_height / y)

            print(distance_to_note)
        
        cv2.imshow("frame", frame)
                 
        if cv2.waitKey(1) == ord('q') & 0xff:
            break

if __name__ == "__main__":
    main()
