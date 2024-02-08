import cv2
import numpy as np
import math
import ntcore

inst = ntcore.NetworkTableInstance.getDefault()
table = inst.getTable("datatable")

dblTopic = inst.getDoubleTopic("/datatable/X")

class Publisher:
    def __init__(self, dblTopic: ntcore.DoubleTopic):

        # start publishing; the return value must be retained (in this case, via
        # an instance variable)
        self.dblPub = dblTopic.publish()

        # publish options may be specified using PubSubOption
        self.dblPub = dblTopic.publish(ntcore.PubSubOptions(keepDuplicates=True))

        # publishEx provides additional options such as setting initial
        # properties and using a custom type string. Using a custom type string for
        # types other than raw and string is not recommended. The properties string
        # must be a JSON map.
        self.dblPub = dblTopic.publishEx("double", '{"myprop": 5}')

    def periodic(self):
        # publish a default value
        self.dblPub.setDefault(0.0)

        
        # publish a value with a specific timestamp with microsecond resolution.
        # On the roboRIO, this is the same as the FPGA timestamp (e.g.
        # RobotController.getFPGATime())
        self.dblPub.set(3.0, ntcore._now())

    # often not required in robot code, unless this class doesn't exist for
    # the lifetime of the entire robot program, in which case close() needs to be
    # called to stop publishing
    def close(self):
        # stop publishing
        self.dblPub.close()

def main():
    cap = cv2.VideoCapture(4)
    cap.set(cv2.CAP_PROP_EXPOSURE, -1)

    pub = Publisher(dblTopic)
    
    while True:
        ret, frame = cap.read()
        if not ret: break 

        bndreclist = []

        hsvim = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        upper_bound = np.array([25, 255, 255])
        lower_bound = np.array([0, 0, 0])
        ring_imrange = cv2.inRange(hsvim, lower_bound, upper_bound)

        red_upper_bound = np.array([7, 255, 255])
        red_lower_bound = np.array([0, 100, 0])
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
            if cv2.contourArea(c) < 800:
                continue
            
            x, y, w, h = cv2.boundingRect(c)

            a = x + (w//2)
            b = y + (h//2)
            
            bndreclist.append([x,y,w,h,a,b])


        bndreclist.sort(reverse = True, key = lambda rectangle: rectangle[2] * rectangle[3])
            
        if len(bndreclist) == 0:
            continue
        
        b = bndreclist[0]
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

        pub.dblPub.set(distance_to_note, ntcore._now())
    
        cv2.imshow("frame", frame)

        if cv2.waitKey(1) == ord('q') & 0xff:
            pub.close()
            break

if __name__ == "__main__":
    main()
