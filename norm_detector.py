from dt_apriltags import Detector
import numpy as np
import argparse
import pathlib
import yaml
import math
import cv2
import os
import ntcore
import time
from cscore import CameraServer

CameraServer.enableLogging()

camera = CameraServer.putVideo("backcam", 640//8, 480//8)

def start_network_table():
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("azathoth")
    inst.startClient4("visionclient")
    inst.setServerTeam(3656)
    xPub = table.getDoubleTopic("robotposX").publish()
    zPub = table.getDoubleTopic("robotposZ").publish()
    thetaPub = table.getDoubleTopic("robotposTheta").publish()
    tagSeenPub = table.getBooleanTopic("tagSeen").publish()
    return xPub, zPub, thetaPub, tagSeenPub

#def init_network_tables() -> tuple[ntcore.DoubleTopic, ntcore.DoubleTopic, ntcore.DoubleTopic]:
#    inst = ntcore.NetworkTableInstance.getDefault()
#    table = inst.getTable("SmartDashboard")
#    return (table.getDoubleTopic("robotposX").publish(), table.getDoubleTopic("robotposZ").publish(), table.getDoubleTopic("robotposTheta").publish())

def main():
    xPub, zPub, thetaPub, tagSeenPub = start_network_table()
    parser = argparse.ArgumentParser(prog='Apriltag Detector')
    parser.add_argument("intrinsics_file", type=pathlib.Path)
    args = parser.parse_args()

    #xPub, zPub, thetaPub = init_network_tables()

    with open("field.yaml", 'r') as f:
        tag_positions = yaml.safe_load(f)

    at_detector = Detector(searchpath=['apriltags'],
                           families='tag36h11',
                           nthreads=2,
                           quad_sigma=0.2,
                           decode_sharpening=0.25)

    camera_params = [0] * 4

    print(args.intrinsics_file);
    with open(args.intrinsics_file, 'r') as f:
        params = yaml.safe_load(f)
        camera_params[0] = params["fx"]
        camera_params[1] = params["fy"]
        camera_params[2] = params["cx"]
        camera_params[3] = params["cy"]

    cap = cv2.VideoCapture(cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, .25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -13)
    ret, frame = cap.read()
    print(ret)
    print("is the frame healthy")

    lastTime = 0
    while True:
        ret, frame = cap.read()

        deltaTime = time.process_time() - lastTime
        if deltaTime >= 1/18:
            lastTime = time.process_time()
            camera.putFrame(cv2.resize(frame, (640//8, 480//8)))

        if not ret:
            continue

        #cv2.imshow("frame", frame)

        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        all_tags = at_detector.detect(grayscale,
                                      estimate_tag_pose=True,
                                      camera_params=camera_params,
                                      tag_size=0.163525)
        tagSeenPub.set(not not all_tags)
        for tag in all_tags:
            if (tag.tag_id != 4):
                continue
            tag_info = tag_positions[f"tag{tag.tag_id}"]
            tag_t = 0.0254 * np.array([[tag_info["z"]], [tag_info["y"]], [tag_info["x"]]])
            true_t = np.matmul(np.linalg.inv(tag.pose_R), tag.pose_t)

            alpha = math.atan2(-tag.pose_R[2][0], math.sqrt(tag.pose_R[2][1]**2 + tag.pose_R[2][2]**2));
            conv_angle = -(math.pi - alpha - math.radians(tag_info["theta"]))
            conv_angle_x = 26 * math.pi / 180

            Rx_robot_world = np.linalg.inv(np.array([[1, 0, 0], [0, math.cos(conv_angle_x), -math.sin(conv_angle_x)], [0, math.sin(conv_angle_x), math.cos(conv_angle_x)]]))
            Ry_robot_world = np.linalg.inv(np.array([[math.cos(conv_angle), 0, math.sin(conv_angle)], [0, 1, 0], [-math.sin(conv_angle), 0, math.cos(conv_angle)]]))

            print(np.matmul(Ry_robot_world, np.matmul(Rx_robot_world, tag.pose_t)))

            print(math.degrees(alpha))
            print(np.matmul(Ry_robot_world, tag.pose_t))
            full_t = tag_t - np.matmul(Ry_robot_world, np.matmul(Rx_robot_world, tag.pose_t))
            print(full_t)
            print("---")

            xPub.set(1.14935 + full_t[0])
            zPub.set(full_t[2])
            thetaPub.set(conv_angle)

        #if cv2.waitKey(1) == ord('q') & 0xff:
        #    break

if __name__ == "__main__":
    main()
