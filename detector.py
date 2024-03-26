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
import dataclasses
import wpiutil
from wpiutil import wpistruct
#from cscore import CameraServer

INCHES_TO_METERS = 0.0254
CAMERA_PITCH = math.radians(90 - 26)
CAMERA_TO_ROBOT_OFFSET = 0.3937

# IF NECESSARY :
# run this to list camera properties
# v4l2-ctl -d /dev/video0 -l
# check brightness, auto_exposure, exposure_time_absolute
# auto_exposure MUST be 1
# brightness SHOULD be high
# exposure_time_absolute SHOULD be very low (<10)
# if necessary
# v4l2-ctl -d /dev/video0 -c exposure_time_absolute=10,brightness=300

#CameraServer.enableLogging()

#camera = CameraServer.putVideo("backcam", 640//8, 480//8)
# @wpiutil.wpistruct.make_wpistruct(name="position")
class Position:
    def __init__(self, x: wpiutil.wpistruct.double, y: wpiutil.wpistruct.double, ID: wpiutil.wpistruct.int16):
        self.x = x
        self.y = y
        self.ID = ID

    def pack(self):
        bb = bytearray()
        self.packInto(bb)
        return bytes(bb)

    def packInto(self, bb: bytearray):
        bb.extend(self.x.tobytes())
        bb.extend(self.y.tobytes())
        bb.extend(self.ID.to_bytes(4, byteorder='little'))

    def unpack(bb: bytearray):
        raise NotImplementedError("who called this?")

    WPIStruct = wpiutil.wpistruct.StructDescriptor(
        "struct:position",
        "double x;double y;int ID",
        8 + 8 + 4,
        pack,
        packInto,
        unpack,
        None,
    )

def start_network_table():
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("azathoth")
    inst.startClient4("visionclient")
    inst.setServerTeam(3656)
    positionPub = table.getStructArrayTopic("visionPos", Position).publish()
    latencyPub = table.getDoubleTopic("visionLatency").publish()
    tagSeenPub = table.getBooleanTopic("tagSeen").publish()
    return tagSeenPub, latencyPub, positionPub

#def init_network_tables() -> tuple[ntcore.DoubleTopic, ntcore.DoubleTopic, ntcore.DoubleTopic]:
#    inst = ntcore.NetworkTableInstance.getDefault()
#    table = inst.getTable("SmartDashboard")
#    return (table.getDoubleTopic("robotposX").publish(), table.getDoubleTopic("robotposZ").publish(), table.getDoubleTopic("robotposTheta").publish())

def test_main():
    tagSeenPub, latencyPub, positionPub = start_network_table()
    while True:
        tagSeenPub.set(True)
        positionPub.set(list([Position(np.float64(1.0),np.float64(2.0),3)]))

def main():
    parser = argparse.ArgumentParser(prog='Apriltag Detector')
    parser.add_argument("intrinsics_file", type=pathlib.Path)
    args = parser.parse_args()

    tagSeenPub, latencyPub, positionPub = start_network_table()

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

    cap = cv2.VideoCapture(-1)
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, .25)
    # cap.set(cv2.CAP_PROP_EXPOSURE, -100)
    ret, frame = cap.read()
    print(ret)
    print("is the frame healthy")

    lastTime = 0
    
    while True:
        frame_start = time.process_time()
        ret, frame = cap.read()

        deltaTime = time.process_time() - lastTime
        if deltaTime >= 1/18:
            lastTime = time.process_time()
            #camera.putFrame(cv2.resize(frame, (640//8, 480//8)))
        #cv2.imshow("frame", frame)

        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        all_tags = at_detector.detect(grayscale,
                                      estimate_tag_pose=True,
                                      camera_params=camera_params,
                                      tag_size=0.163525)

        seen_tag = False

        visionPositions = []
        for tag in all_tags:
            #if ((tag.tag_id != 4 and allianceSub.get()) or (tag.tag_id != 7 and not allianceSub.get())):
            if tag.pose_err > 5.0e-7:
                continue
            seen_tag = True
            
            # tag.pose_t[2] += 0.4064
            # tag_info = tag_positions[f"tag{tag.tag_id}"]
            # tag_t = INCHES_TO_METERS * np.array([[tag_info["z"]], [tag_info["y"]], [tag_info["x"]]])
            # gyro_angle = -gyroThetaSub.get()

            tag_in_robot_frame = calculate_tag_in_robot_frame(tag.pose_t)            # robot_rotation = np.array([
            #     [math.cos(gyro_angle), math.sin(gyro_angle), 0, 0],
            #     [-math.sin(gyro_angle), math.cos(gyro_angle), 0, 0],
            #     [0, 0, 1, 0],
            #     [0, 0, 0, 1]
            # ], dtype="object")
            # tag_in_world_frame = np.matmul(robot_rotation, tag_in_robot_frame)
            xPos = tag_in_robot_frame[0][0][0]
            yPos = tag_in_robot_frame[1][0][0]
            tagID = tag.tag_id
            visionPositions.append(Position(xPos, yPos, tagID))
            
            # print(tag.pose_t)
            # print(tag_in_robot_frame)
            # print(tag_in_world_frame)
        positionPub.set(visionPositions)
        latencyPub.set(time.process_time() - frame_start)
        tagSeenPub.set(seen_tag)
        #if cv2.waitKey(1) == ord('q') & 0xff:
        #    break

def calculate_tag_in_robot_frame(tag_in_camera_frame):
    tag_in_camera_frame = np.array([[tag_in_camera_frame[0]], [tag_in_camera_frame[1]], [tag_in_camera_frame[2]], [1]], dtype="object")
    pitch_rotation = np.array([
        [1, 0, 0, 0],
        [0, math.cos(CAMERA_PITCH), math.sin(CAMERA_PITCH), 0],
        [0, -math.sin(CAMERA_PITCH), math.cos(CAMERA_PITCH), 0],
        [0, 0, 0, 1]
    ], dtype="object")
    camera_axes_to_robot_axes = np.array([
        [math.cos(math.pi / 2), math.sin(math.pi / 2), 0, 0],
        [-math.sin(math.pi / 2), math.cos(math.pi / 2), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype="object")
    camera_origin_to_robot_origin = np.array([
        [1, 0, 0, CAMERA_TO_ROBOT_OFFSET],
        [0, 1, 0, 0], # -1 due to terrible left handedness of coordinate system
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype="object")
    overall_transformation = np.matmul(camera_origin_to_robot_origin, np.matmul(camera_axes_to_robot_axes, pitch_rotation))
    tag_in_robot_frame = np.matmul(overall_transformation, tag_in_camera_frame)
    return tag_in_robot_frame

if __name__ == "__main__":
    main()
    # test_main()
