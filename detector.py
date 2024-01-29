from dt_apriltags import Detector
import numpy as np
import argparse
import pathlib
import yaml
import math
import cv2
import os

def main():
    parser = argparse.ArgumentParser(prog='Apriltag Detector')
    parser.add_argument("intrinsics_file", type=pathlib.Path)
    args = parser.parse_args()

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

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        cv2.imshow("frame", frame)

        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        all_tags = at_detector.detect(grayscale,
                                      estimate_tag_pose=True,
                                      camera_params=camera_params,
                                      tag_size=0.163525)

        for tag in all_tags:
            tag_info = tag_positions[f"tag{tag.tag_id}"]
            tag_t = 0.0254 * np.array([[tag_info["z"]], [tag_info["y"]], [tag_info["x"]]])
            true_t = np.matmul(np.linalg.inv(tag.pose_R), tag.pose_t)

            theta = tag_info["theta"]

            Ry_april_world = np.linalg.inv(np.array([math.cos(theta), 0, math.sin(theta)], [0, 1, 0], [-math.sin(theta), 0, math.cos(theta)]))
            Ry_camera_april = np.linalg.inv(tag.pose_R)

            print(tag_t - np.matmul(Ry_april_world, np.matmul(Ry_camera_april, tag.pose_t)))
            
            break

        if cv2.waitKey(1) == ord('q') & 0xff:
            break

if __name__ == "__main__":
    main()
