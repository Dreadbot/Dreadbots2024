import cv2
from dt_apriltags import Detector
import numpy as np
import os
import yaml
import re

def main():
    at_detector = Detector(searchpath=['apriltags'],
                           families='tag36h11',
                           nthreads=2,
                           quad_sigma=0.2,
                           decode_sharpening=0.25)

    camera_params = [0] * 4

    with open("intrinsics.yaml") as f:
        params = yaml.safe_load(f)
        camera_params[0] = params["fx"]
        camera_params[1] = params["fy"]
        camera_params[2] = params["cx"]
        camera_params[3] = params["cy"]

    directory = 'out'
    for filename in os.listdir(directory):
        fname = os.path.join(directory, filename)
        print(f"--- {fname} ---")
        img = cv2.imread(fname)
        grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        all_tags = at_detector.detect(grayscale,
                                      estimate_tag_pose=True,
                                      camera_params=camera_params,
                                      tag_size=0.163525)
        for tag in all_tags:
            pose_t = tag.pose_t
            estimated = np.linalg.norm([pose_t[0], pose_t[2]]) * 3.28084
            actual = int(re.findall(r"\d+", fname)[0])
            print(tag.tag_id)
            print(f"Actual: {actual}\nEstimated: {estimated}")
            print(pose_t)
            print("")
        print("\n")

if __name__ == "__main__":
    main()
