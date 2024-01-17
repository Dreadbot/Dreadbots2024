import numpy as np
import argparse
import tempfile
import pathlib
import sys
import os
import cv2

def main():
    parser = argparse.ArgumentParser(prog='Dreadbot Calibrator')
    parser.add_argument('-c', '--camera-id', type=int, help="Specify the id of the camera used to capture images.")
    parser.add_argument('-o', '--output-path', type=pathlib.Path, help="Specify filename of the outputted calibrations. Defaults to \x1B[34mintrinsics.yaml\x1B[0m.")
    parser.add_argument('-p', '--precaptured-images', type=pathlib.Path, metavar="DIR_PATH", help="Skip capturing images, instead loading them from a directory.")
    parser.add_argument('-s', '--store-images', type=pathlib.Path, metavar="DIR_PATH", help="Store captured images in a permanent directory.")
    parser.add_argument('-i', '--skip-calibration', action='store_true', help="Skip calibrating the camera, terminating the program after capturing images.")
    parser.add_argument('-x', '--rows', type=int, default=8, help="Specify the number of rows on the checkerboard. Defaults to 8.")
    parser.add_argument('-y', '--cols', type=int, default=7, help="Specify the number of columns on the checkerboard. Defaults to 7.")
    parser.add_argument('-v', action='count', default=0, help="Verbosity; the more this flag appears, the more debug information will be shown.")
    args = parser.parse_args()

    num_points_hori = args.cols - 1
    num_points_vert = args.rows - 1

    if args.v < 2:
        cv2.setLogLevel(0)

    if args.precaptured_images is not None and args.store_images is not None:
        print ("\x1B[91mYou cannot combine flags \x1B[34m-p\x1B[91m and \x1B[34m-s\x1B[91m together.\x1B[0m")
        exit(2)

    if args.precaptured_images is not None:
        dir = args.precaptured_images
        hard_store = True
    elif args.store_images is not None:
        dir = args.store_images
        hard_store = True
    else:
        dir = pathlib.Path(tempfile.mkdtemp())
        hard_store = False

    def get_camera_id():
        if args.camera_id is None:
            print("\x1B[91mNo camera id specified. Use \x1B[34m-c\x1B[91m or \x1B[34m--camera-id\x1B[91m to set it.\x1B[0m")
            exit(2)

    # Gather images
    def gather():
        camera_id = get_camera_id()
        cap = cv2.VideoCapture(camera_id)

        if cap is None:
            print(f"\x1B[91mCould not open camera.\x1B[0m")
            exit(1)

        img_num = 0
        while True:
            ret, frame = cap.read()
            if ret == False:
                print("\x1B[91mCould not read from camera.\x1B[0m")
                exit(1)
            cv2.imshow("frame", frame)

            key = cv2.waitKey(1)
            if key == ord('c') & 0xff:
                path = f"{dir}/img{img_num}.png"
                cv2.imwrite(path)
                if args.verbosity >= 2 or (hard_store and args.verbosity == 1):
                    print(f"Wrote to \x1B[34m{path}\x1B[0m")
                img_num += 1
            elif key == ord('q') & 0xff:
                break

        cap.close()
        
    # Calibrate from images
    def calibrate():
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        cv2.TERM_CRITERIA_EPS
    
        objp = np.zeros((num_points_hori*num_points_vert, 3), np.float32)
        objp[:,:2] = np.mgrid[0:num_points_vert,0:num_points_hori].T.reshape(-1,2)

        objpoints = []
        imgpoints = []

        paths = os.listdir(dir)
        for path in paths:
            img = cv2.imread(str(dir / path))
            if img is None:
                if args.v >= 1:
                    print(f"\x1B[34m{path}\x1B[33m is not an image, skipping\x1B[0m")
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, (num_points_vert, num_points_hori), None)

            if not ret:
                if args.v >= 1:
                    print(f"\x1B[34m{path}\x1B[33m has no detectable checkerboard, skipping\x1B[0m")
                continue

            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            if args.v >= 1:
                cv2.drawChessboardCorners(img, (num_points_vert, num_points_hori), corners2, ret)
                cv2.imshow("img", img)
                cv2.waitKey(500)
    
        cv2.destroyAllWindows()

        if len(objpoints) == 0:
            print(f"\x1B[91mNot enough data to calibrate.\x1B[0m")
            exit(1)

        val = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        print(val)

    if args.precaptured_images is None:
        gather()
    if not args.skip_calibration:
        calibrate()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\x1B[91mAborted\x1B[0m")
        exit(130)
