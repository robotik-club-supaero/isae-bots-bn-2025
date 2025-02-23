""" Camera calibration script """
"""   Command line version    """

# To calibrate the camera, we need to take several pictures of a chessboard pattern from different angles and distances.
# The script will detect the corners of the chessboard in each image and use them to calibrate the camera, using the calibration.json file.
# The calibration results will be saved to a file in the output directory.


import json, argparse, traceback, glob, datetime
import os, sys
import numpy as np
import cv2
from loadImage import loadImage
from undistort import Undistort


# Parse arguments
parser = argparse.ArgumentParser(prog='Camera calibration')
parser.add_argument('calibration_file')
parser.add_argument('--output', '-o', default=".", required=False, help="output directory") 
parser.add_argument('--draw', '-d', action="store_true", help="draw detected corners on calibration images") 
parser.add_argument('--epsilon', required=False, type=float, default=1e-3, help="TERM_CRITERIA_EPS in cv2.cornerSubPix") 
parser.add_argument('--max-count', '-c', required=False, type=int, default=30, help="TERM_CRITERIA_MAX_ITER in cv2.cornerSubPix")
args = parser.parse_args()

# Command example:
# python calibrateCamera.py ../json/calibration.json -o ../../calibration/calibrationResults --draw --epsilon 1e-3 --max-count 30

# The calibration file should be a JSON file with the following structure:
# {
#     "cols": 9,
#     "rows": 6,
#     "images": "./calibration_images/*.jpg"
# }
# where:
# - "cols" and "rows" are the number of corners in the chessboard pattern
# - "images" is either a list of paths to the calibration images or a "glob" string to match the images


# Output directory
os.makedirs(args.output, exist_ok=True)

EPSILON = args.epsilon
MAX_COUNT = args.max_count

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, MAX_COUNT, EPSILON)

with open(args.calibration_file) as file:
    value = json.load(file)
    SIZE = (value["cols"], value["rows"])

    CHESSBOARD = np.zeros((SIZE[0] * SIZE[1], 3), dtype=np.float32)
    CHESSBOARD[:, :2] = np.mgrid[:SIZE[0], :SIZE[1]].T.reshape(-1, 2)

    IMAGE_SIZE = None

    objpoints = []
    imgpoints = []

    if isinstance(value["images"], list):
        images = value["images"]
    elif isinstance(value["images"], str):
        print("C'est un string !\n")
        images = glob.glob(value["images"])
    else:
        raise ValueError("`images` must be either a list of paths or a \"glob\" string")
    
    print("images: ", images)
    print("\n")
    
    for path in images:
        print("Dans le for\n")
        image = cv2.imread(path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        if IMAGE_SIZE is None:
            IMAGE_SIZE = gray.shape[::-1]
        elif IMAGE_SIZE != gray.shape[::-1]:
            raise ValueError("All the images should have the same size")

        ret, corners = cv2.findChessboardCorners(gray, SIZE, None)
        if ret:
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)

            objpoints.append(CHESSBOARD)
            print(objpoints)
            print(IMAGE_SIZE)
            print("\n")
            imgpoints.append(corners2)

            if args.draw:
                try:
                    cv2.drawChessboardCorners(image, SIZE, corners2, ret)
                    o_path = os.path.join(args.output, os.path.basename(path))
                    cv2.imwrite(o_path, image)
                    print(f"Drawn corners to `{o_path}`")
                except:
                    print(f"Warning: Failed to draw corners for image `{path}`", file=sys.stderr)
                    traceback.print_exc()

        else:
            print(f"Warning: Cannot detect corners in image `{path}` (skipped)", file=sys.stderr)
    
    if IMAGE_SIZE is None or len(objpoints) == 0:
        raise ValueError("At least one successful image is required for calibration!")

    print("Corner detection complete. Calibrating camera... This may take a few seconds...")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, IMAGE_SIZE, None, None)
    
    if args.draw:
        undistort = Undistort(mtx=mtx, dist=dist)
        for path in images:
            image = undistort(path)
            o_path = os.path.join(args.output, "undistorted_" + os.path.basename(path))
            cv2.imwrite(o_path, image)
            print(f"Drawn undistorted image to `{o_path}`")
    
    if not ret:
        print("Warning: Camera calibration failed! Output results may be inaccurate.", file=sys.stderr)
    
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )
    
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, IMAGE_SIZE, 1, IMAGE_SIZE)

    o_path = datetime.datetime.now().strftime("calibration_%d_%m_%Y_%H_%M_%S.npz")
    o_path = os.path.join(args.output, o_path)

    np.savez(o_path, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs, newcameramtx=newcameramtx, roi=roi)
    print(f"Calibration results saved to {o_path}")