""" Script to normalize images using Aruco tags on the platform """
"""                    Command line version                     """

# To normalize an image of the platform, we need to take a reference image of the platform with the markers,
#  and a real image of the same scene taken by the camera.
# The script will detect the Aruco markers of the platform in both images and use them to calculate the homography matrix.
# The homography matrix will be used to normalize the real image to match the reference image.
# The normalized image will be saved to the output directory.
# To perform the calculation, the camera calibration file has to be provided to undistort the images prior to detecting the Aruco markers.
# This calibration file can be generated using the calibrateCamera.py script.


import cv2
import argparse, os, datetime
import numpy as np
from loadImage import loadImage
from undistort import Undistort
from aruco import ArucoDetector, ArucoTags, ImageNormalize 

# Parse arguments
parser = argparse.ArgumentParser(prog='Aruco-tags-based homography')
parser.add_argument('reference', help="path of the reference image")
parser.add_argument('capture', help="path of a real image of the same scene taken by the camera")
parser.add_argument('--calibration', '-c', required=False, help="camera calibration file used to undistorted images prior to detecting Aruco tags") 
parser.add_argument('--output', '-o', default=".", required=False, help="output directory") 
args = parser.parse_args()

# Command example:
# python normalizeImage.py ../platformImages/reference.jpg ../platformImages/capture.jpg -c ../npz/calibration.npz -o ../normalizedImages

# Output directory
os.makedirs(args.output, exist_ok=True)
    
preprocess = Undistort(path=args.calibration) if args.calibration is not None else None    
normalize = ImageNormalize(args.reference, preprocess=preprocess)

tagged_capture = normalize.calibrate(args.capture, draw=True)

o_path = os.path.join(args.output, "tagged_" + os.path.basename(args.capture))
cv2.imwrite(o_path, tagged_capture)
print(f"Tagged capture saved to {o_path}")
    
normalized_capture = normalize(args.capture)

o_path = os.path.join(args.output, "normalized_" + os.path.basename(args.capture))
cv2.imwrite(o_path, normalized_capture)
print(f"Normalized capture saved to {o_path}")