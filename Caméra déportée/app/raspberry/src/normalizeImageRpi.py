""" Script to normalize images using Aruco tags on the platform """
"""                  Version for Raspberry Pi                   """

import cv2
import os
import numpy as np
from loadImage import loadImage
from undistort import Undistort
from aruco import ArucoDetector, ArucoTags, ImageNormalize 

def normalize_image(reference_path, capture_path, calibration_path=None, output_dir="."):
    """
    Normalize an image using Aruco tags on the platform.

    Args:
        reference_path (str): Path to the reference image.
        capture_path (str): Path to the real image of the same scene taken by the camera.
        calibration_path (str, optional): Path to the camera calibration file used to undistort images prior to detecting Aruco tags.
        output_dir (str, optional): Output directory to save the normalized image. Defaults to the current directory.

    Returns:
        str: Path to the saved normalized image.
    """
    # Output directory
    os.makedirs(output_dir, exist_ok=True)
    
    preprocess = Undistort(path=calibration_path) if calibration_path is not None else None    
    normalize = ImageNormalize(reference_path, preprocess=preprocess)

    tagged_capture = normalize.calibrate(capture_path, draw=True)

    tagged_output_path = os.path.join(output_dir, "tagged_" + os.path.basename(capture_path))
    cv2.imwrite(tagged_output_path, tagged_capture)
    print(f"Tagged capture saved to {tagged_output_path}")
        
    normalized_capture = normalize(capture_path)

    normalized_output_path = os.path.join(output_dir, "normalized_" + os.path.basename(capture_path))
    cv2.imwrite(normalized_output_path, normalized_capture)
    print(f"Normalized capture saved to {normalized_output_path}")

    return 0

# Example usage:
# normalized_image_path = normalize_image(
#     reference_path="../platformImages/reference.jpg",
#     capture_path="../platformImages/capture.jpg",
#     calibration_path="../npz/calibration.npz",
#     output_dir="../normalizedImages"
# )