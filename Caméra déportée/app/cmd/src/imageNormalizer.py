""" Script to normalize images using Aruco tags on the platform """

import cv2
import os
import numpy as np
from undistort import Undistort
from aruco import ArucoDetector, ArucoTags

class ImageNormalizer:

    def __init__(self, reference_image_path, calibration_image_path, npz_file_path=None):

        # Set the preprocess to undistort the image if a calibration path is provided
        self._preprocess = Undistort(path=npz_file_path) if npz_file_path is not None else None
        
        # Compute the camera matrix for the preprocess
        self._dstMap1, self._dstMap2, self._roi = self._preprocess.computeCameraMatrix(calibration_image_path) if self._preprocess else (None, None)
        
        # Detector for Aruco tags
        self._detector = ArucoDetector()

        # Load the reference image for Aruco tags
        self._reference = self._detector(cv2.imread(reference_image_path))
        # Shape of the reference image
        self._shape = self._reference._image.shape
        
        self._homographyMatrixComputed = False
        
        # Homography matrix to normalize the image
        self._H = None

    def computeHomographyMatrix(self, calibration_image_path, draw=True):
        
        print("Begin computeHomographyMatrix...")

        calibrator = self._preprocess.undistort(calibration_image_path, self._dstMap1, self._dstMap2, self._roi)

        print("Applying detector on calibration image...")
        calibrator = self._detector(calibrator)

        print("Loading reference...")        
        reference = self._reference
     
        src_pts = []
        dst_pts = []

        print("Calculating source points and distance points...")
        for tag in calibrator:
            if tag in reference:
                src_pts.extend(reference[tag])
                dst_pts.extend(calibrator[tag])

        print("Computing homography...")
        self._H, _ = cv2.findHomography(np.array(src_pts), np.array(dst_pts), cv2.RANSAC)
        self._homographyMatrixComputed = True
        print("Computation complete.")
        
        if draw:
            return calibrator.draw()
    
    def normalizeImage(self, image_path):     
        
        print("Begin normalizeImage...")   

        if not self._homographyMatrixComputed:
            print("Homography Matrix not computed...")
            self.computeHomographyMatrix(image_path)

        size = self._shape[1::-1] # width, height

        output = np.uint8(255 * np.ones((size[0], size[1], 3))) # white image     
        
        image = self._preprocess.undistort(image_path, self._dstMap1, self._dstMap2, self._roi)
        
        print("Warping...")
        output = cv2.warpPerspective(image, self._H, size, output, flags=cv2.INTER_LINEAR+cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_TRANSPARENT)
        return output # normalized image
