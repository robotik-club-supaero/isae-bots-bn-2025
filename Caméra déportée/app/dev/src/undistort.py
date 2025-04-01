""" Class to undistort images using camera matrix and distortion coefficients """

import numpy as np
import cv2
from loadImage import loadImage

class Undistort:

    def __init__(self, mtx=None, dist=None, path=None):
        if mtx is None and dist is None and path is None:
            raise ValueError("Either 'mtx' and 'dist' or 'path' must be given")
        if ((mtx is None) != (dist is None)) or ((mtx is None) == (path is None)):
            raise ValueError("'mtx' and 'dist' cannot be given along with 'path'")
        
        if path is not None:
            data = np.load(path)
            self._mtx, self._dist = data["mtx"], data["dist"]
        else:
            self._mtx, self._dist = mtx, dist

    def computeCameraMatrix(self, image_path):

        print("Computing camera matrix...")

        image = loadImage(image_path)
        h,  w = image.shape[:2]
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(self._mtx, self._dist, (w,h), 0, (w,h))
        mapx, mapy = cv2.initUndistortRectifyMap(self._mtx, self._dist, None, newCameraMatrix, (w, h), 5)
        return mapx, mapy, roi

        
    def undistort(self, image_path, mapx, mapy, roi):
        
        print("Undistorting image...")
        
        image = loadImage(image_path)
        
        dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
        
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        return dst