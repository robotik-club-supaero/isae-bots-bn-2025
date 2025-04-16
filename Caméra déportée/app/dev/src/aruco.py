""" Classes for detecting Aruco tags on images and normalizing images with Aruco tags """

import cv2
import numpy as np

class ArucoDetector:
    _dictionary =  cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _params = cv2.aruco.DetectorParameters()
   
    def __init__(self):
        self._detector = cv2.aruco.ArucoDetector(ArucoDetector._dictionary, ArucoDetector._params)
    
    def detect(self, image):
        tags = {}
        corners, ids, _ = self._detector.detectMarkers(image)
        for box, tag in zip(corners, ids):
            # topLeft, topRight, bottomRight, bottomLeft
            tags[tag.item()] = box.squeeze()

        return ArucoTags(image, tags)

    def __call__(self, image):
        return self.detect(image)
    
class ArucoTags:

    def __init__(self, image, tags):
        self._image = image
        self._tags = tags
    
    def __getitem__(self, index):
        return self._tags[index]
    
    def __iter__(self):
        return iter(self._tags)
    
    def __contains__(self, tag):
        return tag in self._tags
    
    def draw(self):
        image = self._image.copy()
        for tag in self:            
            corners = self[tag].reshape((4, 2)).astype(int)
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            cv2.putText(image, str(tag),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
        
        return image