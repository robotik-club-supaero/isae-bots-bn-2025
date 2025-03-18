""" Function to load an image from a file path or return the image as-is """

import cv2
import numpy as np

def loadImage(image):
    """Loads the image.

    - If `image` is `str`, it will be interpreted as the image's file path.
    - Otherwise, `image` is returned as-is
    
    Raises:
    ---------
    ValueError: If `image` is `str` and the image cannot be loaded from the file path."""
    if isinstance(image, str):
        loaded_image = cv2.imread(image)
        if loaded_image is None:
            raise ValueError(f"Image at path '{image}' could not be loaded.")
        return loaded_image

    return image