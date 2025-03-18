""" Test of the main script for the Raspberry Pi """

import os
import subprocess
import time
import cv2
import json
from normalizeImageRpi import normalize_image
from detectRobotsInsideArucoRpi import detect_objects_inside_aruco
from lora import ModemConfig, LoRa

# File paths
capture_image_path = "../platformImages/capture.jpg"
reference_image_path = "../platformImages/reference.jpg"
normalized_image_path = "../normalizedImages/normalized_platform.jpg"
calibration_path="../npz/calibration.npz"
aruco_json_path = "../json/aruco.json"
distances_json_path = "../json/distances.json"

# Function to capture an image using the camera
def capture_image(output_path):
    # Command to capture an image using the camera
    # libcamera-jpeg is a custom command to capture images using the Raspberry Pi camera
    # -n no preview
    # -o output file
    command = f"libcamera-jpeg -n -o {output_path}"
    subprocess.run(command, shell=True)


if __name__ == "__test__":

    #lora = LoRa(channel=0, interrupt=17, this_address=2, modem_config=ModemConfig.Bw500Cr45Sf128, tx_power=14, acks=False)
    

    # Capture an image using the camera
    capture_image(capture_image_path)

    # Normalize the image using Aruco tags
    normalize_image(reference_image_path, capture_image_path, calibration_path=None, output_dir=normalized_image_path)
    
    # Detect objects inside Aruco tags
    rectangles, aruco_tags_in_objects = detect_objects_inside_aruco(
        reference_image_path, normalized_image_path, aruco_json_path, distances_json_path
    )
    
    # Prepare the coordinates to be sent via LoRa
    coordinates = {
        "rectangles": rectangles,
        "aruco_tags_in_objects": aruco_tags_in_objects
    }
    
    # Send the coordinates via LoRa
    #header_to = 1
    #lora.send(coordinates, header_to)
    
    # Wait for the next iteration
    time.sleep(10)  # Adjust this value as needed