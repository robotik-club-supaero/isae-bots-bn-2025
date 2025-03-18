""" Test of the main script for the Raspberry Pi """

import os
import subprocess
import time
print("Importing opencv")
import cv2
import json
print("Importing files")
from normalizeImageRpi import normalize_image
from detectRobotsInsideArucoRpi import detect_objects_inside_aruco
from lora import ModemConfig, LoRa

# File paths
capture_image_path = "../platformImages/capture.jpg"
reference_image_path = "../platformImages/reference.jpg"
normalized_image_dir = "../normalizedImages"
normalized_image_path = "../normalizedImages/normalized_capture.jpg"
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


print("Setting up  LoRa.\n")
lora = LoRa(channel=0, interrupt=17, this_address=2, modem_config=ModemConfig.Bw500Cr45Sf128, tx_power=14, acks=False)
header_to = 255 # Send to all

# Capture an image using the camera
print("Capturing image.\n")
#capture_image(capture_image_path)

# Normalize the image using Aruco tags
print("Normalizing image.\n")
normalize_image(reference_image_path, capture_image_path, calibration_path, output_dir=normalized_image_dir)

# Detect objects inside Aruco tags
rectangles, aruco_tags_in_objects = detect_objects_inside_aruco(
    reference_image_path, normalized_image_path, aruco_json_path, distances_json_path
)

# Prepare the coordinates to be sent via LoRa
coordinates = {
    "rectangles": rectangles,
    "aruco_tags_in_objects": aruco_tags_in_objects
}
print(coordinates)
print("\n")

# Convert the coordinates to a list
coordinates = list(coordinates.items())
print(coordinates)
print("\n")

# Send the coordinates via LoRa
print("Sending data.\n")
lora.send(coordinates, header_to)

# Wait for the next iteration
print("Sleeping.\n")
time.sleep(10)  # Adjust this value as needed
lora.close()
