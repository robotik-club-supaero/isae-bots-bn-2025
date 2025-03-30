""" Test of the main script for the Raspberry Pi """

import os
import subprocess
import time
print("Importing opencv")
import cv2
import json
print("Importing files")
from imageNormalizerRpi import ImageNormalizer
from detectRobotsInsideArucoRpi import detect_objects_inside_aruco
from lora import ModemConfig, LoRa

# File paths
calibration_image_path = "../platformImages/calibration.jpg"

capture_image_path = "../platformImages/capture.jpg"
reference_image_path = "../platformImages/reference.jpg"

normalized_image_dir = "../normalizedImages"

npz_file_path="../npz/calibration.npz"

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


print("Setting up  LoRa...")
lora = LoRa(channel=0, interrupt=17, this_address=2, modem_config=ModemConfig.Bw500Cr45Sf128, tx_power=14, acks=False)
header_to = 255 # Send to all

# Output directory for normalized images
print("Setting up output directory for normalized images...")
os.makedirs(normalized_image_dir, exist_ok=True)

# Capture an image of the platform for calibration
#print("Capturing image for calibration.\n")
#capture_image(calibration_image_path)

print("Setting up image normalizer...")
imageNormalizer = ImageNormalizer(reference_image_path, calibration_image_path, npz_file_path)
imageNormalizer.computeHomographyMatrix(calibration_image_path, draw=False)

# Capture an image of the platform
#print("Capturing image of the platform...")
#capture_image(capture_image_path)

# Normalize the image using Aruco tags
normalizedImage = imageNormalizer.normalizeImage(capture_image_path)

print("Writing normalized capture...")
normalized_output_path = os.path.join(normalized_image_dir, "normalized_" + os.path.basename(capture_image_path))
cv2.imwrite(normalized_output_path, normalizedImage)
print(f"Normalized capture saved to {normalized_output_path}")

# Detect objects inside Aruco tags and return the coordinates
rectangles = detect_objects_inside_aruco(
    reference_image_path,normalized_output_path, aruco_json_path, distances_json_path
)
print(rectangles)
print("\n")

# Send the coordinates via LoRa
print("Sending data.\n")
for rectangle in rectangles:
    for item in rectangle:
        item = int(item)
        lora.send(item, header_to)

# Wait for the next iteration
print("Sleeping.\n")
time.sleep(5)  # Adjust this value as needed

lora.close()
