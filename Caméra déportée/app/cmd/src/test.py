""" Test of the image processing functions """

import os
import subprocess
import time
print("Importing opencv")
import cv2
import json
import struct
print("Importing files")
from imageNormalizerRpi import ImageNormalizer
from detectRobotsInsideArucoRpi import detect_objects_inside_aruco
from lora import ModemConfig, LoRa
#from picamera2 import Picamera2

# File paths
calibration_image_path = "../platformImages/calibration.jpg"

camera_test_image_path = "../platformImages/test_camera.jpg"

capture_image_path = "../platformImages/capture.jpg"
reference_image_path = "../platformImages/reference.jpg"

normalized_image_dir = "../normalizedImages"

npz_file_path="../npz/calibration.npz"

aruco_json_path = "../json/aruco.json"
distances_json_path = "../json/distances.json"

""" # Uncomment the following lines if you want to use the camera
print("Setting up camera...")
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (320, 240)})
picam2.configure(config)
picam2.start()

"""

"""# Uncomment the following lines if you want to use the lora module
print("Setting up  LoRa...")
lora = LoRa(channel=0, interrupt=17, this_address=2, modem_config=ModemConfig.Bw500Cr45Sf128, tx_power=14, acks=False)
header_to = 255 # Send to all
lora.set_mode_tx()
"""

# Output directory for normalized images
print("Setting up output directory for normalized images...")
os.makedirs(normalized_image_dir, exist_ok=True)


# Capture an image of the platform for calibration
#print("Capturing image for calibration.\n")
#picam2.capture_file("test.jpg")

print("Setting up image normalizer...")
imageNormalizer = ImageNormalizer(reference_image_path, calibration_image_path, npz_file_path)
imageNormalizer.computeHomographyMatrix(calibration_image_path, draw=False)

start = time.time()

# Capture an image of the platform
#print("Capturing image of the platform...")
#picam2.capture_file(camera_test_image_path)

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
flatlist = [coord for rectangle in rectangles for coord in rectangle]
nb_rectangles = len(rectangles)
len_flatlist = len(flatlist)
payload = struct.pack(f'i{len_flatlist}i', nb_rectangles,*flatlist)
print("Sending data.\n")
#lora.send(payload, header_to)

print(time.time() - start)

#lora.close()
#picam2.close()


