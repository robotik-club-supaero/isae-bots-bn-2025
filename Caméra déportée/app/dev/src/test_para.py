""" Test of the main script for the Raspberry Pi """

import os
import subprocess
import time
print("Importing opencv")
import cv2
print("Importing multiprocessing")
import multiprocessing
import json
print("Importing files")
from imageNormalizerRpi import ImageNormalizer
from detectRobotsInsideArucoRpi import detect_objects_inside_aruco
from lora import ModemConfig, LoRa
from picamera2 import Picamera2

# File paths
calibration_image_path = "../platformImages/calibration.jpg"

camera_test_image_path = "../platformImages/test_camera.jpg"

capture_image_path = "../platformImages/capture.jpg"
reference_image_path = "../platformImages/reference.jpg"

normalized_image_dir = "../normalizedImages"

npz_file_path="../npz/calibration.npz"

aruco_json_path = "../json/aruco.json"
distances_json_path = "../json/distances.json"

# Function to capture an image using the camera
def capture_image(output_path): # add ,q in final version
    # Command to capture an image using the camera
    # libcamera-jpeg is a custom command to capture images using the Raspberry Pi camera
    # -n no preview
    # -o output file
    command = f"libcamera-jpeg -n --output {output_path} --width 320 --height 240"
    subprocess.run(command, shell=True)

    #q.put(output_path)

def processImage(capture_image_path,normalized_image_dir,q):

    # Normalize the image using Aruco tags
    normalizedImage = imageNormalizer.normalizeImage(capture_image_path)

    print("Writing normalized capture...")
    normalized_output_path = os.path.join(normalized_image_dir, "normalized_" + os.path.basename(capture_image_path))
    cv2.imwrite(normalized_output_path, normalizedImage)
    print(f"Normalized capture saved to {normalized_output_path}")

    q.put(normalized_output_path)


def detectArucos(reference_image_path,normalized_output_path, aruco_json_path, distances_json_path,q):
    
    # Detect objects inside Aruco tags and return the coordinates
    rectangles = detect_objects_inside_aruco(
        reference_image_path,normalized_output_path, aruco_json_path, distances_json_path
    )
    print(rectangles)
    print("\n")

    q.put(rectangles)

def sendData(rectangles, lora, header_to):
    # Send the coordinates via LoRa
    print("Sending data.\n")
    for rectangle in rectangles:
        for item in rectangle:
            item = int(item)
            lora.send(item, header_to)

if __name__ == "__main__":

    print("Setting up camera...")
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (320, 240)})
    picam2.configure(config)
    picam2.start()


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
    print("Capturing image of the platform...")
    capture_image(camera_test_image_path)
    
    q = multiprocessing.Queue()

    start = time.time()

    #Prise de la première image
    p0 = multiprocessing.Process(target = capture_image, args = (camera_test_image_path,))
    p0.start()

    p0.join()

    #capture_image_path = q.get()
    # Processing de la première image
    p1 = multiprocessing.Process(target=processImage, args=(capture_image_path,normalized_image_dir,q))
    p1.start()

    # Prise de la deuxième image
    p0 = multiprocessing.Process(target = capture_image, args = (camera_test_image_path,))
    p0.start()

    p1.join()
    normalized_output_path = q.get()
    # Détection des Arucos sur la première image
    p2 = multiprocessing.Process(target=detectArucos, args=(reference_image_path,normalized_output_path, aruco_json_path, distances_json_path,q))
    p2.start()

    p0.join()
    #capture_image_path = q.get()
    #Processing de la deuxième image
    p1 = multiprocessing.Process(target=processImage, args=(capture_image_path,normalized_image_dir,q))
    p1.start()

    p2.join()
    rectangles = q.get()
    # Envoi des données de la première image
    p3 = multiprocessing.Process(target=sendData, args=(rectangles, lora, header_to))
    p3.start()

    p1.join()
    normalized_output_path = q.get()
    # Détection des Arucos sur la deuxième image
    p2 = multiprocessing.Process(target=detectArucos, args=(reference_image_path,normalized_output_path, aruco_json_path, distances_json_path,q))
    p2.start()
    p2.join()
    
    p3.join()
    rectangles = q.get()
    # Envoi des données de la deuxième image
    p3 = multiprocessing.Process(target=sendData, args=(rectangles, lora, header_to))
    p3.start()
    
    p3.join()

    lora.close()
    picam2.close()

    print(time.time() - start)
