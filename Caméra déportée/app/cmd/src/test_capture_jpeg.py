#!/usr/bin/python3

# Capture a JPEG while still running in the preview mode. When you
# capture to a file, the return value is the metadata for that image.

import time

from picamera2 import Picamera2

picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (320, 240)})
picam2.configure(config)

print("Start...")
picam2.start()

print("Sleep...")
time.sleep(2)

print("Saving...")
picam2.capture_file("test.jpg")

picam2.close()
