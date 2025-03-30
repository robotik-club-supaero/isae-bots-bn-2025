""" Script to detect objects inside an Aruco tag in a reference image and a capture image """
"""                                Version for Raspberry Pi                               """

import cv2
import sys
import json
from loadImage import loadImage
from aruco import ArucoDetector, ArucoTags

def detect_objects_inside_aruco(reference_image_path, capture_image_path, aruco_json_path, distances_json_path):
    # Load images
    image_reference = loadImage(reference_image_path)
    image_with_objects = loadImage(capture_image_path)

    # Load Aruco tags
    aruco_tags = json.load(open(aruco_json_path))

    # Convert the dictionary to a dictionary with keys as values and values as keys
    id_to_key = {v: k for k, v in aruco_tags.items()}

    # Load safety distances
    safety_distances = json.load(open(distances_json_path))

    # Define the Aruco tags of the platform
    platform_tags_id = [
        aruco_tags["table_up_left"],
        aruco_tags["table_up_right"],
        aruco_tags["table_down_left"],
        aruco_tags["table_down_right"]
    ]

    # Check if the images were loaded successfully
    if image_reference is None:
        print("Erreur: Impossible de charger l'image référence")
        sys.exit(1)
    if image_with_objects is None:
        print("Erreur: Impossible de charger l'image normalisée")
        sys.exit(1)

    # Detect Aruco tags in the reference and capture images
    aruco_detector = ArucoDetector()
    tags_reference = aruco_detector(image_reference)
    tags_with_objects = aruco_detector(image_with_objects)

    # Extract the corners of the Aruco tags
    def get_corners(tags, platform_tag_ids, platform):
        corners = []
        ids = []
        if platform:
            for tag_id in platform_tag_ids:
                if tag_id in tags:
                    tag_corners = tags[tag_id].reshape((4, 2)).astype(int)
                    corners.append(tag_corners)
                    ids.append(tag_id)
        else:
            for tag_id in tags:
                if tag_id not in platform_tag_ids:
                    tag_corners = tags[tag_id].reshape((4, 2)).astype(int)
                    corners.append(tag_corners)
                    ids.append(tag_id)
        return corners, ids

    corners_reference, _ = get_corners(tags_reference, platform_tags_id, platform=True)
    corners_with_objects, _ = get_corners(tags_with_objects, platform_tags_id, platform=True)

    # Define the region of interest (ROI) in the images
    def get_roi(corners):
        x_coords = [corner[0][0] for corner in corners]
        y_coords = [corner[0][1] for corner in corners]
        x_min, x_max = int(min(x_coords)), int(max(x_coords))
        y_min, y_max = int(min(y_coords)), int(max(y_coords))
        return x_min, y_min, x_max, y_max

    x_min_ref, y_min_ref, x_max_ref, y_max_ref = get_roi(corners_reference)
    x_min_obj, y_min_obj, x_max_obj, y_max_obj = get_roi(corners_with_objects)

    # Extract the region of interest (ROI) from the images
    roi_reference = image_reference[y_min_ref:y_max_ref, x_min_ref:x_max_ref]
    roi_with_objects = image_with_objects[y_min_obj:y_max_obj, x_min_obj:x_max_obj]

    # Convert the ROIs to grayscale
    gray_reference = cv2.cvtColor(roi_reference, cv2.COLOR_BGR2GRAY)
    gray_with_objects = cv2.cvtColor(roi_with_objects, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to the images
    gray_reference = cv2.GaussianBlur(gray_reference, (21, 21), 0)
    gray_with_objects = cv2.GaussianBlur(gray_with_objects, (21, 21), 0)

    # Calculate the absolute difference between the images
    diff = cv2.absdiff(gray_reference, gray_with_objects)
    # Apply a threshold to the difference image
    _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

    # Number of iterations for the erosion and dilation steps
    nb_iterations_erode = 4
    nb_iterations_dilate = 4

    # Define the kernel for the morphological operations
    kernel_size = (3, 3)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)

    # Apply morphological operations to the thresholded image
    thresh = cv2.erode(thresh, kernel, iterations=nb_iterations_erode)

    # Apply morphological operations to the thresholded image
    thresh = cv2.dilate(thresh, kernel, iterations=nb_iterations_dilate)
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize lists to store the rectangles and Aruco tags in the objects
    rectangles = []
    aruco_tags_in_objects = []

    # Detect Aruco tags in the ROI with objects
    tags_in_roi = aruco_detector(roi_with_objects)

    # Extract the corners and IDs of the Aruco tags in the ROI
    corners_in_roi, ids_in_roi = get_corners(tags_in_roi, platform_tags_id, platform=False)

    # Draw the contours of the Aruco tags in the ROI
    for contour in contours:
        if cv2.contourArea(contour) < 500:  # Ignore small contours
            continue
        (x, y, w, h) = cv2.boundingRect(contour)

        # Store the coordinates of the rectangles
        rectangles.append([x, y, x + w, y + h])

        # Check if the center of the Aruco tag is inside the rectangle
        for tag_corners, tag_id in zip(corners_in_roi, ids_in_roi):
            tag_center_x = tag_corners[0][0] + tag_corners[2][0]
            tag_center_x //= 2
            tag_center_y = tag_corners[0][1] + tag_corners[2][1]
            tag_center_y //= 2
            tag_center = [tag_center_x, tag_center_y]
            
            if x <= tag_center[0] <= x + w and y <= tag_center[1] <= y + h:
                
                if tag_id in id_to_key:
                    tag_key = id_to_key[tag_id]
                    safety_distance = safety_distances[tag_key]
                    
                    rectangles[-1] = [tag_center_x - safety_distance, tag_center_y - safety_distance,
                                      tag_center_x + safety_distance, tag_center_y + safety_distance]

                    (x, y, w, h) = (tag_center_x - safety_distance, tag_center_y - safety_distance,
                                    2 * safety_distance, 2 * safety_distance)

                    aruco_tags_in_objects.append({
                        "tag_id": tag_id,
                        "corners": tag_corners.tolist(),
                        "rectangle": [x, y, x + w, y + h]
                    })

                # Draw the Aruco tag in the ROI with objects
                cv2.polylines(roi_with_objects, [tag_corners], isClosed=True, color=(255, 0, 0), thickness=2)
            
            # Draw the rectangles around the objects
            cv2.rectangle(roi_with_objects, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Return the rectangles and Aruco tags in the objects
    return rectangles, aruco_tags_in_objects
