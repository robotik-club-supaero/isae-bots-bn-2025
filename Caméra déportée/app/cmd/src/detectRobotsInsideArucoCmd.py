""" Script to detect objects inside Aruco tags on a platform """
"""                   Command line version                   """        

# This script compares a referenceimage of the platform with a captured image containing objects.
# It detects the Aruco tags on the platform to define the region of interest (ROI) and then
# detects the objects inside the Aruco tags in the captured image.
# If the objects detected have Aruco tags, the scirpt defines a safety distance around the tags
# and draws rectangles around the objects detected in the captured image.
# The script returns the coordinates of the corners of the rectangles and the Aruco tags in the objects.


import cv2
import sys
import argparse
import json
from loadImage import loadImage
from aruco import ArucoDetector, ArucoTags

# Parse arguments
parser = argparse.ArgumentParser(prog='Detect robots')
parser.add_argument('reference', help="path of the reference image")
parser.add_argument('capture', help="path of the captured image")
parser.add_argument('aruco_json', help="path of the json file with the Aruco tags")
parser.add_argument('distances_json', help="path of the json file with the safety distances")
args = parser.parse_args()

# Command example:
# python detectRobotsInsideAruco.py ../platformImages/reference.jpg ../normalizedImages/normalized_platform.jpg ../json/aruco.json ../json/distances.json

# Charger les images
image_reference = loadImage(args.reference)
image_with_objects = loadImage(args.capture)

# Charger les tags Aruco
aruco_tags = json.load(open(args.aruco_json))

# Créer un dictionnaire inversé pour rechercher les clés par ID
id_to_key = {v: k for k, v in aruco_tags.items()}

# Charger les distances de sécurité autour de chaque robot équipé d'un tag Aruco
safety_distances = json.load(open(args.distances_json))

# Liste des tags de la plateforme
platform_tags_id = []
platform_tags_id.append(aruco_tags["table_up_left"])
platform_tags_id.append(aruco_tags["table_up_right"])
platform_tags_id.append(aruco_tags["table_down_left"])
platform_tags_id.append(aruco_tags["table_down_right"])

# Vérifier si les images ont été chargées correctement
if image_reference is None:
    print("Erreur: Impossible de charger l'image référence")
    sys.exit(1)
if image_with_objects is None:
    print("Erreur: Impossible de charger l'image normalisée")
    sys.exit(1)

# Détecter les tags Aruco
aruco_detector = ArucoDetector()
tags_reference = aruco_detector(image_reference)
tags_with_objects = aruco_detector(image_with_objects)

# Extraire les coins des tags de la plateforme et les convertir en entiers
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

# Déterminer les coordonnées du rectangle formé par les tags
def get_roi(corners):
    x_coords = [corner[0][0] for corner in corners]
    y_coords = [corner[0][1] for corner in corners]
    x_min, x_max = int(min(x_coords)), int(max(x_coords))
    y_min, y_max = int(min(y_coords)), int(max(y_coords))
    return x_min, y_min, x_max, y_max

x_min_ref, y_min_ref, x_max_ref, y_max_ref = get_roi(corners_reference)
x_min_obj, y_min_obj, x_max_obj, y_max_obj = get_roi(corners_with_objects)

# Extraire la région d'intérêt (ROI)
roi_reference = image_reference[y_min_ref:y_max_ref, x_min_ref:x_max_ref]
roi_with_objects = image_with_objects[y_min_obj:y_max_obj, x_min_obj:x_max_obj]

# Convertir les images en niveaux de gris
gray_reference = cv2.cvtColor(roi_reference, cv2.COLOR_BGR2GRAY)
gray_with_objects = cv2.cvtColor(roi_with_objects, cv2.COLOR_BGR2GRAY)

# Appliquer un flou pour réduire le bruit
gray_reference = cv2.GaussianBlur(gray_reference, (21, 21), 0)
gray_with_objects = cv2.GaussianBlur(gray_with_objects, (21, 21), 0)

# Calculer la différence absolue entre les deux images
diff = cv2.absdiff(gray_reference, gray_with_objects)
# Appliquer un seuil pour obtenir une image binaire
# Ajuster le seuil pour obtenir une image binaire plus précise
_, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

# Nombre d'itérations pour l'érosion et la dilatation
# Ajuster le nombre d'itérations pour l'érosion et la dilatation
nb_iterations_erode = 4
nb_iterations_dilate = 4

# Taille du noyau pour l'érosion et la dilatation
# Ajuster la taille du noyau pour l'érosion et la dilatation
kernel_size = (3, 3)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)

# Ajouter une étape d'érosion pour séparer les objets proches
thresh = cv2.erode(thresh, kernel, iterations=nb_iterations_erode)

# Dilater l'image pour combler les trous et trouver les contours
thresh = cv2.dilate(thresh, kernel, iterations=nb_iterations_dilate)
contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Liste pour stocker les coordonnées des coins des rectangles
rectangles = []
aruco_tags_in_objects = []

# Détecter les tags Aruco dans la ROI avec les objets
tags_in_roi = aruco_detector(roi_with_objects)

# Convertir les coins des tags dans la ROI en entiers
corners_in_roi, ids_in_roi = get_corners(tags_in_roi, platform_tags_id, platform=False)

# Dessiner les contours sur l'image originale et stocker les coordonnées des coins
for contour in contours:
    if cv2.contourArea(contour) < 500:  # Ignorer les petits contours
        continue
    (x, y, w, h) = cv2.boundingRect(contour)

    # Stocker les coordonnées des coins du rectangle dans le repère de la ROI
    rectangles.append([x, y, x + w, y + h])

    # Vérifier si un tag Aruco se trouve à l'intérieur du contour
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

            # Dessiner les contours des tags Aruco détectés
            cv2.polylines(roi_with_objects, [tag_corners], isClosed=True, color=(255, 0, 0), thickness=2)
        
        # Dessiner les rectangles autour des objets détectés
        cv2.rectangle(roi_with_objects, (x, y), (x + w, y + h), (0, 255, 0), 2)

# Afficher l'image avec les objets détectés
cv2.imshow('Detected Objects', roi_with_objects)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Retourner les coordonnées des coins des rectangles et des tags Aruco dans les objets
print("Rectangles:", rectangles)
print("Aruco Tags in Objects:", aruco_tags_in_objects)