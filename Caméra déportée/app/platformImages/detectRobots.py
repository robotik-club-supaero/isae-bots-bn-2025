import cv2
import sys

# Charger les images
image_reference = cv2.imread('D:/Robotik/Caméra déportée/app/platformImages/reference.jpg')
image_with_objects = cv2.imread('D:/Robotik/Caméra déportée/app/platformImages/normalized_table.jpg')

# Vérifier si les images ont été chargées correctement
if image_reference is None:
    print("Erreur: Impossible de charger 'reference.jpg'")
    sys.exit(1)
if image_with_objects is None:
    print("Erreur: Impossible de charger 'normalized_table.jpg'")
    sys.exit(1)

# Convertir les images en niveaux de gris
gray_reference = cv2.cvtColor(image_reference, cv2.COLOR_BGR2GRAY)
gray_with_objects = cv2.cvtColor(image_with_objects, cv2.COLOR_BGR2GRAY)

# Appliquer un flou pour réduire le bruit
gray_reference = cv2.GaussianBlur(gray_reference, (21, 21), 0)
gray_with_objects = cv2.GaussianBlur(gray_with_objects, (21, 21), 0)

# Calculer la différence absolue entre les deux images
diff = cv2.absdiff(gray_reference, gray_with_objects)

# Appliquer un seuil pour obtenir une image binaire
_, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

# Dilater l'image pour combler les trous et trouver les contours
thresh = cv2.dilate(thresh, None, iterations=2)
contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Dessiner les contours sur l'image originale
for contour in contours:
    if cv2.contourArea(contour) < 500:  # Ignorer les petits contours
        continue
    (x, y, w, h) = cv2.boundingRect(contour)
    cv2.rectangle(image_with_objects, (x, y), (x + w, y + h), (0, 255, 0), 2)

# Afficher l'image avec les objets détectés
cv2.imshow('Detected Objects', image_with_objects)
cv2.waitKey(0)
cv2.destroyAllWindows()