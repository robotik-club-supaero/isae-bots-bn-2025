import cv2

# Création du soustracteur de fond (MOG2 est plus robuste)
fgbg = cv2.createBackgroundSubtractorMOG2()

cap = cv2.VideoCapture(0)  # Ou chemin de la caméra

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    fgmask = fgbg.apply(frame)  # Extraction du masque de mouvement
    cv2.imshow('Foreground Mask', fgmask)

    if cv2.waitKey(30) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
