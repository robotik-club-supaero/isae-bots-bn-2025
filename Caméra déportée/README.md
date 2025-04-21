### Répertoire Caméra déportée ###

Les programmes se situent dans app.

app > cmd: programmes de test sur PC.

app > raspberry: dernière version fonctionnelle des codes téléversés sur le Raspberry Pi 3 B+.

app > dev: contient les programmes en cours de dévelopepement. A chaque fois qu'une version stable est mise au point, elle est copiée collée dans raspberry.

Ces trois dossiers (cmd, dev et raspberry) ont la même architecture:

cmd/dev/raspberry   > src               > aruco.py
                                        > constants.py
                                        > detectRobotsInsideAruco.py
                                        > imageNormalizer.py
                                        > lora.py
                                        > test_capture_jpeg.py
                                        > test_lora.py
                                        > test_para.py
                                        > test.py
                                        > undistort.py
                                        > calibrateCamera.py (présent dans cmd uniquement)
                                 
                    > platformImages
                    > normalizedImages
                    > npz
                    > json

- npz contient le fichier NPZ obtenu à la calibration de la caméra, utilisé par les codes Python

- json contient des fichiers JSON utilisés par les codes Python  

- platformImages contient l'image de référence de la plateforme et les images de la plateforme prises par la caméra telles quelles

- normalizedImages contient les images traitées

- src contient les codes Python:
    - aruco.py est une librairie utilisée pour détecter les tags Aruco
    - constants.py contient les constantes utiles au module Lora
    - detectRobotsIndideAruco.py détecte les robots et objets situés à l'intérieur du rectangle définir par les quatre tags Aruco de la plateforme
    - imageNormalizer.py traite les images pour corriger les déformations de la lentille de la caméra et aligner l'image prise avec l'image de référence, pour ensuite détecter les objets avec detectRobotsInsideAruco.py
    - lora.py est la librairie du module Lora
    - test_capture_jpeg.py est un test pour prendre une image avec la caméra en utilisant picamera2
    - test_lora.py teste l'envoi des données via Lora
    - test_para effectue les mêmes tâches que test.py mais utilise du multiprocessing
    - test.py effectue un test général des fonctions attendues par la caméra déportée (prise d'une image, traitement, détection des objets et envoi des données via Lora)
    - undistort.py est une librairire utilosée pour corriger les distorsions dues à la lentille de la caméra. Cette librairie est appelée par imageNormalizer.py
    - calibrateCamera.py sert à calibrer la caméra et obtenir le fichier NPZ.


Le Pi contient déjà les codes du dossier raspberry. Pour faire une démo des codes, se connecter en SSH à la Pi (mot de passe: raspberry) puis lancer les commandes suivantes:

"""
cd ./raspberry

source ./rasbperry_venv/bin/activate

cd ./src

python test.py

"""