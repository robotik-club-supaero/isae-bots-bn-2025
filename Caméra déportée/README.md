### Répertoire Caméra déportée ###

Les programmes se situent dans app.

app > cmd: anciens programmes de tests sur PC, fonctionnent en ligne de commande. Pas à jour.

app > raspberry: dernière version fonctionnelle des codes téléversés sur la Raspberry Pi 3 B+.

app > dev: contient les programmes en cours de dévelopepement. A chaque fois qu'une version stable est mise au point, elle est copiée collée dans raspberry.

Les dossiers dev et raspberry ont la même architecture:

dev/raspberry   > src
                > platformImages
                > normalizedImages
                > npz
                > json

- src contient les codes Python
- npz contient le fichier NPZ obtenu à la calibration de la caméra, utilisé par les codes Python
- json contient des fichiers JSON utilisés par les codes Python  
- platformImages contient l'image de référence de la plateforme et les images de la plateforme prises par la caméra telles quelles
- normalizedImages contient les images traitées

La Pi contient déjà les codes du dossier dev. Pour faire une démo des codes, se connecter en SSH à la Pi (mot de passe: raspberry) puis lancer les commandes suivantes:

"""
cd ./raspberry

source ./rasbperry_venv/bin/activate

cd ./src

python test.py

python test_para.py

"""

/!\ LES EXPLICATIONS SUIVANTES SUR LES PROGRAMMES SONT SIMPLIFIEES.

Chaque programme calibre le procédé de traitement des images, puis effectue 2 fois tout le processus de prise de l'image + traitement de l'image + envoi des données. test.py le fait séquentiellement, test_para.py le fait (à peu près) en parallèle.
