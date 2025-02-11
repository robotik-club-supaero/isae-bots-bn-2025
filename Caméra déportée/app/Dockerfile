# Image compatible avec Raspberry Pi (ARM) et PC Windows/Linux
FROM python:3.9

# Définir le répertoire de travail
WORKDIR /app  

# Copier tout le code source
COPY src/ ./src/
COPY json/ ./json
COPY npz/ ./npz

# Installer les dépendances générales
# On n'utilise pas de fichier requirements.txt pour éviter les problèmes de compatibilité
RUN pip install --no-cache-dir numpy opencv-python  

# Vérifier l’architecture et installer les bons paquets
RUN python -c "import platform; \
    import os; \
    if platform.system() == 'Linux': os.system('pip install tflite-runtime'); \
    elif platform.system() == 'Windows': os.system('pip install tensorflow')"

# Définir le répertoire de travail pour le script principal
WORKDIR /app/src

# Exécuter le script principal
CMD ["python", "main.py"]