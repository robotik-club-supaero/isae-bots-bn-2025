# Code asserv Base Roulante
Code de l'asservissement en position du gros robot avec un noeud ROS2 Jazzy.
L'asservissement est un PID avec filtre passe-bas sur la dérivée. L'accélération maximale du robot est contrôlée par des rampes.
Les gains du PID sont définies avec une valeur par défaut dans le code, on peut les modifier avec les topics appropriés (la modification n'est valable que jusqu'au redémarrage de la Teensy).

Le fichier br_controller/include/configuration.hpp contient la plupart des paramètres de l'asservissement, s'il y a besoin de les modifier de manière permanente.

## Compilation 

Attention : la racine du projet ROS est ici, mais la racine du projet Platform.IO est dans le dossier br_controller, et il y a un lien symbolique vers br_messages dans br_controller/extra_packages.
Cela était nécessaire pour que le code soit compatible à la fois avec colcon et Platform.IO.

### Compiler la simulation
En pratique, la BR est intégrée au HN en tant que sous-module Git et est compilée en même temps que le reste du code du HN.

Conseils pour faciliter le développement :
- Ajoutez le dépôt local en tant que source pour le sous-module :
```
cd [HN]/dev/lib/br
git remote add local [BR]
```

- Ainsi, vous pouvez compiler les modifications apportées à la BR sans être obligé de "push" (vous devez quand même faire un commit) :
```
cd [HN]/dev/lib/br
git fetch local
git checkout <id du commit local>
```

Consultez la documentation du HN pour plus d'informations.

La simulation peut aussi être compilée en exécutant la commande `colcon build` à la racine du projet (nécessite ROS2, SFML et pybind11).

### Compiler pour la BR (Arduino Teensy)
Compiler avec Platform.IO.
Attention la convention de code est en c++20 et ne compilera pas en Arduino pur, qui est en c++9x de base (en plus des Décodeurs HW)

## Utilisation (sur le robot)

Compilez et téléversez le code sur la Teensy BR avec Platform.IO.

Pour communiquer avec la BR depuis le HN, il faut démarrer micro_ros_agent dans le Docker sur le Raspberry :
`ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyXXX`

Consultez la section sur les erreurs fréquentes ci-dessous et la documentation du HN pour plus d'informations.

## Les entrees sur l'asserv

Tous les topics sont exactement les mêmes entre la simulation et le vrai robot, sauf pour le topic "/br/odosCount".

### Le topic /br/goTo

Ce topic reçoit les ordres de déplacement. L'ordre est constitué de :
- kind: Une combinaison de "flags" binaires parmi :
    * REVERSE (1) : Provoque le déplacement en marche arrière plutôt qu'en marche avant
    * FINAL_ORIENTATION (2) : Provoque la rotation finale du robot à la fin du déplacement.
    * ALLOW_CURVE (4) : Autorise l'utilisation des trajectoires courbes. Si ce flag n'est pas utilisé, une trajectoire
      en ligne brisée (avec arrêt et rotation à chaque point) sera utilisée.
- path: Une liste de points par lesquels le robot doit passer. La position initiale du robot ne doit PAS être incluse.
- theta: L'orientation finale du robot attendue à la fin du déplacement (en radians). `theta` est ignoré si le flag `FINAL_ORIENTATION` n'est pas utilisé.

Si `path` est vide, le robot ne se déplacera pas et les flags `REVERSE` et `ALLOW_CURVE` n'ont pas d'effet. Si le flag `FINAL_ORIENTATION` est utilisé, le robot effectue une rotation sur place, sinon l'ordre de déplacement n'a aucun effet.

#### Consignes obsolètes ou pas encore implémentées

recalage avant : le robot s'oriente selon theta = z, puis avance en marche avant jusqu'au contact d'un mur et reset sa position (x ou y)
recalage arriere : le robot s'oriente selon theta = z, puis recule en marche arriere jusqu'au contact

[SUPPRIME] point final marche avant, mais avec une dynamique assouplie - voir le topic /br/setSpeed pour modifier la vitesse de déplacement
[SUPPRIME] fake_recal_avant : le robot se plaque au mur mais sans reset sa position
[SUPPRIME] fake_recal_arriere : idem

Les consignes marquées [SUPPRIME] ne seront pas réimplémentées sauf si elles sont vraiment nécessaires.

### Le topic /br/command

Ce topic reçoit les ordres de commande manuelle. L'ordre est contitué de :
- linear : vitesse linéaire, entre -255 et 255
- angular : vitesse angulaire, entre -255 et 255

Attention, les vitesses ne sont pas en m/s mais sont relatives aux vitesse maximales définies par la configuration de la BR.
255 = 100% de la vitesse maximale

### Le topic /br/stop

Ce topic reçoit les ordres de freinage d'urgence.

### Le topic /br/idle

Activation ou désactivation de l'asservissement. Les ordres ci-dessus n'ont aucun effet quand l'asservissement n'est pas activé.
L'asservissement est automatiquement activé avec la simulation, mais pas avec le vrai robot.

Assurez-vous d'allumer la puissance avant d'activer l'asservissement.

### Le topic /br/resetPosition

Réinitialise la position simulée ou estimée du robot. Ce topic ne devrait pas être utilisé quand le robot est en mouvement.

### Le topic /br/setSpeed

Ce topic réduit temporairement la vitesse du robot (par exemple à l'approche d'un obstacle).
La valeur à donner est un entier entre 1 et 100 qui correspond à la nouvelle vitesse limite du robot, exprimée en pourcentage de la vitesse maximale globale.
La limitation est appliquée uniquement au déplacement en cours. Si aucun déplacement n'est en cours, le topic n'a aucun effet.
```
ros2 topic pub /br/setSpeed std_messages/Int16 "{data: <percentage>}"
```

### Le topic /br/gains
Ce topic regle les gains du PID:
```
ros2 topic pub /br/gains br_messages/GainsPid "{kp: <value>
ti: <value>
td: <value>}"
```
Publier sur ce topic réinitialise la valeur de l'intégrale et de la dérivée de l'erreur dans le PID.

## Les feedbacks 

### Le topic /br/currentPosition

Ce topic retourne la position actuelle du robot toutes les 100ms (ou toutes les 10ms pour la simulation)

### Le topic /br/callbacks

Ce topic effectue le feedback des étapes de l'asservissement. Le message est un entier parmi :
```
1 : okPos, le robot a fini son avance lineaire [OBSOLETE]
2 : okTurn, le robot a fini de tourner [OBSOLETE]
3 : le robot a fini une marche arrière (ce code est toujours immédiatement suivi de okPos) [OBSOLETE]
5 : okReady, contrôleur passé en état ready (activation asservissement ODrive)
6 : okIdle, contrôleur passé en état idle (désactivation asservissement ODrive)
7 : okOrder, ordre (déplacement ou rotation) terminé
0 : [jamais envoyé actuellement] erreur dans l'asserv (timeout, derapage, divergence etc...) donc erreur qui peut reussir apres un deuxieme essai
-1 : [jamais envoyé actuellement] erreur bloquante (gains non sets ou non valides, erreurs internes etc...)
```

### Le topic /br/logTotaleArray 

NB: L'asserv publie sur ce topic uniquement si la constante `_BR_DEBUG` est définie lors de la compilation (elle est définie par défaut, voir br_controller/CMakeLists.txt et br_controller/platformio.ini pour la désactiver).

Ce topic effectue un log global des variables internes de l'asserv. Pour l'exploiter :
```
ros2 topic echo -p /br/logTotaleArray > fichierdelog.log
python GraphPos.py fichierdelog.log # TODO: script obsolete [où est-il ?]
```

Si la teensy est connectée a la pi, un `scp pi@ip:fichierdelog.log` . permet de le recuperer
@Yoann a concu un affichage a la volée de ces log

### Le topic /br/odosCount

Publie le nombre de "ticks" comptés sur les roues encodeuses de position (utile pour la calibration).

## Possibles erreurs à detecter **Avant** la coupe 

- Tester asserv en idle : --> Topic /br/idle 
- Tester asserv en vitesse --> Topic /br/command
- Tester asserv en Position --> Topic /br/goTo. Tester tous les angles et les directions puis adapter si besoin les paramètres dans le fichier ['configuration.hpp'](br_controller/include/configuration.hpp).

- Si le robot va trop vite ou a une trop forte accélération, il y a un risque de glissement et de dérive de la position estimée par les odomètres. Penser à vérifier que les vitesses et accélérations sont adaptées (cela inclut l'accélération du freinage d'urgence). Si la position estimée n'est pas suffisamment précise, la stratégie du haut niveau ne fonctionnera pas.

### IMPORTANT
**Refaire le test après avoir redemarré le robot pour vérifier que les variables se sont bien initialisées !**
*Il est arrivé dans les versions précédentes du code que après un redémarrage de la teensy , certaines variables de trajectoires ne soient pas bien initialisées et donc le robot ne bouge pas ou devient fou (calcul de trajectoire infini)*

L'interface graphique (dans le code du haut niveau) permet d'envoyer un ordre de déplacement au topic /br/goTo et de visualiser le retour de position sur le topic /br/currentPosition.
Il suffit de faire un clic droit à l'emplacement où on veut faire aller le robot, puis de déplacer la souris dans la direction que l'on veut pour l'orientation finale avant de relacher le clic.

Sur le Docker HN :
```
source /app/install/setup.bash
ros2 run uix interface_node
```
(Penser à faire `xhost +` si besoin avant d'aller dans le Docker)

### DEL clignotante

La Teensy doit clignoter en permanence à partir du moment où `micro_ros_agent` est lancé.

- Si la Teensy se met à clignoter très lentement lorsque vous activez l'asservissement avec /br/idle, vérifiez que la puissance est bien allumée et que les moteurs et la carte ODrive sont bien connectés.
- Si la Teensy arrête de clignoter totalement, cela signifie que le programme a planté. Cela peut être dû soit à une perte de connexion avec micro_ros_agent, soit à un problème plus grave dans le programme (erreur de segmentation ou autre).

## [TOUT CE QUI SUIT EST POTENTIELLEMENT OBSOLETE, TODO à vérifier]
## Les scripts 
[Où sont les scripts ?]

Pour se simplifier la tache, plusieurs scripts sont disponibles dans le dossier Script

### InputTeensyAsserv

Reformule simplement un envoi sur le topic nextPositionTeensy, on entre simplement x,y,theta et le type d'ordre

### rotation.py

Fais effectuer 10 tours sur lui-meme au robot, utile pour regler l'odometrie

### SetGainsTeensy

Permet de régler tous les gains de la teensy, utile pendant les reglages

### SpamSpeed 
spam les ordres de consigne pour tester la robustesse

### suite_point.py

Permet d'effectuer une suite de points, simulation d'une trajectoire avec un path-finder haut niveau

### GraphPos.py

Effectue le tracé des logs de l'asserv 

### Dossier plot

Contient tous les scripts pour effectuer des logs et les tracer.
Les noms sont explicites : 
- acquireLogs est à lancer SUR la pi
- les autres se lancent sur l'ordinateur où on effectue les plots
- getAndPlotLastLog va automatiquement chercher le dernier log sur la pi, le télécharge et le plot

NOTE : Les fichiers sont effacés au redémarrage de la pi/de l'ordinateur, peut-être à modifier :-D

## utiliser les graphes en Live

Pour afficher les graphes en live, qques commandes a lancer en plus : 

Sur la Pi (dans le meme terminal que roscore et rosrun): 
```
export ROS_IP=192.168.217.11
export ROS_MASTER_URI=http://192.168.217.11:11311
```
Ou alors utiliser les scripts export_roscore.sh et export_rosrun.sh sur la pi

Sur le pc fixe 
```
#IP perso ici
export ROS_IP=192.168.217.131
export ROS_MASTER_URI=http://192.168.217.11:11311
cd Robotik/Robotik_2019/Sources/Robot/Haut_niveau/Dev/
source devel/setup.bash
rosrun isae_robotics_graph GraphNode.py 
rosrun isae_robotics_graph InterfaceNode.py 
```
Ou utiliser export_costa_launcher.sh ou export_titanic_launcher.sh dans le dossier Scripts

