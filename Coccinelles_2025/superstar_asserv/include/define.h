/**
 * @file define_ewan.h
 * @brief fichier de configuration des pins pour la pami walle ( cramptesque :) )
 * Voir avec l'elec pour les pins
 */
// TODO REGLER LES PINS AVANT LA COUPE AVEC L'ELEC
#ifndef DEFINE_H
#define DEFINE_H

/*
ENCODEURS
*/
#define CLK_L 5 // CLK LEFT  ENCODER
#define DT_L 18 // DT LEFT Encoder

#define CLK_R 17 // CLK RIGHT ENCODER
#define DT_R 16  // DT RIGHT Encoder

/*
MOTEURS
*/

#define EN_R 27  // EN RIGHT MOTEUR
#define IN1_R 13 // IN1 RIGHT MOTEUR
#define IN2_R 12 // IN2 RIGHT MOTEUR

#define EN_L 26  // EN LEFT MOTEUR
#define IN1_L 15 // IN1 LEFT MOTEUR
#define IN2_L 14 // IN2 LEFT MOTEUR

/*
SENSORS
*/

/*
SERVO
*/
#define ANGLE1 20  
#define ANGLE2 60 
#define TEMPS_BLINK 1  //Temps clignotement (secondes).

/*
MACHINE A ETAT
*/
#define START_TIME 85  //Les groupies commence dans les 15 dernières minutes.
#define DMIN 15 // En cm, à modifier.

/*
Capteur ultrason
*/
*
#define ECHO 7 // Broche Echo du HC-SR04 sur D7 //
#define TRIGGER 8 // Broche Trigger du HC-SR04 sur D8 //
#define DIST_MAX = 300; // Distance maxi a mesurer //
#define DIST_MINs = 3; // Distance mini a mesurer //


/*
SUIVEUR DE LIGNE
*/

#define dt 10
#define time_global 10000
#define time_sensor 8000

#define SPEED 25        // Vitesse en cm/s 25 est la vitesse max des moteurs
#define DISTANCE_MIN 80 // Distance minimale pour éviter un obstacle en mm
#define WHITE_MIN 90 
#define BLACK_MAX 30
#define YELLOW_MIN 45 
#define YELLOW_MAX 80 // Temporary
#define INL 4 
#define INR 5 




#endif