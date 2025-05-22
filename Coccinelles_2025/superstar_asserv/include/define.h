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
#define CLK_R 21 // CLK LEFT  ENCODER
#define DT_R 22 // DT LEFT Encoder

#define CLK_L 13 // CLK RIGHT ENCODER
#define DT_L 16  // DT RIGHT Encoder

#define INV_L 0 // Inversion du sens de rotation de l'encodeur gauche
#define INV_R 0 // Inversion du sens de rotation de l'encodeur droit





/*
MOTEURS
*/

#define EN_L 14  // EN RIGHT MOTEUR
#define IN1_L 27 // IN1 RIGHT MOTEUR
#define IN2_L 26 // IN2 RIGHT MOTEUR

#define EN_R 33  // EN LEFT MOTEUR
#define IN1_R 25 // IN1 LEFT MOTEUR
#define IN2_R 32 // IN2 LEFT MOTEUR

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
#define TOURNE_SUPERSTAR_X 120    
#define TOURNE_SUPERSTAR_Y 0 //Position ou la superstar va tourner
#define DEPART_SUPERSTAR_X 0
#define DEPART_SUPERSTAR_Y 0
#define FIN_SUPERSTAR_X 120
#define FIN_SUPERSTAR_Y -38 //position finale superstar
#define EPSP 1 //Incertitude position, cm
#define EPSA 0.1 //0,1 ? //Incertitude position, radian

/*
Capteur ultrason
*/
#define ECHO 7 // Broche Echo du HC-SR04 sur D7 //
#define TRIGGER 8 // Broche Trigger du HC-SR04 sur D8 //
#define DIST_MAX = 300; // Distance maxi a mesurer //
#define DIST_MINs = 3; // Distance mini a mesurer //



#endif