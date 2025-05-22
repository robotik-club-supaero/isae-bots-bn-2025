
#ifndef MACHINE_ETATS_H
#define MACHINE_ETATS_H

#include <Arduino.h>
#include <Mesure_pos.h>
#include <Moteur.h>
#include <Irsensor.h>
#include <Asserv.h>
#include <Define_map.h>
#include "Ultrason.hpp"
#include "Serv.h"
#include "define.h"

#define K 1
#define dt 10
#define time_global 100000
#define time_sensor 8000

#define SPEED 20      // Vitesse en cm/s 25 est la vitesse max des moteurs
// c'est faux 25 n'est pas la vitesse max des moteurs, à trouver la bonne valeur (pour moi c'est 255)
#define DISTANCE_MIN 80 // Distance minimale pour éviter un obstacle en mm

class Machine_etats
{
    enum Pami_State
    {
        INIT,
        MOVE,
        STOP,
        END,

    };

public:
    Pami_State etat;
    long m_time;
    long m_time_global;
    long m_time_sensor;
    bool condx_turn ;
    bool condy_turn ;
    bool condx_arret ;
    bool condy_arret ;

// public:
    int tirette = 1; // TODO Etat par défaut de la tirette , CHANGER SI NECESSAIRE
    
    float pos_init_x = DEPART_SUPERSTAR_X; // TODO : A MODIFIER en foction de la stratégie
    float pos_init_y = DEPART_SUPERSTAR_Y;// TODO : A MODIFIER en foction de la stratégie
    
    float pos_finit_x = TOURNE_SUPERSTAR_X; // Premier point clef
    float pos_finit_y = TOURNE_SUPERSTAR_Y;// Premier point clef
    
    float pos_x = 0;
    float pos_y = 0;
    float angle = 0;
    int m_minimum_distance = 1000; 

    Asserv *m_p_asserv;
    Serv *m_p_servo ;

    Mesure_pos *m_p_mesure_pos;
    Ultrason *m_p_ultrason;

    Machine_etats(Asserv *p_asserv, Mesure_pos *p_mesure_pos, Ultrason *p_ultrason);
    void setup();
    void loop();
};

#endif
