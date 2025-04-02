
#ifndef SUIVEUR_DE_LIGNE_H
#define SUIVEUR_DE_LIGNE_H

#include <Arduino.h>
#include <Mesure_pos.h>
#include <Moteur.h>
#include <Irsensor.h>
#include <Asserv.h>
#include <Define_map.h>


class Machine_etats_sl
{
    enum Pami_State
    {
        INIT,
        STRAIGHT,
        TLEFT, // 
        TRIGHT,
        STOP, // Si on est trop proche d'un obstacle.
        END,

    };

private:
    Pami_State etat;
    long m_time;
    long m_time_global;
    long m_time_sensor;

public:
    int tirette = 1; // TODO Etat par défaut de la tirette , CHANGER SI NECESSAIRE
    
    int m_minimum_distance = 1000;

    Asserv *m_p_asserv ;
    Serv *m_p_servo ;
    Irsensor *m_p_ir_sensor ;

    Machine_etats_sl(Asserv *p_asserv, Irsensor *p_ir_sensor, Serv *p_servo);
    void setup();
    void loop();
};

#endif
