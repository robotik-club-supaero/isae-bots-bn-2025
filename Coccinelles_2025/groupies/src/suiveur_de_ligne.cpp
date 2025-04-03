//Machine à état pour robot suiveur de ligne.

#include "Machine_etats.h"
#include <Arduino.h>
#include <cmath>
#include "trajectories/PathTrajectory.hpp"
#include "Serv.h"
#include "suiveur_de_ligne.h"
#include "define.h"

Machine_etats_sl::Machine_etats_sl(Asserv *p_asserv, Irsensor *p_ir_sensor, Serv *p_servo)
{
    m_p_asserv = p_asserv;
    m_p_ir_sensor = p_ir_sensor;
    m_p_servo = p_servo ;
}

void Machine_etats_sl::setup()
{
    pinMode(34, INPUT);
    pinMode(INL,INPUT);
    pinMode(INR,INPUT);
    etat = INIT;
    m_time = millis();
    m_time_global = millis();
}

void Machine_etats_sl::loop()
{
        if (millis() - m_time >= dt) {
        int lsensor = analogRead(INL); //Valeur capteur gauche.
        int rsensor = analogRead(INR); //Valeur capteur droit.

        if (millis() - m_time_global >= time_global)
        {
            m_p_asserv->asservissement(0, 0);
            etat = END; //Tps dépassé.
        }
        // Lire l'état de la tirette
        tirette = digitalRead(34);
        Serial.println(tirette);

        // Récupère la distance au danger le plus proche
        m_minimum_distance = m_p_ir_sensor->ir_minimum_distance ;
        
       
        switch (etat)
        {
        case INIT:
            if (tirette == 0) {
                m_time_global = millis();
                etat = STRAIGHT ;
            } 
            else {
                etat = INIT ;
            }
            break;
        case STRAIGHT:
            m_p_asserv->asservissement(SPEED, SPEED);
            if (m_minimum_distance <= DISTANCE_MIN) {
                etat = STOP ;
            } 
            
            else if (lsensor > WHITE_MIN && rsensor < BLACK_MAX) {
                etat = TRIGHT ;
            }
            else if (rsensor > WHITE_MIN && lsensor < BLACK_MAX) {
                etat = TLEFT ;
            }
            else if (rsensor > WHITE_MIN && lsensor > WHITE_MIN) {
                etat = END ;
            }
            else {
                etat = STRAIGHT ;
            }
            break;

        case TLEFT:
            m_p_asserv->asservissement(20, 5);
            if (m_minimum_distance <= DISTANCE_MIN) {
                etat = STOP ;
            }
            else if (rsensor < BLACK_MAX && lsensor < BLACK_MAX) {
                etat = STRAIGHT ;
            }
            else if (rsensor > WHITE_MIN && lsensor > WHITE_MIN) {
                etat = END ;
            }
            else {
                etat = TLEFT ;
            }
            break;
        
        case TRIGHT:
            m_p_asserv->asservissement(5, 20) ;
            if (m_minimum_distance <= DISTANCE_MIN) {
                etat = STOP ;
            }
            else if (rsensor < BLACK_MAX && lsensor < BLACK_MAX) {
                etat = STRAIGHT ;
            }
            else if (rsensor > WHITE_MIN && lsensor > WHITE_MIN) {
                etat = END ;
            }
            else {
                etat = TRIGHT ;
            }            
            break;

        case STOP:
            m_p_asserv->asservissement(0, 0);
            if (m_minimum_distance > DISTANCE_MIN)
            {
                etat = STRAIGHT;
            }
            break;

        case END:
            m_p_asserv->asservissement(0, 0) ;
            m_p_servo->blink(1, ANGLE1, ANGLE2) ;
            etat = END ;
            break;
        }
        m_time = millis();
    }
}