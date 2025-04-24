#include <Machine_etats.h>
#include <Arduino.h>
#include <cmath>
#include "trajectories/PathTrajectory.hpp"

#include <define.h>


Machine_etats::Machine_etats(Asserv *p_asserv, Mesure_pos *p_mesure_pos, Irsensor *p_ir_sensor_right,  Irsensor *p_ir_sensor_left)
{
    m_p_asserv = p_asserv;
    m_p_mesure_pos = p_mesure_pos;
}

void Machine_etats::setup()
{
    pinMode(34, INPUT);
    etat = INIT;
    m_time = millis();
    m_time_global = millis();
}

void Machine_etats::loop()
{
    if (millis() - m_time >= dt)
    {
        if (millis() - m_time_global >= time_global)
        {
            m_p_asserv->asserv_global(0, 0, 0);
            etat = END;
        }
        // Lire l'état de la tirette
        tirette = digitalRead(34);
        
        // Récupère la distance au danger le plus proche
        if (m_p_ir_sensor_left->ir_minimum_distance < m_p_ir_sensor_right->ir_minimum_distance){
            m_minimum_distance = m_p_ir_sensor_left->ir_minimum_distance ;
        } else {
            m_minimum_distance = m_p_ir_sensor_right->ir_minimum_distance ;
        }
       
        switch (etat)
        {
        case INIT:
            if ((millis() - m_time_global >= START_TIME) && tirette == 0) {
                m_time_global = millis() ;
                etat = STRAIGHT ;
            } 
            else {
                etat = INIT ;
            }
            break;
        case STRAIGHT:
            m_p_asserv->asserv_global(SPEED, SPEED, 0);
            if (m_minimum_distance <= DISTANCE_MIN) {
                etat = STOP;
            }

            else {
                bool condx_turn = (m_p_mesure_pos->position_x <= TOURNE_SUPERSTAR_X + EPSP) && (m_p_mesure_pos->position_x >= TOURNE_SUPERSTAR_X - EPSP) ;
                bool condy_turn = (m_p_mesure_pos->position_y <= TOURNE_SUPERSTAR_Y + EPSP) && (m_p_mesure_pos->position_y >= TOURNE_SUPERSTAR_Y - EPSP) ;
                if ( condx_turn && condy_turn) {
                    etat = TURN ;
                }

                bool condx_arret = (m_p_mesure_pos->position_x <= FIN_SUPERSTAR_X + EPSP) && (m_p_mesure_pos->position_x >= FIN_SUPERSTAR_X - EPSP) ;
                bool condy_arret = (m_p_mesure_pos->position_y <= FIN_SUPERSTAR_Y + EPSP) && (m_p_mesure_pos->position_y >= FIN_SUPERSTAR_Y - EPSP) ;
                if (condx_arret && condy_arret) {
                    etat = END ;
                }
                else {
                    etat = STRAIGHT ;
                }
            }
            break;

        case TURN:
            // Tournez de 45 degrés à droite
            m_p_asserv->asservissement(-SPEED, SPEED);
            if (m_minimum_distance <= DISTANCE_MIN) {
                etat = STOP;
            }
            
            else {
                bool condtheta = (m_p_mesure_pos->position_theta <= PI/2 + EPSA) && (m_p_mesure_pos->position_theta >= PI/4 - EPSA) ; //A tourner de 90 degrés.
                if (!condtheta) {
                    etat = TURN ;
                }
                else {
                    etat = STRAIGHT;
                }
            }
            break;

        case STOP:
            m_p_asserv->asserv_global(0, 0, 0);
            if (m_minimum_distance > DISTANCE_MIN)
            {
                etat = STRAIGHT;
            }
            else {
                etat = STOP ;
            }
            break;

        case END:
            m_p_asserv->asserv_global(0, 0, 0);
            m_p_servo->blink(1, ANGLE1, ANGLE2) ;
            break;

        }
        m_time = millis();
    }
}