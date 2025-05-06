#include <Machine_etats.h>
#include <Arduino.h>
#include <cmath>


#include <define.h>


Machine_etats::Machine_etats(Asserv *p_asserv, Mesure_pos *p_mesure_pos, Ultrason *p_ultrason)
{
    m_p_asserv = p_asserv;
    m_p_mesure_pos = p_mesure_pos;
}

void Machine_etats::setup()
{
    pinMode(4, INPUT);
    etat = INIT;
    m_time = millis();
    Serial.println(m_time);
    m_time_global = millis();
    Serial.println("setup fin");
}

void Machine_etats::loop()
{
    Serial.println("herein") ;
    Serial.print("m_time =");
    Serial.println(m_time);

    if (millis() - m_time >= dt)
    {   Serial.println("m_time_global = ") ;
        Serial.println(m_time_global) ;
        Serial.println(millis()-m_time_global) ;
        if (millis() - m_time_global >= time_global)
        {
            Serial.println("starth") ;
            m_p_asserv->asserv_global(0, 0, 0);
            etat = END;
        }
        // Lire l'état de la tirette
        tirette = digitalRead(4);
        
        // Récupère la distance au danger le plus proche
        m_minimum_distance = m_p_ultrason->m_distance ;
        
        switch (etat)
        {
            case INIT:
            Serial.println(tirette) ;
            if ((millis() - m_time_global >= START_TIME) && tirette == 0) {
                m_time_global = millis() ;
                etat = MOVE ;
                Serial.println("herein1");
            } 
            else {
                etat = INIT ;
            }
            break;
        case MOVE:
            Serial.println("heremoven2");
            if (m_minimum_distance <= DISTANCE_MIN) {
                etat = STOP;
            }
            
            pos_x = m_p_mesure_pos->position_x + pos_init_x;
            pos_y = m_p_mesure_pos->position_y + pos_init_y;
            angle = atan2(pos_y - pos_finit_y, pos_finit_x - pos_x);
            
            m_p_asserv->asserv_global(SPEED, SPEED, angle); //corrige l'angle. 
            
            condx_turn = (pos_x <= TOURNE_SUPERSTAR_X + EPSP) && (pos_x >= TOURNE_SUPERSTAR_X - EPSP) ;
            condy_turn = (pos_y <= TOURNE_SUPERSTAR_Y + EPSP) && (pos_y >= TOURNE_SUPERSTAR_Y - EPSP) ;
            if ( condx_turn && condy_turn) {
                pos_finit_x = FIN_SUPERSTAR_X;
                pos_finit_y = FIN_SUPERSTAR_Y;
                pos_init_x = TOURNE_SUPERSTAR_X ;
                pos_init_y = TOURNE_SUPERSTAR_Y ;
                m_p_mesure_pos->reinitialise() ;
                etat = MOVE ;
            }

            condx_arret = (pos_x <= FIN_SUPERSTAR_X + EPSP) && (pos_x >= FIN_SUPERSTAR_X - EPSP) ;
            condy_arret = (pos_y <= FIN_SUPERSTAR_Y + EPSP) && (pos_y >= FIN_SUPERSTAR_Y - EPSP) ;
            if (condx_arret && condy_arret) {
                etat = END ;
            }
            else {
                etat = MOVE ;
            }
            break;

        case STOP:
            Serial.println("herestopn3");
            m_p_asserv->asserv_global(0, 0, 0);
            if (m_minimum_distance > DISTANCE_MIN)
            {
                etat = MOVE;
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