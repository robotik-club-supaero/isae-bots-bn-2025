#include <Machine_etats.h>
#include <Arduino.h>
#include <cmath>
#include "trajectories/PathTrajectory.hpp"
#include <Pathfinder.h>

Machine_etats::Machine_etats(Asserv *p_asserv, Mesure_pos *p_mesure_pos, Irsensor *p_ir_sensor_right, Irsensor *p_ir_sensor_left)
{
    m_p_asserv = p_asserv;
    m_p_mesure_pos = p_mesure_pos;
    m_p_ir_sensor_right = p_ir_sensor_right;
    m_p_ir_sensor_left = p_ir_sensor_left;
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
        Serial.println(tirette);

        // Récupère la vision des 2 ir_sensor et les mets dans 1 seul liste (taille 16)
        for (int i = 0; i < 8; i++){ m_vision[i] = m_p_ir_sensor_left->vision[i] ; } 
        for (int i = 0; i < 8; i++){ m_vision[8 + i] = m_p_ir_sensor_right->vision[i] ; }
        // Récupère la distance au danger le plus proche
        if (m_p_ir_sensor_left->ir_minimum_distance < m_p_ir_sensor_right->ir_minimum_distance){
            m_minimum_distance = m_p_ir_sensor_left->ir_minimum_distance ;
        } else {
            m_minimum_distance = m_p_ir_sensor_right->ir_minimum_distance ;
        }
       
        switch (etat)
        {
        case INIT:
            if (tirette == 0)
            {
                m_time_global = millis();
                etat = STOP;
            } else
            {
                etat = INIT;
            }
            break;
        case MVT:
            if (m_minimum_distance <= DISTANCE_MIN)
            {
                m_p_asserv->asserv_global(0, 0, 0);
                etat = EMERGENCY;
            } else {
                pos_x = m_p_mesure_pos->position_x + pos_init_x;
                pos_y = m_p_mesure_pos->position_y + pos_init_y;

                point pos = {.x = pos_x, .y = pos_y};
                point target = {.x = pos_finit_x, .y = pos_finit_y};

                list_points path = pathfinding_Astar(NULL, pos, target, m_vision);
                point sub_target = path.first;

                // trajectory = PathTrajectory(angle, path)
                // target_reached = trajectory.advance(SPEED * dt)
                // pos = trajectory.getCurrentPosition()

                if (m_minimum_distance < DISTANCE_MIN) {
                    etat = EMERGENCY;
                } else {
                    m_p_asserv->asserv_global(SPEED, SPEED, target.argument());
                }
            }
            break;
        case EMERGENCY:
            // Faites tourner le robot dans une direction spécifique
            // (par exemple, en ajustant l'angle)
            angle += PI / 4; // Tournez de 45 degrés à droite
            m_p_asserv->asserv_global(-SPEED, -SPEED, angle);

            // Si l'obstacle n'est plus détecté, revenez à l'état MVT
            if (m_minimum_distance > DISTANCE_MIN)
            {
                etat = MVT;
            }
            break;

        case STOP:
            m_p_asserv->asserv_global(0, 0, angle);
            if (m_minimum_distance > DISTANCE_MIN)
            {
                etat = MVT;
            }
            break;

        case END:
            m_p_asserv->asserv_global(0, 0, angle);
            break;

        }
        m_time = millis();
    }
}