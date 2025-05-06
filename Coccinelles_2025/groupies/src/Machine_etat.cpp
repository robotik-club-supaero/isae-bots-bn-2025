#include <Machine_etats.h>
#include <Arduino.h>
#include <cmath>
#include "trajectories/PathTrajectory.hpp"
#include <Pathfinder.h>
#include <define.h>
#include <vector>


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
    
    GridLocation start ;
    start.x = STARTX ;
    start.y = STARTY ;
    GridLocation goal ;
    goal.x = FINX ;
    goal.y = FINY ;
    m_trajectoire = pathfinding_Astar(START_GRID, start, goal) 
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
        
        // Récupère la vision des 2 ir_sensor et les mets dans 1 seul matrice (taille 16*8)
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
            if ((millis() - m_time_global >= START_TIME) && tirette == 0) {
                m_time_global = millis() ;
                etat = MVT ;
            } 
            else {
                etat = INIT ;
            }
            break;
        case MVT:
            if (m_minimum_distance < DMIN ){
                m_p_asserv->asserv_global(0, 0, 0);
                etat = STOP ;
            }
            if (m_minimum_distance <= DMAX && m_minimum_distance >= DMIN) {
                m_p_asserv->asserv_global(0, 0, 0);
                etat = AVOID;
            } 
            

            else {
                pos_x = m_p_mesure_pos->position_x + pos_init_x;
                pos_y = m_p_mesure_pos->position_y + pos_init_y;

                GridLocation pos;
                pos.x = pos_x;
                pos.y = pos_y;
                GridLocation target;
                target.x = pos_finit_x;
                target.y = pos_finit_y;

                GridLocation sub_target = m_trajectoire[m_traj_index];
                m_traj_index++ ;

                suppr_obstacle(pos_x, pos_y, angle, m_vision, &grid) ; 

                //trajectory = PathTrajectory(angle, path);
                //target_reached = trajectory.advance(SPEED * dt);
                pos = trajectory.getCurrentPosition();

                if (m_minimum_distance < DISTANCE_MIN) {
                    etat = AVOID;
                } else {
                    m_p_asserv->asserv_global(SPEED, SPEED, target.argument());
                    etat = MVT ;
                }
            }
            break;
        case STOP:
            m_p_asserv->asserv_global(0, 0, angle);
            // Faites tourner le robot dans une direction spécifique
            // (par exemple, en ajustant l'angle)
            angle += PI / 4; // Tournez de 45 degrés à droite
            m_p_asserv->asserv_global(-SPEED, -SPEED, angle);

            // Si l'obstacle n'est plus détecté, revenez à l'état MVT
            if (m_minimum_distance > DISTANCE_MIN)
            {
                etat = AVOID;
            }
            break;

        case AVOID:
            
            if (m_minimum_distance < DMIN) {
                etat = STOP;
            } 
            else if (m_minimum_distance <= DMAX && m_minimum_distance >= DMIN) {
                pos_x = m_p_mesure_pos->position_x + pos_init_x;
                pos_y = m_p_mesure_pos->position_y + pos_init_y;

                GridLocation pos;
                pos.x = pos_x;
                pos.y = pos_y;
                GridLocation target;
                target.x = pos_finit_x;
                target.y = pos_finit_y;

                add_obstacle(pos_x, pos_y, angle, m_vision, &grid) ;
                std::vector<GridLocation> path = pathfinding_Astar(&grid, pos, target);
            
                m_traj_index = 1 ;
                GridLocation sub_target = path[m_traj_index];
                m_traj_index++ ;
                
                suppr_obstacle(pos_x, pos_y, angle, m_vision, &grid) ; 

                //trajectory = PathTrajectory(angle, path);
                //target_reached = trajectory.advance(SPEED * dt);
                //pos = trajectory.getCurrentPosition();
            }
            

            else {
                m_p_asserv->asserv_global(SPEED, SPEED, target.argument());
                etat = MVT ;
            }
            break;

        case END:
            m_p_asserv->asserv_global(0, 0, angle);
            break;

        }
        m_time = millis();
    }
}
