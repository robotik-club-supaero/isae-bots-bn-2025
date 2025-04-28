/**
 * @file Ultrason.h
 * @brief Classe servant à controler un capteur ultrason
 */

/**
 * Classe ultrason
 */
#ifndef ULTRASON_H
#define ULTRASON_H
#include <Arduino.h>
#include <ESP32Servo.h>

class Ultrason
{
private:
    int m_echo_PIN;     // Broche Echo du HC-SR04
    int m_trigger_PIN;  // Broche Trigger du HC-SR04
    long m_duration; // Durée de l'echo
    long m_dt = 10; // Délai entre les lectures (10ms)
public:
    long m_distance; // Distance mesurée
    long m_time;
    Ultrason(int m_echo_PIN, int m_trigger_PIN); // Constructeur
    /**
     * @brief Fonction d'initialisation du capteur ultrason
     */
    void setup();
    /**
     * @brief Boucle de lecture du capteur ultrason, met à jour la distance
     */
    void loop();
};

#endif