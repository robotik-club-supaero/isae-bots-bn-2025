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

class ultrason
{
private:
    int echo_PIN;     // Broche Echo du HC-SR04
    int trigger_PIN;  // Broche Trigger du HC-SR04
    long duration; // Durée de l'echo
    long dt = 10; // Délai entre les lectures (10ms)
public:
    long distance; // Distance mesurée
    Ultrason(int echo_PIN, int trigger_PIN); // Constructeur
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