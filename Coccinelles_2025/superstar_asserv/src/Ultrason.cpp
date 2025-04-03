#include "Ultrason.hpp"

ultrason::ultrason(int echo_PIN, int trigger_PIN)
{
    m_echo_PIN = echo_PIN;
    m_trigger_PIN = trigger_PIN;
    distance = 0;
    duration = 0;
}

void ultrason::setup()
{
    pinMode(echo_PIN, INPUT);     // Broche Echo en entrée
    pinMode(trigger_PIN, OUTPUT); // Broche Trigger en sortie
    digitalWrite(trigger, LOW);
}

void ultrason::loop()
{
    if (millis() - m_time >= dt) // 10ms delay between readings
    {
        // Envoi d'une impulsion de 10 microsecondes sur la broche Trigger
        // à voir si on doit le faire en asynchrone ou pas (10 micro secondes c'est pas long)
        digitalWrite(trigger_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger_PIN, LOW);

        // Lecture de la durée de l'echo
        duration = pulseIn(echo_PIN, HIGH);

        // Calcul de la distance en cm
        distance = duration * 0.034 / 2;
        
        //affichage de la distance (à commenter si pas besoin)
        Serianl.print("Distance: ");
        Serial.println(distance);
    }
}