#include "Ultrason.hpp"

Ultrason::Ultrason(int echo_PIN, int trigger_PIN)
{
    m_echo_PIN = echo_PIN;
    m_trigger_PIN = trigger_PIN;
    m_distance = 0;
    m_duration = 0;
}

void Ultrason::setup()
{
    pinMode(m_echo_PIN, INPUT);     // Broche Echo en entrée
    pinMode(m_trigger_PIN, OUTPUT); // Broche Trigger en sortie
    digitalWrite(m_trigger_PIN, LOW);
}

void Ultrason::loop()
{
    if (millis() - m_time >= m_dt) // 10ms delay between readings
    {
        // Envoi d'une impulsion de 10 microsecondes sur la broche Trigger
        // à voir si on doit le faire en asynchrone ou pas (10 micro secondes c'est pas long)
        digitalWrite(m_trigger_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(m_trigger_PIN, LOW);

        // Lecture de la durée de l'echo
        m_duration = pulseIn(m_echo_PIN, HIGH);

        // Calcul de la distance en cm
        m_distance = m_duration * 0.034 / 2;
        
        //affichage de la distance (à commenter si pas besoin)
        Serial.print("Distance: ");
        Serial.println(m_distance);
    }
}