#include <Moteur.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Encodeur.h>
#include <Mesure_pos.h>

#include <define.h>
#include <Asserv.h>



#define IR_PIN 4 //  ne sert à rien pour le moment , sert pour l'interruption



Moteur moteur_d = Moteur(EN_R, IN1_R, IN2_R);
Moteur moteur_g = Moteur(EN_L, IN1_L, IN2_L);
Encodeur encoder_R = Encodeur(CLK_R, DT_R);
Encodeur encoder_L = Encodeur(CLK_L, DT_L);
Mesure_pos mesure_pos = Mesure_pos(&encoder_R, &encoder_L);
Asserv asserv = Asserv(&moteur_d, &moteur_g, &mesure_pos);


void setup()
{

  
  
  mesure_pos.setup();
  // Si on veut tester les encodeurs , on les setup
  /*
  encoder_L.setup();
  encoder_R.setup();
  */

  Serial.begin(115200); // Initialisation de la communication série
  moteur_g.setup();     // Initialisation des moteurs
  moteur_d.setup();
  Serial.println("here");
  // Test moteur
  /*
  moteur_g.set_speed(255); //TODO : regler la vitesse pour tester la vitesse max
  moteur_d.set_speed(255);
  */
  asserv.setup();

}

void loop()
{
  /*
  #DEBUG
  encoder_L.loop();
  encoder_R.loop();
  */

  
  asserv.asserv_global(25, 25, 0) ;
  mesure_pos.loop();
 
  // #DEBUG Si on veut tester les asservissements , on decommente la lige suivante et on commente machine_etats.loop()
  asserv.loop();
  
  
  delay(100); // Delay de 100ms entre chaque boucle
}