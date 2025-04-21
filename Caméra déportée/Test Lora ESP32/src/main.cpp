#include <SPI.h>
#include <RH_RF95.h>  // Librairie RadioHead pour LoRa SX1276/SX1278

// === 📌 Définition des broches (GPIO ESP32) ===

// --- Bus SPI (pins par défaut de l’ESP32) ---
#define LORA_SCK   35  // Clock
#define LORA_MISO  33  // Master In Slave Out
#define LORA_MOSI  32  // Master Out Slave In

// --- Contrôle LoRa ---
#define LORA_CS    34   // NSS / CS (Chip Select du LoRa)
#define LORA_RST   23  // Reset matériel du module LoRa
#define LORA_DIO0  26  // DIO0 (Interruption pour détection réception)

// === 🌐 Fréquence LoRa (doit être la même que l’émetteur) ===
#define RF95_FREQ 868.0  // Europe : 868 MHz / US : 915 MHz

// === 📦 Objet LoRa (RadioHead) ===
RH_RF95 rf95(LORA_CS, LORA_DIO0);  // CS + DIO0 (SPI utilise pins par défaut)

void setup() {
  Serial.begin(115200);
  delay(1000);

  // === 🔁 Reset manuel du module LoRa ===
  pinMode(LORA_RST, OUTPUT);      //broche reset utilisé en sortie (envoyer des signaux)
  digitalWrite(LORA_RST, HIGH);
  delay(10);
  digitalWrite(LORA_RST, LOW);    // Pulse LOW pour reset
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  // Code de detection d'anomalies

  // === ⚙️ Initialisation du module LoRa ===
  if (!rf95.init()) {
    Serial.println("❌ Erreur : Module LoRa non détecté !");
    while (1);
  }

  // Configuration de la fréquence de réception
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("❌ Erreur : Impossible de définir la fréquence !");
    while (1);
  }

  // FIn de la detection

  // Configuration de la puissance d’émission (utile si émission plus tard)
  rf95.setTxPower(13, false);  // 13 dBm, sans PA_BOOST
  Serial.println("✅ Récepteur LoRa prêt à recevoir !");
  Serial.println("===================================");
}

void loop() {
  // === 🔍 Vérifie si un message est disponible ===
  if (rf95.available()) {
    // Creation d'un message grace à un tableau de bytes
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];  // Buffer de réception
    uint8_t len = sizeof(buf);

    // Si un message est reçu avec succès
    if (rf95.recv(buf, &len)) {
      // On stocke les données  du messag dans buf : à ce point precis on a recu le message radio
      Serial.print("📨 Message reçu : ");
      for (int i = 0; i < len; i++) {
        Serial.print((char)buf[i]);  // Affiche caractère par caractère
      }
      Serial.println();

      // Affiche la puissance du signal reçu
      Serial.print("📶 RSSI : ");
      Serial.print(rf95.lastRssi());
      Serial.println(" dBm");
    } else {
      Serial.println("⚠️ Erreur lors de la réception du message !");
    }
  }
}
