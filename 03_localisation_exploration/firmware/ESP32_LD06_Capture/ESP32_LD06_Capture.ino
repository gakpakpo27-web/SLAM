// ============================================================================
//  ESP32 LD06 – Capture de scans (coordonnées polaires) pour le TP cartographie
//  Basé sur l'exemple de la librairie LD06 (Robot-Maker-SAS / Mike118)
//
//  Le robot reste silencieux et envoie un scan complet (360°) au PC
//  UNIQUEMENT quand on le lui demande. Format envoyé :
//
//      #SCAN,<id>,<nbPoints>,<vitesse °/s>
//      angle_deg,distance_mm,intensite
//      ...
//      #END
//
//  Commandes (Moniteur série ou script capture_lidar.py) :
//      s      -> envoie 1 scan
//      1..9   -> envoie N scans consécutifs
//      c      -> flux continu CSV        x -> stop
//      t      -> flux Teleplot (visualisation temps réel sur teleplot.fr)
//      ?      -> aide
// ============================================================================

#include "ld06.h"
// Ce firmware ne fait pas rouler le robot : il garde seulement les 4 moteurs ARRÊTÉS
// (mêmes broches que ESP32_LD06_WiFi.ino, moteurs pilotés directement par l'ESP32). Mettre 0 si aucun moteur n'est câblé.
#define STOP_MOTORS 1

#define RX_LIDAR      33
#define TX_LIDAR      -1          // TX non utilisé
#define LD06_PWM_PIN  32
#define LIDAR_BAUD    230400
#define PC_BAUD       115200      // 921600 possible : ~8x plus rapide (adapter --baud côté Python)

LD06 ld06(Serial2, LD06_PWM_PIN);
#if STOP_MOTORS
// 2 broches par moteur : les deux à LOW = moteur arrêté (roue libre)
const uint8_t MOTOR_PINS[8] = {15, 2,  4, 16,  17, 5,  18, 19};
void motorPinsLow() {
  for (uint8_t k = 0; k < 8; k++) { pinMode(MOTOR_PINS[k], OUTPUT); digitalWrite(MOTOR_PINS[k], LOW); }
}
// Exécuté pendant l'initialisation des variables globales, AVANT setup() : les roues s'arrêtent au plus tôt
struct EarlyMotorOff { EarlyMotorOff() { motorPinsLow(); } };
EarlyMotorOff earlyMotorOff;
#endif

enum Mode { MODE_IDLE, MODE_STREAM_CSV, MODE_TELEPLOT };
Mode     mode        = MODE_IDLE;
uint8_t  scansToSend = 0;
uint32_t scanId      = 0;
bool     skipNext    = false;     // le scan qui suit un envoi peut être incomplet

// ----------------------------------------------------------------------------
void printHelp() {
  Serial.println(F("# Commandes : s=1 scan | 1..9=N scans | c=flux CSV | t=Teleplot | x=stop"));
}

void sendScanCSV() {
  uint16_t n = ld06.getNbPointsInScan();
  Serial.print(F("#SCAN,"));
  Serial.print(scanId++);   Serial.print(',');
  Serial.print(n);          Serial.print(',');
  Serial.println(ld06.getSpeed());

  for (uint16_t i = 0; i < n; i++) {
    const auto* pt = ld06.getPoints(i);
    if (pt->distance == 0) continue;           // mesure invalide
    Serial.print(pt->angle, 2);   Serial.print(',');
    Serial.print(pt->distance);   Serial.print(',');
    Serial.println(pt->intensity);
  }
  Serial.println(F("#END"));

  // Pendant l'envoi (bloquant) le buffer Serial2 a débordé :
  // on le vide et on ignore le prochain scan, potentiellement tronqué.
  while (Serial2.available()) Serial2.read();
  skipNext = true;
}

void handleCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S')            scansToSend = 1;
    else if (c >= '1' && c <= '9')       scansToSend = c - '0';
    else if (c == 'c' || c == 'C')       mode = MODE_STREAM_CSV;
    else if (c == 't' || c == 'T')       mode = MODE_TELEPLOT;
    else if (c == 'x' || c == 'X')     { mode = MODE_IDLE; scansToSend = 0; }
    else if (c == '?')                   printHelp();
  }
}

// ----------------------------------------------------------------------------
void setup() {
#if STOP_MOTORS
  motorPinsLow();                                // roues arrêtées en tout premier
#endif
  Serial.begin(PC_BAUD);

  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, RX_LIDAR, TX_LIDAR);
#if STOP_MOTORS
  // TX_LIDAR = -1 : selon la version du core, Serial2 reprend sa broche TX par défaut, GPIO17 = moteur 3 (niveau haut
  // au repos -> la roue tournerait). On rend la broche aux moteurs juste après.
  motorPinsLow();
#endif
  Serial2.setTimeout(10);
  ld06.init();

  // Le lidar renvoie les points dans son propre repère : pas d'offset ici,
  // la pose de chaque scan est gérée ensuite dans l'outil de visualisation.
  // ld06.setUpsideDown(true);   // à activer si le LD06 est monté à l'envers

  delay(200);
  Serial.println(F("# ESP32 LD06 capture prete"));
  printHelp();
}

void loop() {
  handleCommands();
#if STOP_MOTORS
  static uint32_t lastStop = 0;                  // rappel de l'arrêt toutes les 300 ms (sécurité)
  if (millis() - lastStop >= 300) { lastStop = millis(); motorPinsLow(); }
#endif

  if (ld06.readScan()) {                 // true quand un tour complet est prêt
    if (skipNext) { skipNext = false; return; }

    if (scansToSend > 0) {
      sendScanCSV();
      scansToSend--;
    } else if (mode == MODE_STREAM_CSV) {
      sendScanCSV();
    } else if (mode == MODE_TELEPLOT) {
      ld06.printScanTeleplot(Serial);
    }
  }
}
