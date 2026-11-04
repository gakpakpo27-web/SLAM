// ============================================================================
//  test_moteurs.ino – Diagnostic des 4 moteurs (pilotés directement par l'ESP32)
//  Programme minimal : pas de WiFi, pas de lidar. Mêmes broches que ESP32_LD06_WiFi.ino :
//      moteur 1 : GPIO15 / GPIO2      moteur 2 : GPIO4  / GPIO16
//      moteur 3 : GPIO17 / GPIO5      moteur 4 : GPIO18 / GPIO19
//
//  Commandes (moniteur série 115200 bauds, « Pas de fin de ligne » ou « Nouvelle ligne ») :
//    1..4 = ce moteur seul, sens + pendant 1 s      ! puis 1..4 = ce moteur, sens - pendant 1 s
//    a = les 4 moteurs, sens + pendant 1 s          s = stop      ? = état
//    + / - = puissance (par pas de 25, de 50 à 255)
//
//  À lire dans le moniteur série après la mise sous tension :
//    - « Démarrage n°… » qui augmente tout seul -> l'ESP32 redémarre en boucle
//      (cause affichée : BROWNOUT = batterie / alimentation trop faible)
//    - les roues tournent alors que « STOP » défile -> câblage (broche, masse commune
//      ESP32 / carte moteur) ou carte moteur qui n'a pas d'entrée « au repos » à 0 V
//    - une roue tourne quelques instants puis s'arrête à la mise sous tension : broches
//      15 et 5 (moteurs 1 et 3) sont à l'état haut pendant le démarrage de l'ESP32,
//      avant que le programme ne tourne (voir le README)
// ============================================================================
#include "esp_system.h"

struct MotorPins { uint8_t a; uint8_t b; };
const MotorPins MOTOR_PINS[4] = { {15, 2}, {4, 16}, {17, 5}, {18, 19} };

void motorPinsLow() {
  for (int m = 0; m < 4; m++) {
    pinMode(MOTOR_PINS[m].a, OUTPUT); digitalWrite(MOTOR_PINS[m].a, LOW);
    pinMode(MOTOR_PINS[m].b, OUTPUT); digitalWrite(MOTOR_PINS[m].b, LOW);
  }
}
// avant setup() : arrêt au plus tôt
struct EarlyMotorOff { EarlyMotorOff() { motorPinsLow(); } };
EarlyMotorOff earlyMotorOff;

const int PWM_FREQ = 1000, PWM_RES = 8;
int power = 128;
RTC_NOINIT_ATTR uint32_t boots;      // garde sa valeur après un redémarrage logiciel / brownout
uint32_t runUntil = 0, stopsSent = 0, lastStop = 0, lastPrint = 0;
bool negNext = false;

void pwmWrite(uint8_t pin, uint8_t ch, int v) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, v);
#else
  ledcWrite(ch, v);
#endif
}
// v de -255 à +255 (sens brut, sans les inversions de l'interface)
void motorRaw(uint8_t m, int v) {
  v = constrain(v, -255, 255);
  if (v > 0)      { pwmWrite(MOTOR_PINS[m].a, m * 2, 0);  pwmWrite(MOTOR_PINS[m].b, m * 2 + 1, v); }
  else if (v < 0) { pwmWrite(MOTOR_PINS[m].a, m * 2, -v); pwmWrite(MOTOR_PINS[m].b, m * 2 + 1, 0); }
  else            { pwmWrite(MOTOR_PINS[m].a, m * 2, 0);  pwmWrite(MOTOR_PINS[m].b, m * 2 + 1, 0); }
}
void stopAll() { for (int m = 0; m < 4; m++) motorRaw(m, 0); }

const char* reasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "mise sous tension";
    case ESP_RST_BROWNOUT: return "BROWNOUT (tension trop faible : batterie, moteurs qui tirent trop)";
    case ESP_RST_SW:       return "redémarrage logiciel";
    case ESP_RST_PANIC:    return "PLANTAGE du programme";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "chien de garde (programme bloqué)";
    case ESP_RST_EXT:      return "bouton EN / reset";
    default:               return "autre";
  }
}

void setup() {
  motorPinsLow();
  Serial.begin(115200);
  esp_reset_reason_t r = esp_reset_reason();
  if (r == ESP_RST_POWERON) boots = 0;
  boots++;
  for (int m = 0; m < 4; m++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PINS[m].a, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR_PINS[m].b, PWM_FREQ, PWM_RES);
#else
    ledcSetup(m * 2, PWM_FREQ, PWM_RES);     ledcAttachPin(MOTOR_PINS[m].a, m * 2);
    ledcSetup(m * 2 + 1, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_PINS[m].b, m * 2 + 1);
#endif
  }
  stopAll();
  delay(300);
  Serial.println();
  Serial.println(F("=== test_moteurs (moteurs directs) ==="));
  Serial.print(F("Démarrage n°")); Serial.print(boots); Serial.print(F(" · cause : ")); Serial.println(reasonText(r));
  Serial.println(F("Roues ARRÊTÉES. Commandes : 1..4 = un moteur 1 s, !1..!4 = sens inverse, a = tous, s = stop, +/- = puissance, ? = état"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '!') { negNext = true; continue; }
    if (c >= '1' && c <= '4') {
      const int m = c - '1', v = negNext ? -power : power;
      stopAll(); motorRaw(m, v); runUntil = millis() + 1000;
      Serial.printf("-> moteur %d (GPIO%d / GPIO%d) sens %c, puissance %d, pendant 1 s\n", m + 1, MOTOR_PINS[m].a, MOTOR_PINS[m].b, v > 0 ? '+' : '-', power);
    }
    else if (c == 'a') { for (int m = 0; m < 4; m++) motorRaw(m, power); runUntil = millis() + 1000; Serial.println(F("-> les 4 moteurs, sens +, 1 s")); }
    else if (c == 's') { runUntil = 0; stopAll(); Serial.println(F("-> STOP")); }
    else if (c == '+') { power = min(255, power + 25); Serial.printf("puissance %d\n", power); }
    else if (c == '-') { power = max(50, power - 25); Serial.printf("puissance %d\n", power); }
    else if (c == '?') { Serial.printf("démarrage n°%u, arrêts envoyés : %u, puissance %d\n", boots, stopsSent, power); }
    if (c != '\n' && c != '\r') negNext = false;
  }
  if (runUntil && millis() > runUntil) { runUntil = 0; stopAll(); Serial.println(F("-> fin, STOP")); }
  if (!runUntil && millis() - lastStop >= 100) { lastStop = millis(); stopAll(); stopsSent++; }
  if (millis() - lastPrint >= 3000) {
    lastPrint = millis();
    Serial.printf("STOP maintenu (%u fois). Les roues tournent-elles ?\n", stopsSent);
  }
}
