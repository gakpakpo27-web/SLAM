// ============================================================================
//  ESP32 LD06 – WiFi (méthode du cours, ESP32RobotControl) + USB (CSV)
//
//  Partie WiFi reprise du firmware du module « ESP32RobotControl.ino » :
//    - point d'accès « RobotESP32 », mot de passe « 12345678 », http://192.168.4.1
//    - serveur ESPAsyncWebServer (bibliothèques AsyncTCP et ESPAsyncWebServer)
//    - routes identiques : /scan (binaire), /pose, /forward, /backward, /left, /right,
//      /rotateleft, /rotateright (?time=ms), /stop, /ping
//    => compatible avec RobotControl.html ET avec lidar_viewer.html (mode WiFi)
//
//  Ajout pour ce projet : les commandes USB de ESP32_LD06_Capture.ino
//    s = 1 scan | 1..9 = N scans | c = flux CSV | t = Teleplot | x = stop | ? = infos
//    Format : #SCAN,<id>,<nbPoints>,<vitesse> / angle_deg,distance_mm,intensite / #END
//
//  Bibliothèques : LD06 (ld06.h), MecanumCar_v2, AsyncTCP, ESPAsyncWebServer
//  Carte : ESP32 Dev Module
// ============================================================================

#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ld06.h"
#include "MecanumCar_v2.h"

// ============================================================
// WIFI AP (identique au firmware du cours)
// ============================================================
const char* ssid = "RobotESP32";
const char* password = "12345678";
AsyncWebServer server(80);

// ROBOT POSE (saisie par /pose, renvoyée avec chaque scan)
int16_t posex = 0;     // mm
int16_t posey = 0;     // mm
uint16_t posez = 0;    // centièmes de degré

// ============================================================
// LIDAR LD06
// ============================================================
#define RX_LIDAR 33
#define TX_LIDAR -1 // Keep default/current UART TX assignment; leave it unconnected.
#define LD06_PWM_PIN 32
#define LIDAR_BAUD 230400
#define PC_BAUD 115200
LD06 lidar(Serial2, LD06_PWM_PIN);

// ============================================================
// OUTILS HTTP
// ============================================================
int getArg(AsyncWebServerRequest* req, const char* name, int def) {
  return req->hasParam(name) ? req->getParam(name)->value().toInt() : def;
}

void addCORS(AsyncWebServerResponse* resp) {
  resp->addHeader("Access-Control-Allow-Origin", "*");
  resp->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  resp->addHeader("Access-Control-Allow-Headers", "*");
}

void sendOK(AsyncWebServerRequest* r, const char* txt) {
  auto x = r->beginResponse(200, "text/plain", txt);
  addCORS(x); r->send(x);
}

// ============================================================
// MOTEURS
// ============================================================
bool moving = false;
uint32_t moveStarted = 0;
uint32_t moveDuration = 0;
SemaphoreHandle_t motorMutex;
const int MAX_MOVE_MS = 2000;

mecanumCar mecanumCar(19, 21);

void stopRobot() {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  mecanumCar.Stop();
  moving = false;
  xSemaphoreGive(motorMutex);
}

void doMove(void (*fn)(), int t) {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  if (t <= 0) {
    mecanumCar.Stop();
    moving = false;
  } else {
    moveDuration = (uint32_t)constrain(t, 1, MAX_MOVE_MS);
    moveStarted = millis();
    fn();
    moving = true;
  }
  xSemaphoreGive(motorMutex);
}

void forward(int t)     { doMove([](){ mecanumCar.Advance(); }, t); }
void backward(int t)    { doMove([](){ mecanumCar.Back(); }, t); }
void left(int t)        { doMove([](){ mecanumCar.L_Move(); }, t); }
void right(int t)       { doMove([](){ mecanumCar.R_Move(); }, t); }
void rotateleft(int t)  { doMove([](){ mecanumCar.Turn_Left(); }, t/2); }
void rotateright(int t) { doMove([](){ mecanumCar.Turn_Right(); }, t/2); }

// ============================================================
// SCAN BINAIRE (WiFi, format du cours)
//   [int16 x mm][int16 y mm][uint16 θ 1/100°][uint16 N] + N × [uint16 angle 1/100°][uint16 dist mm][uint8 intensité]
// ============================================================
const uint16_t MAX_SCAN_POINTS = 460;
const size_t MAX_SCAN_BYTES = 8 + 5 * MAX_SCAN_POINTS;
uint8_t publishedScan[MAX_SCAN_BYTES];
size_t publishedSize = 0;
portMUX_TYPE scanMux = portMUX_INITIALIZER_UNLOCKED;

void publishScan() {
  const uint16_t n = lidar.getNbPointsInScan();
  if (n == 0 || n > MAX_SCAN_POINTS) return;
  uint8_t snapshot[MAX_SCAN_BYTES];
  snapshot[6] = n & 0xff;
  snapshot[7] = n >> 8;
  size_t off = 8;
  for (uint16_t i = 0; i < n; ++i) {
    const auto* pt = lidar.getPoints(i);
    const uint16_t angle = (uint16_t)(pt->angle * 100);
    snapshot[off++] = angle & 0xff;
    snapshot[off++] = angle >> 8;
    snapshot[off++] = pt->distance & 0xff;
    snapshot[off++] = pt->distance >> 8;
    snapshot[off++] = pt->intensity;
  }
  portENTER_CRITICAL(&scanMux);
  memcpy(publishedScan + 6, snapshot + 6, off - 6);
  publishedSize = off;
  portEXIT_CRITICAL(&scanMux);
}

void sendScanBinary(AsyncWebServerRequest* req) {
  uint8_t snapshot[MAX_SCAN_BYTES];
  size_t size;
  portENTER_CRITICAL(&scanMux);
  size = publishedSize;
  memcpy(snapshot, publishedScan, size);
  snapshot[0] = posex & 0xff; snapshot[1] = (uint16_t)posex >> 8;
  snapshot[2] = posey & 0xff; snapshot[3] = (uint16_t)posey >> 8;
  snapshot[4] = posez & 0xff; snapshot[5] = posez >> 8;
  portEXIT_CRITICAL(&scanMux);
  if (!size) {
    auto response = req->beginResponse(503, "text/plain", "NO_VALID_SCAN");
    addCORS(response); req->send(response);
    return;
  }
  auto response = req->beginResponseStream("application/octet-stream");
  response->write(snapshot, size);
  addCORS(response);
  req->send(response);
}

// ============================================================
// USB : commandes série (comme ESP32_LD06_Capture.ino)
// ============================================================
enum SerialMode { SER_IDLE, SER_CSV, SER_TELEPLOT };
SerialMode serialMode = SER_IDLE;
uint8_t serialToSend = 0;
uint32_t scanId = 0;
bool skipNext = false;   // après un envoi série (bloquant), le tour suivant peut être tronqué

void printInfo() {
  Serial.println(F("# ---------------------------------------------"));
  Serial.print(F("# WiFi : ")); Serial.print(ssid); Serial.print(F("  mot de passe : ")); Serial.println(password);
  Serial.print(F("# Adresse du robot : http://")); Serial.println(WiFi.softAPIP());
  Serial.println(F("# USB : s=1 scan | 1..9=N scans | c=flux CSV | t=Teleplot | x=stop"));
  Serial.println(F("# ---------------------------------------------"));
}

void sendScanCSV() {
  uint16_t n = lidar.getNbPointsInScan();
  Serial.print(F("#SCAN,")); Serial.print(scanId++); Serial.print(',');
  Serial.print(n); Serial.print(','); Serial.println(lidar.getSpeed());
  for (uint16_t i = 0; i < n; i++) {
    const auto* pt = lidar.getPoints(i);
    if (pt->distance == 0) continue;
    Serial.print(pt->angle, 2); Serial.print(',');
    Serial.print(pt->distance); Serial.print(',');
    Serial.println(pt->intensity);
  }
  Serial.println(F("#END"));
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S')        serialToSend = 1;
    else if (c >= '1' && c <= '9')   serialToSend = c - '0';
    else if (c == 'c' || c == 'C')   serialMode = SER_CSV;
    else if (c == 't' || c == 'T')   serialMode = SER_TELEPLOT;
    else if (c == 'x' || c == 'X') { serialMode = SER_IDLE; serialToSend = 0; }
    else if (c == '?')               printInfo();
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(PC_BAUD);
  delay(200);

  motorMutex = xSemaphoreCreateMutex();
  if (!motorMutex) { while (true) delay(1000); }
  stopRobot();

  // LIDAR UART (buffer plus grand que dans le firmware du cours : l'envoi USB est bloquant)
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, RX_LIDAR, TX_LIDAR);
  Serial2.setTimeout(2);
  lidar.init(); // Sets the Lidar PWM input on GPIO32 HIGH (no periodic PWM).

  // WIFI AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  delay(200);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // ROUTES (identiques au firmware du cours)
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* r){ stopRobot(); sendOK(r, "STOPPED"); });
  server.on("/forward", HTTP_GET, [](AsyncWebServerRequest* r){ forward(getArg(r, "time", 300)); sendOK(r, "OK"); });
  server.on("/backward", HTTP_GET, [](AsyncWebServerRequest* r){ backward(getArg(r, "time", 300)); sendOK(r, "OK"); });
  server.on("/left", HTTP_GET, [](AsyncWebServerRequest* r){ left(getArg(r, "time", 300)); sendOK(r, "OK"); });
  server.on("/right", HTTP_GET, [](AsyncWebServerRequest* r){ right(getArg(r, "time", 300)); sendOK(r, "OK"); });
  server.on("/rotateleft", HTTP_GET, [](AsyncWebServerRequest* r){ rotateleft(getArg(r, "time", 300)); sendOK(r, "OK"); });
  server.on("/rotateright", HTTP_GET, [](AsyncWebServerRequest* r){ rotateright(getArg(r, "time", 300)); sendOK(r, "OK"); });

  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest* r){ sendScanBinary(r); });

  server.on("/pose", HTTP_GET, [](AsyncWebServerRequest* r){
    const int xPose = constrain(getArg(r, "x", 0), -32768, 32767);
    const int yPose = constrain(getArg(r, "y", 0), -32768, 32767);
    const int angle = getArg(r, "z", 0);
    portENTER_CRITICAL(&scanMux);
    posex = xPose;
    posey = yPose;
    posez = ((angle % 36000) + 36000) % 36000;
    portEXIT_CRITICAL(&scanMux);
    sendOK(r, "OK");
  });

  // /ping avec CORS (dans le firmware du cours, /ping n'a pas l'en-tête : une page ouverte depuis le PC ne peut pas lire sa réponse)
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest* r){ sendOK(r, "OK"); });

  server.begin();
  printInfo();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  handleSerial();

  // Lire le lidar régulièrement ; chaque tour complet est publié pour le WiFi
  if (lidar.readScan()) {
    if (skipNext) {
      skipNext = false;
    } else {
      publishScan();
      bool usedSerial = false;
      if (serialToSend > 0 || serialMode == SER_CSV) {
        sendScanCSV();
        if (serialToSend > 0) serialToSend--;
        usedSerial = true;
      } else if (serialMode == SER_TELEPLOT) {
        lidar.printScanTeleplot(Serial);
        usedSerial = true;
      }
      if (usedSerial) {                       // l'envoi série à 115200 bauds est lent : buffer lidar vidé
        while (Serial2.available()) Serial2.read();
        skipNext = true;
      }
    }
  }

  // Laisser respirer la pile WiFi
  static uint32_t lastYield = 0;
  if (millis() - lastYield > 20) {
    lastYield = millis();
    yield();
  }

  // Arrêt auto des moteurs
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  if (moving && (uint32_t)(millis() - moveStarted) >= moveDuration) {
    mecanumCar.Stop();
    moving = false;
  }
  xSemaphoreGive(motorMutex);
}
