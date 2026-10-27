// ===== ESP32 véhicule mecanum : reçoit les commandes du T-Display en UDP =====
// - Se connecte au Wi-Fi du T-Display
// - Se signale toutes les 500 ms ("CAR") pour que le T-Display connaisse son IP
// - Reçoit vx (latéral), vy (avant/arrière), w (rotation) entre -1000 et +1000
// - Calcule la vitesse des 4 roues et pilote 4 drivers PWM + DIR
// - FAILSAFE : arrêt des moteurs si plus de commande depuis 300 ms

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- Réseau du T-Display ---
const char*     AP_SSID = "TDISPLAY-VIDEO";
const char*     AP_PASS = "video1234";
const IPAddress TDISPLAY_IP(192, 168, 4, 1);
const uint16_t  CTRL_PORT = 4210;

// --- Moteurs, dans l'ordre : avant-gauche, avant-droit, arrière-gauche, arrière-droit ---
//const char*   MOTOR_NAME[4] = {"AvG", "AvD", "ArG", "ArD"};
const char*   MOTOR_NAME[4] = {"ArG", "AvD", "ArD", "AvG"};
const uint8_t PIN_PWM[4]    = {33, 26, 25, 27};
const uint8_t PIN_DIR[4]    = {22, 19, 18, 21};
// Les moteurs du côté droit sont montés "à l'envers" : on inverse leur sens.
// À ajuster roue par roue lors du premier test (véhicule sur cales !)
const bool    INVERT[4]     = {false, false, false, false};

const uint32_t PWM_FREQ      = 20000;   // 20 kHz : silencieux
const uint8_t  PWM_BITS      = 10;      // 0..1023
const int      PWM_MAX       = (1 << PWM_BITS) - 1;
const int      MAX_SPEED_PCT = 100;     // limite globale (mettre 50 pour les premiers essais)
const uint32_t FAILSAFE_MS   = 300;
const float    RAMP_PER_MS   = 4.0f;    // accélération max : 0 -> 1000 en 250 ms

// --- Paquet de commande (identique côté T-Display) ---
struct __attribute__((packed)) DrivePacket {
  uint16_t magic;     // 0xD71E
  uint16_t seq;
  int16_t  vx;        // latéral   : -1000 (gauche) .. +1000 (droite)
  int16_t  vy;        // longitud. : -1000 (arrière) .. +1000 (avant)
  int16_t  w;         // rotation  : -1000 (gauche) .. +1000 (droite)
  uint8_t  flags;     // bit 0 : moteurs armés
  uint8_t  buttons;   // bit 0 : bouton 1, bit 1 : bouton 2 (pelle), bit 2 : joystick droit
  uint8_t  checksum;  // XOR de tous les octets précédents
};
const uint16_t PACKET_MAGIC = 0xD71E;

WiFiUDP udp;
float    target[4]  = {0, 0, 0, 0};
float    current[4] = {0, 0, 0, 0};
uint32_t lastRx     = 0;
uint16_t lastSeq    = 0;
bool     armed      = false;
bool     failsafe   = true;

// PWM compatible core ESP32 2.x (PlatformIO) et 3.x (IDE Arduino récent)
void pwmInit(int i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_PWM[i], PWM_FREQ, PWM_BITS);
#else
  ledcSetup(i, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_PWM[i], i);
#endif
}

void pwmWrite(int i, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PWM[i], duty);   // 3.x : par broche
#else
  ledcWrite(i, duty);            // 2.x : par canal
#endif
}

uint8_t checksumOf(const uint8_t* d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= d[i];
  return c;
}

void setMotor(int i, float speed) {
  bool fwd = speed >= 0;
  if (INVERT[i]) fwd = !fwd;
  digitalWrite(PIN_DIR[i], fwd ? HIGH : LOW);
  uint32_t duty = (uint32_t)(fabsf(speed) * PWM_MAX / 1000.0f * MAX_SPEED_PCT / 100.0f);
  pwmWrite(i, min(duty, (uint32_t)PWM_MAX));
}

void stopAll() {
  for (int i = 0; i < 4; i++) {
    target[i] = current[i] = 0;
    pwmWrite(i, 0);
  }
}

// Cinématique mecanum (rouleaux formant un "X" vus de dessus)
void computeMecanum(int vx, int vy, int w) {
  float v[4] = {
    (float)(vy + vx + w),   // avant-gauche
    (float)(vy - vx - w),   // avant-droit
    (float)(vy - vx + w),   // arrière-gauche
    (float)(vy + vx - w),   // arrière-droit
  };
  // Si une roue dépasse 1000, on réduit tout proportionnellement (garde la direction)
  float m = 1000;
  for (float x : v) m = max(m, fabsf(x));
  for (int i = 0; i < 4; i++) target[i] = v[i] * 1000.0f / m;
}

void handlePackets() {
  int size;
  while ((size = udp.parsePacket()) > 0) {
    if (size != sizeof(DrivePacket)) continue;   // le prochain parsePacket() jette ce paquet

    DrivePacket p;
    udp.read((uint8_t*)&p, sizeof(p));
    if (p.magic != PACKET_MAGIC) continue;
    if (p.checksum != checksumOf((uint8_t*)&p, sizeof(p) - 1)) continue;

    // Ignore un paquet plus ancien que le dernier reçu (sauf si la liaison était coupée)
    bool stale = millis() - lastRx > 1000;
    if (!stale && (int16_t)(p.seq - lastSeq) <= 0) continue;

    lastSeq = p.seq;
    lastRx  = millis();
    if (failsafe) Serial.println("Commandes recues");
    failsafe = false;

    static uint8_t lastButtons = 0;
    if (p.buttons != lastButtons) {
      Serial.printf("Boutons : B1=%d B2=%d JoyD=%d\n",
                    p.buttons & 1, (p.buttons >> 1) & 1, (p.buttons >> 2) & 1);
      lastButtons = p.buttons;
    }

    bool nowArmed = p.flags & 0x01;
    if (nowArmed != armed) Serial.println(nowArmed ? "Moteurs ARMES" : "Moteurs DESARMES");
    armed = nowArmed;

    if (armed) computeMecanum(p.vx, p.vy, p.w);
    else       stopAll();
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Vehicule mecanum ===");

  for (int i = 0; i < 4; i++) {
    pinMode(PIN_DIR[i], OUTPUT);
    digitalWrite(PIN_DIR[i], LOW);
    pwmInit(i);
    pwmWrite(i, 0);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);     // latence minimale
  WiFi.begin(AP_SSID, AP_PASS);
  udp.begin(CTRL_PORT);
}

void loop() {
  static uint32_t lastHello = 0, lastLog = 0, lastRamp = millis();

  if (WiFi.status() != WL_CONNECTED) {
    stopAll();
    failsafe = true;
    if (millis() - lastLog > 2000) {
      Serial.printf("Recherche du Wi-Fi \"%s\"...\n", AP_SSID);
      lastLog = millis();
    }
    delay(50);
    return;
  }

  // Signale notre présence au T-Display
  if (millis() - lastHello > 500) {
    udp.beginPacket(TDISPLAY_IP, CTRL_PORT);
    udp.write((const uint8_t*)"CAR", 3);
    udp.endPacket();
    lastHello = millis();
  }

  handlePackets();

  // FAILSAFE
  if (!failsafe && millis() - lastRx > FAILSAFE_MS) {
    Serial.println("FAILSAFE : plus de commande, arret moteurs");
    failsafe = true;
    armed = false;
    stopAll();
  }

  // Rampe d'accélération puis application aux moteurs
  uint32_t now = millis();
  float step = RAMP_PER_MS * (now - lastRamp);
  lastRamp = now;
  for (int i = 0; i < 4; i++) {
    float d = target[i] - current[i];
    current[i] += constrain(d, -step, step);
    setMotor(i, current[i]);
  }

  if (millis() - lastLog > 500 && armed) {
    Serial.printf("%s %5.0f | %s %5.0f | %s %5.0f | %s %5.0f\n",
                  MOTOR_NAME[0], current[0], MOTOR_NAME[1], current[1],
                  MOTOR_NAME[2], current[2], MOTOR_NAME[3], current[3]);
    lastLog = millis();
  }

  delay(2);
}