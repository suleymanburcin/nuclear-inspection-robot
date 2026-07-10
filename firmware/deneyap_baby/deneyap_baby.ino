/*
 * ====== DENEYAP KART - ELEKTRIKLI ROBOT (WiFi + UDP) ======
 * Skid-steer, CALISAN Bluetooth kodunun pin duzenine gore (kablolar ayni).
 *   Sag motor: D0/D1 (yon) | Sol motor: D8/D7 (yon) | PWM: D9 (hiz, her iki taraf)
 * Jetson/PC hotspot'una baglanir, UDP broadcast komut dinler.
 *
 * WiFi: PC/Jetson mobil hotspot (2.4 GHz)
 *   Ag: "<AGINIZIN_ADI>"  Sifre: "<PAROLA>"  UDP port: 4210
 *
 * KOMUTLAR (UDP paket): "F,s1,s2,s3,s4"
 *   F ileri | B geri | L sola | R saga | S dur
 *   s1..s4 servo acilari (0-180)
 *
 * Arduino IDE: Board = "Deneyap Kart 1A" (Dydk1a) SECILI OLMALI!
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ---- WiFi ----
// WiFi kimlik bilgileri ayri dosyada tutulur (GitHub'a yuklenmez).
// "secrets_ornek.h" dosyasini "secrets.h" olarak kopyalayip doldurun.
#include "secrets.h"   // WIFI_SSID, WIFI_PASS
const uint16_t UDP_PORT = 4210;
WiFiUDP udp;
char pkt[32];

// ---- Motor pinleri (CALISAN kod ile ayni) ----
int motor1_IN1 = D0;   // sag
int motor1_IN2 = D1;
int motor2_IN1 = D8;   // sol
int motor2_IN2 = D7;
int motor3_IN1 = D0;   // sag (paralel)
int motor3_IN2 = D1;
int motor4_IN1 = D8;   // sol (paralel)
int motor4_IN2 = D7;
int pwmPin1 = D9;      // sag PWM
int pwmPin2 = D9;      // sol PWM (ayni pin)
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmFreq = 5000;
const int pwmResolution = 8;

// ---- Servo pinleri (bos, PWM'li) ----
// NOT: D7 sol motorda, D9 PWM'de. Servolar icin D4,D5,D6 + D10 kullaniyoruz.
#define S1_PIN D4    // en alt servo (taban)
#define S2_PIN D5    // bir ust servo (orta eklem)
#define S3_PIN D6    // sondaki servo (kiskac AC/KAPA - butondan)
Servo s1, s2, s3;    // 3 servo

uint32_t lastCmdMs = 0;
bool timeoutWarned = false;
const uint32_t CMD_TIMEOUT = 1500;

void setPWM(int dutyCycle) {
  ledcWrite(pwmChannel1, dutyCycle);
  ledcWrite(pwmChannel2, dutyCycle);
}

// --- Hareket fonksiyonlari (CALISAN koddan birebir) ---
void forward() {
  digitalWrite(motor1_IN1, HIGH); digitalWrite(motor1_IN2, LOW);
  digitalWrite(motor3_IN1, HIGH); digitalWrite(motor3_IN2, LOW);
  digitalWrite(motor2_IN1, HIGH); digitalWrite(motor2_IN2, LOW);
  digitalWrite(motor4_IN1, HIGH); digitalWrite(motor4_IN2, LOW);
  setPWM(128);
}
void backward() {
  digitalWrite(motor1_IN1, LOW); digitalWrite(motor1_IN2, HIGH);
  digitalWrite(motor3_IN1, LOW); digitalWrite(motor3_IN2, HIGH);
  digitalWrite(motor2_IN1, LOW); digitalWrite(motor2_IN2, HIGH);
  digitalWrite(motor4_IN1, LOW); digitalWrite(motor4_IN2, HIGH);
  setPWM(128);
}
void turnRight() {
  digitalWrite(motor1_IN1, LOW); digitalWrite(motor1_IN2, HIGH);
  digitalWrite(motor3_IN1, LOW); digitalWrite(motor3_IN2, HIGH);
  digitalWrite(motor2_IN1, HIGH); digitalWrite(motor2_IN2, LOW);
  digitalWrite(motor4_IN1, HIGH); digitalWrite(motor4_IN2, LOW);
  setPWM(90);
}
void turnLeft() {
  digitalWrite(motor1_IN1, HIGH); digitalWrite(motor1_IN2, LOW);
  digitalWrite(motor3_IN1, HIGH); digitalWrite(motor3_IN2, LOW);
  digitalWrite(motor2_IN1, LOW); digitalWrite(motor2_IN2, HIGH);
  digitalWrite(motor4_IN1, LOW); digitalWrite(motor4_IN2, HIGH);
  setPWM(90);
}
void stopAll() {
  digitalWrite(motor1_IN1, LOW); digitalWrite(motor1_IN2, LOW);
  digitalWrite(motor2_IN1, LOW); digitalWrite(motor2_IN2, LOW);
  digitalWrite(motor3_IN1, LOW); digitalWrite(motor3_IN2, LOW);
  digitalWrite(motor4_IN1, LOW); digitalWrite(motor4_IN2, LOW);
  setPWM(0);
}

void setup() {
  Serial.begin(115200);
  pinMode(motor1_IN1, OUTPUT); pinMode(motor1_IN2, OUTPUT);
  pinMode(motor2_IN1, OUTPUT); pinMode(motor2_IN2, OUTPUT);
  pinMode(motor3_IN1, OUTPUT); pinMode(motor3_IN2, OUTPUT);
  pinMode(motor4_IN1, OUTPUT); pinMode(motor4_IN2, OUTPUT);
  // PWM (calisan koddaki gibi)
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin1, pwmChannel1);
  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin2, pwmChannel2);
  stopAll();

  // Servolar (sirayla baslat - ani akim koruma)
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  s1.setPeriodHertz(50); s2.setPeriodHertz(50);
  s3.setPeriodHertz(50);
  s1.attach(S1_PIN, 500, 2400); s1.write(90); delay(120);  // en alt orta
  s2.attach(S2_PIN, 500, 2400); s2.write(90); delay(120);  // bir ust orta
  s3.attach(S3_PIN, 500, 2400); s3.write(0);  delay(120);  // kiskac kapali basla

  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi baglaniyor");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("BAGLANDI. IP: "); Serial.println(WiFi.localIP());
    udp.begin(UDP_PORT);
    Serial.printf("UDP dinleniyor port %d\n", UDP_PORT);
  } else {
    Serial.println("\nWiFi BAGLANAMADI - ag/sifre/hotspot kontrol");
  }
  lastCmdMs = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    stopAll();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(500);
    return;
  }

  int n = udp.parsePacket();
  if (n > 0) {
    int len = udp.read(pkt, sizeof(pkt) - 1);
    if (len > 0) {
      pkt[len] = '\0';
      Serial.print("[HAM] "); Serial.println(pkt);
      char c = pkt[0];
      const char* hareket = "?";
      switch (c) {
        case 'F': backward();  hareket="ILERI"; break;   // F->geri fonk (ters duzeltme)
        case 'B': forward();   hareket="GERI";  break;   // B->ileri fonk
        case 'L': turnRight(); hareket="SOLA";  break;   // L->sag fonk (ters duzeltme)
        case 'R': turnLeft();  hareket="SAGA";  break;   // R->sol fonk
        case 'S': stopAll();   hareket="DUR";   break;
        default:  hareket="GECERSIZ"; break;
      }
      int sv[4] = {-1,-1,-1,-1};
      char *p = strchr(pkt, ',');
      if (p) {
        char *tok = strtok(p + 1, ",");
        int i = 0;
        while (tok && i < 4) { sv[i++] = atoi(tok); tok = strtok(NULL, ","); }
        if (sv[0] >= 0) s1.write(constrain(sv[0], 0, 180));  // en alt
        if (sv[1] >= 0) s2.write(constrain(sv[1], 0, 180));  // bir ust
        if (sv[2] >= 0) s3.write(constrain(sv[2], 0, 180));  // kiskac (ac/kapa)
      }
      lastCmdMs = millis();
      timeoutWarned = false;
      Serial.print("[KOMUT] hareket="); Serial.print(hareket);
      Serial.print(" servo="); Serial.print(sv[0]); Serial.print(",");
      Serial.print(sv[1]); Serial.print(","); Serial.print(sv[2]);
      Serial.print(","); Serial.println(sv[3]);
    }
  }

  if (millis() - lastCmdMs > CMD_TIMEOUT) {
    if (!timeoutWarned) {
      Serial.println("[GUVENLIK] Komut kesildi -> motorlar durduruldu.");
      timeoutWarned = true;
    }
    stopAll();
  }
}
