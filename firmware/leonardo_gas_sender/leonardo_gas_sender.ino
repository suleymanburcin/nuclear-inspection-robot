/*
 * ARDUINO LEONARDO - Gaz + BME280 + Joystick Shield + GPS (MASTER)
 * ================================================================
 * Funduino Joystick Shield V1.A uzerinde calisir.
 *
 * GAZ sensorleri (analog) - shield joystick'e yer acmak icin kaydirildi:
 *   MQ2 -> A2   MQ8 -> A3   MQ135 -> A4   MQ7 -> A5   MiCS -> A9
 *
 * BME280 (I2C):  SDA->D2  SCL->D3   Adres 0x76/0x77
 *
 * JOYSTICK (shield sabit pinleri):
 *   X ekseni -> A0    Y ekseni -> A1
 *   Butonlar: C->D4  D->D5  E->D6  F->D7   K(joystick bas)->D8
 *   (A->D2, B->D3 BME280 ile cakisir, KULLANILMIYOR)
 *   Butonlar shield'de pull-up'li: basili=LOW, serbest=HIGH
 *
 * GPS (GY-NEO6MV2, SoftwareSerial):
 *   GPS TX -> Leonardo D10 (SoftwareSerial RX)  [sadece dinleme]
 *   GPS RX -> D11 (kullanilmiyor)
 *   GPS VCC->5V  GND->GND   9600 baud NMEA
 *
 * Kutuphaneler: Adafruit BME280 + Adafruit Unified Sensor + TinyGPSPlus
 *
 * Frame (saniyede bir):
 *   GAS,joyX,joyY,btns,mq2,mq8,mq135,mq7,mics,tC,pPa,hC,lat,lon,sats,spd,cog,dist,chk\n
 *   joyX/joyY: 0-1023 | btns: "CDEFK" sirasi "00000"/"10100"...
 *   Gaz mV | tC x100 | pPa Pa | hC x100 | lat/lon x1e6 | sats uydu
 *   spd: hiz m/s x100 | cog: gidis yonu derece x10 (-1=gecersiz/duruyor)
 *   dist: JSN-SR04T mesafe cm (-1=olcum yok)
 *   BME yoksa tC/pPa/hC=-1 | GPS fix yoksa lat=0,lon=0,sats=0,spd=0,cog=-1
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// ---------- Gaz pinleri (kaydirilmis) ----------
const uint8_t PIN_MQ2   = A2;
const uint8_t PIN_MQ8   = A3;
const uint8_t PIN_MQ135 = A4;
const uint8_t PIN_MQ7   = A5;
const uint8_t PIN_MICS  = A9;

// ---------- Joystick shield ----------
const uint8_t PIN_JOY_X = A0;
const uint8_t PIN_JOY_Y = A1;
const uint8_t PIN_BTN_C = 4;    // buton C
const uint8_t PIN_BTN_D = 5;    // buton D
const uint8_t PIN_BTN_E = 6;    // buton E
const uint8_t PIN_BTN_F = 7;    // buton F
const uint8_t PIN_BTN_K = 8;    // joystick basma

// ---------- GPS (SoftwareSerial) ----------
const uint8_t GPS_RX_PIN = 10;  // GPS TX buraya
const uint8_t GPS_TX_PIN = 11;  // kullanilmiyor
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

// ---------- JSN-SR04T ULTRASONIK (su seviyesi / mesafe) ----------
// Su gecirmez ultrasonik. HC-SR04 gibi calisir. Menzil ~25cm-450cm.
const uint8_t US_TRIG_PIN = 12;  // tetikleme cikisi
const uint8_t US_ECHO_PIN = 13;  // yanki girisi
const unsigned long US_TIMEOUT = 30000UL;  // us; ~5m otesi = olcum yok

// ---------- Ayarlar ----------
const int      SAMPLES   = 16;
const uint32_t PERIOD_MS      = 40;    // FRAME hizi (joystick icin ~25 Hz)
const uint32_t SENSOR_PERIOD  = 1000;  // agir sensorler (gaz/BME/ultrasonik) saniyede 1
const uint32_t WARMUP_S  = 30;
const float    ADC_REF_MV = 5000.0;

Adafruit_BME280 bme;
bool bmeOk = false;
uint32_t lastSend = 0;
uint32_t lastSensor = 0;
// Agir sensorlerin son okunan degerleri (saniyede 1 guncellenir, her frame'de kullanilir)
int  g_mq2=0, g_mq8=0, g_mq135=0, g_mq7=0, g_mics=0;
long g_tC=-1, g_pPa=-1, g_hC=-1;
long g_dist=-1;

int readAvg_mV(uint8_t pin) {
  uint32_t acc = 0;
  for (int i = 0; i < SAMPLES; i++) { acc += analogRead(pin); }  // delay kaldirildi (bloklamasin)
  return (int)(((float)acc / SAMPLES) * ADC_REF_MV / 1023.0);
}

uint8_t xorChecksum(const String &s) {
  uint8_t c = 0;
  for (uint16_t i = 0; i < s.length(); i++) c ^= (uint8_t)s[i];
  return c;
}

// GPS'i surekli besle (karakter dusurmesin)
void feedGPS() {
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
}

// JSN-SR04T mesafe olcumu (cm). Olcum yoksa -1 doner.
long readUltrasonic() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);
  // echo suresini olc (timeout: ~5m)
  unsigned long dur = pulseIn(US_ECHO_PIN, HIGH, US_TIMEOUT);
  if (dur == 0) return -1;               // yanki yok / menzil disi
  long cm = (long)(dur * 0.0343 / 2.0);  // ses hizi 343 m/s
  // JSN-SR04T kor bolgesi ~20-25 cm (uretici belirtimi). 2 cm alt siniri HC-SR04
  // icindir; bu sensorde 20 cm altindaki okumalar guvenilir degildir.
  if (cm < 20 || cm > 500) return -1;    // gecersiz aralik (kor bolge + menzil)
  return cm;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);        // ESP32.ye giden hat (hizlandirildi)
  gpsSerial.begin(9600);        // GPS

  analogReference(DEFAULT);

  // Butonlar: shield pull-up'li, INPUT_PULLUP guvenli
  pinMode(PIN_BTN_C, INPUT_PULLUP);
  pinMode(PIN_BTN_D, INPUT_PULLUP);
  pinMode(PIN_BTN_E, INPUT_PULLUP);
  pinMode(PIN_BTN_F, INPUT_PULLUP);
  pinMode(PIN_BTN_K, INPUT_PULLUP);

  // JSN-SR04T ultrasonik
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);

  // BME280
  Wire.begin();
  if (bme.begin(0x76) || bme.begin(0x77)) {
    bmeOk = true;
    Serial.println(F("BME280 hazir."));
  } else {
    Serial.println(F("BME280 BULUNAMADI. Gaz devam eder."));
  }

  Serial.println(F("GPS D10'da dinleniyor (acik alanda fix birkac dk surer)."));
  Serial.print(F("Isinma "));
  for (uint32_t s = 0; s < WARMUP_S; s++) {
    feedGPS();
    Serial.print('.');
    delay(1000);
  }
  Serial.println(F(" tamam. Yayin basliyor.\n"));
}

void loop() {
  feedGPS();                    // GPS'i SUREKLI besle

  // ---- AGIR SENSORLER: saniyede 1 oku (gaz/BME/ultrasonik), global'e sakla ----
  if (millis() - lastSensor >= SENSOR_PERIOD) {
    lastSensor = millis();
    g_mq2   = readAvg_mV(PIN_MQ2);
    g_mq8   = readAvg_mV(PIN_MQ8);
    g_mq135 = readAvg_mV(PIN_MQ135);
    g_mq7   = readAvg_mV(PIN_MQ7);
    g_mics  = readAvg_mV(PIN_MICS);
    if (bmeOk) {
      g_tC  = (long)(bme.readTemperature() * 100.0);
      g_pPa = (long)(bme.readPressure());
      g_hC  = (long)(bme.readHumidity() * 100.0);
    }
    g_dist = readUltrasonic();   // 30ms pulseIn - sadece saniyede 1 (frame'i bloklamaz)
  }

  // ---- FRAME: hizli (joystick+buton taze) - ~25 Hz ----
  if (millis() - lastSend < PERIOD_MS) return;
  lastSend = millis();

  // ---- Joystick (HER FRAME taze oku - hizli tepki) ----
  int joyX = analogRead(PIN_JOY_X);
  int joyY = analogRead(PIN_JOY_Y);

  // ---- Butonlar (HER FRAME taze) ----
  char btns[6];
  btns[0] = (digitalRead(PIN_BTN_C) == LOW) ? '1' : '0';
  btns[1] = (digitalRead(PIN_BTN_D) == LOW) ? '1' : '0';
  btns[2] = (digitalRead(PIN_BTN_E) == LOW) ? '1' : '0';
  btns[3] = (digitalRead(PIN_BTN_F) == LOW) ? '1' : '0';
  btns[4] = (digitalRead(PIN_BTN_K) == LOW) ? '1' : '0';
  btns[5] = '\0';

  // ---- Agir sensorler: son okunan global degerler (saniyede 1 guncellenmis) ----
  int mq2=g_mq2, mq8=g_mq8, mq135=g_mq135, mq7=g_mq7, mics=g_mics;
  long tC=g_tC, pPa=g_pPa, hC=g_hC;
  long dist=g_dist;

  // ---- GPS (her frame taze, hafif) ----
  long lat = 0, lon = 0;
  int  sats = 0;
  long spd = 0;
  long cog = -1;
  if (gps.location.isValid()) {
    lat = (long)(gps.location.lat() * 1000000.0);
    lon = (long)(gps.location.lng() * 1000000.0);
  }
  if (gps.satellites.isValid()) sats = gps.satellites.value();
  if (gps.speed.isValid())  spd = (long)(gps.speed.mps() * 100.0);
  if (gps.course.isValid()) cog = (long)(gps.course.deg() * 10.0);

  // ---- Frame ----
  String payload = String(joyX) + "," + String(joyY) + "," + String(btns) + "," +
                   String(mq2)  + "," + String(mq8)  + "," + String(mq135) + "," +
                   String(mq7)  + "," + String(mics) + "," +
                   String(tC)   + "," + String(pPa)  + "," + String(hC) + "," +
                   String(lat)  + "," + String(lon)  + "," + String(sats) + "," +
                   String(spd)  + "," + String(cog)  + "," + String(dist);
  uint8_t chk = xorChecksum(payload);
  String frame = "GAS," + payload + "," + String(chk);

  Serial1.println(frame);       // -> ESP32 (hizli)
  Serial.println(frame);        // -> USB (test: her frame gorunur, joystick canli)
}
