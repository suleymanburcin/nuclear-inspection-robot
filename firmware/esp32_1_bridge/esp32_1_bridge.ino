/*
 * ESP32 - Leonardo'dan alir + BNO055 pusula okur, JETSON'a JSON yollar
 * =====================================================================
 * Leonardo --UART 9600 (Serial2/GPIO16)--> ESP32 --UART 115200
 *                                          (Serial1/GPIO2)--> Jetson ttyTHS1
 * BNO055 --I2C (SDA=GPIO21, SCL=GPIO22)--> ESP32
 * Ortak GND SART.
 *
 * KABLOLAMA:
 *   Leonardo TX  --(bolen 5V->3.3V)--> ESP32 GPIO16 (RX2)   [giris]
 *   ESP32 GPIO2 (TX1) ---------------> Jetson pin10 (UART1_RX / ttyTHS1)
 *   ESP32 GND ------------------------ Jetson pin6  (GND)
 *
 *   BNO055 VIN --> ESP32 3.3V   (BNO055'e 5V VERME)
 *   BNO055 GND --> ESP32 GND
 *   BNO055 SDA --> ESP32 GPIO21
 *   BNO055 SCL --> ESP32 GPIO22
 *   BNO055 ADR --> bos (adres 0x28)  |  3.3V'a baglarsan 0x29
 *
 * GEREKLI KUTUPHANELER (Arduino IDE -> Library Manager):
 *   - Adafruit BNO055
 *   - Adafruit Unified Sensor
 *   - Adafruit BusIO   (otomatik bagimlilik)
 *
 * Cikis JSON'a EKLENEN alanlar:
 *   "heading" : 0-360 derece (pusula, N=0, dogu=90, guney=180, bati=270)
 *   "imuCal"  : 0-3 (BNO kalibrasyon; 3=tam kalibre, dusukse pusulayi gezdir)
 *   heading=null ise BNO bulunamadi/okunamadi.
 *
 * Jetson tarafi:  python3 jetson_gas_dock.py /dev/ttyTHS1
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Preferences.h>          // ESP32 dahili flash kayit (kalibrasyon icin)

// ---- UART pinleri ----
#define RXD2 16          // Leonardo'dan giris (Serial2 RX)
#define TXD2 17          // Serial2 TX (kullanilmiyor, bosta)
#define UART_BAUD 115200 // Leonardo hatti (hizlandirildi)

#define JETSON_TX 2      // ESP32 GPIO2 -> Jetson pin10 (Serial1 TX)
#define JETSON_RX 4      // Serial1 RX (kullanilmiyor, bosta ama tanimli olmali)
#define JETSON_BAUD 115200

// ---- BNO055 I2C ----
#define BNO_SDA 21       // ESP32 I2C SDA
#define BNO_SCL 22       // ESP32 I2C SCL
#define BNO_ADDR 0x28    // ADR pini bos -> 0x28  (3.3V'a baglarsan 0x29 yap)
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDR, &Wire);

bool  bnoOk       = false;   // BNO bulundu mu
float bnoHeading  = 0.0;     // 0-360 derece (yaw)
uint8_t bnoCal    = 0;       // 0-3 kalibrasyon (sistem)
uint8_t bnoCalG   = 0;       // jiroskop kalibrasyonu
uint8_t bnoCalA   = 0;       // ivmeolcer kalibrasyonu
uint8_t bnoCalM   = 0;       // manyetometre kalibrasyonu (pusula icin kritik)

// ---- Kalibrasyon kalici kayit (flash) ----
Preferences prefs;
bool     calLoaded  = false;   // flash'tan kalibrasyon yuklendi mi
bool     calSaved   = false;   // bu oturumda kaydettik mi (tekrar kaydetme)
uint32_t lastCalChk = 0;       // periyodik kalibrasyon kontrol zamani

struct SensorData {
  int  joyX, joyY;                   // 0..1023
  char btns[8];                      // "CDEFK" -> "00000"/"10100"
  int  mq2, mq8, mq135, mq7, mics;   // mV
  float tempC, presHpa, humPct;      // BME280
  bool bmeValid = false;
  long latE6, lonE6;                 // enlem/boylam x1e6
  int  sats;                         // uydu sayisi
  long spdE2;                        // hiz m/s x100
  long cogE1;                        // gidis yonu derece x10 (-1=gecersiz)
  long distCm;                       // JSN-SR04T mesafe cm (-1=olcum yok)
  bool gpsValid = false;             // GPS fix var mi
  bool valid = false;
  uint32_t lastMs = 0;
} d;

char    lineBuf[200];
uint8_t lineLen = 0;

uint8_t xorChecksum(const char *s, uint16_t len) {
  uint8_t c = 0;
  for (uint16_t i = 0; i < len; i++) c ^= (uint8_t)s[i];
  return c;
}

// ---- BNO055'ten heading (yaw) + kalibrasyon oku ----
// NDOF modu absolute orientation verir: euler.x = yaw (0-360, N=0).
void readBNO() {
  if (!bnoOk) return;

  // Kalibrasyon durumu
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  bnoCal   = sys;
  bnoCalG  = gyro;
  bnoCalA  = accel;
  bnoCalM  = mag;

  // 1) Once fusion yaw'i dene (sys kalibreyse en dogru sonuc)
  sensors_event_t ev;
  bno.getEvent(&ev);
  float yaw = ev.orientation.x;

  // 2) Fusion 0 veriyorsa (sys=0, heading calismiyor) -> MANYETIK vektorden hesapla
  //    MAG=3 oldugu icin bu guvenilir. Robot yatay dururken gecerli.
  if (yaw == 0.0 || bnoCal == 0) {
    imu::Vector<3> m = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    // yatay duzlemde manyetik kuzey acisi (atan2)
    float h = atan2(m.y(), m.x()) * 180.0 / PI;
    if (h < 0) h += 360.0;
    yaw = h;
  }

  if (yaw < 0)   yaw += 360.0;
  if (yaw >= 360) yaw -= 360.0;
  bnoHeading = yaw;
}

// Gelen frame gecerliyse d'yi doldurur ve true doner
// Format: GAS,joyX,joyY,btns,mq2,mq8,mq135,mq7,mics,tC,pPa,hC,lat,lon,sats,chk (14 veri)
bool parseLine(char *line) {
  if (strncmp(line, "GAS,", 4) != 0) return false;   // prefix: GAS

  char *payload = line + 4;                            // "GAS," sonrasi
  char *lastComma = strrchr(payload, ',');
  if (!lastComma) return false;

  *lastComma = '\0';                                   // payload'i checksum'dan ayir
  int rxChk = atoi(lastComma + 1);
  if (xorChecksum(payload, strlen(payload)) != rxChk) return false;  // bozuk -> at

  // payload: joyX,joyY,btns,mq2,mq8,mq135,mq7,mics,tC,pPa,hC,lat,lon,sats (14 alan)
  char *tok = strtok(payload, ",");
  int n = 0;
  long joyX = 0, joyY = 0;
  char btns[8] = {0};
  long g[5]   = {0};    // mq2,mq8,mq135,mq7,mics
  long bme[3] = {0};    // tC,pPa,hC
  long gpsLat = 0, gpsLon = 0, gpsSats = 0, gpsSpd = 0, gpsCog = -1, usDist = -1;

  while (tok) {
    switch (n) {
      case 0:  joyX = atol(tok); break;
      case 1:  joyY = atol(tok); break;
      case 2:  strncpy(btns, tok, sizeof(btns) - 1); break;  // "10100" string
      case 3:  g[0] = atol(tok); break;   // mq2
      case 4:  g[1] = atol(tok); break;   // mq8
      case 5:  g[2] = atol(tok); break;   // mq135
      case 6:  g[3] = atol(tok); break;   // mq7
      case 7:  g[4] = atol(tok); break;   // mics
      case 8:  bme[0] = atol(tok); break; // tC
      case 9:  bme[1] = atol(tok); break; // pPa
      case 10: bme[2] = atol(tok); break; // hC
      case 11: gpsLat  = atol(tok); break; // lat x1e6
      case 12: gpsLon  = atol(tok); break; // lon x1e6
      case 13: gpsSats = atol(tok); break; // uydu
      case 14: gpsSpd  = atol(tok); break; // hiz m/s x100
      case 15: gpsCog  = atol(tok); break; // gidis yonu derece x10 (-1=gecersiz)
      case 16: usDist  = atol(tok); break; // ultrasonik mesafe cm (-1=yok)
    }
    n++;
    tok = strtok(NULL, ",");
  }
  if (n != 17) return false;   // alan sayisi tutmadi -> at

  d.joyX = joyX; d.joyY = joyY;
  strncpy(d.btns, btns, sizeof(d.btns));
  d.mq2 = g[0]; d.mq8 = g[1]; d.mq135 = g[2]; d.mq7 = g[3]; d.mics = g[4];

  if (bme[0] == -1 && bme[1] == -1 && bme[2] == -1) {
    d.bmeValid = false;
  } else {
    d.bmeValid = true;
    d.tempC   = bme[0] / 100.0;
    d.presHpa = bme[1] / 100.0;
    d.humPct  = bme[2] / 100.0;
  }

  d.latE6 = gpsLat; d.lonE6 = gpsLon; d.sats = gpsSats;
  d.spdE2 = gpsSpd; d.cogE1 = gpsCog;
  d.distCm = usDist;
  d.gpsValid = (gpsLat != 0 || gpsLon != 0);

  d.valid = true;
  d.lastMs = millis();
  return true;
}

// Ayni JSON'u iki porta da basar: Jetson (Serial1) + USB (debug).
void sendJson() {
  String j = "{\"ts\":";        j += millis();
  j += ",\"joyX\":";            j += d.joyX;
  j += ",\"joyY\":";            j += d.joyY;
  j += ",\"btns\":\"";          j += d.btns;   j += "\"";
  j += ",\"mq2\":";             j += d.mq2;
  j += ",\"mq8\":";             j += d.mq8;
  j += ",\"mq135\":";           j += d.mq135;
  j += ",\"mq7\":";             j += d.mq7;
  j += ",\"mics\":";            j += d.mics;
  if (d.bmeValid) {
    j += ",\"temp\":";  j += String(d.tempC, 2);
    j += ",\"pres\":";  j += String(d.presHpa, 2);
    j += ",\"hum\":";   j += String(d.humPct, 2);
  } else {
    j += ",\"temp\":null,\"pres\":null,\"hum\":null";
  }
  // ---- BNO055 pusula ----
  if (bnoOk) {
    j += ",\"heading\":"; j += String(bnoHeading, 1);   // 0-360 derece
    j += ",\"imuCal\":";  j += bnoCal;                  // 0-3
  } else {
    j += ",\"heading\":null,\"imuCal\":0";
  }
  // ---- GPS ----
  if (d.gpsValid) {
    // lat/lon'u x1e6 tamsayidan ondaliga cevir (6 hane hassasiyet)
    j += ",\"lat\":";  j += String(d.latE6 / 1000000.0, 6);
    j += ",\"lon\":";  j += String(d.lonE6 / 1000000.0, 6);
    j += ",\"sats\":"; j += d.sats;
    j += ",\"spd\":";  j += String(d.spdE2 / 100.0, 2);        // m/s
    // cog: -1 ise gecersiz (duruyor), null gonder
    if (d.cogE1 >= 0) { j += ",\"cog\":"; j += String(d.cogE1 / 10.0, 1); }
    else              { j += ",\"cog\":null"; }
  } else {
    j += ",\"lat\":null,\"lon\":null,\"sats\":"; j += d.sats;
    j += ",\"spd\":0,\"cog\":null";
  }
  // ---- JSN-SR04T ultrasonik mesafe (cm) ----
  if (d.distCm >= 0) { j += ",\"dist\":"; j += d.distCm; }
  else               { j += ",\"dist\":null"; }
  j += "}";

  Serial1.println(j);   // -> Jetson (ttyTHS1)  ASIL HAT
  Serial.println(j);    // -> USB (PC debug)    istege bagli
}

// ---- Kalibrasyon FLASH'a kaydet (MAG=3 olunca) ----
void saveCalibration() {
  adafruit_bno055_offsets_t offs;
  bno.getSensorOffsets(offs);                    // BNO'dan mevcut offset'leri al
  prefs.begin("bno", false);                      // namespace "bno", yazma modu
  size_t yazilan = prefs.putBytes("cal", &offs, sizeof(offs));  // kac byte yazildi
  prefs.putBool("has", true);                      // "kayit var" bayragi
  prefs.end();
  calSaved = true;
  // net teshis: gercekten yazildi mi (yazilan == 22 olmali)
  Serial.printf("{\"bno_debug\":\"FLASH'A YAZILDI: %d byte (22 bekleniyor) - power kes-ver ile test et\"}\n",
                (int)yazilan);
}

// ---- Kalibrasyon FLASH'tan yukle (acilista) ----
bool loadCalibration() {
  prefs.begin("bno", true);                        // okuma modu
  bool has = prefs.getBool("has", false);
  if (!has) {
    prefs.end();
    Serial.println("{\"bno_debug\":\"flash'ta 'has' bayragi YOK (hic kaydedilmemis)\"}");
    return false;                                  // kayit yok
  }
  adafruit_bno055_offsets_t offs;
  size_t n = prefs.getBytes("cal", &offs, sizeof(offs));
  prefs.end();
  Serial.printf("{\"bno_debug\":\"flash'tan OKUNDU: %d byte (22 bekleniyor)\"}\n", (int)n);
  if (n != sizeof(offs)) return false;             // bozuk kayit
  bno.setSensorOffsets(offs);                      // BNO'ya yukle
  return true;
}

// Belirli adreste BNO055 baslatmayi dener. Basarili + chip dogru ise true.
bool tryBNO(uint8_t addr) {
  // I2C'de adres yanit veriyor mu (chip fiziksel orada mi)
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0) {
    Serial.printf("{\"bno_debug\":\"0x%02X I2C yanit yok\"}\n", addr);
    return false;
  }
  // begin() kutuphane baslatmasi - BASARISIZSA 5 KEZ TEKRAR DENE
  // (BNO055 ilk acilista hazir olmayabilir; birkac deneme cogu zaman tutar)
  for (int deneme = 1; deneme <= 5; deneme++) {
    if (bno.begin()) {
      return true;                    // basardi
    }
    Serial.printf("{\"bno_debug\":\"0x%02X begin() deneme %d/5 basarisiz, tekrar...\"}\n", addr, deneme);
    delay(300);                       // biraz bekle, tekrar dene
  }
  Serial.printf("{\"bno_debug\":\"0x%02X begin() 5 denemede de basarisiz\"}\n", addr);
  return false;
}

void setup() {
  Serial.begin(115200);                                        // USB (PC debug)
  Serial2.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);            // Leonardo'dan giris (115200)
  Serial1.begin(JETSON_BAUD, SERIAL_8N1, JETSON_RX, JETSON_TX);// Jetson'a cikis (115200)

  // ---- BNO055 baslat (teshisli: iki adresi de dener) ----
  Wire.begin(BNO_SDA, BNO_SCL);
  delay(1000);   // BNO055 boot suresi (~650ms) + guc stabilizasyonu icin BEKLE
                 // (bu bekleme olmadan ESP32 cok erken yokluyor, begin() basarisiz oluyordu)

  // Once 0x28, olmazsa 0x29 dene. Hangisi tutarsa onu kullan.
  bnoOk = tryBNO(0x28);
  if (!bnoOk) {
    Serial.println("{\"bno_debug\":\"0x28 basarisiz, 0x29 deneniyor\"}");
    bno = Adafruit_BNO055(55, 0x29, &Wire);   // adresi degistirip yeniden dene
    bnoOk = tryBNO(0x29);
  }

  if (bnoOk) {
    delay(50);
    Serial.println("{\"bno_debug\":\"BNO055 BULUNDU\"}");

    // ---- ONCE kayitli kalibrasyonu yukle (setExtCrystalUse'dan ONCE!) ----
    // Adafruit sirasi: begin -> setSensorOffsets -> setExtCrystalUse
    delay(100);
    calLoaded = loadCalibration();
    if (calLoaded) {
      Serial.println("{\"bno_debug\":\"*** Kayitli kalibrasyon YUKLENDI - kuzey hazir ***\"}");
    } else {
      Serial.println("{\"bno_debug\":\"Kayit YOK - 8 cizerek gezdir, MAG=3 olunca kaydedilir\"}");
    }

    // SONRA kristal (offset yuklemesini bozmasin diye en son)
    delay(50);
    bno.setExtCrystalUse(true);         // harici kristal -> daha kararli
  } else {
    Serial.println("{\"bno_debug\":\"BNO055 begin() BASARISIZ - kutuphane/adres/kablo\"}");
  }

  delay(300);
  // Acilis mesaji her iki hatta da
  String hello = "{\"status\":\"esp32_ready\",\"bno\":";
  hello += (bnoOk ? "true" : "false");
  hello += ",\"in_baud\":9600,\"out_baud\":115200}";
  Serial1.println(hello);   // Jetson
  Serial.println(hello);    // PC
}

void loop() {
  // BNO'yu her dongude tazele (heading guncel kalsin)
  readBNO();

  // ---- Heading + kalibrasyon teshis: SADECE MAG degisince bas ----
  // (surekli akmasin diye. MAG stabilse sessiz kalir.)
  static int lastMagDbg = -1;
  if (bnoOk && bnoCalM != lastMagDbg) {
    lastMagDbg = bnoCalM;
    Serial.printf("{\"hdbg\":\"MAG=%d/3  heading=%.1f  (sys=%d gyro=%d accel=%d)\"}\n",
                  bnoCalM, bnoHeading, bnoCal, bnoCalG, bnoCalA);
  }

  // ---- Kalibrasyon kaydet: MANYETOMETRE (mag) 3 olunca yeter (pusula icin) ----
  // sys=3 beklemek zor (accel de ister); pusula icin mag=3 kritik olan.
  if (bnoOk && !calSaved && millis() - lastCalChk > 1000) {
    lastCalChk = millis();
    if (bnoCalM == 3 && !calLoaded) {
      // Manyetometre tam kalibre -> kaydet (pusula/kuzey hazir)
      saveCalibration();
    }
  }

  // Leonardo'dan satir topla
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0 && parseLine(lineBuf)) {
        sendJson();          // her gecerli frame'i (saniyede bir) ilet
      }
      lineLen = 0;
    } else if (c != '\r') {  // CR (\r) temizle
      if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
      else lineLen = 0;      // tasma -> sifirla
    }
  }

  // Baglanti koptu mu? PC'ye durum satiri
  static uint32_t lastWarn = 0;
  if (d.valid && millis() - d.lastMs > 3000 && millis() - lastWarn > 3000) {
    Serial.println("{\"error\":\"no_data_3s\"}");
    lastWarn = millis();
  }
}
