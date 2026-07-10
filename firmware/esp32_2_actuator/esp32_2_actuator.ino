/*
 * ============ 2. ESP32 - MOTOR + ROBOT KOL KONTROL ============
 * Jetson'dan komut alir, motorlari + robot kolu surer. SENSOR YOK.
 *
 * BAGLANTI:
 *   Jetson ttyTHS1 TX (pin 8, 3.3V) --> 2. ESP32 GPIO16 (RX2)
 *   Jetson GND (pin 6)              --> 2. ESP32 GND      [ORTAK GND SART]
 *
 * SURUS MOTORLARI:
 *   Servo GAZ    -> GPIO18   (6V ayri guc)
 *   Servo FREN   -> GPIO19
 *   Step DIREKS. -> PUL GPIO26, DIR GPIO27   (MD5042 + 57HB84)
 *
 * ROBOT KOL:
 *   Step TABAN   -> PUL GPIO25, DIR GPIO33   (MD5042 + 57HB84, aynisi)
 *   Servo DIRSEK -> GPIO23    (0-180)
 *   Servo PARMAK -> GPIO22     (0-180, kiskac ac/kapa)
 *   TUM STEP PUL+/DIR+ -> 5V | TUM GUC ORTAK GND
 *
 * KOMUT PROTOKOLU (Jetson -> ESP32, \n sonlu):
 *   M,<gaz>,<fren>,<dir>,<kolBaz>,<dirsek>,<parmak>\n
 *     gaz    : 0..90    (gaz servosu)
 *     fren   : 0..90    (fren servosu)
 *     dir    : -STEER_MAX_LEFT..+STEER_MAX_RIGHT (asimetrik; 0=duz)
 *     kolBaz : -600..600 (kol taban step hedef; 0=merkez)
 *     dirsek : 0..180   (dirsek servosu)
 *     parmak : 0..180   (parmak/kiskac servosu)
 *   Ornek: M,40,0,20,100,90,180  -> yarim gaz, saga, kol saga, dirsek orta, parmak acik
 *   Komut gelmezse (CMD_TIMEOUT = 1500ms): GUVENLIK -> gaz kapa, fren serbest.
 *
 * ACILISTA: direksiyon VE kol tabani ELLE ORTA/DUZ yap (merkez=0).
 * Kutuphane: ESP32Servo
 */

#include <ESP32Servo.h>

// ============================================================================
//  DERLEME KORUMASI — PSRAM
//  PSRAM acikken ESP32 cekirdegi GPIO16 ve GPIO17'yi PSRAM icin ayirir.
//  Serial2 bu iki pinde oldugundan Jetson komutlari HIC ULASMAZ.
//  Belirti: "E (91) psram: PSRAM ID read error" + surekli [GUVENLIK] mesaji.
//  Cozum:   Tools > Board: "ESP32 Dev Module"   ve   Tools > PSRAM: Disabled
// ============================================================================
#ifdef BOARD_HAS_PSRAM
#error "PSRAM ACIK! GPIO16/17 PSRAM'e ayrilir, Serial2 calismaz. Tools > PSRAM: Disabled yap."
#endif


// ---- Jetson komut hatti ----
#define JETSON_RX 16
#define JETSON_TX 17
#define CMD_BAUD  115200

// ---- Surus motor pinleri ----
#define SERVO_GAS_PIN   18   // (yer degistirildi: eskiden 19, ters calisiyordu)
#define SERVO_BRAKE_PIN 19   // (yer degistirildi: eskiden 18)
#define STEER_PUL_PIN   26
#define STEER_DIR_PIN   27

// ---- Robot kol pinleri ----
#define ARM_PUL_PIN     25   // kol taban step
#define ARM_DIR_PIN     33
#define SERVO_ELBOW_PIN 23   // dirsek
#define SERVO_GRIP_PIN  22   // parmak/kiskac (5ten 22ye tasindi)

Servo servoGas, servoBrake, servoElbow, servoGrip;

// ---- Aci limitleri ----
const int GAS_MIN = 0, GAS_MAX = 90;
const int BRK_MIN = 0, BRK_MAX = 90;
const int ELB_MIN = 0, ELB_MAX = 180;
const int GRP_MIN = 0, GRP_MAX = 180;

// ---- Direksiyon step ----
// ---- DIREKSIYON ACI SINIRLARI (asimetrik) ----
// Aci -> adim donusumu: adim = derece * STEPS_PER_REV * GEAR / 360
//   STEPS_PER_REV : surucunun mikroadim ayari (tam tur basina adim)
//   STEER_GEAR    : motor mili -> direksiyon mili orani (dogrudan tahrik = 1.0)
// Degistirmek icin SADECE asagidaki dort satiri duzenle.
// !!! STEER_STEPS_PER_REV, MD5042'nin DIP anahtarindaki mikroadim ayariyla
//     BIREBIR AYNI OLMALI. Uyusmazsa direksiyon eksik/fazla doner.
//     (Onceki deger 400 idi; olculen donusun 1/3 kalmasi uzerine 1200 yapildi.)
const float STEER_STEPS_PER_REV = 1200.0;  // MD5042 mikroadim ayari (DIP'ten oku!)
const float STEER_GEAR          = 1.0;     // dogrudan tahrik (disli/kayis varsa oran gir)
const float STEER_DEG_RIGHT     = 15.0;    // saga azami aci
const float STEER_DEG_LEFT      = 30.0;    // sola azami aci

const long STEER_MAX_RIGHT = (long)(STEER_DEG_RIGHT * STEER_STEPS_PER_REV * STEER_GEAR / 360.0 + 0.5);  // 50 adim
const long STEER_MAX_LEFT  = (long)(STEER_DEG_LEFT  * STEER_STEPS_PER_REV * STEER_GEAR / 360.0 + 0.5);  // 100 adim
const int  STEER_MIN_US = 600;   // darbe 200us + bekleme (adim hizi)
long steerCurrent = 0, steerTarget = 0;
uint32_t lastSteerUs = 0;

// ---- Kol taban step ----
const long ARM_MAX    = 600;   // kol taban +-600 adim (~270 derece, 6 kat)
const int  ARM_MIN_US = 1500;  // kol yavas donsun (adimlar arasi daha genis)
long armCurrent = 0, armTarget = 0;
uint32_t lastArmUs = 0;

// ---- Komut alimi ----
char cmdBuf[80];
uint8_t cmdLen = 0;
uint32_t lastCmdMs = 0;
const uint32_t CMD_TIMEOUT = 1500;  // joystick ~1Hz geldigi icin genis tut (gaz kesilmesin)
bool timeoutWarned = false;   // guvenlik uyarisi bir kez bassin

// ---- Hat kalitesi sayaclari (gurultu teshisi) ----
uint32_t okCount = 0;        // gecerli komut
uint32_t badCount = 0;       // bozuk / sagalama tutmayan satir
uint32_t lastErrMs = 0;      // hata mesajini saniyede 1 kez bas
uint32_t lastStatMs = 0;     // 5 saniyede bir ozet
const bool VERBOSE_HAM = false;  // true yaparsan her satiri ham basar

// Genel bloklamayan step surucu (hangi motor olursa)
void stepTickGeneric(long &cur, long tgt, uint32_t &lastUs, int minUs,
                     int pulPin, int dirPin) {
  if (cur == tgt) return;
  uint32_t now = micros();
  if (now - lastUs < (uint32_t)minUs) return;
  lastUs = now;
  bool dir = (tgt > cur);
  digitalWrite(dirPin, dir ? HIGH : LOW);
  delayMicroseconds(10);          // yon oturması icin
  digitalWrite(pulPin, HIGH);
  delayMicroseconds(100);         // HIGH darbe genisligi (MD5042 icin yeterli)
  digitalWrite(pulPin, LOW);
  delayMicroseconds(100);         // LOW arasi (darbe net ayrilsin)
  cur += dir ? 1 : -1;
}

// Guvenli mod: gaz kapa, fren serbest. Kol/direksiyon oldugu yerde kalir.
void safeStop() {
  servoGas.write(GAS_MIN);
  servoBrake.write(BRK_MIN);
  // step hedefleri degismez -> son konumda kalir (kol dusmez)
}

// Sagalama: "M,...*7F" -> '*' oncesi tum karakterlerin XOR'u, hex olarak.
// '*' yoksa (eski panel) sagalama atlanir, komut kabul edilir.
// -1 = sagalama yok | 0 = TUTMADI | 1 = TUTTU
int checksumOk(const char *line) {
  const char *star = strchr(line, '*');
  if (star == NULL) return -1;
  uint8_t x = 0;
  for (const char *q = line; q < star; q++) x ^= (uint8_t)(*q);
  uint8_t got = (uint8_t)strtol(star + 1, NULL, 16);
  return (x == got) ? 1 : 0;
}

// Komut isle: M,gaz,fren,dir,kolBaz,dirsek,parmak[*XOR] (6 sayi)
bool parseCmd(char *line) {
  // Bastaki cop/gurultu byte'larini atla, "M," ile baslayan yeri bul
  char *m = strstr(line, "M,");
  if (m == NULL) return false;
  line = m;   // M,'den itibaren isle (bastaki cop atlanir)
  if (line[0] != 'M' || line[1] != ',') return false;

  // Sagalama varsa DOGRULA. Tutmuyorsa satiri sessizce at (gurultu).
  int cs = checksumOk(line);
  if (cs == 0) { badCount++; return false; }

  // '*' varsa oradan kes -> strtok sayilari temiz alsin
  char *star = strchr(line, '*');
  if (star) *star = '\0';

  char *p = line + 2;
  char *tok = strtok(p, ",");
  int n = 0;
  int gaz=0, fren=0, dirsek=0, parmak=0;
  long dir=0, kolBaz=0;
  while (tok) {
    long v = atol(tok);
    switch (n) {
      case 0: gaz    = (int)v; break;
      case 1: fren   = (int)v; break;
      case 2: dir    = v;      break;
      case 3: kolBaz = v;      break;
      case 4: dirsek = (int)v; break;
      case 5: parmak = (int)v; break;
    }
    n++;
    tok = strtok(NULL, ",");
  }
  if (n != 6) return false;

  // limitlere kis (guvenlik)
  gaz    = constrain(gaz,    GAS_MIN, GAS_MAX);
  fren   = constrain(fren,   BRK_MIN, BRK_MAX);
  dir    = constrain(dir,   -STEER_MAX_LEFT, STEER_MAX_RIGHT);   // asimetrik: sol 30 derece, sag 15 derece
  kolBaz = constrain(kolBaz, -ARM_MAX,  ARM_MAX);
  dirsek = constrain(dirsek, ELB_MIN, ELB_MAX);
  parmak = constrain(parmak, GRP_MIN, GRP_MAX);

  // uygula
  servoGas.write(gaz);
  servoBrake.write(fren);
  steerTarget = dir;
  armTarget   = kolBaz;
  servoElbow.write(dirsek);
  servoGrip.write(parmak);

  lastCmdMs = millis();
  timeoutWarned = false;   // komut geldi -> guvenlik uyarisini sifirla
  okCount++;

  // --- DEBUG: gelen komutu ve uygulanan degerleri Serial'e (USB) bas ---
  Serial.print("[KOMUT] gaz="); Serial.print(gaz);
  Serial.print(" fren="); Serial.print(fren);
  Serial.print(" dir="); Serial.print(dir);
  Serial.print(" kolBaz="); Serial.print(kolBaz);
  Serial.print(" dirsek="); Serial.print(dirsek);
  Serial.print(" parmak="); Serial.println(parmak);

  return true;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(CMD_BAUD, SERIAL_8N1, JETSON_RX, JETSON_TX);

  // Step motorlar (direksiyon + kol taban)
  pinMode(STEER_PUL_PIN, OUTPUT); pinMode(STEER_DIR_PIN, OUTPUT);
  pinMode(ARM_PUL_PIN, OUTPUT);   pinMode(ARM_DIR_PIN, OUTPUT);
  digitalWrite(STEER_PUL_PIN, LOW); digitalWrite(STEER_DIR_PIN, LOW);
  digitalWrite(ARM_PUL_PIN, LOW);   digitalWrite(ARM_DIR_PIN, LOW);
  steerCurrent = 0; steerTarget = 0;    // ACILISTA DIREKSIYON ELLE DUZ
  armCurrent = 0;   armTarget = 0;      // ACILISTA KOL TABANI ELLE ORTA

  // Servolar (4 adet -> 4 timer)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoGas.setPeriodHertz(50);   servoBrake.setPeriodHertz(50);
  servoElbow.setPeriodHertz(50); servoGrip.setPeriodHertz(50);
  servoGas.attach(SERVO_GAS_PIN, 500, 2400);
  servoBrake.attach(SERVO_BRAKE_PIN, 500, 2400);
  servoElbow.attach(SERVO_ELBOW_PIN, 500, 2400);
  servoGrip.attach(SERVO_GRIP_PIN, 500, 2400);

  // baslangic: gaz kapali, fren serbest, dirsek orta, parmak kapali
  servoGas.write(GAS_MIN);
  servoBrake.write(BRK_MIN);
  servoElbow.write(90);
  servoGrip.write(GRP_MIN);

  // Baud ve pinleri bas -> uc cihazin ayarini karsilastirmak kolay olsun
  Serial.print("[UART] Serial2 RX=GPIO"); Serial.print(JETSON_RX);
  Serial.print("  baud="); Serial.println(CMD_BAUD);
  Serial.println("[UART] Panel (BAUD) ve 1.ESP32 (JETSON_BAUD) AYNI olmali!");

  // Hesaplanan limitleri bas (yanlis mikroadim/oran ayarini erken yakalamak icin)
  Serial.print("[LIMIT] direksiyon: sag +"); Serial.print(STEER_MAX_RIGHT);
  Serial.print(" adim ("); Serial.print(STEER_DEG_RIGHT, 1); Serial.print(" derece)  |  sol -");
  Serial.print(STEER_MAX_LEFT); Serial.print(" adim ("); Serial.print(STEER_DEG_LEFT, 1);
  Serial.println(" derece)");
  Serial.print("[LIMIT] kol taban: +-"); Serial.print(ARM_MAX); Serial.println(" adim");
  Serial.println("[UYARI] Jetson paneli dir icin +-300 gonderiyor; burada kisilir.");

  Serial.println("{\"motor\":\"2.ESP32 hazir. Direksiyon+kol ELLE ORTA yap. Jetson komutu bekleniyor.\"}");

  lastCmdMs = millis();
}

void loop() {

  // Jetson'dan komut topla
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0) {
        if (VERBOSE_HAM) { Serial.print("[HAM] "); Serial.println(cmdBuf); }
        if (!parseCmd(cmdBuf)) {
          badCount++;
          // Gurultulu hatta ekrani bogmamak icin saniyede en fazla 1 uyari
          if (millis() - lastErrMs > 1000) {
            lastErrMs = millis();
            Serial.println("[BOZUK] satir elendi (gurultu). Ozet icin 5 sn bekle.");
          }
        }
      }
      cmdLen = 0;
    } else if (c != '\r') {
      if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
      else cmdLen = 0;
    }
  }

  // Guvenlik: komut kesilirse dur
  if (millis() - lastCmdMs > CMD_TIMEOUT) {
    if (!timeoutWarned) {
      Serial.print("[GUVENLIK] "); Serial.print(CMD_TIMEOUT);
      Serial.println(" ms komut yok -> gaz kapandi, kol/direksiyon sabit.");
      timeoutWarned = true;
    }
    safeStop();
  }

  // ---- 5 saniyede bir HAT KALITESI ozeti ----
  if (millis() - lastStatMs > 5000) {
    lastStatMs = millis();
    uint32_t toplam = okCount + badCount;
    if (toplam > 0) {
      float oran = 100.0 * okCount / toplam;
      Serial.print("[HAT] gecerli="); Serial.print(okCount);
      Serial.print("  bozuk=");       Serial.print(badCount);
      Serial.print("  basari=%");     Serial.print(oran, 1);
      if (oran > 98)      Serial.println("  -> hat TEMIZ");
      else if (oran > 80) Serial.println("  -> hat gurultulu (kablo/GND kontrol)");
      else                Serial.println("  -> hat COK KOTU: ortak GND yok veya baud yanlis");
    } else {
      Serial.println("[HAT] hic satir gelmedi.");
    }
    okCount = 0; badCount = 0;
  }

  // Iki step motoru da bloklamadan surekli adimla
  stepTickGeneric(steerCurrent, steerTarget, lastSteerUs, STEER_MIN_US,
                  STEER_PUL_PIN, STEER_DIR_PIN);   // direksiyon
  stepTickGeneric(armCurrent, armTarget, lastArmUs, ARM_MIN_US,
                  ARM_PUL_PIN, ARM_DIR_PIN);       // kol taban
}
