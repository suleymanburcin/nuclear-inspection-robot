# Nükleer Denetim Robotu

**TEKNOFEST 2026 · Nükleer Enerji Teknolojileri Tasarım Yarışması**
Uygulama Kategorisi — Robotik Sistem Tasarımı

Küçük ve mikro modüler reaktörlerde (SMR/MMR) insan girişi olmadan denetim yapmak üzere
geliştirilmiş, **keseli (marsupial)** mimariye sahip iki platformlu robotik sistem.

---

## Fikir

Kaza sonrası bir nükleer tesiste iki farklı problem vardır ve bunların çözümleri
birbiriyle çelişir:

- **Dış sahada** menzil ve yük taşıma kapasitesi gerekir → yüksek özgül enerjili yakıt
- **Reaktör binası içinde** kıvılcım ve egzoz üretilmemelidir → hidrojen birikimi riski

Literatürdeki robotlar tek platformlu ve tamamen elektriklidir; ikinci kısıtı sağlarlar
ama birincisinden ödün verirler.

Bu çalışma görevi **iki uzmanlaşmış platforma** ayırır:

| Platform | Tahrik | Görev |
|---|---|---|
| **Benzinli taşıyıcı** | 7 BG içten yanmalı, Ackermann direksiyon | Dış saha, menzil, yük |
| **Elektrikli yavru robot** | Deneyap Kart 1A, kayma-yönlendirmeli | Kapalı hacim, emisyonsuz |

Literatürdeki hibritler ağırlıklı olarak *hareket-modu* hibritleridir (tekerlek–palet,
tekerlek–bacak). Buradaki ise bir **enerji-modu hibrididir.**

Kapalı hacme yalnızca emisyonsuz robot girer. Kaza sonrası ortamda hidrojen birikimi
ihtimali göz önüne alındığında bu bir tercih değil, **güvenlik gereğidir.**

Taşıyıcı platform, atıl bir go-kart şasisinden yeniden kazanılmıştır.

---

## Sistem mimarisi

Dört katman, dört ayrı işlemci. Katmanlar fiziksel olarak ayrılmıştır; bu sayede hata
kaynağı ayrıştırılabilir ve alt sistemler paralel geliştirilebilir.

```
  ALGILAMA          KÖPRÜLEME          KARAR              EYLEYİCİ
  Arduino Leonardo  1. ESP32           Jetson Orin Nano   2. ESP32
  10 sensör         BNO055 yönelim     PyQt5 panel        6 eyleyici
       │                 │                  │                  ▲
       └──D0/D1─────────►└──GPIO2──────────►└──pin8────────────┘
          115.200            JSON              M,…*XOR
```

Buna ek olarak, karar katmanından **tamamen yalıtılmış** bir FPV hattı bulunur:
termal kamera ve radyasyon dedektörünün ekranları ortak bir kadrajda, analog video
üzerinden operatöre aktarılır. Jetson çökse dahi radyolojik ve termal durum
farkındalığı sürer.

---

## Doğrulanmış işlevler

- Manuel kontrollü sürüş: gaz, fren, direksiyon
- Çok modaliteli algılama: beş gaz kanalı (H₂, CO, LPG/duman, NH₃/CO₂, UOB),
  sıcaklık/nem/basınç, ultrasonik seviye, GNSS, dokuz eksen mutlak yönelim
- Her iki platformda üç eksenli manipülatör
- Elektrikli yavru robotun kablosuz denetimi ve emniyet zaman aşımı
- Çift kameralı görüş, RealSense derinlik akışı
- Karar katmanından bağımsız FPV hattı (termal + radyasyon)
- Yalıtılmış ağda salt-okunur panel yayını (MJPEG, çoklu istemci)
- Uzaktan kontrol: fiziksel joystick her zaman öncelikli

## Doğrulanmamış / kapsam dışı

Bunlar raporda da açıkça belirtilmiştir; hiçbiri görev senaryolarının başarı
ölçütüne dâhil değildir.

- **Otonom seyrüsefer** — planlama katmanı yazıldı, eyleyici katmanına bağlanmadı,
  saha denemesi yapılmadı. İç mekân modülünde **lokalizasyon yoktur**; robot kendi
  konumunu ölçmez, konum varsayılır.
- **Yüksek tork gerektiren manipülasyon** — mevcut aktüatör kapasitesi dışında.
- **Zırhlamanın fiziksel doğrulaması** — yalnızca Geant4 simülasyonu yapıldı,
  ışınlama testi yapılmadı.

---

## Radyasyon zırhlaması

COTS bileşenlerin literatürde bildirilen **~120 Gy** arıza eşiği veri olarak alınmış,
bu eşiğin altında kalınmasını sağlayacak zırhlama Geant4 ile boyutlandırılmıştır.
Böylece dayanım problemi, pahalı bileşen tedarikinden **ölçülebilir bir mühendislik
hesabına** dönüşmüştür.

| Malzeme | Gama 1,6 MeV | Nötron 2 MeV |
|---|---|---|
| WNiFe95 | %7,93 | %36,80 |
| **WNiFe97** | **%7,40** | **%36,22** |
| WNiCu97 | %7,53 | %36,37 |
| Cam (Pb + boro) | %57,18 | %70,88 |

Seçilen yapılandırma: **1 cm parafin + 4 cm WNiFe97**. Parafin hızlı nötronları elastik
saçılmayla yavaşlatır; tungsten alaşımı gama fotonlarını soğurur ve parafindeki nötron
yakalanmasından doğan ikincil gamaları da zayıflatır.

---

## Depo yapısı

```
geant4/           Zırhlama analizi: girdi makrosu, koşu çıktıları, kaynak
firmware/         Gömülü yazılım — dört Arduino eskizi
  leonardo_gas_sender/    Algılama katmanı
  esp32_1_bridge/         Köprüleme katmanı
  esp32_2_actuator/       Eyleyici katmanı
  deneyap_baby/           Elektrikli yavru robot
jetson/           Karar katmanı ve seyrüsefer modülleri
  remote_control/         Uzak istasyon istemcisi
cad/              Üç boyutlu baskı parçalarının teknik çizimleri
```

---

## Kurulum

### Gömülü kartlar (Arduino IDE)

> **ÖNEMLİ.** ESP32 kart tanımı yanlış seçilirse `Serial2` sessizce ölür.

```
Board            : ESP32 Dev Module      ← "Wrover Kit" DEĞİL
PSRAM            : Disabled
Flash Mode       : DIO
Flash Frequency  : 40MHz
Upload Speed     : 115200
Core Debug Level : None
```

PSRAM açıkken çekirdek **GPIO16 ve GPIO17'yi PSRAM için ayırır.** `Serial2` bu iki
pinde olduğundan komutlar hiç ulaşmaz. Firmware bunu derleme zamanında engeller:

```c
#ifdef BOARD_HAS_PSRAM
#error "PSRAM ACIK! GPIO16/17 PSRAM'e ayrilir, Serial2 calismaz."
#endif
```

Gerekli kütüphaneler: `ESP32Servo` · `Adafruit BNO055` · `Adafruit BME280` · `TinyGPS++`

### Wi-Fi kimlik bilgileri

`firmware/deneyap_baby/secrets_ornek.h` dosyasını `secrets.h` olarak kopyalayıp doldurun.
`secrets.h` `.gitignore` içindedir; **depoya yüklenmez.**

### Karar katmanı (Jetson Orin Nano)

```bash
pip3 install -r jetson/requirements.txt
python3 jetson/jetson_gas_dock.py
```

---

## Haberleşme protokolleri

| Hat | Biçim | Hız |
|---|---|---|
| Leonardo → 1. ESP32 | 17 alanlı çerçeve + XOR sağlama | 115.200 |
| 1. ESP32 → Jetson | JSON | 115.200 |
| Jetson → 2. ESP32 | `M,gaz,fren,dir,kolBaz,dirsek,parmak*XOR` | 115.200 |
| Jetson → Deneyap | `F,s1,s2,s3` (UDP yayını) | :4210 |
| Uzak istasyon → Jetson | `joyX,joyY` (UDP) | :9000 |

Komut hatlarında **XOR sağlama toplamı** kullanılır. Gürültülü bir hatta bozulan çerçeve,
eyleyiciye ulaşmadan elenir. Bu, gaz kelebeğinin yanlış bir baytla açılmasını engeller.

---

## Güvenlik

- Panel yayını (`:8080`) ve uzaktan kontrol (`:9000`) **yalıtılmış yerel ağda** kalmalıdır.
  Kimlik doğrulama ve TLS **yoktur**; internete açılmamalıdır. Yalıtım ağ düzeyinde
  sağlanır, uygulama düzeyinde değil.
- Yayın arayüzü **salt-okunurdur.** Yalnızca `GET` yöntemini ve üç tanımlı yolu işler;
  hiçbir eyleyici uç noktası yayımlamaz. Arayüz ele geçirilse dahi robota komut verilemez.
- Uzaktan kontrolde **fiziksel joystick her zaman önceliklidir.** Araç üzerindeki kol
  merkezden ayrıldığı anda uzak komut yok sayılır.
- Üç bağımsız emniyet katmanı: fiziksel öncelik → 800 ms uzak zaman aşımı →
  1500 ms eyleyici zaman aşımı (gaz kapanır).

---

## Geliştirme sırasında belgelenen hata sınıfları

Literatürde nadiren paylaşılan, gerçek bir gömülü sistemde karşılaşılan altı hata sınıfı
raporun 4.1.2.8 bölümünde kök nedenleriyle birlikte açıklanmıştır. Aralarında:

- **Kart tanımının PSRAM'li seçilmesi** → UART2 pinlerinin işgali → sessiz haberleşme kaybı
- **Kanal kapasitesi** — 9600 baud, 25 Hz'te üretilen çerçeveyi taşıyamaz (212 ms > 40 ms)
- **Sürücü asgari darbe genişliği** — adım motoru sürücüsünün gereksinimi
- **Sensör önyükleme penceresi** — BNO055'in I²C'ye hazır olma süresi

Bu bulguların paylaşılması, benzer sistemleri geliştirecek ekiplerin aynı hatalara
harcayacağı süreyi kısaltır.

---

## Maliyet

Toplam donanım maliyeti yaklaşık **129.045 ₺**. Ticari nükleer denetim robotlarının
maliyetinin çok altındadır; sistem tamamen ticari raf ürünü bileşenler ve açık kaynaklı
bir araç zinciri üzerine kuruludur. **Tescilli yazılım bağımlılığı yoktur.**

Sistemi robot hâline getiren unsur donanım değil, yaklaşık **3.161 satırlık** özgün gömülü
ve arayüz yazılımıdır.

---

## Lisans ve atıf

TEKNOFEST 2026 Nükleer Enerji Teknolojileri Tasarım Yarışması kapsamında geliştirilmiştir.

Bu depoyu kullanır veya alıntılarsanız, projeye atıf yapmanız beklenir.
