# Ek-D · Dijital Eklerin İçerik Listesi

Bu arşiv, Final Değerlendirme Raporu'nun **Ek-D** bölümünde tanımlanan yapıya birebir
uygundur.

| Klasör / dosya | İçerik | Durum |
|---|---|---|
| `geant4/utku.mac` | Zırhlama analizi girdi makrosu (Ek-A ile aynı) | ✅ |
| `geant4/output/*.txt` | Beş zırh yapılandırmasının tam koşu çıktıları | ✅ |
| `geant4/src/DetectorConstruction.cc` | Zırh malzemeleri ve geometri tanımı | ✅ |
| `firmware/leonardo_gas_sender/` | Algılama katmanı gömülü yazılımı | ✅ |
| `firmware/esp32_1_bridge/` | Köprüleme katmanı gömülü yazılımı | ✅ |
| `firmware/esp32_2_actuator/` | Eyleyici katmanı gömülü yazılımı | ✅ |
| `firmware/deneyap_baby/` | Elektrikli yavru robot gömülü yazılımı | ✅ |
| `jetson/requirements.txt` | Python paketlerinin sürüm listesi | ✅ Tablo 8.4’ten üretildi |
| `jetson/jetson_gas_dock.py` | Gösterge paneli ve MJPEG yayın sunucusu | ✅ |
| `jetson/remote_control/` | Uzak istasyon istemcisi | ✅ |
| `jetson/indoor_nav.py` | İç mekân yol planlama modülü | ✅ |
| `jetson/outdoor_nav.py` | Dış mekân rota takibi modülü | ✅ |
| `cad/` | Üç boyutlu baskı parçalarının teknik çizimleri | ✅ 12 STL |

## Ek-D'de listelenmeyen, arşive dâhil edilen dosyalar

| Dosya | Neden |
|---|---|
| `firmware/deneyap_baby/secrets_ornek.h` | Wi-Fi kimlik bilgisi şablonu; gerçek değerler depoda tutulmaz |
| `jetson/outdoor_nav_gui.py` | Dış mekân seyrüsefer izleme arayüzü |
| `jetson/rover_gcs.py` | Uydu haritalı yer kontrol istasyonu |

Bu üç dosya raporun Ek-C bölümünde kaynak kod olarak yer almaktadır.

---

## Geant4 koşu ayarları (çıktı dosyalarından doğrulandı)

| | |
|---|---|
| Sürüm | `geant4-11-03-patch-02 [MT]` |
| Fizik listesi | `QGSP_BERT_HP` |
| Birincil parçacık | 10⁶ / parçacık türü |



---

## Yüklemeden önce yapılacaklar

1. `geant4/output/CAM.txt` (cam yapılandırması koşu çıktısı) dosyasını ekleyin.
2. `cad/fpv_tutucu.stl` ve `cad/kol_taban_braketi.stl` adlandırmalarını doğrulayın.
3. `secrets.h` dosyasının arşivde **bulunmadığını** doğrulayın.

---


