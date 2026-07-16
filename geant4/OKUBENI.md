# geant4/ — Radyasyon Zırhlama Analizi

## Koşu ayarları (çıktı dosyalarından doğrulanmıştır)

| | |
|---|---|
| Geant4 sürümü | `geant4-11-03-patch-02 [MT]` (çok iş parçacıklı) |
| Fizik listesi | `QGSP_BERT_HP` |
| Birincil parçacık | 10⁶ / parçacık türü |
| Panel kesiti | 10 cm × 10 cm |

Dört çıktı dosyasının tamamı **aynı sürüm ve aynı fizik listesiyle** koşulmuştur;
karşılaştırma bu nedenle geçerlidir.

`QGSP_BERT_HP` seçimi, `_HP` uzantısı sayesinde 20 MeV altındaki nötronlar için
yüksek hassasiyetli veri kütüphanelerini kullanır. Nötron zırhlaması için gereklidir.

## Mevcut dosyalar

| Dosya | İçerik |
|---|---|
| `src/DetectorConstruction.cc` | Zırh malzemeleri ve geometri tanımı (8 malzeme) |
| `output/WNIFE97.txt` | Tungsten–nikel–demir %97 |
| `output/WNIFE95.txt` | Tungsten–nikel–demir %95 |
| `output/WNICU97.txt` | Tungsten–nikel–bakır %97 |
| `output/WNICU95.txt` | Tungsten–nikel–bakır %95 |

## `utku.mac` hakkında

Makro; 1 cm parafin + 4 cm zırh geometrisini kurar, panel kesitini 10 cm × 10 cm yapar
ve on beş koşu yürütür (her biri 10⁶ birincil parçacık):

| Parçacık | Enerji |
|---|---|
| Gama × 6 | 59,5 keV · 364 keV · 605 keV · 662 keV · 796 keV · 1596 keV |
| Beta (e⁻) × 4 | 606 · 512 · 546 keV · 2,28 MeV |
| Alfa × 2 | 5,16 · 5,49 MeV |
| Nötron × 3 | 0,025 eV (termal) · 100 keV (epitermal) · 2 MeV (hızlı) |

