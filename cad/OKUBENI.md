# cad/ — Üç Boyutlu Baskı Parçaları

Tüm dosyalar ikili (binary) STL biçimindedir ve bütünlük denetiminden geçmiştir.

## Mevcut parçalar

| Dosya | Üçgen | Parça |
|---|---|---|
| `gaz1.STL` – `gaz4.STL` | 716 / 770 / 1376 / 656 | Gaz kelebeği servosu braketi |
| `fren1.STL`, `fren2.STL` | 766 / 720 | Fren servosu braketi |
| `direksiyon1.STL`, `direksiyon2.STL` | 2112 / 826 | Direksiyon adım motoru braketi |
| `ustkapak.STL` | 1316 | Algılama muhafazası — üst kapak (230 × 200 × 52 mm) |
| `altkapak.STL` | 628 | Algılama muhafazası — alt kapak (230 × 200 × 10 mm) |
| `fpv_tutucu.stl` | 1718 | FPV kamera ve ölçüm cihazı tutucusu (122 × 100 × 190 mm) |
| `kol_taban_braketi.stl` | 26864 | Kol taban adım motoru braketi (75 × 65 × 94 mm) |

## ⚠ Dosya adı teyidi gerekiyor

Son iki dosya, kaynak adlarından değil **ölçülerinden** yola çıkılarak adlandırılmıştır:

| Özgün ad | Verilen ad | Ölçü | Dayanak |
|---|---|---|---|
| `Magnificent_Amberis__7_.stl` | `fpv_tutucu.stl` | 122 × 100 × 190 mm | Uzun kollu dikey yapı; Şekil 4.9(a) |
| `Assembly.stl` | `kol_taban_braketi.stl` | 75 × 65 × 94 mm | Kompakt montaj parçası; Şekil 4.11 |

Bu eşleştirme **takım tarafından doğrulanmalıdır.** Yanlışsa dosya adları düzeltilmelidir.

## Öneri

Kaynak dosyalar (`.step` veya `.f3d`) da eklenirse parçalar yeniden düzenlenebilir olur.
STL yalnızca baskıya hazır üçgen ağdır; parametrik düzenlemeye uygun değildir.
