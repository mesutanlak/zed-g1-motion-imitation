# Kayıt tabanı ve G1 mimic yol haritası

## Seçilen başlangıç kaydı

`zed_body38_20260727_134332.jsonl` ilk retargeting prototipi için ana başlangıç
kaydıdır. Kayıt durdur/başlat araları üç ayrı aktif segment olarak ele alınır.

| Ölçüm | Sonuç |
|---|---:|
| Aktif süre / kare | 176,34 s / 5268 |
| Segment / aktif FPS | 3 / 29,86 |
| Ortalama body confidence | %96,9 |
| 2–4 m mesafe aralığında | %97,7 |
| Üst gövde görünürlüğü | %95,1 |
| Tüm vücut çekirdek görünürlüğü | %90,8 |
| Sol / sağ Dex3 kaynak görünürlüğü | %84,7 / %87,5 |
| IMU kapsaması | %100 |
| 500°/s üstü quaternion sıçraması | %0,048 |

SVO2 dosyası da bulunduğu için görüntü daha sonra farklı el modeliyle yeniden
işlenebilir. Kemik uzunluğu değişimleri çoğunlukla %1–4 aralığındadır; bu,
önceki kayıtlardan belirgin biçimde daha kararlıdır.

Hareket kapsaması:

- Sol/sağ dirsek yaklaşık 8–180° / 10–180°: güçlü hareket kapsamı.
- Sol/sağ diz yaklaşık 55–180° / 58–180°: güçlü çömelme kapsamı.
- Sol/sağ foot-support görünürlüğü %70,2 / %80,2: ayak bileği ve temas verisi
  gövde/kol verisinden zayıf, ayrı kayıtla güçlendirilmeli.

`124005` tek-parça doğrulama kaydı, `111037` üst gövde yardımcı kaydı olarak
tutulabilir. `152359` ve `112123` ana eğitim/retargeting setine alınmamalıdır.

## Resmî kaynaklara dayalı tasarım

- ZED BODY_38; 2B/3B keypoint, confidence, root ve yerel eklem rotasyonlarını
  sağlar. `enable_body_fitting=True` korunur:
  <https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api>
- JSONL ile birlikte SVO2 tutulur; böylece stereo görüntü, zamanlama ve sensör
  verileri tekrar işlenebilir:
  <https://www.stereolabs.com/docs/development/zed-sdk/modules/camera/recording>
- G1_23 ve Dex3-1 retargeting yapısı için Unitree'nin resmî teleoperation
  projesindeki Pinocchio IK, dex-retargeting, simülasyon ve kayıt ayrımı örnek
  alınır:
  <https://github.com/unitreerobotics/xr_teleoperate>
- ROS 2 iletişiminde Unitree'nin resmî CycloneDDS tabanlı paketi esas alınır:
  <https://github.com/unitreerobotics/unitree_ros2>

## Uygulama sırası

1. **Kalibrasyon klibi:** 10 s hareketsiz A/T pozu; insan ölçeği, nötr pelvis,
   omuz ve el referansı.
2. **Gövde hareket paketi:** omuz pitch/roll, dirsek bükme, gövde yaw; her biri
   30–60 s ve iki yönde.
3. **Bacak paketi:** kontrollü çömelme, diz kaldırma ve küçük yana adım.
   Başlangıçta tek ayaklı hızlı hareket yok.
4. **El paketi:** açık el, yumruk, tripod tutuş, index ve middle ayrı bükme.
   BODY_38 yalnızca kaba kaynak olur; SVO2 üzerinde 21-landmark el takipçisi
   çalıştırılır.
5. **Çevrimdışı retargeting:** G1 23-DOF için Pinocchio IK; Dex3-1 için resmî
   `dex-retargeting` yaklaşımı. Confidence düşükken hedef üretilmez.
6. **RViz:** yalnızca TF ve `JointState`; kayıt zaman çizelgesini oynatır.
7. **MuJoCo:** önce kinematik playback, sonra hız/ivme/limit ve self-collision,
   en son denge policy'si.
8. **Fiziksel G1:** simülasyon metrikleri geçmeden kapalı kalır. İlk test askıda,
   düşük hızda, dead-man ve fiziksel acil durdurmayla yapılır.

## Bir sonraki kabul ölçütleri

- Her klip en az 30 s ve 30 FPS.
- Body confidence ortalaması ≥ 90.
- Hedef eklem grubunun görünürlüğü ≥ %90.
- Düşen kare = 0 veya %0,1 altında.
- Kemik uzunluğu CV değeri temel uzuvlarda ≤ %5.
- 500°/s üzeri quaternion sıçraması ≤ %0,1.
- JSONL, SVO2 ve IMU kapsaması birlikte bulunmalı.

Bu kayıtlar doğrudan motor öğrenmesi hedefi değildir. Önce deterministik
retargeting ve simülasyon doğrulaması için kullanılır; öğrenme/policy aşaması,
güvenli hedef eklem dizileri üretildikten sonra gelir.
