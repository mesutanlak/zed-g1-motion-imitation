# Tek kamera kaydı 20260921_155548 — inceleme ve düzeltmeler

## İncelenen dosyalar

- `recordings/zed_body38_20260921_155548.jsonl`
- `recordings/zed_body38_20260921_155548.svo2`
- `rerun_recordings/rerun_body38_20260921_155435.rrd`
- `rerun_recordings/rerun_body38_20260921_155435/imitation_comparison.jsonl`
- `rerun_recordings/rerun_body38_20260921_155435/imitation_comparison.csv`

## Kayıt bütünlüğü

- JSONL: 1.410 geçerli BODY_38 karesi, bozuk satır yok.
- Süre: 94,01 saniye; etkin hız 14,99 FPS.
- Tahmini kayıp: 1 kamera karesi.
- SVO2: 1.614 kare, 1280x720, 15 FPS, ZED 2i seri 33773329.
- GMR alımı: son sayaçta 1.397 paket, 0 kayıp, yüzde 0 paket kaybı.
- GMR giriş aralığı: medyan 66,67 ms; jitter 1,46 ms.

SVO2'nin JSONL'den 204 kare fazla olması normaldir: SVO2 kamera açık olduğu
sürece stereo kareleri tutar; JSONL yalnız kabul edilen BODY_38 karelerini
yazar.

## Algı kalitesi

- Bütün 1.410 karede takip durumu `OK`.
- Ortalama gövde güveni yüzde 96,68; minimum yüzde 75,71.
- Ortalama mesafe 2,91 m; karelerin yüzde 97,38'i önerilen 2–4 m aralığında.
- Gövde görünür eklem oranı ortalama yüzde 92,36.
- Üst gövde gözlenebilirliği yüzde 86,67.
- Sol kol yüzde 86,67; sağ kol yüzde 97,80.
- Sol Dex3 kaynağı yüzde 59,72; sağ Dex3 kaynağı yüzde 80,71.
- Quaternion normları kararlı; 53.580 ölçümde ortalama norm hatası
  `7,74e-9`.

Sol kol ve sol el, gövde önünde kesişen/üst üste gelen hareketlerde tek kamera
tarafından daha sık örtülüyor. Kayıtta 350 sol ve 246 sağ kol AKC kurtarma
olayı var. Parmak/Dex3 doğruluğu için SVO2 korunmalı; BODY_38 uç noktaları tek
başına yedi el aktüatörünü benzersiz belirlemez.

## GMR ve güvenlik davranışı

- GMR çözüm süresi: medyan 12,32 ms; p95 24,31 ms; maksimum 30,17 ms.
- Kaynak yaşı: medyan 85,42 ms; p95 96,48 ms.
- Benzersiz canlı telemetri örneklerinde güvenlik dağılımı:
  - GREEN: 540
  - YELLOW: 470
  - ORANGE: 37
- En sık güvenlik işlemleri:
  - robot-gövde bariyer projeksiyonu: yüzde 21,9
  - sağ bilek örtülmesi: yüzde 16,0
  - sol bilek örtülmesi: yüzde 12,6
  - çift kol ön sürekliliği: yüzde 7,9
  - yüksek GMR kalıntısı: yüzde 6,8
  - çarpışma riski: yüzde 6,1

ORANGE aralıkları kısa ve çarpışma riskiyle ilişkili. En uzun kümeler yaklaşık
37,7; 62,4; 64,1 ve 65,1 saniyelerde oluşuyor. Güvenlik sistemi bu aralıklarda
SAFE pozunu kaldırmıyor; güvenli dönüş veya bariyer projeksiyonu uyguluyor.

## G1 SAFE görünümünün kesilme nedeni

Kayıt verisinde SAFE eksikliği yoktur:

- 2.085 imitation telemetri satırının tamamında `raw_positions_m`,
  `safe_positions_m` ve `human_positions_m` bulunuyor.
- RAW varken SAFE olmayan kare sayısı sıfırdır.
- SAFE iskeleti ardışık hiçbir benzersiz GMR karesinde donmamıştır.

Eski Rerun yolu aynı diziyi önce GMR canlı paketinden, sonra bütün GMR
geometrisini tekrar içeren Isaac telemetrisinden çiziyordu. 1.202 benzersiz
dizi için 2.085 satır kaydedildi. Bu çift çizim RRD ve görüntüleyici yükünü
ikiye yaklaştırıyor; çizim sırasının sonundaki G1 SAFE katmanı canlı görünümde
gecikebiliyor.

Düzeltme sonrası:

- GMR paketi RAW/SAFE/HUMAN geometrisini bir kez çizer.
- Isaac paketi yalnız Isaac ve sistem ölçümlerini ekler.
- Imitation JSONL/CSV, ölçülen Isaac paketi başına bir satır yazar.
- GMR ve BODY_38 aynı `frame` ve `source_time` zaman çizgilerine bağlanır.

## Retarget öncesi BODY_38'in görünmeme nedeni

Eski oturum manifestinde `frame_count: 0` idi. ZED başlatıcısı yalnız GMR
portu 15050'ye yayın yapıyor, Rerun'ın ham BODY_38 portu 15052'ye monitor
kopyası göndermiyordu. GMR telemetrisi geldiği için HUMAN PRE-GMR ve G1
katmanları görünürken `/world/skeleton` boş kalıyordu.

`start_zed_native_ubuntu.sh` artık her kareyi şu iki yerel hedefe yollar:

- GMR: `127.0.0.1:15050`
- Rerun ham BODY_38: `127.0.0.1:15052`

## Onarılmış Rerun kaydı

Eski BODY_38 JSONL ve imitation telemetrisi dizi numarasıyla birleştirildi:

- RRD: `rerun_recordings/reprocessed_20260921_155548/`
  `rerun_body38_20260921_160923.rrd`
- BODY_38 kareleri: 1.410
- Eşleşmiş Isaac imitation kareleri: 1.038
- Yeni RRD boyutu: yaklaşık 24,5 MB

Bu RRD, BODY_38, HUMAN PRE-GMR, G1 RAW ve G1 SAFE katmanlarını aynı zaman
çizgisinde içerir.

## Sonraki tek kamera denemesi

RTX 5090 üzerinde bir sonraki denemede ZED ve GMR girişini birlikte 30 FPS'e
çıkarmak denenebilir. Rerun görüntüleme 15 Hz'de kalır ve UDP kuyruğundaki en
yeni kareyi seçer; kontrol akışını yavaşlatmaz. GMR p95 çözüm süresi 24,31 ms
olduğu için 33,3 ms'lik 30 FPS bütçesine sığmaktadır, ancak yeni kaydın kaynak
yaşı ve kare kaybı yeniden ölçülmelidir.
