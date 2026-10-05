# Tek kamera Dex3 el IK incelemesi — 2026-09-21 16:53

## Kayıt

- `recordings/zed_body38_20260921_165318.jsonl`
- `recordings/zed_body38_20260921_165318.svo2`
- `rerun_recordings/rerun_body38_20260921_165125.rrd`
- `rerun_recordings/rerun_body38_20260921_165125/`

## Ölçülen giriş kalitesi

- BODY_38: 2606 kare, 87.24 saniye, 29.86 FPS.
- Üst gövde ve çekirdek gözlenebilirliği: `%94.1`.
- El çıkarım paketi: 747, ölçülen `8.60 FPS`.
- Sol el tespiti: `177/747 = %23.69`.
- Sağ el tespiti: `245/747 = %32.80`.
- İki el aynı anda: `%9.24`; en az bir el: `%47.26`.
- MediaPipe çıkarım p50/p95: `19.08/24.29 ms`.
- Ret nedenleri: `MEDIAPIPE_NO_HAND=840`, `DETECTOR_TRACK_LOST=231`,
  `WRIST_OR_ROI_OUTSIDE_IMAGE=1`.

Başarılı tespitlerde el hiçbir zaman ROI kenarına değmiyordu. El kutusu 161
piksel ROI'nin medyan olarak sol elde `%20–23`, sağ elde `%19–27` kadarını
kaplıyordu. Bu nedenle daha büyük kırpma yerine daha sıkı odak seçildi.

## Yumruğun simülasyonda görünmemesinin kök nedeni

Ham kayıtta 747 adet geçerli `unitree_g1_dex3_control/v1` paketi bulunuyordu ve
GMR bunları doğrulayıp Isaac'e geçiriyordu. Veri aktarımı tamamen kopuk değildi.

Eski fallback çözücü parmak bükülmesini MCP ile parmak ucu arasındaki düz
mesafenin kemik zinciri uzunluğuna oranından çıkarıyordu. Dairesel bir yumrukta
bu oran yeterince değişmedi. Kaydın eski hedeflerinde parmak eklemleri medyan
olarak yalnız `0.004–0.016 rad` oynuyordu; Isaac'te görsel karşılığı yoktu.

İkinci sorun `dex3_control` alanının yalnız el çıkarım karelerinde bulunmasıydı.
983 Rerun BODY paketinin 413'ü kontrol taşıyor, 570'i açıkça `null` taşıyordu.
GMR örnekleme ve Isaac watchdog'u bu aralarda hedefi sık sık nötre götürüyordu.

Üçüncü sorun güvenlik filtresinin ölçüm olmayan her BODY karesinde son hedefe
ilerlemeyi durdurup parmak hızını sıfıra doğru frenlemesiydi. 10 Hz el hedefi
ile 30 Hz gövde döngüsü birleşince parmaklar kapanma hedefine ulaşamıyordu.

## Yeni el hattı

1. BODY_38 bileği ve dirseği operatöre kilitli 128 piksel ROI oluşturur.
2. MediaPipe bu ROI'de çalışır. Bulamazsa merkezde yaklaşık 100 piksellik ikinci
   odak kırpmasında yeniden denenir.
3. MediaPipe'ın 21 adet beş parmak noktası avuç merkezli sağ elli koordinata
   normalize edilir.
4. Avuç konumu, SO(3) dönüşü ve ölçeği zamansal `PalmSE3Filter` ile süzülür.
5. MCP, PIP ve DIP eklem bükülme açıları ayrı ayrı ölçülür.
6. İşaret parmağı Dex3 işaret zincirine; orta, yüzük ve serçenin medyan bükülmesi
   Dex3 orta zincirine; başparmak bükülme ve opposition değeri üç başparmak
   eksenine aktarılır.
7. Her 30 Hz BODY karesi geçerli bir Dex3 paketi taşır. Yeni el ölçümü yoksa
   filtre son hedefe hız/ivme sınırları içinde ilerlemeye devam eder.
8. Yumruk algılandıktan sonraki kısa detector kaybında hedef 1.5 saniye tutulur;
   sonra güvenli biçimde nötre döner.
9. Rerun `G1 SAFE` görünümünde üç parmaklı Dex3 hedef geometrisini çizer. Ham
   MediaPipe görünümünde beş insan parmağı korunur.

Bu yol fiziksel robota DDS komutu göndermez.

## Kayıt üzerinde yeni çözücü sonucu

Yeni çözücü eski kayıt üzerinde çevrim dışı çalıştırıldı:

- Dex3 kontrol kapsamı: `2607/2607 = %100` BODY karesi.
- Sol elde `15`, sağ elde `16` ölçüm `FIST` olarak sınıflandı.
- Sol güvenli hedef normu p95: `2.428 rad`.
- Sağ güvenli hedef normu p95: `2.430 rad`.
- Sol karelerin `%65.7`, sağ karelerin `%69.1` bölümünde güvenli hedef normu
  `0.5 rad` üzerinde kaldı.
- Parmak eklemleri her iki tarafta yaklaşık `1.57–1.75 rad` kapanma hedeflerine
  ulaşabildi.

Makine okunur sonuç:
`recordings/analysis/zed_body38_20260921_165318_dex3_retarget_v2.json`.

## Çalıştırma

Isaac mutlaka Dex3 varlığıyla açılmalıdır:

```bash
./start_g1_isaaclab_live_ubuntu.sh \
  --mode upper_body \
  --imitation-mode kinematic_debug \
  --runtime-profile shared_gpu_safe \
  --asset-profile g1_29dof_dex3 \
  --accept-nvidia-eula
```

ZED:

```bash
./start_zed_native_ubuntu.sh \
  --model medium \
  --fps 30 \
  --record \
  --record-svo2 \
  --hand-tracking \
  --hand-model "$PWD/models/hand_landmarker.task" \
  --hand-delegate cpu \
  --hand-inference-fps 10 \
  --hand-roi-scale 1.35 \
  --hand-roi-min-px 128 \
  --hand-roi-max-px 360
```

## Doğrulama

- El/Dex3 birim ve regresyon testleri: `26 passed`.
- Sentetik açık el `OPEN`, tam bükülü beş parmak `FIST` olarak ayrıldı.
- Sentetik yumrukta Dex3 işaret ve orta zincirlerinin ortalama mutlak hedefi
  `1.3 rad` üzerinde ölçüldü.
- ROI ikinci odak denemesi için ayrı regresyon testi eklendi.
- Rerun başsız kayıt testi geçti.
- Resmî 43 eklemli G1-29DOF + Dex3 varlığıyla 600 adımlık başsız uçtan uca
  denemede Isaac 126 el/gövde paketi kabul etti. Her iki el `TRACKING` kaldı.
- İstenen güçlü yumruk hedefleriyle Isaac'teki gerçek 14 el eklemi aynı değerlere
  ulaştı; yalnız resmî USD yumuşak sınırları uygulandı. Yapılandırmadaki Dex3
  sınırları bu deneme sonrasında resmî varlığın sınırlarıyla eşitlendi.
