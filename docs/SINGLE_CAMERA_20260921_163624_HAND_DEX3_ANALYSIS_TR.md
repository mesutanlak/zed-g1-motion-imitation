# Tek ZED BODY_38 + Dex3 incelemesi — 2026-09-21 16:36

## İncelenen kayıtlar

- `recordings/zed_body38_20260921_163624.jsonl`
- `recordings/zed_body38_20260921_163624.svo2`
- `rerun_recordings/rerun_body38_20260921_163411.rrd`
- `rerun_recordings/rerun_body38_20260921_163411/`

## Ölçülen sonuç

- BODY_38: 3438 kare, 115.27 saniye, 29.81 FPS.
- Üst gövde ve bütün çekirdek eklem gözlenebilirliği: `%93.4`.
- El çıkarım paketi: 857; ölçülen hız yaklaşık `5.98 FPS`.
- Sol el geçerli kapsama: `242/857 = %28.24`.
- Sağ el geçerli kapsama: `227/857 = %26.49`.
- İki el aynı anda: `%8.52`; en az bir el: `%46.21`.
- MediaPipe çıkarım süresi: p50 `19.43 ms`, p95 `24.31 ms`.
- Başarılı tespitte ZED derinlik desteği: sol `4.64/5`, sağ `4.97/5` avuç noktası.
- Baskın ret nedeni: `MEDIAPIPE_NO_HAND=1060`; sonra
  `DETECTOR_TRACK_LOST=184`. Yalnız bir örnek bilek uzaklığı kapısından elendi.
- En uzun kesintisiz iz: sol `4.27 s`, sağ `2.27 s`; en uzun kayıp yaklaşık
  `31.10 s` ve `29.60 s`.

Bu sonuç MediaPipe paketinin kurulu ve etkin olduğunu, ancak kamera görüntüsünde
ellerin yeterince sık tanınmadığını gösterir. Çıkarım gecikmesi darboğaz değildir.
Eller gövdeden ayrık, kameraya dönük ve aydınlık tutulmalı; hızlı hareket ve
örtüşme azaltılmalıdır. Tek kamera için `--hand-inference-fps 10` kullanılır.

## Dex3'ün görünmemesinin nedenleri

1. Tek kamera ZED yolu önceki durumda yalnız `zed_operator_hand/v1` ham el
   paketini yayımlıyordu. Dex3 hedef ve kontrol paketine dönüşüm dört kamera
   birleştirme yolunda vardı.
2. Ubuntu Isaac başlatıcısı `g1_23dof` varlığını sabit açıyordu. Bu varlıkta
   Dex3 el geometrisi ve 14 parmak eklemi bulunmaz.
3. Rerun, yeni bir el çıkarım paketi gelmeyen her BODY_38 karesinde önceki el
   geometrisini temizliyordu. El çıkarımı gövdeden daha yavaş olduğu için el
   görünümü yanıp sönüyor veya kayboluyordu.

## Uygulanan düzeltmeler

- Tek kamera MediaPipe şekli avuç çerçevesinde normalize edilip sol ve sağ
  yedişer güvenli Dex3 hedefine çevriliyor.
- ZED canlı paketi `dex3_control` taşıyor; fiziksel DDS çıkışı kapalı kalıyor.
- Isaac başlatıcısı `--asset-profile g1_29dof_dex3` seçeneğiyle resmî Unitree
  USD varlığını açıyor.
- Rerun, aradaki BODY_38 karelerinde son geçerli el geometrisini koruyor.
- MediaPipe tespit kaybında güvenlik makinesi TRACKING, HOLD ve
  FADE_TO_NEUTRAL durumlarıyla hedefi nötre döndürüyor.

Resmî DexPilot Python ortamı kurulu değilse yerel simülasyon normalize 21 nokta
görev uzayı çözümünü kullanır. Bu yol Isaac içi görselleştirme ve güvenli yerel
eklem sürüşü içindir.

## Doğru başlatma sırası

Terminal 1 — Rerun:

```bash
cd "$HOME/g1_isaaclab_project/zed-g1-motion-imitation"
./start_g1_rerun_ubuntu.sh
```

Terminal 2 — Isaac Sim, Dex3 varlığı:

```bash
cd "$HOME/g1_isaaclab_project/zed-g1-motion-imitation"
./start_g1_isaaclab_live_ubuntu.sh \
  --mode upper_body \
  --imitation-mode kinematic_debug \
  --runtime-profile shared_gpu_safe \
  --asset-profile g1_29dof_dex3 \
  --accept-nvidia-eula
```

Terminal 3 — tek ZED, el takibi ve kayıt:

```bash
cd "$HOME/g1_isaaclab_project/zed-g1-motion-imitation"
./start_zed_native_ubuntu.sh \
  --model medium \
  --fps 30 \
  --record \
  --record-svo2 \
  --hand-tracking \
  --hand-model "$PWD/models/hand_landmarker.task" \
  --hand-delegate cpu \
  --hand-inference-fps 10
```

Başlangıç günlüğünde Isaac tarafında `Dex3 local articulation controller ready:
14 joints, no DDS` görülmelidir. ZED tarafında Dex3 yeniden hedefleme etkin
mesajı gelmelidir.

## Doğrulama

- Yeni tek kamera dönüşüm testi dahil `22` test geçti.
- Rerun regresyon testi geçti.
- 857 el paketinin çevrim dışı dönüşümünde 396 pakette en az bir el aktif
  izlendi; güvenlik sınırlayıcıdan çıkan azami mutlak hedef `0.8021 rad` oldu.
- Resmî G1-29DOF + Dex3 USD ile başsız Isaac denemesi 43 eklemi yükledi;
  Dex3 denetleyicisi 14 eklemle hazırlandı ve 120 adımı tamamladı.
