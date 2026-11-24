# Dört ZED için hafif el aç/kapat yolu — 2026-11-24

## Neden

`154917` motorsuz shadow sonucu `FAIL_CLOSED`: taze uygulama 23,21 Hz, çıkış aralığı p99 90,26 ms, en uzun aralık 99,72 ms. Çarpışma ve türev ihlali sıfır. Aynı oturumun Isaac aşama izinde çıkış aralığı p99 84,83 ms. `duplicate` sayacı el paketinin ayrı UDP ve BODY içine gömülü yedek kanalından iki kez gelmesini sayıyordu; bu tek başına paket kaybı veya 90 ms gecikmenin kanıtı değil. Eski canlı kayıtta el füzyonu+retarget p95 5,54 ms idi. Dolayısıyla el yolunun sadeleşmesi tüm 45 ms zamanlama açığını kapatacağını kanıtlamaz.

## Uygulama

`--hand-tracking --hand-gesture-only` kaynakta MediaPipe'i küçük bilek ROI'sinde 8 Hz çalıştırır, sadece dört parmağın bükülmesinden `OPEN/CLOSED/UNKNOWN` çıkarır. El derinliği alınmaz; 21 nokta, derinlik, 3-D üçgenleme ve yinelenen gömülü BODY el paketi ağdan kaldırılır. Ayrı el UDP paketi yaklaşık 0,8 kB'dir. Eski 21 noktalı mod hâlâ seçilebilir.

Füzyon zamana hizalı açık/kapalı oylarını birleştirir. Görüşler çelişirse, veri eskirse, el örtülürse veya el ara pozdaysa durum `UNKNOWN` olur. Bu durumda Dex3 simülasyon hedefi mevcut hız/ivme sınırlarıyla önce son hedefi tutar, sonra nötre döner. GMR/Isaac'teki `--enable-dex3-control` seçeneği ayrıca açılmadıkça parmak hedefleri simülasyonda da uygulanmaz. Mevcut fiziksel kol göndericisi Dex3 el DDS komutu üretmez; bu değişiklik fiziksel robot parmaklarını henüz açıp kapatmaz. Fiziksel parmak çıkışı için ayrı donanım doğrulaması, eklem ölçümü, sınır ve durdurma entegrasyonu gerekir.

## Eski dört kamera kaydıyla motorsuz tekrar oynatma

Kaynak: `recordings/four_body38_fusion_kp120_p25_full_20261124_01_20261124_154831.jsonl`; çıktı: `hardware_recordings/coarse_hand_replay_20261124_154831.json`.

```bash
cd /home/mesut/g1_isaaclab_project/zed-g1-motion-imitation
../envs/zed/bin/python tools/replay_coarse_hand_gesture.py \
  recordings/four_body38_fusion_kp120_p25_full_20261124_01_20261124_154831.jsonl \
  --output hardware_recordings/coarse_hand_replay_20261124_154831.json
```

| Ölçüm | Eski | Hafif aç/kapat |
|---|---:|---:|
| Aynı arşiv girdisinde el CPU p95 | 4,34 ms (yalnız 3-D füzyon) | 0,35 ms (sınıflama+füzyon+Dex3 sim hedefi) |
| El UDP paketi p50 | 5175 bayt | 762 bayt |
| El UDP paketi p95 | 8917 bayt | 918 bayt |
| Canlı ölçülmüş eski el füzyonu+retarget p95 | 5,54 ms | henüz ölçülmedi |

1048 karede el paketi vardı, yalnız 300 karede en az bir kaynak eli açık/kapalı sınıflanabildi. Fused sol `OPEN=135`, `CLOSED=62`, `UNKNOWN=851`; sağ `OPEN=47`, `CLOSED=90`, `UNKNOWN=911`. Bu etiketlerin gerçek insan eliyle doğruluğu için bağımsız video anotasyonu yok; **NOT_MEASURABLE**. Tekrar oynatma kamera/MediaPipe, GMR, Isaac, ağ, DDS ve motor ölçümü süresini içermez. Yeni uçtan uca p95 **NOT_MEASURABLE**; canlı dört kamera motorsuz shadow PASS görülmeden gecikmenin düzeldiği söylenemez.

## Bir sonraki motorsuz canlı ölçüm

Laptop ve ana PC aynı kod sürümünü kullanmalı. Kaynak başlatıcılarında `--hand-tracking --hand-gesture-only`, füzyonda aynı iki bayrak kullanılmalı. İlk A/B'de SVO2 kaydını kapatıp `--record-local` ile BODY/el JSONL'yi koruyun; sonra yalnız SVO2'yi açarak aynı hareketleri tekrarlayın. Her iki koşulda 30 saniyelik `./hardware/run_g1_trajectory_shadow.sh 30 both` raporunu ve tüm kaynak FPS, GMR/Isaac ve PTP kayıtlarını karşılaştırın. Robot kapılarını veya bağımsız IMU durdurmasını değiştirmeyin. Shadow PASS ve fiziksel preflight sağlanırsa 10 saniyelik vücut/kol motor testi, onun raporu incelendikten sonra 60 saniye denenebilir. Bu deney fiziksel parmak hareketini sınamaz.
