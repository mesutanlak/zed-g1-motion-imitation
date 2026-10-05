# Ubuntu 4×ZED + GMR/Isaac + Rerun + el takibi + SVO2 çalıştırma kaydı

Bu kayıt, kalibrasyonu daha önce alınmış iki Ubuntu bilgisayarlı dört ZED 2i
düzeni içindir. Ana PC `192.168.50.10`, kamera laptopu `192.168.50.11` olarak
kabul edilir. Akış fiziksel robota DDS/motor komutu göndermez; Isaac içindeki
G1-29DOF + Dex3 varlığını sürer.

## Beklenen veri yolu

```text
4× ZED BODY_38 + 21 noktalı el verisi
             │
             ▼
4-ZED fusion ├── 15050 ──> GMR ──> 15051 ──> Isaac G1-29 + Dex3
             ├── 15052 ──> Rerun (ham/fused iskelet ve eller)
             ├── 15054 ──> ROS kopyası
             └── recordings/four_body38_fusion_*.jsonl

Her kamera hostu ayrıca yerel recordings/ altında bir JSONL ve bir SVO2 yazar.
```

SVO2 tek bir birleşik dosya değildir. Dört fiziksel kamera için dört ayrı SVO2
oluşur; ana PC'deki iki dosya ana PC'de, laptoptaki iki dosya laptopta kalır.

## 1. Ön kontrol

ZED Explorer, ZED360 ve eski kamera/publisher süreçlerini iki bilgisayarda da
kapatın. Tripodlar kalibrasyondan sonra hareket etmemiş olmalıdır.

Ana PC:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./zed_four_camera_test/test_four_camera_preflight_ubuntu.sh \
  --role main-pc \
  --expected-ip 192.168.50.10 \
  --peer-ip 192.168.50.11 \
  --serial 31571870 \
  --serial 33773329 \
  --expected-sdk 5.4.1

../envs/zed/bin/python \
  ./zed_four_camera_test/validate_distributed_extrinsics.py \
  --input './four json/fourkamera.json' \
  --expected-serials '31571870,33773329,34760587,39504762' \
  --reference-serial 33773329
```

Laptop:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./zed_four_camera_test/test_four_camera_preflight_ubuntu.sh \
  --role laptop \
  --expected-ip 192.168.50.11 \
  --peer-ip 192.168.50.10 \
  --serial 39504762 \
  --serial 34760587 \
  --expected-sdk 5.4.1
```

Her iki ön kontrol de `ON KONTROL: BASARILI` ile bitmelidir. Ayrıca iki hostta
aynı el modelinin bulunduğunu ve hash'lerinin eşleştiğini doğrulayın:

```bash
sha256sum models/hand_landmarker.task
```

## 2. Başlatma sırası

Komutları ayrı terminallerde açın. Isaac ve Rerun ana PC'de çalışır.

### Ana PC terminal 1 — Rerun

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_g1_rerun_ubuntu.sh
```

### Ana PC terminal 2 — GMR ve Isaac G1-29DOF + Dex3

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_g1_isaaclab_live_ubuntu.sh \
  --mode upper_body \
  --imitation-mode kinematic_debug \
  --runtime-profile shared_gpu_safe \
  --asset-profile g1_29dof_dex3 \
  --input-fps 15 \
  --accept-nvidia-eula
```

`G1 yerel Ubuntu zinciri` satırından sonra Isaac penceresinin açılmasını
bekleyin. Logda G1-29DOF + Dex3 varlığının ve DDS kapalı yerel Dex3
denetleyicisinin hazır olduğu görülmelidir.

### Ana PC terminal 3 — dört kamera fusion, el fusion ve araştırma kaydı

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_zed_four_fusion_ubuntu.sh \
  --minimum-sources 3 \
  --fps 15 \
  --preview-hz 8 \
  --hand-tracking \
  --hand-max-age-ms 120 \
  --hand-max-spread-ms 70 \
  --record-detail research \
  --record
```

`research` seviyesi fused BODY_38, fused el noktaları, kamera/detector özeti,
zamanlama, residual ve Dex3 hedeflerini saklar. Bir hatayı nokta nokta yeniden
incelemek gereken kısa tanı kaydında `--record-detail full` kullanılabilir;
dosya boyutu belirgin büyür.

### Laptop terminali — uzak iki kamera, el takibi ve iki SVO2

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_zed_four_sources_ubuntu.sh \
  --role laptop \
  --main-pc-host 192.168.50.10 \
  --fps 15 \
  --model medium \
  --depth-mode neural-light \
  --frame-integrity-mode off \
  --preview-hz 8 \
  --hand-tracking \
  --hand-model "$PWD/models/hand_landmarker.task" \
  --record-svo2
```

### Ana PC terminal 4 — yerel iki kamera, el takibi ve iki SVO2

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_zed_four_sources_ubuntu.sh \
  --role main-pc \
  --fps 15 \
  --model medium \
  --depth-mode neural-light \
  --frame-integrity-mode off \
  --preview-hz 8 \
  --hand-tracking \
  --hand-model "$PWD/models/hand_landmarker.task" \
  --record-svo2
```

CPU delegate, ilk kabul kaydı için kararlı profildir. Ölçülen inference hızı
hedefin altında kalırsa önce aynı hareket senaryosunu `10 Hz` ile A/B test
edin; BODY_38 capture döngüsü el worker'ını beklemez.

## 3. Canlı sağlık kontrolü

Ana PC'de ek terminal:

```bash
ss -lunp | grep -E \
  ':(15050|15051|15052|15053|15054|16000|16002|16004|16006|16100|16102|16104|16106|16200|16202|16204|16206)\b'
```

Beklenen durum:

- fusion arayüzünde `bagli=4/4`, `gmr_kapi=READY`, `gecersiz=0`, `drop=0`;
- çoğu karede `fusion_katki=4/4`, kısa kayıplarda en az `3/4`;
- fusion hızı yaklaşık 14–15 FPS;
- kaynak kapanışında `El paketleri` sayacının artması ve fusion durumunda
  `el_rx=A+Byedek` toplamının sıfırdan büyük olması; `A` ayrı el UDP yolunu,
  `B` BODY datagramındaki güvenilir yedeği gösterir;
- Rerun'da fused insan, G1 RAW/SAFE ve iki el landmark/DEX3 metrikleri;
- Isaac telemetrisinde `asset_profile=g1_29dof_dex3` ve
  `dex3_dds_enabled=false`;
- dört kaynak terminalinde SVO2 kaydının açık olduğu ve frame sayısının arttığı
  görülmelidir.

Fusion'da el verisi görünmüyorsa önce kaynak logundaki model/`El paketleri`
sayacını ve BODY yedeğini kontrol edin. Ayrı UDP için ayrıca
`16200/16202/16204/16206` portlarını denetleyin. BODY geliyor fakat iki taşıma
sayacı da beş saniye boyunca sıfır kalırsa fusion artık açık bir uyarı basar.

## 4. İlk ölçüm senaryosu

Kadraja yalnız tek operatör girsin. En az 90 saniyelik kayıtta sırasıyla:

1. 10 saniye nötr duruş ve eller açık;
2. iki kolu öne, yana ve yukarı kaldırma; dirsek bükme;
3. açık el, yumruk, pinch ve parmakları sırayla kapatma;
4. elleri gövde önünde çaprazlama;
5. sağ ve sol eli ayrı ayrı 2–3 saniye oklüde etme;
6. avuç içini kameraya ve kameradan uzağa çevirme;
7. çalışma hacminde yavaşça sağa/sola ve öne/arkaya hareket;
8. tekrar 10 saniye nötr duruş.

Her fazı sesli söylemek veya süreleri ayrıca not etmek, sonradan bozuk bölümü
SVO2/Rerun zaman çizelgesiyle eşlemeyi kolaylaştırır.

## 5. Güvenli kapatma

1. Fusion penceresinde `Q`/`Esc`; JSONL kapanıp flush edilir.
2. İki kaynak terminalinde `Ctrl+C`; dört SVO2 güvenli kapanır.
3. Rerun'da `Ctrl+C`; `.rrd`, analiz JSONL/CSV ve oturum özeti kapanır.
4. Isaac/GMR terminalinde `Ctrl+C`.

Süreçleri zorla öldürmeyin; özellikle SVO2 ve JSONL sonlandırması eksik
kalabilir.

## 6. Kayıt sonrası sayısal inceleme

Ana PC:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation

latest_fusion="$(find recordings -maxdepth 1 -type f \
  -name 'four_body38_fusion_*.jsonl' -printf '%T@ %p\n' | \
  sort -nr | head -n1 | cut -d' ' -f2-)"

../envs/zed/bin/python \
  ./zed_four_camera_test/summarize_four_body38_recording.py \
  --input "$latest_fusion" | tee recordings/four_body38_summary_latest.txt

../envs/zed/bin/python ./tools/benchmark_hand_tracking.py \
  "$latest_fusion" \
  --output recordings/hand_benchmark_latest.json

find recordings -maxdepth 1 \
  \( -name '*.svo2' -o -name 'zed_body38_*.jsonl' \
     -o -name 'four_body38_fusion_*.jsonl' \) \
  -printf '%TY-%Tm-%Td %TH:%TM:%TS  %12s  %p\n' | sort

find rerun_recordings -maxdepth 1 -type f \
  -printf '%TY-%Tm-%Td %TH:%TM:%TS  %12s  %p\n' | sort
```

Laptopta son iki SVO2/JSONL dosyasını ayrıca listeleyin:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
find recordings -maxdepth 1 \
  \( -name '*.svo2' -o -name 'zed_body38_*.jsonl' \) \
  -printf '%TY-%Tm-%Td %TH:%TM:%TS  %12s  %p\n' | sort
```

İlk kabul değerlendirmesinde şu alanlara bakılır:

- BODY: 4/4 ve 3/4 katkı oranı, kaynak başına katılım, aktif FPS, sync p95,
  `cross_view_mpjpe` ve `post_alignment_mpjpe`, ağ kuyruğu ve record drop;
- el: sol/sağ ve iki-el coverage, kamera başına detector başarısı/inference,
  depth completeness, 0–4 kamera landmark histogramı, capture spread p95,
  fusion residual/reprojection, en uzun takip kaybı;
- Dex3: solver p50/p95, joint-limit clip, velocity/acceleration limit olayları
  ve watchdog hold/fade;
- GMR/Isaac: RAW→SAFE farkı, safety clamp, gecikme, stale dönüşü ve gövde/el
  zaman eşleşmesi;
- veri bütünlüğü: dört SVO2, dört kamera JSONL'si, bir fusion JSONL'si ve bir
  Rerun `.rrd`/oturum özeti bulunmalıdır.

Mutlak anatomik doğruluk için bu metrikler tek başına ground truth değildir;
ancak eksik görüş, zaman senkronu, kalibrasyon uyumsuzluğu, el ROI/detector ve
retarget/safety darboğazını birbirinden ayırmak için yeterlidir.
