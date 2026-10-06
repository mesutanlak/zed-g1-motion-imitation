# G1 BODY_38 canlı aktarım ve fiziksel hazırlık

Bu hattın birincil amacı dört ZED 2i kameradan insan gövde eklemlerini alıp
Unitree G1-29 kinematiğine düşük gecikmeyle aktarmaktır. Dex3 takibi ikincil ve
isteğe bağlıdır. Fiziksel robot komutu bu aşamada bilinçli olarak kapalıdır;
önce shadow doğrulaması tamamlanmalıdır.

## 1. PTP saat senkronizasyonu

Ana PC'de:

```bash
cd "$HOME/g1_isaaclab_project/zed-g1-motion-imitation"
sudo ./install/setup_ptp_sync_ubuntu24.sh --role main-pc --interface enp129s0
./tools/check_ptp_sync_ubuntu.sh --interface enp129s0
```

Laptopta kendi 192.168.50.x Ethernet arayüz adını kullanın:

```bash
sudo ./install/setup_ptp_sync_ubuntu24.sh --role laptop --interface ETHERNET_ADI
./tools/check_ptp_sync_ubuntu.sh --interface ETHERNET_ADI
```

İki bilgisayarda domain varsayılan olarak 24'tür. Mevcut ana PC'deki
`enp129s0` yalnız software timestamping bildiriyor; betik bunu otomatik seçer.
Donanım PTP istenirse her iki NIC'in `ethtool -T` çıktısında hardware transmit,
hardware receive ve bir `/dev/ptpN` saati görünmelidir.

## 2. Önerilen 30 FPS başlatma sırası

Ana PC terminal 1 — bağımsız telemetri/Rerun kaydı, isteğe bağlı ayrı viewer:

```bash
./start_g1_rerun_ubuntu.sh --viewer
```

Viewer kapanırsa ham BODY ve kontrol günlükleri ile RRD sunucusu çalışmaya
devam eder. Viewer istemiyorsanız `--viewer` eklemeyin.

Ana PC terminal 2 — resmi G1-29 + Dex3 Isaac varlığı ve deterministik GMR:

```bash
./start_g1_isaaclab61_live_ubuntu.sh \
  --accept-nvidia-eula \
  --input-fps 30 \
  --asset-profile g1_29dof_dex3
```

Ana PC terminal 3 — dört kamera füzyonu ve sıra-denetimli araştırma kaydı:

```bash
./start_zed_four_fusion_ubuntu.sh \
  --fps 30 \
  --minimum-sources 4 \
  --record \
  --record-detail research
```

Ana PC terminal 4 — ana PC kameraları ve SVO2:

```bash
./start_zed_four_sources_ubuntu.sh \
  --role main-pc \
  --fps 30 \
  --model medium \
  --depth-mode neural-light \
  --frame-integrity-mode off \
  --require-ptp \
  --preview-hz 4 \
  --record-svo2
```

Laptop — laptop kameraları ve SVO2:

```bash
./start_zed_four_sources_ubuntu.sh \
  --role laptop \
  --main-pc-host 192.168.50.10 \
  --fps 30 \
  --model medium \
  --depth-mode neural-light \
  --frame-integrity-mode off \
  --require-ptp \
  --preview-hz 4 \
  --record-svo2
```

Dex3 araştırması yapılacak ayrı oturumda kaynak ve fusion komutlarına
`--hand-tracking` eklenebilir. BODY performans baz koşusunda el takibini kapalı
tutun; böylece gövde hattının gerçek 30 FPS kapasitesi ayrı ölçülür.

## 3. Kayıt mimarisi

Kontrol paketi ayrıntılı analiz paketi seri hale getirilmeden önce GMR'ye
gönderilir. Rerun monitor paketi ayrı worker üzerinde hazırlanır. Dağıtık
kaynaklar gerekli BODY alanlarını altı ondalık hassasiyetle G1Z1/zlib paketine
çevirir; ölçülen eski 31 KB medyan paket yaklaşık 4--7 KB aralığına iner.
Yerel tam JSONL ve SVO2 bundan etkilenmez. Rerun
uygulamasında iki bağımsız ham günlük bulunur:

- `body_telemetry_raw.jsonl`: alınan tüm geçerli fusion BODY paketleri,
- `control_telemetry_raw.jsonl`: alınan tüm GMR ve Isaac paketleri.

Rerun çizimi 12 Hz ve türetilmiş CSV 15 Hz olabilir; ham günlükler giriş
hızındadır. UDP mutlak teslim garantili değildir; bu yüzden sistem kaybı
saklamak yerine `quality_summary.json/transport_integrity` içinde alınan,
yazılan, düşen, sıra boşluğu ve kuyruk üst-seviye sayaçlarını tutar. Fiziksel
teste geçiş için `journal_dropped=0` ve `sequence_gaps=0` aranmalıdır.

## 4. Metrik ChArUco doğrulaması

Board üretin:

```bash
../envs/zed/bin/python tools/create_charuco_reference_board.py \
  --output calibration/charuco_7x5_a3.png \
  --square-mm 55 --marker-mm 41 --dpi 300
```

Board'u yüzde 100 ölçekte düz ve rijit bir yüzeye bastırın. Bir karenin gerçek
kenarını kumpasla ölçün; 55.00 mm değilse analiz komutunda gerçek değeri verin.
Board sabit, dört kamerada tamamen görünür ve odada hareket eden insan yokken
10–20 saniye SVO2 alın. Ardından:

```bash
../envs/zed/bin/python tools/validate_charuco_svo_reference.py \
  recordings/zed_body38_31571870_OTURUM.svo2 \
  recordings/zed_body38_33773329_OTURUM.svo2 \
  recordings/zed_body38_34760587_OTURUM.svo2 \
  recordings/zed_body38_39504762_OTURUM.svo2 \
  --extrinsics "four json/fourkamera.json" \
  --square-mm GERCEK_OLCULEN_MM --marker-mm 41 \
  --output calibration/charuco_metric_report.json
```

Bu rapor kamera reprojection hatasını ve aynı sabit board'un dört kamera
üzerinden world-frame konum/rotasyon dağılımını verir. İnsan eklem doğruluğu
için sonraki aşama marker'ları bilinen omuz/dirsek/bilek noktalarına koyarak
aynı board frame'inde ölçmektir.

## 5. Collision ve ortam kaydı

`config/g1_collision_safety.json` resmi G1-29 + Dex3 varlıklarını ve güvenlik
eşiklerini tanımlar. Ortam engelleri robot world frame'inde ölçülmeden listeye
eklenmemelidir. Desteklenen tipler:

```json
{
  "name": "table",
  "type": "box",
  "center_m": [0.65, 0.0, 0.78],
  "half_extents_m": [0.35, 0.60, 0.04]
}
```

Sphere için `radius_m`, capsule için `radius_m` ve `axis_half_vector_m`
kullanılır. Her son hedefte self/environment collision, adaptif swept yol ve
filtre sonrası son kontrol uygulanır. Bilek Jacobian oranı düşükse yalnız ilgili
kol yüzde 20'ye kadar kademeli yavaşlar.

## 6. Fiziksel robot shadow doğrulaması

Unitree bilgisayarı/ağı erişilebilirken önce yalnız low-state okuyun:

```bash
./start_g1_hardware_shadow_ubuntu.sh ROBOT_ETHERNET_INTERFACE
```

Isaac/GMR'yi hedef kopyasıyla başlatın:

```bash
./start_g1_isaaclab61_live_ubuntu.sh \
  --accept-nvidia-eula --input-fps 30 \
  --asset-profile g1_29dof_dex3 --hardware-shadow
```

Shadow validator yalnız `rt/lowstate` açar; `rt/lowcmd`, `rt/arm_sdk` veya Dex3
komut topic'i açmaz. Hedef, gerçek q/dq, IMU, mode-machine ve watchdog yaşı
`hardware_recordings/` altında saklanır. Düşük seviye komut yayınlayan fiziksel
adaptör; shadow kayıtları, e-stop/deadman ve askı testleri geçmeden etkin hale
getirilmemelidir.

Kanıtları tek, fail-closed raporda değerlendirin:

```bash
../envs/zed/bin/python tools/check_physical_readiness.py \
  rerun_recordings/OTURUM/quality_summary.json \
  --phase physical-shadow \
  --charuco calibration/charuco_metric_report.json \
  --shadow hardware_recordings/g1_shadow_OTURUM.jsonl \
  --output reports/g1_physical_readiness.json
```

Kapı; dört kamera için en az 27 FPS p50, capture spread p95 en çok 40 ms,
kontrol p95 en çok 100 ms, sıfır telemetri sıra boşluğu/disk düşümü, negatif
olmayan son collision marjı, dört-kamera ChArUco doğrulaması ve en az 1000
salt-okunur shadow örneği ister. `ready=true` motor komutunu kendiliğinden
etkinleştirmez; fiziksel e-stop, deadman, askı ve Unitree servis-durumu testleri
ayrıca zorunludur.
