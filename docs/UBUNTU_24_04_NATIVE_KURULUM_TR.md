# Ubuntu 24.04 + Isaac Sim 6.1 yerel kurulum ve çalıştırma

Bu yol, ikinci diskte kurulu Ubuntu 24.04 üzerinde çalışır. Windows kurulumu ve
depodaki PowerShell başlatıcıları korunur. Disk bölümlerine, GRUB'a, Secure
Boot'a veya çalışan NVIDIA sürücüsüne müdahale etmez.

## Doğrulanan sistem

- Ubuntu 24.04, kernel 7.0
- NVIDIA RTX 5090, sürücü 595.91.07
- CUDA Toolkit 12.8
- ZED SDK 5.4.1
- ROS 2 Jazzy + Cyclone DDS
- Isaac Sim 6.1.0.0 + Isaac Lab 3.0.0 EA + Python 3.12
- GMR + Python 3.10

Isaac Sim 6.1 bu sistemde R595 ile açıldığı için R570 geçişi ve MOK kaydı bu
yolda gerekli değildir.

## Ortamların ayrılması

| Ortam | Konum | Python | Amaç |
|---|---|---:|---|
| ZED | `~/g1_isaaclab_project/envs/zed` | 3.12 | Kamera ve BODY_38 |
| GMR | `~/g1_isaaclab_project/envs/gmr_zed` | 3.10 | Hareket retargeting |
| Isaac | `~/g1_isaaclab_project/envs/isaaclab30` | 3.12 | Isaac Sim 6.1 + Isaac Lab 3.0 |
| Unitree SDK2 | `~/g1_isaaclab_project/envs/unitree_sdk2` | 3.10 | İsteğe bağlı robot iletişimi |
| MuJoCo | `~/g1_isaaclab_project/envs/mujoco` | 3.12 | Alternatif simülasyon |

Eski `envs/isaacsim`, `envs/isaacsim51` ve `envs/isaacsim61` ortamları
silinmez. Yeni proje başlatıcısı yalnız `envs/isaaclab30` kullanır.

## 1. Sistem paketleri

```bash
sudo apt update
sudo apt install -y \
  git git-lfs build-essential cmake ninja-build pkg-config \
  libgl1 libglib2.0-0t64 libx11-6 libxext6 libxrender1 libsm6 libglfw3
git lfs install
```

## 2. Sabitlenmiş kaynakları ve Unitree varlıklarını indir

Proje dizininde:

```bash
bash install/fetch_ubuntu24_sources.sh
```

Betik Isaac Lab `v3.0.0-EA`, GMR, Unitree RL Lab, Unitree ROS,
`unitree_sim_isaaclab`, SDK2 Python, XR Teleoperate ve Unitree MuJoCo
depolarını `~/g1_isaaclab_project/repos` altına sabit commitlerle indirir.
Ayrıca resmî G1-29 + Dex3 USD varlıklarını alır.

## 3. GMR ortamı

```bash
bash install/setup_gmr_ubuntu24.sh
```

## 4. Isaac Sim 6.1 + Isaac Lab 3.0

```bash
bash install/setup_isaac_ubuntu24.sh
```

Bu komut Isaac Lab'ın resmî `uv.lock` dosyasını kullanır. Python 3.12,
Isaac Sim 6.1.0.0 ve Isaac Lab 3.0 aynı kilitli ortamda kurulur. İlk kurulumda
NVIDIA extension cache birkaç GB indirebilir.

Paket ve GPU doğrulaması:

```bash
~/g1_isaaclab_project/envs/isaaclab30/bin/python \
  isaaclab_bridge/test_environment.py
```

Kısa Isaac Sim testi:

```bash
export OMNI_KIT_ACCEPT_EULA=YES
~/g1_isaaclab_project/envs/isaaclab30/bin/python \
  isaaclab_bridge/test_isaacsim.py
```

## 5. ZED, ROS 2 ve MuJoCo

```bash
bash install/setup_zed_ubuntu24.sh
bash install/setup_ros2_jazzy_ubuntu24.sh
bash install/setup_mujoco_ubuntu24.sh
```

Gerçek robot bağlantısı ileride gerektiğinde SDK'yı ayrı ortamda hazırlayın:

```bash
bash install/setup_unitree_sdk2_ubuntu24.sh
```

Bu kurulum yalnız SDK importunu doğrular; DDS ağına bağlanmaz ve robota komut
göndermez.

ZED kamerayı doğrudan anakartın USB 3.x portuna bağladıktan sonra:

```bash
lsusb
/usr/local/zed/tools/ZED_Explorer
```

## 6. Toplu doğrulama

```bash
bash install/verify_ubuntu24_native.sh
```

## 7. G1 sahnesini önce başsız çalıştır

```bash
./start_g1_isaaclab_live_ubuntu.sh \
  --mode upper_body \
  --imitation-mode kinematic_debug \
  --headless \
  --max-steps 100 \
  --accept-nvidia-eula
```

Başlatıcı GMR köprüsünü açar, ROS 2 Jazzy ve Cyclone DDS değişkenlerini kurar,
sonra Isaac Sim 6.1 içinde resmî Unitree G1-23DOF sahnesini çalıştırır.

GUI çalışması:

```bash
./start_g1_isaaclab_live_ubuntu.sh \
  --mode upper_body \
  --imitation-mode kinematic_debug \
  --runtime-profile shared_gpu_safe \
  --accept-nvidia-eula
```

Dex3 ellerini sahnede görmek ve 14 parmak eklemini yerel olarak sürmek için
resmî Unitree G1-29DOF + Dex3 varlığını seçin:

```bash
./start_g1_isaaclab_live_ubuntu.sh \
  --mode upper_body \
  --imitation-mode kinematic_debug \
  --runtime-profile shared_gpu_safe \
  --asset-profile g1_29dof_dex3 \
  --accept-nvidia-eula
```

`--asset-profile` verilmezse geriye dönük uyumluluk için 23DOF gövde açılır;
bu varlıkta Dex3 el geometrisi yoktur.

Ayrı terminalde ZED akışı:

```bash
./start_zed_native_ubuntu.sh --model medium --fps 15
```

### Tek kamerada MediaPipe el takibi

MediaPipe yalnız ZED ortamına kurulur:

```bash
./install/setup_hand_tracking_ubuntu24.sh
mkdir -p models
```

Google'ın resmi Hand Landmarker sayfasındaki `HandLandmarker (full)` modelini
indirip `models/hand_landmarker.task` olarak kaydedin. Proje model dosyasını
sessizce indirmez; kullanılan modelin kaynağı açık kalır.

```bash
wget -O models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
ls -lh models/hand_landmarker.task
```

El takibiyle kayıt:

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

CPU delegate başlangıç için en kararlı seçenektir. El çıkarımı ayrı, son-değer
öncelikli iş parçacığında çalışır; 30 FPS BODY_38 yakalamayı bekletmez. Ham
21 nokta ayrı el kanalına ve analiz/Rerun kopyasına gönderilir; GMR BODY_38
kontrol paketi kompakt kalır. Tek kameradaki MediaPipe dünya noktaları
el-merkezli yerel şekildir. Bunlar avuç çerçevesinde normalize edilerek güvenli
14 eklemli Dex3 hedefine çevrilir ve Isaac içindeki yerel artikülasyon
denetleyicisine gönderilir. Fiziksel robota DDS çıkışı bu yolda kapalıdır.
İlk bilek ROI denemesinde el bulunamazsa merkezdeki daha sıkı odak kırpmasında
ikinci deneme yapılır. Avuç dönüşü SO(3), konumu ve ölçeği SE(3) uyumlu zamansal
filtreyle yumuşatılır. Beş insan parmağının bükülme açıları Dex3 başparmak,
işaret ve birleştirilmiş orta parmak zincirlerine aktarılır. El çıkarımının
olmadığı ara BODY_38 kareleri de bağımsız HOLD/FADE watchdog hedefi taşır.

İsteğe bağlı Rerun görünümü:

```bash
./start_g1_rerun_ubuntu.sh
```

## 8. ROS 2 Jazzy / RViz

Her ROS terminalinde önce:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

BODY_38 köprüsü ve görünümü:

```bash
./ros2_bridge/start_body38_ros_bridge_jazzy.sh
./ros2_bridge/view_body38_rviz_jazzy.sh
```

G1 görünümü:

```bash
./ros2_bridge/start_g1_full_rviz_jazzy.sh
```

## Isaac Lab 3.0 geçiş ayrıntıları

`isaaclab_bridge/isaac_g1_23dof_live.py` iki Isaac Lab neslini tanır. Isaac
Lab 3.0 yolunda:

- `ProxyArray` verileri açıkça `.torch` görünümüne çevrilir;
- quaternion verileri Isaac Lab 3.0'ın XYZW düzeninde işlenir;
- G1-23DOF ve G1-29 + Dex3 tanımları proje içindeki 3.0 uyumlu Unitree
  yapılandırmasından yüklenir;
- eski `unitree_rl_lab` 2.x paketi yeni ortama kurulmaz;
- Unitree DDS kanalı açılmaz ve gerçek robota motor komutu gönderilmez.

Eski 5.0 dosyaları tanı ve karşılaştırma için
`install/setup_isaac50_legacy_ubuntu24.sh` ve
`start_g1_isaaclab50_legacy_ubuntu.sh` adlarıyla korunur.
