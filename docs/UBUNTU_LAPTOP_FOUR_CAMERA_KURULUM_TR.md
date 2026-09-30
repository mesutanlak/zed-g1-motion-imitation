# Ubuntu laptop ile 4×ZED BODY_38: sıfırdan kurulum, kalibrasyon ve çalışma

Bu belge mevcut dört-kamera rig'inde **laptop rolünü Windows'tan Ubuntu
24.04'e geçirir**. Fiziksel dağılım değişmez: laptop iki ZED'i çalıştırıp
Ethernet üzerinden ana PC'ye yollar; ana PC diğer iki ZED'i açar, dört akışı
birleştirir ve GMR/Isaac/Rerun'a verir.

```text
Laptop Ubuntu 24.04 / 192.168.50.11
  ZED 39504762 -> BODY 16000, preview 16100 ----+
  ZED 34760587 -> BODY 16006, preview 16106 ----+-- CAT6 --> Ana PC 192.168.50.10

Ana PC
  ZED 31571870 -> BODY 16002, preview 16102 ----+
  ZED 33773329 -> BODY 16004, preview 16104 ----+--> application BODY_38 fusion
                                                        |-> GMR/Isaac 15050
                                                        |-> Rerun     15052
                                                        `-> ROS       15054
```

Bu çalışma doğrudan fiziksel G1 motorlarına komut göndermez. Önce simülasyon
ve kayıt kalite kapıları doğrulanmalıdır.

## 0. Sabit sözleşme ve önemli ayrım

- İşletim sistemi: Ubuntu 24.04 x86_64.
- Ana PC: `192.168.50.10/24`.
- Laptop: `192.168.50.11/24`.
- İki hostta ZED SDK sürümü aynı olmalıdır. Doğrulanmış rig sürümü `5.4.1`dir.
- Başlangıç profili: `HD720`, `15 FPS`, `NEURAL_LIGHT`, `BODY_38 MEDIUM`.
- Laptopta Isaac Sim, GMR, ROS veya Unitree depolarına ihtiyaç yoktur. Yalnız
  ZED SDK, bu proje ve `envs/zed` kurulur.

İki farklı kalibrasyon vardır:

1. **Kamera extrinsic kalibrasyonu:** dört sabit kamerayı ortak dünya
   koordinatına getirir. Tripod/kamera hareket ederse yeniden alınır.
2. **Operatör nötr kalibrasyonu:** her kaynak başladığında kilitlenen kişi için
   yaklaşık 4 saniye alınır. Başlangıçta ortada, dik ve nötr durun.

Bu belgede yeni extrinsic de alınacaktır; eski `fourkamera.json` yeni tripod
pozlarıyla kullanılmayacaktır.

## 1. Ana PC'deki gerçek sürümü kaydet

Ana PC Ubuntu ise:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
~/g1_isaaclab_project/envs/zed/bin/python -c \
  'import pyzed.sl as sl; print(sl.Camera.get_sdk_version())'
nvidia-smi
```

Ana PC Windows ise:

```powershell
cd "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
.\.venv-zed\Scripts\python.exe -c \
  "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"
```

Beklenen SDK `5.4.1`dir. Farklı çıkarsa laptopta aynı sürümü kurun. Dört
kamera oturumu sırasında iki hostu farklı ZED SDK minor sürümlerinde bırakmayın.

## 2. Laptop Ubuntu ve NVIDIA sürücüsü

Ubuntu kurulumu bittikten sonra internet için Wi-Fi'yi açık bırakın. Kamera
Ethernet'i daha sonra gatewaysiz ayrı ağ olacaktır.

```bash
sudo apt update
sudo apt install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo ubuntu-drivers install
sudo reboot
```

Yeniden açılınca:

```bash
nvidia-smi
lsb_release -ds
uname -m
```

`nvidia-smi` GPU ve sürücüyü göstermeli; Ubuntu `24.04`, mimari `x86_64`
olmalıdır. ZED SDK, uyumlu NVIDIA GPU ve CUDA ister. Secure Boot sürücüyü
engelliyorsa `nvidia-smi` düzelmeden ZED kurulumuna geçmeyin.

## 3. Projeyi GitHub'dan sıfırdan al

```bash
sudo apt update
sudo apt install -y git git-lfs
git lfs install

mkdir -p ~/g1_isaaclab_project
cd ~/g1_isaaclab_project
git clone https://github.com/mesutanlak/zed-g1-motion-imitation.git
cd zed-g1-motion-imitation
git checkout main
git pull --ff-only origin main
```

Kontrol:

```bash
test -x ./start_zed_four_sources_ubuntu.sh
test -x ./zed_four_camera_test/start_distributed_source_ubuntu.sh
git status --short
```

İki `test` komutu da sessizce `0` dönmelidir. Dosyalar yoksa Ubuntu laptop
değişiklikleri henüz GitHub `main` dalına gönderilmemiştir; eski Windows-only
kodla devam etmeyin.

Günlük güncelleme komutu:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
git pull --ff-only origin main
```

## 4. Laptopa ZED SDK 5.4.1 kur

Proje rig'i CUDA 12.8 paketli ZED SDK 5.4.1 ile doğrulanmıştır. Stereolabs'ın
resmî sabit yönlendirmesiyle indirin:

```bash
mkdir -p ~/Downloads
cd ~/Downloads
wget --content-disposition \
  https://download.stereolabs.com/zedsdk/5.4/cu12/ubuntu24
ls -lh ZED_SDK_Ubuntu24_cuda12.8_tensorrt10.9_v5.4.1.zstd.run
```

Ardından proje kurulumunu çalıştırın:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./install/setup_four_camera_laptop_ubuntu24.sh \
  --zed-installer \
  ~/Downloads/ZED_SDK_Ubuntu24_cuda12.8_tensorrt10.9_v5.4.1.zstd.run
```

Kurucu lisansını okuyun. Python API, tools ve özellikle Body Tracking/AI
modülünü kurun; `skip_od_module` kullanmayın. Kurucunun sonunda yeniden
başlatma istenirse:

```bash
sudo reboot
```

ZED SDK zaten kurulmuşsa yalnız ortamı kurmak yeterlidir:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./install/setup_four_camera_laptop_ubuntu24.sh
```

Resmî Linux kurucusu `/usr/local/zed` altına kurulur; Python sarmalayıcı
gerektiğinde `/usr/local/zed/get_python_api.py` ile yüklenir. Proje betiği bunu
`~/g1_isaaclab_project/envs/zed` ortamında otomatik yapar.

## 5. İki laptop kamerasını tek tek doğrula

Önce ZED Explorer, Depth Viewer, ZED360 ve eski Python süreçlerini kapatın.
İki ZED'i mümkünse farklı USB root controller'a veya harici adaptörlü kaliteli
USB 3.x hub'a bağlayın.

```bash
lsusb | grep -i -E 'stereo|zed|2b03'
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
~/g1_isaaclab_project/envs/zed/bin/python \
  ./zed_g1_skeleton.py --list-devices
```

Laptopta şu iki seri `AVAILABLE` olmalıdır:

```text
39504762
34760587
```

Grafik test:

```bash
/usr/local/zed/tools/ZED_Explorer
```

Her kamerayı ayrı ayrı açıp USB 3, görüntü bütünlüğü ve 15 FPS'i kontrol edin.
Sonra Explorer'ı tamamen kapatın. İki kamerayı aynı anda açacak proje
başlatıcısından önce hiçbir ZED aracı kamerayı tutmamalıdır.

## 6. Ethernet'i statik kamera ağı yap

Arayüz adını bulun:

```bash
ip -br link
nmcli device status
```

Örneğin arayüz `eno1` ise laptopta:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./install/configure_four_camera_ethernet_ubuntu.sh \
  --interface eno1 \
  --address 192.168.50.11/24
```

Bu bağlantıya gateway/DNS koyulmaz; internet Wi-Fi üzerinden kalır. Ana PC
Ethernet'i `192.168.50.10/24` olmalıdır. Ana PC de Ubuntu ve yapılandırılmamışsa
aynı betiği o makinede farklı adresle kullanın:

```bash
./install/configure_four_camera_ethernet_ubuntu.sh \
  --interface eno1 \
  --address 192.168.50.10/24
```

İki yönde kontrol:

```bash
ping -c 8 -s 1400 192.168.50.10   # laptopta
ping -c 8 -s 1400 192.168.50.11   # ana PC'de
```

Paket kaybı olmamalıdır. Bağlantı hızını görün:

```bash
sudo ethtool eno1 | grep -E 'Speed|Duplex|Link detected'
```

Tercih edilen hız `1000Mb/s` veya `2500Mb/s`, full duplex'tir.

### UFW açıksa

Ana PC Ubuntu'da yalnız laptop IP'sinden gerekli UDP girişlerini açın:

```bash
sudo ufw status
sudo ufw allow from 192.168.50.11 to any port 16000 proto udp
sudo ufw allow from 192.168.50.11 to any port 16006 proto udp
sudo ufw allow from 192.168.50.11 to any port 16100 proto udp
sudo ufw allow from 192.168.50.11 to any port 16106 proto udp
```

El takibi daha sonra açılırsa `16200` ve `16206` da aynı şekilde açılır.
UFW `inactive` ise kural eklemek zorunlu değildir. Ana PC Windows ise mevcut
`zed_four_camera_test/enable_main_pc_calibration_firewall.ps1` kullanılır.

## 7. İki hostun saatini senkronla

Kalibrasyon/fusion capture zamanlarını eşleştirdiği için iki host aynı NTP
kaynağına senkron olmalıdır:

```bash
sudo systemctl enable --now chrony
chronyc tracking
timedatectl status
date -u +'%F %T.%3N'
```

Ana PC Windows ise orada yönetici PowerShell:

```powershell
Start-Service W32Time
w32tm /resync /force
w32tm /query /source
w32tm /query /status
```

## 8. Laptop ön kontrolü

Ana PC'de ölçtüğünüz SDK sürümünü `--expected-sdk` ile verin:

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

`ON KONTROL: BASARILI` görülmeden kalibrasyona geçmeyin. Port kullanım hatası
varsa eski kaynakları bulun:

```bash
ss -lunp | grep -E ':(16000|16006|16100|16106|16200|16206)\b'
pgrep -af 'zed_g1_skeleton|ZED_Explorer|ZED360'
```

## 9. Yeni dört-kamera extrinsic kalibrasyonu

Tripodları son konumuna sabitleyin. Ortak çalışma hacminde yalnız bir kişi
olsun. Eski aktif kalibrasyonu silmeyin; tek tarih değişkeniyle arşive taşıyın:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
archive="config/zed_four/archive_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$archive"
test ! -f "four json/fourkamera.json" || \
  mv "four json/fourkamera.json" "$archive/fourkamera.json"
```

### 9.1 Laptopta iki kaynağı aç

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_zed_four_sources_ubuntu.sh \
  --role laptop \
  --main-pc-host 192.168.50.10 \
  --calibration-mode \
  --fps 15 \
  --model medium \
  --depth-mode neural-light
```

Bu terminal açık kalır. İki kameranın çıktısı seri numarasıyla prefixlenir.
Başlangıçta operatör ortak alanda nötr duruşta yaklaşık 4 saniye beklemelidir.

### 9.2 Ana PC'de iki yerel kaynağı aç

Ana PC Ubuntu ise ikinci terminal:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_zed_four_sources_ubuntu.sh \
  --role main-pc \
  --calibration-mode \
  --fps 15 \
  --model medium \
  --depth-mode neural-light
```

Ana PC Windows ise eşdeğeri:

```powershell
.\start_zed_four_sources.ps1 -Role MainPc -CalibrationMode `
  -Fps 15 -Model medium -DepthMode neural-light
```

### 9.3 Ana PC'de ham dört-görüş kaydı al

Ana PC Ubuntu'da üçüncü terminal:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./zed_four_camera_test/start_distributed_receiver_ubuntu.sh \
  --calibration-record \
  ./recordings/four_body38_static_calibration_NEW.jsonl \
  --fps 15 \
  --minimum-sources 4 \
  --preview-hz 10
```

2×2 pencerede dört preview görünmelidir. Her kamerada aynı kişi seçili ve
`LOCKED` olmalı. Konsolda kaynak `4/4`, ham kayıt sayısı sürekli artmalıdır.

45–90 saniye boyunca:

1. Ortak hacmin farklı noktalarında yavaşça yürüyün.
2. 1–2 saniyelik A ve T pozları verin.
3. Dirsekleri bükün; kolları öne, yana ve kontrollü biçimde arkaya götürün.
4. Tüm kameraların aynı kişiyi ve mümkün olduğunca ayakları görmesini sağlayın.
5. En az 400 ortak örneğe ulaşınca `Ctrl+C` ile alıcıyı kapatın.

Tek noktada sabit T-pozu yeterli değildir. İkinci kişi, hareket eden tripod
veya farklı operatöre kilitlenen kamera kaydı geçersiz kılar.

### 9.4 Kalibrasyonu hesapla ve etkinleştir

Ana PC Ubuntu:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./zed_four_camera_test/start_distributed_calibration_ubuntu.sh \
  --input ./recordings/four_body38_static_calibration_NEW.jsonl \
  --output ./config/zed_four/distributed_body38_extrinsics_NEW.json \
  --world-poses ./config/zed_four/four_camera_world_poses_NEW.jsonl \
  --reference-serial 33773329 \
  --max-samples 5000 \
  --activate
```

Kalite kapısı başarısızsa dosya etkinleşmez. Ortamı/görüşü düzeltip yeni kayıt
alın; eşikleri düşürerek kötü kalibrasyonu zorla kabul etmeyin. Başarılıysa:

```text
config/zed_four/active_distributed_body38_extrinsics.json
config/zed_four/active_four_camera_world_poses.jsonl
four json/fourkamera.json
```

Doğrula:

```bash
~/g1_isaaclab_project/envs/zed/bin/python \
  ./zed_four_camera_test/validate_distributed_extrinsics.py \
  --input "./four json/fourkamera.json" \
  --expected-serials 31571870,33773329,34760587,39504762 \
  --reference-serial 33773329
```

Ana PC Windows ise 9.3 ve 9.4'ün PowerShell karşılıkları
`docs/FOUR_ZED_BODY38_RUNBOOK_TR.md` içindedir; Ubuntu laptop kaynakları aynı
UDP sözleşmesini kullandığı için receiver değişmez.

## 10. Kalibrasyon kabul testi

Dört kaynak açıkken ana PC Ubuntu'da GMR olmadan yalnız fusion testi:

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_zed_four_fusion_ubuntu.sh \
  --minimum-sources 4 \
  --fps 15 \
  --preview-hz 10 \
  --skip-gmr-check \
  --record
```

Beklenen:

- bağlı kaynak `4/4`;
- fusion yaklaşık `14–15 FPS`;
- mümkün olduğunca çok karede katkı `4/4`;
- `cross_view_mpjpe_m` düşük, tercihen `0.10 m` altında;
- `gecersiz=0`, ağ drop `0`;
- dört preview'da aynı operatör.

Kayıt özeti:

```bash
latest="$(ls -1t recordings/four_body38_fusion_*.jsonl | head -n1)"
~/g1_isaaclab_project/envs/zed/bin/python \
  ./zed_four_camera_test/summarize_four_body38_recording.py "$latest"
```

Test bitince fusion'da `Q`/`Esc` veya terminalde `Ctrl+C`; sonra kaynaklarda
`Ctrl+C` kullanın.

## 11. Sistemi günlük çalıştırma — iki host da Ubuntu

Sıra önemlidir.

### Terminal 1 — ana PC: GMR + Isaac

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_g1_isaaclab61_live_ubuntu.sh \
  --mode upper_body \
  --imitation-mode kinematic_debug \
  --runtime-profile shared_gpu_safe \
  --accept-nvidia-eula
```

### Terminal 2 — ana PC: Rerun (isteğe bağlı ama önerilir)

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_g1_rerun_ubuntu.sh
```

### Terminal 3 — laptop: iki uzak kaynak

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
git pull --ff-only origin main
./start_zed_four_sources_ubuntu.sh \
  --role laptop \
  --main-pc-host 192.168.50.10 \
  --fps 15 \
  --model medium \
  --depth-mode neural-light
```

### Terminal 4 — ana PC: iki yerel kaynak

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_zed_four_sources_ubuntu.sh \
  --role main-pc \
  --fps 15 \
  --model medium \
  --depth-mode neural-light
```

### Terminal 5 — ana PC: dört-kamera fusion

```bash
cd ~/g1_isaaclab_project/zed-g1-motion-imitation
./start_zed_four_fusion_ubuntu.sh \
  --minimum-sources 3 \
  --fps 15 \
  --preview-hz 10 \
  --record
```

Günlük kullanımda `--minimum-sources 3`, tek kameranın kısa BODY kaybında
akışı sürdürür; ilk kabul testinde daima `4` kullanın. Tripodlar hareket
etmediyse extrinsic tekrar alınmaz. Her süreç başlangıcında kişi nötr durarak
yerel antropometrik kalibrasyonun `READY` olmasını beklemelidir.

## 12. Ana PC Windows kalacaksa günlük sıra

Laptop komutu yine Bölüm 11 Terminal 3'tür. Ana PC'de mevcut komutlar:

```powershell
.\start_g1_rerun.ps1 -Mode live -ListenPort 15052 -LiveMaxHz 15
.\start_g1_isaaclab_live.ps1 -Mode upper_body `
  -ImitationMode kinematic_debug -AcceptNvidiaEula -InputFps 15
.\start_zed_four_sources.ps1 -Role MainPc
.\start_zed_four_fusion_to_wsl.ps1 `
  -Record -MinimumSources 3 -FusionHz 15 -PreviewHz 10
```

Windows receiver açısından laptop paketlerinin Windows veya Ubuntu'dan
gelmesi fark etmez; `source_host_id=Laptop`, seri numarası ve portlar aynı
kalır.

## 13. Sorun çözme

### Ana PC'de laptop kaynakları yok

Laptopta:

```bash
ping -c 4 192.168.50.10
ip route get 192.168.50.10
sudo tcpdump -ni any 'udp port 16000 or udp port 16006'
```

Ana PC Ubuntu'da:

```bash
ss -lunp | grep -E ':(16000|16006|16100|16106)\b'
sudo tcpdump -ni any 'host 192.168.50.11 and udp'
```

Laptopta paket çıkıyor, ana PC'de görünmüyorsa kablo/VLAN/firewall; ana PC'de
paket görünüyor ama receiver saymıyorsa seri-port sözleşmesini kontrol edin.

### `AVAILABLE` olmayan kamera

```bash
pgrep -af 'ZED|zed_g1_skeleton|python'
lsusb -t
dmesg --ctime | tail -n 80
```

ZED Explorer/başka Python sürecini kapatın; kabloyu/USB portunu değiştirin.
İki kamerayı birlikte 60 saniye 15 FPS test etmeden 30 FPS'e geçmeyin.

### FPS düşük veya USB kopuyor

- `15 FPS` ve `neural-light` profilinde kalın.
- İki kamerayı farklı USB controller'a taşıyın.
- Pasif hub kullanmayın; adaptörlü USB 3.x hub kullanın.
- Önce her kamerayı ayrı, sonra birlikte test edin.
- ZED Diagnostic temiz değilse calibration almayın.

### Fusion `3/4`te kalıyor

2×2 preview'da eksik kamerayı bulun. Kaynak terminalinde BODY kilidi,
confidence, mesafe kapısı ve nötr kalibrasyonu kontrol edin. Kalibrasyon
kaydında `4/4` zorunludur; günlük runtime'da `3/4` kısa süreli fallback'tir.

### Kalibrasyon kalite kapısını geçmiyor

Odada yalnız tek kişi bırakın, dört görüşte aynı kişiyi doğrulayın, 45–90
saniyelik hareketli kayıt alın ve tripodları sabitleyin. Eski kayıt üzerinde
eşikleri gevşetmeyin.

## Resmî referanslar

- [Stereolabs Linux ZED SDK kurulumu](https://docs.stereolabs.com/docs/development/zed-sdk/linux)
- [ZED SDK sürüm indirmeleri](https://www.stereolabs.com/developers/release/)
- [ZED çoklu kamera kurulumu](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/multi-camera)
- [ZED Body Tracking API](https://docs.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api)
