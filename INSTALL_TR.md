# Yeni bilgisayara eksiksiz kurulum

Bu belge ZED 2i BODY_38 verisini GMR ile G1 EDU 23-DOF referansına
dönüştüren ve Isaac Lab/MuJoCo/RViz üzerinde gösteren projeyi Windows 11 +
WSL2 Ubuntu 22.04 bilgisayara kurar.

## 1. Donanım ve işletim sistemi

Önerilen:

- Windows 11 x64 ve WSL2 Ubuntu 22.04
- NVIDIA RTX GPU; Isaac Lab için 16 GB veya daha fazla VRAM
- 32 GB veya daha fazla RAM
- ZED 2i için doğrudan anakart USB 3.x portu
- En az 100 GB boş SSD alanı

ZED ve Isaac Sim aynı GPU'yu kullandığı için önce NVIDIA Production Branch
sürücüsünü kurup bilgisayarı yeniden başlatın. ZED SDK gerçek zamanlı depth
hesabı için CUDA/NVIDIA GPU gerektirir.

## 2. Manuel olarak kurulması gerekenler

1. Git for Windows: <https://git-scm.com/download/win>
2. Python 3.11 x64: <https://www.python.org/downloads/windows/>
3. WSL2 Ubuntu 22.04, yönetici PowerShell:

   ```powershell
   wsl --install -d Ubuntu-22.04
   ```

   Yeniden başlatın, Ubuntu'yu bir kez açıp Linux kullanıcı adı/parolası
   oluşturun.

4. NVIDIA sürücüsü: <https://www.nvidia.com/Download/index.aspx>
5. ZED SDK Windows kurulucusu:
   <https://www.stereolabs.com/developers/release/>

   GPU sürücüsüyle uyumlu Windows sürümünü seçin. Kurulum bitince yeniden
   başlatın. ZED Depth Viewer içinde kameranın çalıştığını doğrulayın.

6. GitHub'dan projeyi klonlayın:

   ```powershell
   cd $HOME\Desktop
   git clone https://github.com/mesutanlak/zed-g1-motion-imitation.git
   cd .\zed-g1-motion-imitation
   ```

## 3. Ön kontrol

```powershell
powershell -ExecutionPolicy Bypass -File .\install\check_prerequisites.ps1
```

Tüm satırlar `OK` olmadan devam etmeyin.

## 4. Windows ZED Python ortamı

ZED uygulaması Isaac ortamından ayrı tutulur:

```powershell
powershell -ExecutionPolicy Bypass -File .\install\setup_zed_windows.ps1
```

Kamera testi:

```powershell
.\.venv-zed\Scripts\python.exe .\zed_g1_skeleton.py --list-devices
```

`id=... serial=... state=AVAILABLE` görülmelidir.

## 5. WSL GMR retargeting ortamı

```powershell
powershell -ExecutionPolicy Bypass -File .\install\setup_gmr_wsl.ps1
```

Betik Ubuntu 22.04 içinde Python 3.10 venv oluşturur, resmi GMR deposunu
kilitli commit'e getirir ve `mujoco/mink` bağımlılıklarını kurar.

## 6. Windows Isaac Sim 5.0 + Isaac Lab

Önce NVIDIA Omniverse EULA'yı okuyun. Kabul ediyorsanız:

```powershell
powershell -ExecutionPolicy Bypass -File .\install\setup_isaac_windows.ps1 `
  -AcceptNvidiaEula
```

Bu işlem `C:\g1il` altında Python 3.11 ortamı kurar ve şu resmi depoları
kilitli commitlerle indirir:

- NVIDIA Isaac Lab
- Unitree RL Lab
- Unitree ROS robot tanımları

İlk kurulum ve ilk RTX shader/URDF→USD dönüşümü uzun sürebilir.

## 7. ROS 2 Humble ve RViz (isteğe bağlı)

Ubuntu terminalinde:

```bash
cd /mnt/c/Users/<WINDOWS_KULLANICI>/Desktop/zed-g1-motion-imitation
bash install/setup_ros2_humble_wsl.sh
```

Ardından iki ayrı PowerShell:

```powershell
.\start_g1_body38_ros_bridge.ps1
.\view_g1_body38_rviz.ps1
```

MuJoCo ve engel parkuru da kullanılacaksa Ubuntu terminalinde:

```bash
cd /mnt/c/Users/<WINDOWS_KULLANICI>/Desktop/zed-g1-motion-imitation
bash install/setup_mujoco_wsl.sh
```

Başlatma:

```powershell
.\start_g1_zed_obstacle_sim.ps1
.\view_g1_sensors_rviz.ps1
```

## 8. Kurulum doğrulaması

```powershell
powershell -ExecutionPolicy Bypass -File .\install\verify_project.ps1
```

## 9. Canlı çalıştırma sırası

Üç ayrı PowerShell açın.

1. Isaac Lab + GMR:

   ```powershell
   .\start_g1_isaaclab_live.ps1 -Mode upper_body -AcceptNvidiaEula
   ```

2. 3B iskelet analiz paneli:

   ```powershell
   .\start_g1_skeleton_panel_wsl.ps1 -Record
   ```

3. ZED canlı yayın:

   ```powershell
   .\start_zed_live_smooth_to_wsl.ps1 -Profile shared_gpu_safe
   ```

ZED betiğinin sistem Python'u yerine özel ortamı kullanması gerekirse önce:

```powershell
.\.venv-zed\Scripts\Activate.ps1
```

## 10. Veri ve güvenlik

- Raw SVO2/JSONL kayıtları GitHub'a yüklenmez; insan görüntüsü içerebilir.
- `datasets/*.npz` simülasyona uygun, küçük, türetilmiş 23-DOF verilerdir.
- Sistem varsayılan olarak `upper_body` kullanır; bacaklar denge policy'sinde
  tutulur.
- Fiziksel G1'e bu repodan doğrudan motor komutu gönderilmez.
- Sim2real öncesinde eklem limitleri, hız/tork sınırları, self-collision,
  watchdog, düşme engelleme, askı ve fiziksel acil durdurma zorunludur.

## 11. İsteğe bağlı Rerun 3B analiz aracı

Mevcut iskelet panelinden bağımsız Rerun aracı için:

```powershell
python -m pip install -r .\requirements-rerun.txt
powershell -ExecutionPolicy Bypass -File .\start_g1_rerun.ps1
```

Ayrıntılar: [RERUN_3D_ANALIZ.md](RERUN_3D_ANALIZ.md)

## Resmi kaynaklar

- ZED Windows kurulumu:
  <https://www.stereolabs.com/docs/installation/windows/>
- ZED sistem gereksinimleri:
  <https://docs.stereolabs.com/docs/development/zed-sdk/specifications>
- ZED BODY_38:
  <https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api>
- Isaac Lab kurulumu:
  <https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index>
- Unitree RL Lab:
  <https://github.com/unitreerobotics/unitree_rl_lab>
- Unitree Isaac Lab simülasyonu:
  <https://github.com/unitreerobotics/unitree_sim_isaaclab>
- GMR:
  <https://github.com/YanjieZe/GMR>
- Rerun Viewer ve Python SDK:
  <https://rerun.io/docs/getting-started/configure-the-viewer>
