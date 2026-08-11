# Yeni bilgisayarda ZED 2i → G1 23-DOF projesi

Bu belge projeyi boş bir Windows 11 bilgisayara kurup tek veya iki ZED 2i ile
Unitree G1 EDU 23-DOF üst-gövde taklidini Isaac Lab, Rerun ve isteğe bağlı
ROS 2/RViz ile çalıştırmak içindir. Varsayılan akış yalnız simülasyondur;
fiziksel G1 motorlarına komut göndermez.

## 1. Doğrulanmış mimari ve sabit sürümler

| Katman | Konum | Sürüm / kaynak |
|---|---|---|
| ZED BODY_38 ve Fusion | Windows | ZED SDK 5.4, resmî Windows SDK |
| Kamera Python ortamı | Windows | Ayrı `.venv-zed` |
| Isaac Sim | Windows | 5.0.0 |
| Isaac Lab | Windows | kilitli commit `46dff135...` |
| Unitree varlıkları | Windows | `unitree_rl_lab`, `unitree_ros`, `unitree_sim_isaaclab` |
| GMR | WSL Ubuntu 22.04 | kilitli commit `bb1bbe40...` |
| ROS 2 / RViz | WSL Ubuntu 22.04 | Humble |
| MuJoCo | WSL Ubuntu 22.04 | resmî `unitree_mujoco` kilitli commit |

Sabit commitlerin tam listesi
`isaaclab_bridge/environment_lock.json` dosyasındadır. Proje çalıştıktan sonra
bu sürümleri topluca yükseltmeyin; her yükseltmeyi kayıt replay testiyle ayrı
doğrulayın.

## 2. Donanım

Asgari pratik yapı:

- Windows 11 x64;
- NVIDIA RTX GPU, tercihen en az 16 GB VRAM;
- 32 GB RAM, Isaac + iki BODY_38 için tercihen 64 GB;
- en az 100 GB boş SSD;
- ZED 2i başına doğrudan anakart USB 3.x portu;
- iki kamera için mümkünse farklı USB host controller/root hub;
- kameraları sabitlemek için rijit tripod veya aparat.

İki kamerayı pasif bir USB hub'a bağlamayın. ZED Explorer'da kare kaybı varsa
önce portları değiştirin ve Windows Aygıt Yöneticisi'nde USB güç tasarrufunu
kapatın. Isaac ve ZED aynı GPU'yu kullandığında dual için önce 30 FPS profilini
kullanın; 60 FPS deneysel profildir.

## 3. Windows'a indirilecekler

Bu sırayı izleyin:

1. Güncel NVIDIA Studio/Production sürücüsü:
   <https://www.nvidia.com/Download/index.aspx>
2. Git for Windows: <https://git-scm.com/download/win>
3. GitHub CLI: <https://cli.github.com/>
4. Python 3.11 x64: <https://www.python.org/downloads/windows/>
   Kurucuda **Add Python to PATH** ve **py launcher** seçeneklerini açın.
5. ZED SDK 5.4 Windows/CUDA paketi:
   <https://www.stereolabs.com/developers/release/>
6. Gerekirse Microsoft Visual C++ 2015–2022 x64 runtime. Eski
   `ZED_BodyFusion.exe` örneği `MSVCR120.dll` isterse ayrıca Visual C++ 2013
   x64 runtime kurulur; projenin Python Fusion yolu bu eski örneğe bağlı değildir.

ZED SDK kurulurken uyumlu CUDA bulunmazsa resmî kurucu CUDA indirmeyi teklif
eder. Kurulumdan sonra Windows'u yeniden başlatın. ZED Depth Viewer ile her
kamerayı ayrı ayrı doğrulayın.

## 4. WSL2 Ubuntu 22.04

Yönetici PowerShell:

```powershell
wsl --install -d Ubuntu-22.04
wsl --update
wsl --set-default-version 2
```

Bilgisayarı yeniden başlatın. Ubuntu 22.04'ü ilk kez açıp Linux kullanıcı adı
ve parolasını oluşturun. Kontrol:

```powershell
wsl -l -v
```

`Ubuntu-22.04` satırında `VERSION 2` görünmelidir.

## 5. GitHub'dan projeyi alma

PowerShell:

```powershell
gh auth login
gh auth setup-git
cd $HOME\Desktop
gh repo clone mesutanlak/zed-g1-motion-imitation
cd .\zed-g1-motion-imitation
```

Dual Fusion değişiklikleri henüz `main` dalına birleştirilmediyse:

```powershell
git fetch origin
git checkout codex/mirror-akc-g1-23dof
git pull --ff-only
```

Klasörün adı veya Windows kullanıcı adı değişebilir; temel başlatıcılar proje
dizinini otomatik hesaplar.

## 6. Ön kontrol

```powershell
powershell -ExecutionPolicy Bypass -File .\install\check_prerequisites.ps1
```

Eksik satır kalmadan ilerleyin. GPU kontrolü ayrıca:

```powershell
nvidia-smi
```

## 7. Windows ZED Python ortamı

Kamera ortamını Isaac ortamından ayrı kurun:

```powershell
powershell -ExecutionPolicy Bypass -File .\install\setup_zed_windows.ps1
```

Tek/çift kamera görünürlüğü:

```powershell
.\.venv-zed\Scripts\python.exe .\zed_g1_skeleton.py --list-devices
```

Tek kamera için en az bir, dual için iki `state=AVAILABLE` satırı gerekir.
ZED Explorer/Depth Viewer/BodyFusion bu testten önce kapalı olmalıdır.

## 8. WSL GMR retargeting ortamı

```powershell
powershell -ExecutionPolicy Bypass -File .\install\setup_gmr_wsl.ps1
```

Kurulum `$HOME/g1_isaaclab_project/envs/gmr_zed` oluşturur ve upstream GMR'yi
kilitli commit'e getirir. Windows kullanıcı adından ve Linux home adından
bağımsızdır.

## 9. Windows Isaac Sim 5.0 + Isaac Lab

NVIDIA Omniverse lisansını okuyun. Kabul ediyorsanız:

```powershell
powershell -ExecutionPolicy Bypass -File .\install\setup_isaac_windows.ps1 `
  -AcceptNvidiaEula
```

Kurulum varsayılan olarak `C:\g1il` altındadır. Yaklaşık onlarca GB indirir ve
ilk shader/URDF→USD hazırlığı uzun sürebilir. Kurulum bitmeden pencereyi
kapatmayın.

Önce headless doğrulayın:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 `
  -Mode upper_body -AcceptNvidiaEula -Headless -MaxSteps 100
```

Ardından GUI kullanılabilir. İlk GUI açılışında `G1 scene initialization
complete` görülene kadar bekleyin.

## 10. Tek kamera ile çalıştırma

İki ayrı PowerShell açın.

Terminal 1 — GMR + Isaac:

```powershell
cd $HOME\Desktop\zed-g1-motion-imitation
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 `
  -Mode upper_body -AcceptNvidiaEula
```

Terminal 2 — tek ZED 2i:

```powershell
cd $HOME\Desktop\zed-g1-motion-imitation
powershell -ExecutionPolicy Bypass -File .\start_zed_live_smooth_to_wsl.ps1 `
  -Profile realtime_60
```

GPU yükünde FPS düşerse `-Profile balanced_30` veya `shared_gpu_safe`
kullanın. `S` JSONL kaydını açar/kapatır, `R` operatör/kalibrasyon/kol
belleğini sıfırlar, `Q` çıkar.

## 11. İki kamera ile çalıştırma

Önce iki kameranın seri numaralarının kalibrasyon dosyasıyla eşleştiğini
kontrol edin. Bu depodaki doğrulanmış dosya:

```text
config\zed_dual\calibration_33773329_39504762.json
```

Terminal 1 yine GMR + Isaac'tır. Terminal 2:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_zed_dual_fusion_to_wsl.ps1 `
  -Profile dual_balanced_30 -Record
```

GPU/USB yükü yüksekse:

```powershell
.\start_zed_dual_fusion_to_wsl.ps1 -Profile dual_safe_15
```

Kameralardan biri diğerine göre hareket eder veya dönerse ZED360 ile yeniden
extrinsic kalibrasyon yapın. Yeni dosya kod değişmeden seçilir:

```powershell
.\start_zed_dual_fusion_to_wsl.ps1 `
  -FusionConfig "C:\kalibrasyonlar\yeni_dual.json" `
  -Profile dual_balanced_30
```

Tek ve dual ZED kaynaklarını aynı anda açmayın; ikisi de GMR UDP `15050`
portuna yayın yapar.

## 12. Rerun analiz paneli

```powershell
.\.venv-zed\Scripts\python.exe -m pip install -r .\requirements-rerun.txt
powershell -ExecutionPolicy Bypass -File .\start_g1_rerun.ps1
```

Rerun kontrol yolundan ayrıdır. GPU zorlanırsa paneli kapatıp önce Isaac/ZED
etkili FPS ve gecikmesini doğrulayın.

## 13. ROS 2 Humble ve RViz (isteğe bağlı)

Ubuntu terminalinde:

```bash
cd /mnt/c/Users/<WINDOWS_KULLANICI>/Desktop/zed-g1-motion-imitation
bash install/setup_ros2_humble_wsl.sh
```

Yeni terminal açın veya `source /opt/ros/humble/setup.bash` çalıştırın.
PowerShell'de:

```powershell
.\start_g1_full_rviz.ps1
```

ZED kaynağı ayrıca açık olduğunda BODY_38, insan/G1 raw/G1 safe hedefleri,
robot mesh/TF ve yapılandırılmış sensör topic'leri RViz'de izlenebilir.

## 14. MuJoCo (isteğe bağlı)

Ubuntu terminalinde:

```bash
cd /mnt/c/Users/<WINDOWS_KULLANICI>/Desktop/zed-g1-motion-imitation
bash install/setup_mujoco_wsl.sh
```

PowerShell:

```powershell
.\start_mujoco_mimic.ps1
```

Isaac ve MuJoCo süreçlerini aynı anda kullanmayın; önceki simülatörü kapatın.

## 15. Tam doğrulama

```powershell
powershell -ExecutionPolicy Bypass -File .\install\verify_project.ps1
```

Canlı kamera olmadan şema ve motion pipeline testleri:

```powershell
.\.venv-zed\Scripts\python.exe .\zed_g1_skeleton.py --self-test
powershell -ExecutionPolicy Bypass -File .\validate_motion_pipeline.ps1
```

Dual kayıt sonrası tek/çift karşılaştırması:

```powershell
python .\tools\compare_single_dual_body38.py `
  --single .\recordings\zed_body38_SINGLE.jsonl `
  --dual .\recordings\zed_dual_body38_DUAL.jsonl `
  --output .\analysis\single_vs_dual
```

## 16. Portlar ve başlatma sırası

| UDP | Üretici → tüketici |
|---:|---|
| 15050 | Windows ZED → WSL GMR |
| 15051 | WSL GMR → Windows Isaac |
| 15052 | ZED → analiz/Rerun |
| 15053 | Isaac/GMR telemetrisi → Rerun/ROS |
| 15054 | ZED BODY_38 → ROS 2 köprüsü |

Doğru sıra: **Isaac/GMR → analiz paneli (isteğe bağlı) → tek veya dual ZED**.

## 17. GitHub'a dahil edilmeyen veriler

`.gitignore` aşağıdakileri özellikle dışarıda tutar:

- `.svo/.svo2`, kamera videoları ve ham insan görüntüsü;
- `recordings/`, `rerun_recordings/`, `analysis/` ve çalışma logları;
- diagnostic JSON dosyaları;
- Python sanal ortamları, CUDA/SDK paketleri ve makineye özel `.env` dosyaları.

Yeni bilgisayara ham deney verileri gerekiyorsa Git LFS yerine erişimi
kontrollü kurum depolaması veya şifreli harici disk kullanın.

## 18. Sık hatalar

- **Yalnız `/parameter_events` ve `/rosout`:** ROS köprü node'u çalışmıyor;
  önce `start_g1_full_rviz.ps1`, sonra ZED kaynağını açın.
- **UDP 15050 dinleyicisi yok:** önce `start_g1_isaaclab_live.ps1` açılmalıdır.
- **ZED `NOT AVAILABLE`:** Explorer/Depth Viewer'ı kapatın, USB portunu ve
  kabloyu değiştirin, yalnız bir kamera ile test edin.
- **Dual kare kaybı:** `dual_safe_15`, farklı USB controller ve daha düşük
  Rerun/ROS yayın hızı kullanın.
- **Isaac GUI device-lost:** doğrulanmış NVIDIA sürücüsüne dönün veya
  `-Headless` test edin; `-AllowUnvalidatedDriver` kalıcı çözüm değildir.
- **Kalibrasyondan sonra iskelet kayık:** kameralar yer değiştirmiştir; ZED360
  extrinsic kalibrasyonu yeniden yapılmalıdır.

## Resmî kaynaklar

- ZED Windows kurulumu: <https://www.stereolabs.com/docs/installation/windows/>
- ZED gereksinimleri: <https://docs.stereolabs.com/docs/development/zed-sdk/specifications>
- ZED BODY_38: <https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api>
- ZED Fusion: <https://docs.stereolabs.com/docs/development/zed-sdk/modules/fusion>
- ZED360: <https://www.stereolabs.com/docs/fusion/zed360>
- Microsoft WSL: <https://learn.microsoft.com/windows/wsl/install>
- ROS 2 Humble: <https://docs.ros.org/en/humble/Installation.html>
- Isaac Lab: <https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html>
- Unitree RL Lab: <https://github.com/unitreerobotics/unitree_rl_lab>
- Unitree Isaac Lab: <https://github.com/unitreerobotics/unitree_sim_isaaclab>
- GMR: <https://github.com/YanjieZe/GMR>
- Rerun: <https://rerun.io/docs/getting-started/installing-viewer>
