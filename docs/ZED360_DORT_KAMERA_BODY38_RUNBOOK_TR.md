# 4× ZED 2i: ZED360 kalibrasyonu, BODY_38 Fusion ve G1/Isaac akışı

Tarih: 2026-09-01  
Ana PC: `192.168.50.10` — ZED `33773329`, `31571870`  
Laptop: `192.168.50.11` — ZED `39504762`, `34760587`

Bu sırayı bozmayın. Her bölümün **Kabul** satırı sağlanmadan sonraki bölüme
geçmeyin. ZED Explorer, ZED360 ve Python aynı kamerayı aynı anda açamaz.

## 0. Bugünkü teşhislerin sonucu

Dört tanı raporunda da kamera ZED 2i, ZED SDK 5.4.1, USB mode 3 ve USB
bandwidth OK. Hata kaydı yok. Buna rağmen her rapor tek kamerayı sınadığı için
iki kameranın aynı hostta 60 saniye birlikte 15 FPS çalışması ayrıca kabul
edilecektir.

Kameraların fiziksel konumları 2026-08-31 kaydından sonra değiştiği için önceki
extrinsic ve dünya-pozu JSONL'si **geçersizdir**. Yanlışlıkla runtime'da
seçilmemeleri için silinmeden şuraya arşivlenmiştir:

- `config\zed_four\archive_20260831_moved_rig\distributed_body38_extrinsics.json`
- `config\zed_four\archive_20260831_moved_rig\four_camera_world_poses.jsonl`

Önceki fit sonuçlarının iyi olması yeni pozlara taşınamaz. Bugünkü düzen için
ZED360 veya Bölüm 5 ile yeni kalibrasyon zorunludur.

## 1. İki bilgisayarı ve Ethernet'i doğrula

Önce tüm ZED uygulamalarını iki bilgisayarda kapatın. Her yeni PowerShell'de:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
```

### 1.1 Ana PC

```powershell
cd "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"

Get-NetAdapter -Name Ethernet | Format-Table Name,Status,LinkSpeed
Get-NetIPAddress -InterfaceAlias Ethernet -AddressFamily IPv4 |
  Format-Table InterfaceAlias,IPAddress,PrefixLength
ping -n 8 -l 1400 192.168.50.11

& .\.venv-zed\Scripts\python.exe .\zed_g1_skeleton.py --list-devices
& .\.venv-zed\Scripts\python.exe -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"
```

Beklenen: Ethernet `Up`, tercihen `2.5 Gbps`, `192.168.50.10/24`, 8/8 ping,
seri `33773329` ve `31571870` AVAILABLE, SDK `5.4.1`.

Yeni ön-kontrol aracı aynı kontrolleri tek komutta da yapar:

```powershell
.\zed_four_camera_test\test_four_camera_preflight.ps1 `
  -Role MainPC -ExpectedIPv4 192.168.50.10 -PeerIPv4 192.168.50.11 `
  -Serial 33773329,31571870
```

### 1.2 Laptop

```powershell
cd "C:\Users\MSI\Desktop\zed-g1-motion-imitation"

Get-NetAdapter -Name Ethernet | Format-Table Name,Status,LinkSpeed
Get-NetIPAddress -InterfaceAlias Ethernet -AddressFamily IPv4 |
  Format-Table InterfaceAlias,IPAddress,PrefixLength
ping -n 8 -l 1400 192.168.50.10

& .\.venv-zed\Scripts\python.exe .\zed_g1_skeleton.py --list-devices
& .\.venv-zed\Scripts\python.exe -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"
```

Beklenen: `192.168.50.11/24`, 8/8 ping, seri `39504762` ve `34760587`
AVAILABLE, SDK `5.4.1`.

### 1.3 Saat ve güvenlik duvarı

İki bilgisayarda da yönetici PowerShell açıp mevcut ortak saat kaynağına yeniden
senkron olun:

```powershell
Start-Service W32Time
w32tm /resync /force
w32tm /query /source
w32tm /query /status
Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"
```

Kaynaklar farklıysa iki bilgisayarı aynı NTP kaynağına ayarlayın. Stereolabs
dağıtık Fusion için PTP önerir; Windows'taki bu NTP kontrolü bugünkü kabul
testidir, donanımsal senkron değildir.

Ethernet profilini iki hostta da `Private` yapın:

```powershell
Get-NetConnectionProfile -InterfaceAlias Ethernet
Set-NetConnectionProfile -InterfaceAlias Ethernet -NetworkCategory Private
```

Norton kullanılıyorsa Norton içinde aşağıdaki uygulamalara Özel Ağ izni verin:

- `C:\Program Files (x86)\ZED SDK\tools\ZED360.exe` (ana PC)
- Projedeki `.venv-zed\Scripts\python.exe` (iki bilgisayar)

Defender etkin olan makinede yönetici PowerShell ile yalnız Özel Ağ için UDP
kuralları eklenebilir:

```powershell
New-NetFirewallRule -DisplayName "ZED Fusion UDP Private" `
  -Direction Inbound -Action Allow -Protocol UDP `
  -LocalPort 30000-30039,16000,16002,16004,16006 -Profile Private
```

Kural zaten varsa ikinci kez oluşturmayın. Antivirüsü kalıcı kapatmayın.

**Kabul:** iki yönde 8/8 büyük ping; doğru iki seri her hostta AVAILABLE; iki
host SDK 5.4.1; saatler aynı kaynağa senkron.

## 2. ZED360 ağ denemesi: teşhis tamamlandı

> **2026-09-01 sonucu:** Hibrit `Load` yolu yerel kameraları açamadı. Dört
> kameranın tamamı BODY_18 (`received_format=0`) publisher yapılıp ZED360'ın
> manuel ağ ekranına eklendiğinde dört kaynak da 14.999 FPS, `Publisher: 4.000`
> ve `FUSION: SUCCESS` verdi; buna rağmen `enableSK` çağrısı
> `ERROR : WRONG BODY FORMAT` ile kapandı. Publisher ayarları resmî örnekle
> aynıdır (`BODY_18`, tracking kapalı, fitting kapalı). Bu nedenle Bölüm 2–4
> artık yalnız tekrar üretim kaydıdır; mevcut SDK 5.4.1 kurulumunda tekrar
> denenmemeli, doğrudan Bölüm 5'teki BODY_38 uygulama Fusion'una geçilmelidir.

Topoloji:

```text
Laptop 39504762 ─ LOCAL_NETWORK 192.168.50.11:30000 ┐
Laptop 34760587 ─ LOCAL_NETWORK 192.168.50.11:30010 ┤
PC     33773329 ─ INTRA_PROCESS (ZED360 açar)        ├─ ZED360 / ana PC
PC     31571870 ─ INTRA_PROCESS (ZED360 açar)        ┘
```

Bu yapı Stereolabs'ın resmi Python Fusion örneğindeki yerel + ağ kamera
ayrımını izler. Ana PC kameraları için ayrıca publisher açmayın.

### 2.1 Laptopta yalnız iki BODY_18 publisher aç

Laptopta tek PowerShell:

```powershell
cd "C:\Users\MSI\Desktop\zed-g1-motion-imitation"
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force

.\zed_four_camera_test\start_publisher.ps1 `
  -Camera "39504762:30000","34760587:30010" `
  -Fps 15 -Model medium -BodyFormat body18 -DepthMode neural-light
```

Her saniye iki kamera için şunları doğrulayın:

- `publisher_fps` yaklaşık 15.0
- `received_format=0` (BODY_18)
- ortak alanda durduğunuzda `bodies=1`
- hiçbir `Corrupted frames`, kapanma veya yırtılma yok

Bu pencere açık kalacak.

### 2.2 Ana PC'de ZED360'ı terminalden aç

Ana PC'de ZED Explorer ve tüm publisher/source pencereleri kapalı olmalı.

```powershell
cd "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
& "C:\Program Files (x86)\ZED SDK\tools\ZED360.exe"
```

Terminali kapatmayın; `WRONG BODY FORMAT` gibi mesajlar burada görünür.

Settings ekranında:

- Depth Mode: `NEURAL LIGHT`
- Body Model: `MEDIUM`
- Detection Threshold: `40`
- `Load` düğmesiyle şu dosyayı seçin:
  `config\zed_four\zed360_hybrid_calibration_seed.json`

**Load'dan sonra Auto Discover'a basmayın.** Auto Discover listeyi yalnız ana
PC'nin fiziksel USB kameralarıyla yeniden kurduğu için elle/ağdan eklenen iki
laptop kamerası kaybolur. Camera List'te dört seri de görünmeli. Ardından
`Setup the room` düğmesine basın.

İlk ekranda dört kamera ikonunun üst üste olması hata değildir: seed dosyası
bilerek sıfır translation içerir. Pozlar kalibrasyon sırasında ayrılmalıdır.

### 2.3 ZED360 kalibrasyonu

1. Görüş alanında yalnız bir kişi olsun.
2. `Start Calibration` düğmesine basın.
3. Dört kameranın ortak kesişiminde başlayın; sonra tüm çalışma hacmini yavaşça
   dolaşın. Kameralara 1.5–4 m uzakta, tam beden ve ayak bilekleri görünür olsun.
4. Her yaklaşık 10 saniyede optimizasyon bekleyin. En az 60–120 saniye devam
   edin; yalnız ortada kalmayın.
5. Dört kamerada Ratio Detection sıfırdan büyük olmalı, kamera ikonları dünya
   konumlarına ayrılmalı ve ham/fused iskelet görünmelidir.
6. `Finish Calibration` ile kaydedin:

```text
C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation\config\zed_four\zed360_calibrated_4cam.json
```

**Kabul:** terminalde format hatası yok; dört kamera aktif; fused iskelet
görünüyor; kaydedilen JSON'da kamera pozları sıfır değil.

### 2.4 ZED360 hata kararı

- `No opened camera`: ana PC'de ZED Explorer/Python kamerayı tutuyordur. Tümünü
  kapatın, iki yerel kamerayı çıkarıp tekrar takın, ZED360'ı yeniden açın.
- Uzak kamera 0 FPS: laptop publisher, IP, Norton ve `30000/30010` kontrolü.
- Dört publisher 15 FPS ve aynı `received_format=0` iken terminalde
  `WRONG BODY FORMAT`: bir kez temiz kapat–çıkar/tak–yeniden başlat deneyin.
  Aynı hata tekrarlanırsa ZED360 ağ yolu bu SDK/kurulumda başarısız kabul edilir;
  rastgele BODY_18/34/38 kombinasyonlarını tekrar tekrar denemeyin ve Bölüm 5'e
  geçin.

## 3. ZED360 başarılıysa BODY_38 ile resmî Fusion kabul testi

Önce ZED360'ı ve laptop BODY_18 publisher'ını kapatın. Laptopta publisher'ı
BODY_38 olarak yeniden açın:

```powershell
.\zed_four_camera_test\start_publisher.ps1 `
  -Camera "39504762:30000","34760587:30010" `
  -Fps 15 -Model medium -BodyFormat body38 -DepthMode neural-light
```

Beklenen `received_format=2` ve kişi varken `bodies=1`.

Ana PC'de kabul testi; bu komut yerel iki ZED'i kendisi açar ve G1'e komut
göndermez:

```powershell
.\zed_four_camera_test\start_fusion_test.ps1 `
  -FusionConfig ".\config\zed_four\zed360_calibrated_4cam.json" `
  -LocalSerial 33773329,31571870 `
  -RemoteCamera "39504762@192.168.50.11:30000","34760587@192.168.50.11:30010" `
  -Fps 15 -Model medium -DepthMode neural-light `
  -MinimumCameras 2 -Duration 120 `
  -Record ".\recordings\zed_sdk_four_body38_fused.jsonl"
```

**Kabul:** `cameras_present=4/4`, Fusion FPS yaklaşık 15, kişi varken
`body_frames>0` ve terminalde `WRONG BODY FORMAT` yok. BODY_38 ağ Fusion burada
da format hatası verirse Bölüm 5 kesin yoldur.

## 4. ZED360 pozlarını BODY_38/G1 dünya dosyasına çevir

Bu bölüm yalnız Bölüm 2 başarılı ve `zed360_calibrated_4cam.json` gerçekten
oluştuysa çalıştırılır. Seed dosyasını verirseniz dönüştürücü kasıtlı olarak
reddeder.

```powershell
.\zed_four_camera_test\convert_zed360_extrinsics.ps1 `
  -InputConfig ".\config\zed_four\zed360_calibrated_4cam.json" `
  -OutputExtrinsics ".\config\zed_four\zed360_body38_extrinsics.json" `
  -WorldPosesJsonl ".\config\zed_four\zed360_four_camera_world_poses.jsonl" `
  -ReferenceSerial 33773329
```

SDK dosyayı `RIGHT_HANDED_Z_UP_X_FWD` ve metre birimine çevirir. JSONL'nin ilk
satırı metadata, sonraki dört satırı her kameranın seri numarası, optik merkez
dünya konumu, quaternion ve 4×4 camera-to-world matrisidir.

## 5. ZED360 olmazsa sağlam BODY_38 uygulama Fusion'u

Bu yol `sl.Fusion` ağ format hatasını atlar; her ZED yine Stereolabs BODY_38
çıkarımını yapar. Ana PC ağ paketlerini kendi varış saatine göre hizalar,
extrinsic ile ortak dünyaya taşır ve güven ağırlıklı/aykırı-değer dayanıklı
joint Fusion uygular.

### 5.1 Ana PC alıcısını kayıt modunda önce aç

```powershell
cd "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force

.\zed_four_camera_test\start_distributed_receiver.ps1 `
  -Source "39504762:16000","31571870:16002","33773329:16004","34760587:16006" `
  -CalibrationRecord ".\recordings\four_body38_static_calibration_20260901.jsonl" `
  -Fps 15 -MinimumSources 4 -MaxSyncMs 110 -SourceTimeoutMs 750
```

### 5.2 Dört kaynağı ayrı PowerShell pencerelerinde aç

Laptop pencere 1:

```powershell
cd "C:\Users\MSI\Desktop\zed-g1-motion-imitation"
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 39504762 -TargetHost 192.168.50.10 -TargetPort 16000 `
  -Fps 15 -Model medium -DepthMode neural-light -FrameIntegrityMode off
```

Laptop pencere 2:

```powershell
cd "C:\Users\MSI\Desktop\zed-g1-motion-imitation"
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 34760587 -TargetHost 192.168.50.10 -TargetPort 16006 `
  -Fps 15 -Model medium -DepthMode neural-light -FrameIntegrityMode off
```

Ana PC pencere 2:

```powershell
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 33773329 -TargetHost 127.0.0.1 -TargetPort 16004 `
  -Fps 15 -Model medium -DepthMode neural-light -FrameIntegrityMode off
```

Ana PC pencere 3:

```powershell
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 31571870 -TargetHost 127.0.0.1 -TargetPort 16002 `
  -Fps 15 -Model medium -DepthMode neural-light -FrameIntegrityMode off
```

Tanılar bugün temiz olduğu için yanlış-pozitif piksel uyarısı üreten ek
heuristic kapalıdır. Gerçek görüntü yırtılması görürseniz testi durdurup ZED
Diagnostic'e dönün.

Ana alıcıda yeni durum satırı şöyle olmalı:

```text
bagli=4/4 [...] | body_taze=4/4 [...] | ham_kayit=... | gecersiz=0
```

`bagli`, status dahil paket geldiğini; `body_taze`, gerçekten güncel iskelet
geldiğini gösterir. Kalibrasyon sırasında dört kameranın hepsinde aynı tek kişi
görünmelidir.

### 5.3 Yeni statik extrinsic kaydı al

Tripodlar sabitken ortak alanda tam beden görünecek şekilde:

1. 8 saniye rahat A-pozunda sabit durun.
2. 8 saniye kolları öne uzatıp sabit durun.
3. 8 saniye T-pozunda sabit durun.
4. Pozlar arasında yavaş geçin; yeri değiştirmeyin.

`ham_kayit` en az 150 olunca ana alıcıda `Ctrl+C`. Dört kaynak açık kalsın.

### 5.4 Extrinsic ve dünya-konumu JSONL üret

```powershell
.\zed_four_camera_test\start_distributed_calibration.ps1 `
  -CapturePath ".\recordings\four_body38_static_calibration_20260901.jsonl" `
  -OutputPath ".\config\zed_four\distributed_body38_extrinsics_20260901.json" `
  -WorldPosesJsonl ".\config\zed_four\four_camera_world_poses_20260901.jsonl" `
  -ReferenceSerial 33773329
```

**Kabul:** her kamera için RMS `<=0.08 m`, p95 `<=0.12 m`. Daha yüksekse ortak
görüşü ve ayak/kol görünürlüğünü düzeltip kaydı tekrarlayın. Kamera hareket
ederse dosya artık geçersizdir.

### 5.5 Dört-kamera canlı Fusion kabulü

```powershell
.\zed_four_camera_test\start_distributed_receiver.ps1 `
  -Source "39504762:16000","31571870:16002","33773329:16004","34760587:16006" `
  -Extrinsics ".\config\zed_four\distributed_body38_extrinsics_20260901.json" `
  -Fps 15 -MinimumSources 4 -MaxSyncMs 110 -SourceTimeoutMs 750 `
  -Duration 120
```

**Kabul:** `body_taze=4/4`; `fusion_cikis` yaklaşık saniyede 15 artar;
`son_katki` dört seri içerir; `yayilim_ms<=110`; `gecersiz=0`. Ortada dönüp
kolları sırayla farklı kameralardan saklayın; Fusion kesilmemeli.

## 6. G1/Isaac simülasyonuna BODY_38 bağla

Önce dört kaynak açık kalsın; Bölüm 5.5 alıcısını kapatın. Isaac/GMR'yi ana
PC'de ayrı PowerShell'de başlatın:

```powershell
cd "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
.\start_g1_isaaclab_live.ps1 `
  -Mode upper_body -ImitationMode kinematic_debug `
  -InputFps 15 -AcceptNvidiaEula
```

Komut açıldıktan sonra WSL IP'sini bulun:

```powershell
$wslIp = ((wsl.exe -d Ubuntu-22.04 -- hostname -I).Trim() -split "\s+")[0]
$wslIp
```

Yeni ana-PC PowerShell'de BODY_38 Fusion'u bu IP'ye gönderin. Aşağıdaki
`$wslIp` aynı pencere içinde hesaplanmalıdır:

```powershell
cd "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
$wslIp = ((wsl.exe -d Ubuntu-22.04 -- hostname -I).Trim() -split "\s+")[0]

.\zed_four_camera_test\start_distributed_receiver.ps1 `
  -Source "39504762:16000","31571870:16002","33773329:16004","34760587:16006" `
  -Extrinsics ".\config\zed_four\distributed_body38_extrinsics_20260901.json" `
  -OutputHost $wslIp -OutputPort 15050 `
  -Fps 15 -MinimumSources 3 -MaxSyncMs 110 -SourceTimeoutMs 750
```

İlk simülasyon için `kinematic_debug`, `upper_body` ve `MinimumSources 3`
kullanın; fiziksel robota bağlanmayın. Akış kararlı olduktan sonra deney
protokolünde 4/4 kabul testi, kamera-örtülme testi, gecikme ve eklem sürekliliği
ayrı kaydedilmelidir.

## 7. Hangi dosya ne işe yarıyor?

- `zed360_hybrid_calibration_seed.json`: yalnız ZED360'a dört kaynağı tanıtan
  sıfır-poz başlangıcı; runtime extrinsic değildir.
- `zed360_calibrated_4cam.json`: ZED360 başarılıysa esas Stereolabs kalibrasyonu.
- `zed360_four_camera_world_poses.jsonl`: ZED360 sonucunun SDK ile Z-up dünyaya
  çevrilmiş satır-bazlı kamera pozları.
- `four_body38_static_calibration_20260901.jsonl`: custom kalibrasyonda ham,
  eşzamanlı dört BODY_38 iskelet örnekleri.
- `distributed_body38_extrinsics_20260901.json`: custom Fusion'un kullandığı
  kamera→dünya dönüşümleri.
- `four_camera_world_poses_20260901.jsonl`: her fiziksel kameranın dünya pozu.

## Resmî kaynaklar ve bilinen SDK riski

- [Stereolabs ZED360](https://docs.stereolabs.com/docs/development/zed-tools/zed-360)
- [Stereolabs Fusion API](https://docs.stereolabs.com/docs/development/zed-sdk/modules/fusion)
- [Resmî multi-camera Python örneği](https://github.com/stereolabs/zed-sdk/blob/master/body%20tracking/multi-camera/python/fused_cameras.py)
- [Stereolabs multi-camera USB ve saat senkronizasyonu](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/multi-camera)
- [`WRONG BODY FORMAT` için 2026 tekrar üretimleri](https://community.stereolabs.com/t/zed360-crashing-and-not-showing-skeletal-data/10694)

Ayrıntılı kaynak/gap matrisi:
`research\zed360_four_camera_2026-09-01\report-source.md`.
