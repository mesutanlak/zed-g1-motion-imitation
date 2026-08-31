# 4 ZED 2i / 2 bilgisayar: ZED360 kalibrasyonu ve BODY_38 Fusion

Bu akış iki aşamadan oluşur: ZED360 ağ kalibrasyonu için geçici BODY_18
yayını; G1 retarget yolunda kullanılacak nihai test için BODY_38 Fusion.
G1, Isaac, WSL veya fiziksel robot motorlarına veri/komut göndermez.

```text
ZED A + ZED B -> WD19S/USB -> MSI Raider laptop -> Ethernet -> ana PC
ZED C + ZED D -> USB -------------------------------> ana PC
```

Her iki bilgisayarda aynı ZED SDK 5.4.1, güncel NVIDIA sürücüsü, Python 3.11 ve projenin `.venv-zed` ortamı olmalıdır.
Bu izole ZED360 kabul aracı ZED360'ın beklediği `RIGHT_HANDED_Y_UP` koordinat
sistemini kullanır; G1/ROS dönüşümü bu testten sonra ayrı uygulanmalıdır.

## 1. Ağ ve seri numaraları

Doğrudan CAT6 bağlantısı için örnek IPv4 değerleri: ana PC `192.168.50.10/24`, laptop `192.168.50.11/24`. İki yönde `ping` çalışmalıdır.

Her bilgisayarda yalnız ona bağlı kameraları görün:

```powershell
.\.venv-zed\Scripts\python.exe .\zed_g1_skeleton.py --list-devices
```

Her seri numarasını not edin. Bir kamera yalnız bir bilgisayarda görünmelidir.

## 2. ZED360 için dört ağ yayını (kalibrasyon: BODY_18)

ZED360 ağ kalibrasyonunda dört kameranın da `start_publishing` ile yayın yapması gerekir. `L1,L2` laptop; `P1,P2` ana PC seri numaralarıdır.

Laptopta:

```powershell
.\zed_four_camera_test\start_publisher.ps1 `
  -Camera "L1:30000","L2:30002" -Fps 15 -Model medium -BodyFormat body18 -DepthMode performance
```

Ana PC'de ikinci PowerShell penceresinde:

```powershell
.\zed_four_camera_test\start_publisher.ps1 `
  -Camera "P1:30004","P2:30006" -Fps 15 -Model medium -BodyFormat body18 -DepthMode performance
```

Önce HD720@15 ile doğrulayın. `HAZIR | ... | BODY18` ve yaklaşık 15 publisher FPS görmelisiniz. Güvenlik duvarı izin sorarsa yalnız **Özel ağ** için izin verin. Dört publisher penceresinde de ortak alanda duran kişi için `bodies=1` görmeden ZED360'ı başlatmayın.

## 3. Ağ ZED360 kalibrasyonu

Mevcut `fourkamera.json` önceki masa düzenine aittir; tripodlar yerleşince geçersizdir. Yalnız seri numaralarını taşıyan yeni bir ağ seed'i üretmek için ana PC'de:

```powershell
.\zed_four_camera_test\new_network_calibration_seed.ps1 `
  -SourceConfig "C:\Program Files (x86)\ZED SDK\tools\fourkamera.json" `
  -Camera "L1@192.168.50.11:30000","L2@192.168.50.11:30002","P1@192.168.50.10:30004","P2@192.168.50.10:30006" `
  -OutputConfig ".\config\zed_four\network_seed.json"
```

Ana PC'de `ZED360.exe` açın; ilk ekranda **LOAD** ile `network_seed.json` dosyasını seçin ve Setup the room ile kalibre edin. Bir kişi tüm çalışma alanında yavaş yürüsün. Kameralar/tripodlar kalibrasyon bitene kadar hareket etmemelidir. Sonucu `config\zed_four\tripod_calibrated.json` diye kaydedin.

ZED360'daki **Auto Discover** düğmesine basmayın ve senderları sonradan elle eklemeyin. Bu yol tamamen ağ yayınıdır: PC'deki iki kamera da Python publisher tarafından açılır.

## 4. Dört-kamera BODY_38 Fusion kabul testi

ZED360 kaydından sonra iki bilgisayardaki geçici BODY_18 publisher'ı `Ctrl+C` ile kapatın. Nihai testte laptop publisher'ını BODY_38 ile yeniden açın; ana PC, kendi iki kamerasını Fusion testinin içinde doğrudan USB'den açar.

Laptopta:

```powershell
.\zed_four_camera_test\start_publisher.ps1 `
  -Camera "L1:30000","L2:30002" -Fps 15 -Model fast -BodyFormat body38 -DepthMode neural-light
```

Ana PC'de:

```powershell
.\zed_four_camera_test\start_fusion_test.ps1 `
  -FusionConfig ".\config\zed_four\tripod_calibrated.json" `
  -LocalSerial P1,P2 `
  -RemoteCamera "L1@192.168.50.11:30000","L2@192.168.50.11:30002" `
  -Fps 15 -Model fast -DepthMode neural-light -Duration 300 `
  -MinimumCameras 2 `
  -Record ".\recordings\four_zed_body38_test.jsonl"
```

Konsolda `cameras_present=4/4`, yaklaşık 15 Fusion FPS ve operatör varken sıfır olmayan `body_frames` görmelisiniz. Önce `-MinimumCameras 2`, sonra 3 ve 4 ile tekrarlayın.

Başlangıç kabulü: 5 dakika kopmasız çalışma, `cameras_present=4/4`, 15 FPS talebinde yaklaşık 14+ Fusion FPS ve JSONL kaydında BODY_38 eklemleri. Bu geçmeden G1 kontrol hattına bağlamayın.
