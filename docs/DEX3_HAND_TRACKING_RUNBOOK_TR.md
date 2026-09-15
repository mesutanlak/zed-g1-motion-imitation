# 4×ZED BODY_38 kilitli el takibi ve Dex3-1 runbook

> Resmi G1-29DOF + Dex3 Isaac entegrasyonu ve güncel performans profili için
> `docs/DEX3_OFFICIAL_ISAAC_PIPELINE_TR.md` esas alınmalıdır. Bu dosyanın kalan
> kısmı ayrıntılı/geriye dönük paket ve replay açıklaması olarak korunur.

Bu ek katman mevcut G1 23-DOF gövde yolunu değiştirmez. Varsayılanı kapalıdır.
Kaynak süreçler tam RGB görüntüyü ağdan göndermez; yalnız BODY_38 ile **LOCKED**
durumdaki operatörün bilek ROI'lerinde MediaPipe çalıştırır ve ayrı UDP
portlarından kompakt 21-landmark/depth metadata'sı yollar. Kodda Unitree DDS
importu, fiziksel el komut topic'i veya fiziksel robot publisher'ı yoktur.

## Mimari audit ve doğrulanan kaynaklar

Aktif yol `start_zed_four_sources.ps1` → kamera başına `zed_g1_skeleton.py` →
`zed_body38_live/v1` UDP → `distributed_body38_fusion.py` yoludur. Bu,
`sl.Fusion` publisher yolu değil, uygulama düzeyi distributed BODY_38 yoludur.
El verisi bu yüzden aynı işi ikinci bir ZED Fusion protokolünde çoğaltmadan bu
yola eklendi. BODY UDP portları değişmedi; el portları sırasıyla 16200, 16202,
16204 ve 16206'dır.

Teknik kararlar 4 Eylül 2026 tarihinde şu birincil kaynaklarla kontrol edildi:

- MediaPipe Hand Landmarker Python: VIDEO modunda her tracker için monoton
  milisaniye timestamp gerekir; 21 normalized ve world landmark ayrıdır.
  MediaPipe world landmark'ı ZED dünya noktası olarak kullanılmaz.
  <https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python>
- ZED SDK body tracking: BODY_38 `keypoint_2d` ile 3B `keypoint` sağlar ve 3B
  referans çerçevesi runtime seçimine bağlıdır.
  <https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api>
- ZED SDK depth: `grab()` sonrası `retrieveMeasure(..., MEASURE.DEPTH)` sol
  görüntüyle hizalı 32-bit depth verir. Bu projede birim `METER`'dır.
  <https://www.stereolabs.com/docs/development/zed-sdk/modules/depth-sensing/using-the-api>
- Unitree `xr_teleoperate` Dex3 config'i 7 hedef joint'i ve DexPilot uç/link
  görevlerini tanımlar. Resmi checkout Apache-2.0'dır; repo içine URDF/YAML
  kopyalanmamıştır. Kullanılırsa dış checkout doğrudan okunur.
  <https://github.com/unitreerobotics/xr_teleoperate>

Resmi Unitree backend 25×3 sahte giriş almaz. MediaPipe'ın gerçek
wrist/thumb-tip/index-tip/middle-tip indekslerinden resmi DexPilot'in altı
vektörü doğrudan üretilir. `-Dex3OfficialRoot` verilmezse donanımdan bağımsız
normalize-21 görev fallback'i yalnız analiz/simülasyon hedefi üretir.
Joint sırası resmi config/controller ile kontrol edilmiştir. Position limitleri
yalnız repository'nin mevcut `config/g1_23dof_dex3.json` kaynağından okunur;
koda ikinci tablo gömülmez. Unitree'nin herkese açık sayfasında bu limitlerin
makine-okunur güncel tablosu doğrulanamadığından fiziksel kullanıma uygunluk
iddiası yoktur ve bu da donanım çıkışının kapalı kalmasının ek nedenidir.

## Kurulum — hem ana PC hem laptop

Mevcut `.venv-zed` ortamlarını birleştirmeyin. Her bilgisayarda kendi proje
klasöründe:

```powershell
Set-Location "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
.\install\setup_hand_tracking.ps1
```

Google'ın Hand Landmarker sayfasından resmi `hand_landmarker.task` modelini
elle indirin ve her iki bilgisayarda `C:\ZED_G1\models\hand_landmarker.task`
olarak saklayın. Uygulama modeli sessizce indirmez. Dosyayı doğrulayıp oturum
notuna hash'i yazın:

```powershell
Get-FileHash -Algorithm SHA256 "C:\ZED_G1\models\hand_landmarker.task"
```

Ana PC'de, yalnız ilk kurulumda PowerShell'i **Yönetici olarak** açın ve
laptopun aynı ağdaki gerçek IPv4 adresini vererek BODY_38, önizleme ve el UDP
portlarını açın:

```powershell
Set-Location "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
.\zed_four_camera_test\enable_main_pc_calibration_firewall.ps1 -LaptopIPv4 "192.168.50.11"
```

Ana PC adresi `192.168.50.10` veya laptop adresi `192.168.50.11` değilse
`ipconfig` çıktısındaki aynı Ethernet ağına ait IPv4 adreslerini kullanın.

İsteğe bağlı resmi Dex3 optimizer kurulumu yalnız ana PC'dedir:

```powershell
Set-Location "C:\src"
git clone --recurse-submodules https://github.com/unitreerobotics/xr_teleoperate.git
Set-Location ".\xr_teleoperate"
git submodule status
```

Checkout commit'ini ve `teleop/robot_control/dex-retargeting` submodule
commit'ini kabul kaydına yazın. Bu proje Unitree'nin DDS controller dosyasını
import etmez; yalnız config, URDF ve optimizer kullanılır.

## Canlı dört kamera — kopyalanabilir komutlar

Önce ana PC'de receiver:

```powershell
Set-Location "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
.\zed_four_camera_test\start_distributed_receiver.ps1 `
  -Source "39504762:16000","31571870:16002","33773329:16004","34760587:16006" `
  -PreviewSource "39504762:16100","31571870:16102","33773329:16104","34760587:16106" `
  -Extrinsics ".\config\zed_four\active_distributed_body38_extrinsics.json" `
  -MinimumSources 2 -MaxSyncMs 80 -PreferredFullSetSpreadMs 40 `
  -MaxTemporalPredictionMs 70 -HandTracking -HandMaxAgeMs 120 -HandMaxSpreadMs 70 `
  -Record -OutputDir ".\recordings"
```

Resmi optimizer checkout'i hazırsa aynı komuta şunu ekleyin:

```powershell
-Dex3OfficialRoot "C:\src\xr_teleoperate"
```

Sonra ana PC'nin iki yerel kamerası:

```powershell
Set-Location "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
.\start_zed_four_sources.ps1 -Role MainPc -MainPcHost "192.168.50.10" `
  -Fps 15 -DepthMode neural-light -FrameIntegrityMode off `
  -DistanceMin 1.0 -DistanceMax 5.25 `
  -HandTracking -HandModel "C:\ZED_G1\models\hand_landmarker.task" `
  -HandDelegate cpu -HandInferenceFps 8 -RecordSvo2
```

Laptopta iki uzak kamera:

```powershell
Set-Location "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
.\start_zed_four_sources.ps1 -Role Laptop -MainPcHost "192.168.50.10" `
  -Fps 15 -DepthMode neural-light -FrameIntegrityMode off `
  -DistanceMin 1.0 -DistanceMax 5.25 `
  -HandTracking -HandModel "C:\ZED_G1\models\hand_landmarker.task" `
  -HandDelegate cpu -HandInferenceFps 8 -RecordSvo2
```

Operatör yaklaşık 3 m'de tutulur. ROI boyutu sabit piksel değildir: önkolun
görüntü uzunluğu ×1,45, ele doğru %32 offset, 160–420 px kapısı kullanır. İkinci
kişi varsa kaynak BODY_38 kilidi değişmeden el de değişmez. `R`, kişi kilidiyle
birlikte el association belleğini sıfırlar. Kilit kaybolunca başka ele atlanmaz;
merkez watchdog kısa hold sonrası nötre fade eder.

## Offline SVO2 yeniden işleme

Her SVO2 ait olduğu fiziksel kamerada veya pyzed kurulu Windows makinede:

```powershell
& ".\.venv-zed\Scripts\python.exe" ".\zed_g1_skeleton.py" `
  --svo-input "C:\ZED_G1\recordings\zed_body38_39504762_SESSION.svo2" `
  --serial 39504762 --headless --record `
  --hand-tracking --hand-model "C:\ZED_G1\models\hand_landmarker.task" `
  --hand-stream-host 127.0.0.1 --hand-stream-port 16200 --hand-inference-fps 8
```

Yalnız ekran videosu metrik depth, fabrika intrinsics'i veya kalibre edilmiş
çoklu kamera capture timestamp'i içermez; ondan 3B multi-view fusion üretildiği
iddia edilmez.

## Sürekli Isaac + Rerun + dört kamera bağlantısı

Canlı zincirde başlatma sırası önemlidir. Önce ana PC'de Rerun, sonra
Isaac/GMR ve en son dört-kamera fusion receiver açılır. Elin tam 21-landmark
verisi `127.0.0.1:15052` Rerun kanalına gider. `15050` kontrol paketine yalnız
14 hedef ve watchdog içeren küçük `dex3_control/v1` alanı eklenir; BODY_38 IK
sözleşmesi değişmez. Fiziksel robot el çıkışı etkinleştirilmez.

Ana PC pencere 1 — Rerun:

```powershell
.\start_g1_rerun_analysis.ps1 -Mode live -LiveMaxHz 10 -GmrLogMaxHz 10
```

Ana PC pencere 2 — Isaac/GMR:

```powershell
.\start_g1_isaaclab_live.ps1 -Mode upper_body `
  -ImitationMode kinematic_debug -InputFps 15 -Dex3 -AcceptNvidiaEula
```

Ana PC pencere 3 — sürekli dört-kamera fusion:

```powershell
.\start_zed_four_fusion_to_wsl.ps1 -Record -MinimumSources 3 `
  -FusionHz 15 -PreviewHz 8 -WorkspaceXMinM 2 -WorkspaceXMaxM 4 `
  -HandTracking -HandMaxAgeMs 120 -HandMaxSpreadMs 70
```

Kaynaklar iki hostta `start_zed_four_sources.ps1 -HandTracking` ile açık
olmalıdır. Rerun canlı sahnesinde `world/skeleton/hands`, zaman serilerinde
`world/metrics/dex3`; receiver kaydında `hand_tracking` ve `dex3_targets`
alanları bulunur.

Oturumdan sonra son fusion kaydını sayısal incelemek için:

```powershell
$Latest = Get-ChildItem ".\recordings\four_body38_fusion_*.jsonl" |
  Sort-Object LastWriteTime -Descending | Select-Object -First 1
& ".\.venv-zed\Scripts\python.exe" ".\tools\benchmark_hand_tracking.py" `
  $Latest.FullName --output ".\recordings\hand_benchmark_latest.json"
Get-Content ".\recordings\hand_benchmark_latest.json"
```

## Replay, Rerun ve benchmark

```powershell
& ".\.venv-zed\Scripts\python.exe" ".\tools\replay_hand_tracking.py" `
  ".\recordings\four_body38_fusion_SESSION.jsonl" --rerun --landmark-only

& ".\.venv-zed\Scripts\python.exe" ".\tools\replay_hand_tracking.py" `
  ".\recordings\four_body38_fusion_SESSION.jsonl" `
  --output ".\recordings\hand_replay.jsonl" --simulation

& ".\.venv-zed\Scripts\python.exe" ".\tools\benchmark_hand_tracking.py" `
  ".\recordings\four_body38_fusion_SESSION.jsonl" `
  --output ".\recordings\hand_benchmark.json"
```

## Donanımsız regresyon testleri

```powershell
Set-Location "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
& ".\.venv-zed\Scripts\python.exe" -m pytest `
  ".\tests\test_hand_tracking.py" `
  ".\tests\test_distributed_body38_fusion.py" -q
& ".\.venv-zed\Scripts\python.exe" -m pytest ".\tests" -q
& ".\.venv-zed\Scripts\python.exe" ".\zed_g1_skeleton.py" --self-test
```

15 Eylül 2026 doğrulaması: el + distributed fusion `46 passed`; tüm donanımsız
`tests/` paketi `113 passed, 2 skipped`; ZED self-test SDK 5.4.1 ile geçti.
Skip'ler opsiyonel ortamlara aittir. Gerçek MediaPipe inference kabulü için
`hand_landmarker.task`, gerçek kamera görüntüsü ve dört ZED gereklidir.

Mevcut sim asset'in parmak eklemleri doğrulanmadığı için bu teslimde simülasyon
adaptörü gövde/sol el/sağ el joint gruplarını ayrı callback'lere yazar ve hedefi
görselleştirir; parmakların hareket ettiğini sahte biçimde göstermez. Resmi
eklemli Dex3 asset'i bağımsız olarak doğrulanana kadar durum
**landmark/target visualization only**'dir.

Fusion kaydı `q_left_dex3[7]` ve `q_right_dex3[7]` üretir. Gövde retargeting'i
mevcut ayrı GMR hattında kaldığı için bu aşamada `q_body`/`q_target` null ve
composition status açıktır. `HandBodyTargetJoiner`, timestamp farkı ≤40 ms olan
mevcut `q_body[23]` ile elleri birleştirerek 37 elemanlı, yalnız simülasyon için
`q_target` sözleşmesini üretir. Böylece 23-DOF davranışı el katmanına taşınmaz.

## İlk gerçek kabul testi

En az 60 saniye; eller açık, yumruk, pinch, çapraz el, avuç ters, tek-el kaybı
ve ikinci kişi senaryolarını kaydedin. `hand_benchmark.json` için geçme kapıları:

- kamera başına inference ≥10 FPS;
- end-to-end p95 ≤100 ms;
- capture spread p95 ≤40 ms; 40–70 ms hızlı parmak fusion'unda **kabul değil**;
- landmark fusion residual p95 ≤10 mm sentetikte, gerçekte başlangıç hedefi
  ≤30 mm ve reprojection p95 ≤5 px (gerçek run'da ölçülecek);
- geçerli el coverage ≥%80;
- annotate edilmiş senaryoda sol/sağ ID switch = 0;
- stale rejection, kamera/inlier sayısı, solver p50/p95, residual ve saturation
  raporda görünür olmalı.

Donanım çalıştırılmadan FPS/gecikme/coverage değerleri başarılı sayılmaz. Dört
ZED ve gerçek Dex3 asset doğrulaması bu makinede yapılmadı. Fiziksel G1/Dex3
komut çıkışı her modda kapalıdır.
