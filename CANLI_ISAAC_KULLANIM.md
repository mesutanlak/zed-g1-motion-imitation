# ZED 2i → GMR → G1 23-DOF Isaac Lab

Üç ayrı PowerShell penceresi kullanılır. Fiziksel Unitree DDS kanalı açılmaz.

## 1. Isaac Lab, GMR ve denge politikası

```powershell
cd "C:\Users\Misafir\Desktop\ZED_G1_Projesi"
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 `
  -Mode upper_body `
  -StanceMode fixed_double_support `
  -InputFps 15 `
  -UpperCutoffHz 10 `
  -MimicBlend 1.0 `
  -UpperStiffnessScale 2.0 `
  -UpperDampingScale 2.0 `
  -HumanHeightM 1.80 `
  -RuntimeProfile shared_gpu_safe `
  -AcceptNvidiaEula
```

Terminalde `G1 scene initialization complete` görülene kadar bekleyin.
Üst gövde taklidinde:

- bacaklar, ayak bilekleri ve bel: resmî nominal çift-temas pozunda sabit,
- omuz, dirsek ve bilek roll: GMR referansı,
- düşme emniyeti: fizik tabanlı esnek askı,
- giriş kesilmesi: son güvenli üst-gövde hedefi tutulur, sabit çift-temas
  alt-beden duruşu devam eder.

Logda `stance=fixed_double_support`, birbirine yakın `foot_z` değerleri ve
`foot_delta=0.000` beklenir. Kamera 15 Hz çalışırken Isaac fiziği 200 Hz
çalışır; her yeni kamera hedefi gecikme biriktirmeden en son hedef olarak
uygulanır.

Bilek roll, BODY_38 işaret/serçe parmak düzleminin önkol ekseni etrafındaki
hareketinden çıkarılır. Kamera başladıktan sonra yaklaşık ilk 12 sağlam kare
nötr bilek kalibrasyonudur; bu sırada elleri doğal ve sabit tutun.

G1 23-DOF resmî URDF'de `head_joint` sabit eklemdir. Baş hareketi ZED ve analiz
panelinde ölçülür fakat bu modelde fiziksel bir baş aktüatörü olmadığı için
robota baş açısı uygulanamaz.

Askısız politika testi yalnız simülasyon tanısı içindir:

```powershell
.\start_g1_isaaclab_live.ps1 -Mode upper_body -AcceptNvidiaEula -NoFallArrest
```

### Isaac Sim Simulation Output

Canlı GMR hedefleri Isaac Lab'in GPU/Fabric fizik tamponuna yazılır. Bu nedenle
`Simulation Settings > Simulation Output` seçimi **Fabric GPU** olmalıdır.
`USD` seçiliyse UDP ve fizik çalışsa bile viewport başlangıç pozunu gösterebilir.
Başlatıcı artık Fabric GPU ayarını otomatik uygular. Pencereden elle değiştirilirse
yeniden `Fabric GPU` seçilmelidir; `Fabric CPU` bu CUDA çalışması için kullanılmaz.

Isaac terminalindeki `packets`, `cmd_delta`, `actual_delta` ve `tracking_err`
alanları sırasıyla alınan paket sayısını, komut hareketini, gerçek fizik
hareketini ve takip hatasını gösterir.

## 2. İsteğe bağlı 3B analiz paneli

```powershell
cd "C:\Users\Misafir\Desktop\ZED_G1_Projesi"
powershell -ExecutionPolicy Bypass -File .\start_g1_skeleton_panel_wsl.ps1
```

Panel yalnız tutarlı bütün-vücut karelerini çizer. Parçalı algılamada son
sağlam iskeleti korur ve `LOW_QUALITY` gösterir.

Panelde `UDP RX FPS` ZED uygulamasından gelen bütün durum/veri paketlerini,
`Geçerli FPS` ise çizilebilen tutarlı BODY_38 karelerini gösterir. Böylece
`NO_BODY` durumunda ağ bağlantısı çalışırken geçerli iskelet hızının sıfır
olduğu açıkça görülebilir.

Panel küçük, içi boş noktalarla ham ZED ölçümlerini; renkli çizgilerle
güven-eşikli filtrelenmiş iskeleti gösterir. Kaynak Hz, capture→UDP gecikmesi,
ham/filtre RMS farkı, mesafe, dirsek açıları, IMU açısal hız ve baş yana eğimi
canlı gösterilir. Alt beden eksik olsa bile `upper_body=HAZIR` ise panel
`UPPER_LIVE` durumunda kol ve gövdeyi günceller.

## 3. ZED 2i canlı yayın

```powershell
cd "C:\Users\Misafir\Desktop\ZED_G1_Projesi"
powershell -ExecutionPolicy Bypass -File .\start_zed_live_smooth_to_wsl.ps1
```

Varsayılan `shared_gpu_safe` profili Isaac ile aynı anda kullanım içindir:
HD720@15, BODY_38 MEDIUM ve düşük GPU yükü kullanır. Isaac kapalıyken yalnız
kamera kalitesi incelemesi için `-Profile quality` seçilebilir.

İnsan kameradan yaklaşık 2–4 m uzakta, tüm vücudu görünür ve iki ayağı zeminde
olmalıdır. `Q`/`ESC` çıkış, `R` kişi kilidi sıfırlama, `D` tanı panelidir.

Canlı Isaac oturumunda SVO2 kaydı açılmaz. Son bozuk kaydın ham SVO2
karelerinde yatay parçalanma doğrulanmıştır. Canlı oturumda yalnız JSONL:

```powershell
.\start_zed_live_smooth_to_wsl.ps1 -Record
```

SVO2 veri seti kaydı, Isaac kapalıyken ayrı oturumda:

```powershell
.\start_zed_live_smooth_to_wsl.ps1 -SkipGmrCheck -RecordSvo2
```

## Güvenli durum

- `NO_BODY`, `LOW_QUALITY` veya `STALE`: yeni GMR hareketi uygulanmaz.
- `CORRUPT_FRAME`: yatay parçalanmış USB karesi BODY_38/GMR'ye sokulmaz.
  Altı ardışık bozuk karede kamera otomatik kapatılıp yeniden açılır.
- Anatomik oranı veya kareler arası hızı gerçek dışı olan noktalar GMR'ye girmez.
- GMR optimizasyonu kamera hızının gerisinde kalırsa eski kuyruk atılır ve
  yalnız en yeni kare çözülür.
- Sonlu olmayan (`NaN/Inf`) hedefler hem GMR çıkışında hem Isaac girişinde
  reddedilir.
- Robot yerinde durmayı sürdürür; fiziksel robot ağına hiçbir paket gönderilmez.
- Yürüme/koşma bu canlı denetleyicide kapalıdır; sonraki aşama, temiz GMR
  referans kliplerinden resmi 23-DOF tracking göreviyle politika eğitimidir.

## USB bağlantısı

ZED 2i doğrudan anakartın arka USB 3.x portuna bağlanmalıdır. Ön panel, hub ve
uzatma kablosu kullanılmamalıdır. Bu bilgisayarda Windows USB selective suspend
AC ve batarya için kapatılmıştır. Buna rağmen `CORRUPT_FRAME` tekrar ederse
kamerayı farklı bir arka USB 3 denetleyicisine taşıyın ve kabloyu değiştirin;
yazılım bozulmuş UVC karesinden güvenilir iskelet üretemez.
