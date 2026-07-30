# 30 Temmuz BODY_38 veri seçimi ve canlı smoothness sonucu

Bu rapor algı ve simülasyon verisi içindir. Hiçbir çıktı fiziksel G1 motor
komutu olarak doğrudan kullanılmamalıdır.

## Seçilen kayıtlar

Ham ZED kayıtları içinde:

| Kayıt | Kullanım | Temel gerekçe |
|---|---|---|
| `130903` | ana üst-gövde baseline | 3323 kare, 15 Hz, %98.89 üst-gövde görünürlüğü, mesafenin %99.52'si 2–4 m |
| `102827` | bağımsız doğrulama | 2926 kare, %98.43 üst-gövde görünürlüğü |
| `114421` | el/örtülme doğrulama | sol/sağ BODY_38 el-proxy görünürlüğü %88.51/%85.53 |
| `153223` | yalnız zor-vaka testi | uzun kayıt fakat bütün-vücut çekirdek görünürlüğü yalnız %40.18 |

3B analiz kayıtlarında `144046` temiz baseline, `153334` ise kol–gövde
örtülmesi içeren challenge setidir. Metadata-only ve çok kısa kayıtlar eğitim
seçimine alınmamıştır. Makine tarafından okunabilir sonuç:
`datasets/analysis_quality_manifest.json`.

`130903` kaydı upstream GMR üzerinden G1 23-DOF uzayına çevrildi:

- 3245 geçerli retarget edilmiş kare;
- iki kesintisiz clip: 1204 kare/80.2 s ve 2042 kare/136.07 s;
- eğitim clip'leri 15 Hz ve 4 Hz sıfır-fazlı offline filtre ile hazırlandı.

BODY_38 noktaları G1 motor açıları değildir. Yalnız GMR çıktısı, G1 limitleri
ve simülasyon güvenlik katmanı sonrasındaki referanslar tracking eğitiminde
kullanılabilir.

## Canlı hatta bulunan ve düzeltilen sorun

Kamera 15 FPS olmasına rağmen analiz UDP kaydı 7.5–9.6 Hz civarındaydı.
Yayınlayıcı `1/15 s` eşiğini tam karşılaştırdığı için zamanlayıcı jitter'ında
neredeyse her ikinci kareyi atıyordu. Kamera hızı ile istenen yayın hızı aynı
olduğunda artık her başarılı BODY_38 karesi gönderilir. Bu değişiklik tek
başına canlı hareketin zaman çözünürlüğünü yaklaşık iki katına çıkarır.

GMR çıkışındaki sabit low-pass filtre de adaptif One-Euro tarzı filtreyle
değiştirildi:

- yavaş/durağan harekette `2 Hz` minimum cutoff ile jitter azaltılır;
- gerçek eklem hareketinde hızla birlikte cutoff en fazla giriş Nyquist
  sınırının %90'ına çıkar;
- varsayılan hız katsayısı `beta=1.2`;
- mevcut eklem hız/slew limitleri korunur.

Deterministik testte 0.03 rad gürültü RMS değeri 0.021 rad'a inerken 1 rad
basamak hedefi iki kamera karesinde 0.963 rad'a ulaşmıştır. Böylece sabit ağır
filtrenin gecikmesi olmadan durağan pozdaki titreşim azaltılır.

## Önerilen canlı profil

```powershell
.\start_g1_isaaclab_live.ps1 `
  -Mode upper_body `
  -AcceptNvidiaEula `
  -InputFps 15 `
  -UpperMinCutoffHz 2.0 `
  -UpperCutoffHz 10.0 `
  -UpperVelocityBeta 1.2
```

ZED ayrı terminalde:

```powershell
.\start_zed_live_smooth_to_wsl.ps1 -Profile shared_gpu_safe
```

Canlı logda kaynak hızının yaklaşık `15 Hz` olması beklenir. `accepted/hz`
değeri bunun belirgin altında kalırsa sonraki darboğaz GMR çözüm süresidir;
önce `solve_ms`, `superseded` ve `source_age_ms` ölçülmelidir.

## Dayanak

- Stereolabs BODY Tracking API, `skeleton_smoothing`, confidence,
  minimum-keypoint ve prediction-timeout parametrelerini tanımlar:
  https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api
- GMR, insan hareketini farklı humanoid yapılara gerçek zamanda retarget etmek
  için kullanılan upstream çözücüdür:
  https://github.com/YanjieZe/GMR
- Isaac Lab eklem hedefleri actuator stiffness/damping ve hız/efor limitleri
  üzerinden fizik motoruna uygular:
  https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.actuators.html
