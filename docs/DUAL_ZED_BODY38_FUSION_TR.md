# İki ZED 2i ile ortak BODY_38 ve G1 üst-gövde taklidi

Bu yol **tek-kamera sisteminden ayrıdır**. Tek kamera için mevcut
`start_zed_live_smooth_to_wsl.ps1`, çift kamera için yeni
`start_zed_dual_fusion_to_wsl.ps1` kullanılır. Her ikisi de aynı
`zed_body38_live/v1` UDP sözleşmesini ürettiği için GMR, feasibility ve Isaac
Lab tarafı değiştirilmeden çalışır.

## Metodoloji

1. İki ZED 2i, kalibrasyon dosyasındaki seri numarası ve extrinsic pozlarla açılır.
2. Her kamera `BODY_38/FULL` çıkarır ve resmî ZED SDK `INTRA_PROCESS` yerel yayıncısı olur.
3. ZED Fusion zaman senkronizasyonu, geometrik kalibrasyon ve ortak `BASELINK` koordinatında BODY_38 füzyonunu yapar.
4. Bir kamera geçici olarak göremezse `skeleton_minimum_allowed_camera=1` sayesinde Fusion tek geçerli kamerayla kimliği korumaya çalışır.
5. Birleşik iskelet yetersizse yalnız daha önce kilitlenmiş operatöre ait, SDK'nın ortak koordinata dönüştürdüğü ham iskeletler değerlendirilir. Üst-gövde görünürlüğü, keypoint güveni ve son pelvis konumuna göre en iyi kamera seçilir. Başka kişiye otomatik geçiş yapılmaz.
6. Gövde önü oklüzyonunda her kameranın 2B torso poligonu kendi piksel düzleminde ayrı hesaplanır. En az bir kamera tam kol zincirini güvenilir ve açık görüyorsa birleşik 3B hedef korunur. İki görünüm de belirsizse mevcut olay-tetiklemeli AKC, dirsek dalı belleği ve anatomik sınırlar devreye girer.
7. Çıktı pelvis-yerel hale getirilir; mevcut GMR nominal çözümü, feasibility, `safe_q` ve Isaac üst-gövde uygulaması aynen korunur.

Bu tasarım confidence ağırlıklı çoklu-görünüm kanıtı kullanır; yeni bir öğrenilmiş
triangulation ağı eklemez. ZED SDK zaten stereo derinlik, kalibrasyon ve Fusion
çözümünü sağladığı için ilk aşamada bu yaklaşım daha az gecikme ve regresyon riski taşır.

## Kalibrasyon

Varsayılan dosya `config\zed_dual\calibration_33773329_39504762.json` olup
`calibrationdual.json` değerlerinin proje içindeki sabit kopyasıdır.

Kameralardan biri diğerine göre taşınır veya döndürülürse ZED360 ile yeniden
kalibrasyon zorunludur. İki kamera rijit bir aparat üzerinde birbirlerine göre
değişmeden birlikte taşınırsa relative extrinsic korunur; yine de zemin ve görüş
alanı kontrol edilmelidir. Yeni dosya kod değiştirmeden seçilebilir:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_zed_dual_fusion_to_wsl.ps1 `
  -FusionConfig "C:\kalibrasyonlar\yeni_dual.json"
```

## Çalıştırma

ZED Explorer, ZED360 ve BodyFusion uygulamasını kapatın. Önce Isaac/GMR:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 `
  -Mode upper_body -AcceptNvidiaEula
```

Ayrı PowerShell'de önerilen çift-kamera profili:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_zed_dual_fusion_to_wsl.ps1 `
  -Profile dual_balanced_30 -Record
```

Bu komut varsayılan olarak iki ZED 2i görüntüsünü yan yana açar. Her panelde
kameranın kendi BODY_38 iskeleti, üst çubukta ise Fusion kaynağı, kilitli
operatör, katkı veren kamera sayısı ve kayıt durumu gösterilir. Önizleme
penceresi odaktayken `S` JSONL kaydını açıp kapatır, `R` operatör kilidi ve
kalibrasyonu sıfırlar, `Q`/`ESC` uygulamayı kapatır. Görüntüsüz kullanım için
ayrıca `-Headless` verilebilir.

Tuşlar: `S` JSONL kaydını açar/kapatır, `R` operatör kilidi, kalibrasyon ve kol
belleğini sıfırlar, `Q` güvenli çıkış yapar.

GPU/USB yükü yüksekse `-Profile dual_safe_15` kullanın.
`dual_60_experimental` vardır ancak iki BODY_38 çıkarımı + Isaac aynı GPU'da
60 Hz garantisi değildir. Varsayılan 30 Hz; etkin BODY FPS, kare düşümü ve p95
gecikme ölçülmeden 60 Hz makale sonucu olarak kullanılmamalıdır.

Tek kameraya dönmek için dual süreci kapatıp mevcut komutu çalıştırın:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_zed_live_smooth_to_wsl.ps1 `
  -Profile realtime_60
```

İki kaynak aynı anda UDP 15050'ye yayınlanmamalıdır.

## Ölçümler ve tek/çift kamera karşılaştırması

Dual JSONL içindeki `multi_camera` alanı; `fusion/single_fallback` modu, katkı
veren kamera sayısı, kamera başına görünürlük/güven, açık kol görüşü sayısı,
iki kameranın ortak koordinattaki keypoint MPJPE/p95 farkı, resmî Fusion kamera
katılım sayısı, kamera başına alınan FPS, senkron gecikme dağılımı ve fallback
nedenini saklar.

Mevcut 7 Ağustos tek-kamera kaydından yeniden üretilebilir baz çizgi:

| Metrik | Tek kamera |
|---|---:|
| Kare | 5749 |
| Etkin BODY FPS (medyan aralık) | 59.72 |
| Kare aralığı p95 | 33.38 ms |
| Tüm keypoint görünürlük oranı | 0.854 |
| Üst-gövde görünürlük oranı | 0.985 |
| Tam üst-gövde kare oranı | 0.923 |
| Sol dirsek kareler-arası açı değişimi p95 | 1.03° |
| Sağ dirsek kareler-arası açı değişimi p95 | 0.74° |
| AKC/kol kurtarma kullanılan kare oranı | 0.308 |

Bu değerler ground-truth doğruluk değil, iç tutarlılık ve süreklilik
metrikleridir. İlk gerçek dual operatör kaydından sonra:

```powershell
python .\tools\compare_single_dual_body38.py `
  --single .\recordings\zed_body38_20260807_171809.jsonl `
  --dual .\recordings\zed_dual_body38_YYYYMMDD_HHMMSS.jsonl `
  --output .\analysis\single_vs_dual_experiment
```

CSV/JSON çıktısı görünürlük, üst-gövde tamamlığı, kemik uzunluğu CV, dirsek açı
sürekliliği, AKC oranı, bilateral torso-overlap oranı, FPS, kaynak modları ve
cross-view MPJPE ile Fusion senkron/FPS ölçülerini verir. Dual kayıt alınmadan
iyileşme yüzdesi iddia edilmemelidir.

## Kaynaklar

- Stereolabs Fusion: https://docs.stereolabs.com/docs/development/zed-sdk/modules/fusion
- Stereolabs ZED360: https://www.stereolabs.com/docs/fusion/zed360
- Resmî multi-camera BODY örneği: https://github.com/stereolabs/zed-sdk/tree/master/body%20tracking/multi-camera/python
- Resmî BODY Tracking API: https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api
- Iskakov vd., confidence ağırlıklı çoklu-görünüm triangulation, ICCV 2019: https://openaccess.thecvf.com/content_ICCV_2019/html/Iskakov_Learnable_Triangulation_of_Human_Pose_ICCV_2019_paper.html
- CMU Panoptic Studio, oklüzyonda çoklu-görünüm entegrasyonu: https://www.cs.cmu.edu/~hanbyulj/panoptic-studio/
