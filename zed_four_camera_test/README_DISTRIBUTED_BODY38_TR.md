# Laptop USB hub + ana PC USB: 4 ZED 2i BODY_38 akışı

> Güncel, ZED360 denemesini de içeren uçtan uca çalışma sırası için
> `docs/ZED360_DORT_KAMERA_BODY38_RUNBOOK_TR.md` belgesini kullanın. Bu belge
> yalnız uygulama-seviyesi fallback'in teknik özetidir.

Bu belge, seçtiğin fiziksel düzen için üretim-deney akışıdır:

```text
ZED 39504762 + ZED 34760587 -> laptop USB 3 hub -> laptop
                                                  | CAT6 Ethernet
ZED 33773329 + ZED 31571870 -> ana PC USB -------+-> ana PC alici/fusion
```

Canlı veri yolu `sl.Fusion` Network Workflow'unu kullanmaz; iki hostta ortaya
çıkan `WRONG BODY FORMAT` hatasını atlar. Kamera pozlarını tercihen ZED360
`Finish Calibration` JSON'undan (`four json` klasörü), gerekirse BODY_38
fallback kalibrasyonundan alır. ZED'in **BODY_38** algılamasını korur ve mevcut
G1 alıcılarının bildiği `zed_body38_live/v1` UDP paketi üretir. Fiziksel robota
komut göndermez.

## Ön koşullar

- Dört kamerada da ZED SDK 5.4.1 ve proje `.venv-zed` ortamı kurulu olmalı.
- PC IP: `192.168.50.10`, laptop IP: `192.168.50.11`; iki yönlü ping başarılı
  olmalı.
- Dört kamera da HD720@15 kullanır. Aynı anda açık olan başka ZED uygulaması
  (ZED Explorer, Depth Viewer, ZED360, eski `start_publisher.ps1`) kalmamalı.
- Laptop hub'ı **harici adaptörlü USB 3.x hub** olmalı. Her kamera ayrı kısa ve
  sağlam USB 3 kablosuyla hub'a bağlanmalı. Görüntü yırtılması/USB uyarısı
  olursa bu akışı durdurup önce o bağlantıyı çöz.

İlk defa çalıştırırken Windows güvenlik duvarı sorarsa ana PC'de yalnız **Özel
ağ** için Python'a izin ver. UDP `16000`, `16002`, `16004`, `16006` portları
ana PC'ye gelen trafiğe açık olmalı.

Her yeni PowerShell penceresinde önce şunu çalıştır:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
```

## A. Dört ham BODY_38 kaynağını başlat

Her komut tek fiziksel kamerayı seri numarasıyla açar. Böylece iki uygulama
aynı "ilk ZED"i açmaz. Dört komutun her birini **ayrı PowerShell penceresinde**
çalıştır; açık kalmaları gerekir.

Laptopta, `C:\Users\MSI\Desktop\zed-g1-motion-imitation` dizininde:

```powershell
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 39504762 -TargetHost 192.168.50.10 -TargetPort 16000 `
  -Fps 15 -Model medium -DepthMode neural-light -FrameIntegrityMode off
```

```powershell
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 34760587 -TargetHost 192.168.50.10 -TargetPort 16006 `
  -Fps 15 -Model medium -DepthMode neural-light -FrameIntegrityMode off
```

Ana PC'de, `C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation`
dizininde. Yerel kameralar da aynı UDP biçimiyle **loopback** üzerinden ana
PC alıcısına gider:

```powershell
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 33773329 -TargetHost 127.0.0.1 -TargetPort 16004 `
  -Fps 15 -Model medium -DepthMode neural-light -FrameIntegrityMode off
```

```powershell
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 31571870 -TargetHost 127.0.0.1 -TargetPort 16002 `
  -Fps 15 -Model medium -DepthMode neural-light -FrameIntegrityMode off
```

Her pencerede `Acik ZED seri numarasi: ...` yazısı kendi seri numarasıyla
eşleşmelidir. Başka seri numarası görülürse tüm kaynakları `Ctrl+C` ile kapat;
o kamerayı tutan eski uygulamayı kapat ve yalnız o komutu yeniden başlat.

## B. Tripod yerleşimi ve ilk kalibrasyon kaydı

Kameraları dört köşeye koy; her biri operatörün orta çalışma alanını görsün.
En az iki kameranın her omuzu, dirseği ve bileği görmesi gerekir. Tripodlar
kalibrasyon tamamlanana kadar hareket etmemeli. Kalibrasyon için dört manuel
kaynak yerine iki hostta `start_zed_four_sources.ps1 ... -CalibrationMode`
kullanın; odada yalnız tek kişi bulunsun.

Ana PC'de beşinci PowerShell penceresinde aşağıdaki alıcıyı aç:

```powershell
.\zed_four_camera_test\start_distributed_receiver.ps1 `
  -Source "39504762:16000","31571870:16002","33773329:16004","34760587:16006" `
  -CalibrationRecord ".\recordings\four_body38_static_calibration_v2.jsonl" `
  -Fps 15 -MinimumSources 4 -MaxSyncMs 80 -SourceTimeoutMs 250
```

Konsolda `DURUM | kaynak=4/4 [...] | ham_kayit=...` görmelisin. `ham_kayit`
sayısı artıyorsa ağ ve dört BODY_38 kaynağı doğrudur.

45-60 saniye ortak hacimde yavaş yürüyün; farklı grid noktalarında 1-2 saniye
A/T pozu, kollar önde ve bükülü dirsek pozları verin. Tüm kameralar aynı kişiyi
ve mümkün olduğunca tam bedeni görmelidir. `ham_kayit` tercihen 400'ü geçince
alıcıyı ve dört kaynağı `Ctrl+C` ile kapatın. Kısa tek-nokta T-poz kaydı
extrinsic için yeterli geometrik çeşitlilik sağlamaz.

## C. Extrinsic dosyasını üret

Ana PC'de:

```powershell
.\zed_four_camera_test\start_distributed_calibration.ps1 `
  -CapturePath ".\recordings\four_body38_static_calibration_v2.jsonl" `
  -OutputPath ".\config\zed_four\distributed_body38_extrinsics_v2.json" `
  -ReferenceSerial 33773329 -Activate
```

Her referans dışı kamera için örnek, inlier oranı, `rms`, `p95` ve pelvis p95
yazılır. En az 60 örnek, 300 inlier, %25 inlier oranı ve pelvis p95 `<=0.25 m`
sağlanmazsa dosya kasıtlı olarak oluşturulmaz. Tripod açılarını/ortak görünümü
düzeltip B adımını tekrarlayın. Kamera veya tripod sonradan hareket ederse bu
JSON geçersiz olur.

## D. Dört kamera BODY_38 birleşimini test et

İki hostta kaynakları `start_zed_four_sources.ps1` ile bu kez
`-CalibrationMode` olmadan yeniden başlatın. Böylece kamera-yerel kaba kapı
etkinleşir. Sonra alıcıyı aktif kalibrasyonla çalıştırın:

```powershell
.\zed_four_camera_test\start_distributed_receiver.ps1 `
  -Source "39504762:16000","31571870:16002","33773329:16004","34760587:16006" `
  -Extrinsics ".\config\zed_four\active_distributed_body38_extrinsics.json" `
  -Fps 15 -MinimumSources 4 -MaxSyncMs 80 -SourceTimeoutMs 250 `
  -MaxAlignmentTranslationM 0.25 -WorkspaceXMinM 2 -WorkspaceXMaxM 4
```

`fusion_cikis` sayısı yaklaşık 14-15/s artmalıdır. Bu ilk kabul testinde birleşik
veri henüz G1'e yönlendirilmez. Ortada yürüyüp, kolları sırayla kameraların
görmediği tarafa çevir: bilekler için `per_joint_contributions` birden fazla
kamerada artar; tek kamera kaybolsa diğer üçünden takip devam eder.

## E. G1/Isaac alıcısına yönlendirme (yalnız test geçince)

Mevcut G1 UDP alıcın ana PC `15050` portunu dinliyorsa D adımındaki komuta şu
iki parametreyi ekle:

```powershell
-OutputHost 127.0.0.1 -OutputPort 15050
```

Alıcı WSL ya da başka bir makinedeyse `127.0.0.1` yerine yalnız o alıcının IP
adresini yaz. Birleşik paket hâlâ `zed_body38_live/v1` şemasındadır; ayrıca
`fusion.implementation=application_level_weighted_body38/v1` alanıyla
izlenebilir. Gerçek robot/aktüatör kontrolünü bu kabul testi tamamlanmadan
başlatma.

## Sorun bulma

- `kaynak=2/4`: O kameranın kaynak penceresi açık değil, yanlış seri açılmış
  veya PC güvenlik duvarı laptop UDP paketini engelliyor. Laptop komutlarının
  hedefi mutlaka `192.168.50.10`; PC yerel komutlarının hedefi `127.0.0.1`.
- `ham_kayit=0`: Dört kaynağın hepsinde aynı anda geçerli operatör iskeleti
  yoktur. Kameraları ortak kesişime çevir, ortada görünür ol ve her kaynakta
  BODY_38 kilidinin oluşması için birkaç saniye bekle.
- Kalibrasyon RMS yüksek: kameralar/tripodlar oynamış olabilir veya aynı anda
  farklı kişiler eşleştirilmiştir. Odada tek kişiyle ortak hacmi yavaşça
  dolaşarak yeni ham kayıt alın.
- USB yırtılması, FPS düşüşü: önce yalnız sorunlu kamerayı doğrudan USB 3
  porta bağlayıp ZED Diagnostic ile doğrula. ZED360/UDP kodu bozuk görüntüyü
  düzeltemez.

ZED Diagnostic temiz olduğu halde eski satır-süreksizliği sezgisi masa,
pencere veya raf gibi yatay kenarlarda yanlış pozitif verebilir. Dört-kamera
otomatik başlatıcısı bu nedenle varsayılan olarak `-FrameIntegrityMode off`
kullanır; SDK'nin gerçek `grab` hataları yine yakalanır. Ek tanı gerekirse
kaynak komutuna `-FrameIntegrityMode monitor` eklenebilir. Bu mod şüpheli
kareleri UDP durum paketi olarak işaretler fakat BODY_38 testini kesmez. Bu
modda G1'e `OutputHost` ile veri yönlendirmeyin; önce alıcıdaki dört-kamera ve
kalibrasyon sonuçlarını doğrulayın.

Bu mod gerekirse dört kaynak komutunun tamamına uygulanmalıdır. Alıcıdaki
`durum=...` sayısı yalnız bu tanı paketlerini sayar; `gecersiz=0` ise UDP BODY_38
paketleri bozuk değildir.
