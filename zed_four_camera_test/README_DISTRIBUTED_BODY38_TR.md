# Laptop USB hub + ana PC USB: 4 ZED 2i BODY_38 akışı

Bu belge, seçtiğin fiziksel düzen için üretim-deney akışıdır:

```text
ZED 39504762 + ZED 31571870 -> laptop USB 3 hub -> laptop
                                                  | CAT6 Ethernet
ZED 33773329 + ZED 34760587 -> ana PC USB -------+-> ana PC alici/fusion
```

Bu yol ZED360/`sl.Fusion` Network Workflow'u kullanmaz. Bu iki hostta ortaya
çıkan `WRONG BODY FORMAT` hatasını atlar; ancak ZED'in **BODY_38** algılamasını
korur ve sonuçta mevcut G1 alıcılarının bildiği `zed_body38_live/v1` UDP paketi
üretir. Fiziksel robota komut göndermez.

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
  -Fps 15 -Model medium -DepthMode performance
```

```powershell
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 31571870 -TargetHost 192.168.50.10 -TargetPort 16002 `
  -Fps 15 -Model medium -DepthMode performance
```

Ana PC'de, `C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation`
dizininde. Yerel kameralar da aynı UDP biçimiyle **loopback** üzerinden ana
PC alıcısına gider:

```powershell
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 33773329 -TargetHost 127.0.0.1 -TargetPort 16004 `
  -Fps 15 -Model medium -DepthMode performance
```

```powershell
.\zed_four_camera_test\start_distributed_source.ps1 `
  -Serial 34760587 -TargetHost 127.0.0.1 -TargetPort 16006 `
  -Fps 15 -Model medium -DepthMode performance
```

Her pencerede `Acik ZED seri numarasi: ...` yazısı kendi seri numarasıyla
eşleşmelidir. Başka seri numarası görülürse tüm kaynakları `Ctrl+C` ile kapat;
o kamerayı tutan eski uygulamayı kapat ve yalnız o komutu yeniden başlat.

## B. Tripod yerleşimi ve ilk kalibrasyon kaydı

Kameraları dört köşeye koy; her biri operatörün orta çalışma alanını görsün.
En az iki kameranın her omuzu, dirseği ve bileği görmesi gerekir. Tripodlar
kalibrasyon tamamlanana kadar hareket etmemeli.

Ana PC'de beşinci PowerShell penceresinde aşağıdaki alıcıyı aç:

```powershell
.\zed_four_camera_test\start_distributed_receiver.ps1 `
  -Source "39504762:16000","31571870:16002","33773329:16004","34760587:16006" `
  -CalibrationRecord ".\recordings\four_body38_static_calibration.jsonl" `
  -Fps 15
```

Konsolda `DURUM | kaynak=4/4 [...] | ham_kayit=...` görmelisin. `ham_kayit`
sayısı artıyorsa ağ ve dört BODY_38 kaynağı doğrudur.

Ortadaki ortak görüş alanında 25-30 saniye boyunca T-pozda veya kollar hafif
açık, **olabildiğince sabit** dur. Bu sırada yürüme, dönme ya da tripod
oynatma. Sonra ana PC alıcısında `Ctrl+C` yap. Dört kaynak penceresi açık
kalsın.

Bu kalibrasyon, operatörün aynı anki iskelet noktalarından her kamerayı
`33773329` kamera koordinatına dönüştürür. Bu yüzden kalibrasyon sırasında
hareket etmek kaliteyi düşürür.

## C. Extrinsic dosyasını üret

Ana PC'de:

```powershell
.\zed_four_camera_test\start_distributed_calibration.ps1 `
  -Input ".\recordings\four_body38_static_calibration.jsonl" `
  -Output ".\config\zed_four\distributed_body38_extrinsics.json" `
  -ReferenceSerial 33773329
```

Her referans dışı kamera için `rms` ve `p95` değerleri yazılır. Hedef, RMS'in
`0.10 m` veya daha düşük olmasıdır. `0.16 m` üstünde dosya kasıtlı olarak
oluşturulmaz; tripod açılarını/ortak görünümü düzeltip B adımını tekrarla.
Kamera veya tripod sonradan hareket ederse bu JSON geçersiz olur.

## D. Dört kamera BODY_38 birleşimini test et

Kaynak dört pencere açıkken, B adımında kapattığın alıcıyı bu defa kalibrasyon
dosyasıyla tekrar çalıştır:

```powershell
.\zed_four_camera_test\start_distributed_receiver.ps1 `
  -Source "39504762:16000","31571870:16002","33773329:16004","34760587:16006" `
  -Extrinsics ".\config\zed_four\distributed_body38_extrinsics.json" `
  -Fps 15
```

`fusion_cikis` sayısı yaklaşık 15/s artmalıdır. Bu ilk kabul testinde birleşik
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
  farklı pozlar eşleştirilmiştir. Yeni ham kayıt alırken sabit dur.
- USB yırtılması, FPS düşüşü: önce yalnız sorunlu kamerayı doğrudan USB 3
  porta bağlayıp ZED Diagnostic ile doğrula. ZED360/UDP kodu bozuk görüntüyü
  düzeltemez.

ZED Diagnostic temiz olduğu halde uygulamanın satır-süreksizliği tanısı yanlış
pozitif verirse, yalnız ham kalibrasyon/fusion kabul testi için kaynak komutuna
`-FrameIntegrityMode monitor` eklenebilir. Bu mod şüpheli kareleri UDP durum
paketi olarak işaretler fakat BODY_38 testini kesmez. Bu modda G1'e `OutputHost`
ile veri yönlendirmeyin; önce alıcıdaki dört-kamera ve kalibrasyon sonuçlarını
doğrulayın.
