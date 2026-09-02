# 4 ZED 2i BODY_38 + Isaac + Rerun calistirma kaydi

Bu yol iki Windows bilgisayarinda dört ZED 2i kullanir. ZED360'in ag uzerinde
verdigi `WRONG BODY FORMAT` hatasina bagli degildir. Her ZED kendi hostunda
BODY_38 ve dusuk hizli JPEG onizleme yayinlar; ana PC extrinsic kalibrasyonu
uygulayip tek BODY_38 akisi uretir.

Sabit esleme:

| Host | ZED seri | BODY UDP | Onizleme UDP |
|---|---:|---:|---:|
| Laptop `192.168.50.11` | 39504762 | 16000 | 16100 |
| Laptop `192.168.50.11` | 34760587 | 16006 | 16106 |
| Ana PC `192.168.50.10` | 31571870 | 16002 | 16102 |
| Ana PC `192.168.50.10` | 33773329 | 16004 | 16104 |

## Gunluk calistirma

ZED Explorer, ZED360 ve eski publisher sureclerini kapat. Iki bilgisayarda da
proje klasorunde guncel kodu al:

```powershell
git pull --ff-only origin main
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
```

Laptopta iki kaynak penceresini otomatik ac:

```powershell
cd "C:\Users\MSI\Desktop\zed-g1-motion-imitation"
.\start_zed_four_sources.ps1 -Role Laptop -MainPcHost 192.168.50.10
```

Ana PC'de iki yerel kaynak penceresini otomatik ac:

```powershell
cd "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
.\start_zed_four_sources.ps1 -Role MainPc
```

Ana PC'de ayri bir PowerShell'de Rerun'i ac. Bu surec canli analizi gosterir ve
her oturumu otomatik olarak `rerun_recordings` altina `.rrd`, `.jsonl`, `.csv`
ve oturum ozeti olarak yazar:

```powershell
.\start_g1_rerun.ps1 -Mode live -ListenPort 15052 -LiveMaxHz 15
```

Ana PC'de ayri bir PowerShell'de Isaac/GMR'i ac:

```powershell
.\start_g1_isaaclab_live.ps1 -Mode upper_body -ImitationMode kinematic_debug -AcceptNvidiaEula -InputFps 15
```

Bu dort-kamera profili eklem uzayinda sabit-durus deadband'i kullanir: omuzda
yaklasik `0.018 rad`, dirsekte `0.022 rad`, bilekte `0.032 rad`. Gercek hareket
deadband'i astiginda hiz-duyarli filtre gecikmeyi azaltir.
`-StationaryDeadbandScale 0` yalniz A/B tani icin eski davranisi geri getirir.
Arka kol erisimi varsayilan
olarak aciktir; resmi eklem limitleri ve surekli govde/kapsul bariyeri kolun
govdenin icinden gecmesini engeller. Eski dar arka-erisim davranisi gerekirse
`-RestrictBackwardArms` ile secilebilir.

Son olarak ana PC'de 4-ZED fusion ve 2x2 arayuzu ac. Bu komut `dual json`
akisiyla ayni sekilde proje kokundeki `four json` klasorunde bulunan tek JSON'u
otomatik secer; ZED360 ve dogrulanmis BODY_38 extrinsic tiplerini ayirt eder:

```powershell
.\start_zed_four_fusion_to_wsl.ps1 -Record -MinimumSources 3 -FusionHz 15 -PreviewHz 10
```

`-MinimumSources 3`, tek gorusun anlik BODY kaybinda Isaac akisini kesmez;
ekranda ve kayitta her karenin gercek katkisi `fusion_katki=3/4` veya `4/4`
olarak kalir. Alici, dort kaynak canliyken dorduncu ayni-dongu paketini en
fazla 20 ms bekler ve `<=40 ms` sikiliktaki gercek 4'lu paketi daha yeni 3'lu
pakete tercih eder. Yalniz dort goruslu kare uretmek icin `-MinimumSources 4`
kullan; gunluk canli kontrol icin 3 emniyetli geri dusustur.

Fusion penceresi tuslari: `S` JSONL kaydini acip kapatir; `Q`/`Esc` guvenli
cikis yapar. Kaynak penceresindeki `R` yalniz o kameranin operator kilidini ve
yerel notr kalibrasyonunu sifirlar. Olculen kayit sirasinda gereksiz `R`
kullanma.

## Kayitlar ve olculecek alanlar

- `recordings/four_body38_fusion_YYYYMMDD_HHMMSS.jsonl`: tam fusion paketleri,
  dört kameranin fusion-world BODY_38 noktaları, confidence, kamera pozlari,
  senkron yayilimi, cross-view MPJPE/p95, kaynak/fusion FPS ve kuyruk kaybi.
- `rerun_recordings/`: Rerun `.rrd`, ham analiz JSONL/CSV ve oturum ozeti.
- Isaac/GMR `15050`: kontrol icin oncelikli ve kompakt fused BODY_38.
- Rerun `15052`: dört kamera ham iskeleti dahil tam analiz paketi.
- WSL/ROS `15054`: kompakt ek kopya.

Saglikli bir kayitta `bagli=4/4`, `gmr_kapi=READY`, `gecersiz=0`, `drop=0`, fusion FPS yaklasik
14-15 ve mumkun oldugunca cok `fusion_katki=4/4` beklenir. `cross_view_mpjpe_m`
dusuk olmalidir; 0.10 m uzeri kalibrasyon/ortak gorus kontrolu gerektirir.
Her kamera satirinda `lat` saat-ofseti duzeltilmis capture gecikmesi, `net`
yalniz ag kuyrugu, `clk` iki Windows hostu arasindaki tahmini saat farkidir.
Laptop satirindaki ham 70 ms degeri tek basina gercek ag gecikmesi sayilmaz.

Alıcı iki ayrı uyum metriği kaydeder:

- `cross_view_mpjpe_m`: statik extrinsic sonrasındaki **ham** kamera uyumu;
  kalibrasyon, zamanlama ve yanlış kişi kilidi tanısıdır.
- `post_alignment_mpjpe_m`: aynı operatöre ait görüşler ortak pelvis merkezine
  taşındıktan sonraki, gerçekten fusion'a giren iskelet uyumudur. Rerun'daki
  kamera iskeletleri ve birleşik BODY_38 bu hizalanmış noktaları kullanır.

Pelvis-yerel gövde biçimi uyuşmayan veya ortak pelvise taşınması 1 m'den fazla
gereken bir görüş farklı kişi/poz adayı sayılır ve o karede fusion dışına
alınır. Konsolda `fusion_katki` gerçek kabul edilen görüş sayısıdır. Aynı anda
birden fazla kişi varken dört bağımsız kaynak farklı kişilere kilitlenebilir;
tek-operatör deneyi için çalışma alanında yalnız hedef kişi bulunsun ve gerekirse
ölçümden önce dört kaynak penceresinde de `R` ile kilidi yenile.

Kayit bittikten sonra son oturumun sayisal kabul ozetini al:

```powershell
.\zed_four_camera_test\summarize_four_body38_recording.ps1
```

Belirli bir dosya icin `-InputPath ".\recordings\four_body38_fusion_....jsonl"`
verilebilir.

`start_zed_four_sources.ps1` doğrulanmış düzen için kare-bant sezgisini
varsayılan olarak `off` açar. ZED SDK'nin gerçek `grab` hataları yine kaynak
penceresinde hata olarak görünür. Tanı amaçlı eski sezgiyi açmak istersen
`-FrameIntegrityMode monitor`, şüpheli karede süreci durdurmak istersen
`-FrameIntegrityMode strict` ver. Normal odadaki masa/pencere gibi yatay
kenarlar artık USB bozulması diye sürekli yazdırılmaz.

## Tripod hareket ederse yeniden kalibrasyon

Tercih edilen resmi yol ZED360'da dört kamerayi BODY_18 ile kalibre edip
`Finish Calibration` sonucunu kaydetmektir. Kaynaklari durdurduktan sonra yeni
dosyayi proje kokundeki `four json` klasorune `fourkamera.json` adi ile koyun;
klasorde baska `.json` birakmayin. Runtime BODY_38 olarak devam eder:

```powershell
Copy-Item -LiteralPath "C:\Program Files (x86)\ZED SDK\tools\fourkamera.json" `
  -Destination ".\four json\fourkamera.json" -Force
.\start_zed_four_fusion_to_wsl.ps1 -ValidateCalibrationOnly `
  -ReferenceSerial 33773329
```

ZED360 sonucu uretemezse asagidaki BODY_38 tabanli uygulama kalibrasyonu
fallback'tir. Bu yolda odada yalniz tek kisi bulunmali ve tum ortak hacimde
45-60 saniye hareketli, cok pozlu kayit alinmalidir.

Kaynaklar acik, Isaac/Rerun/fusion alicisi kapali olsun. Ana PC'de 45-60 saniye
ham kalibrasyon kaydi al:

```powershell
.\zed_four_camera_test\start_distributed_receiver.ps1 `
  -Source "39504762:16000","31571870:16002","33773329:16004","34760587:16006" `
  -CalibrationRecord ".\recordings\four_body38_static_calibration_NEW.jsonl" `
  -Fps 15 -MinimumSources 4
```

Ortak gorus alaninda tek kisiyle farkli noktalarda T/A pozlari, bukulu dirsek
ve kollar onde/arkada hareketleri yapin. `ham_kayit` tercihen 400'u gecince
`Ctrl+C` yap. Sonra extrinsic ve world-pose dosyalarini uretip kalite kapisindan
gecirin:

```powershell
.\zed_four_camera_test\start_distributed_calibration.ps1 `
  -CapturePath ".\recordings\four_body38_static_calibration_NEW.jsonl" `
  -OutputPath ".\config\zed_four\distributed_body38_extrinsics_NEW.json" `
  -WorldPosesJsonl ".\config\zed_four\four_camera_world_poses_NEW.jsonl" `
  -ReferenceSerial 33773329 -Activate
```

Aktif kopyalar:

```text
config/zed_four/active_distributed_body38_extrinsics.json
config/zed_four/active_four_camera_world_poses.jsonl
four json/fourkamera.json
```

Yeni kalite kapisi yalniz nokta RMS'ine bakmaz. Her kameranin referansla ayni
hareketli pelvisi izlemesini de zorunlu tutar: referans hareketi `>=0.10 m`,
hareket genisligi orani `>=0.60` ve pelvis yol korelasyonu `>=0.75`. Bir ZED
arkadaki sabit kisiye kilitlenirse dusuk Kabsch RMS'i uretse bile kalibrasyon
yazilmaz. `-Activate`, kaliteyi gecen dosyayi `four json/fourkamera.json` olarak
da etkinlestirir; gunluk launcher ek `-Extrinsics` istemeden bunu okur.

Varsayilan `start_zed_four_fusion_to_wsl.ps1`, `four json` klasorundeki tek
ZED360 veya BODY_38 extrinsic dosyasini otomatik okur. Klasoru kullanmadan
BODY_38 fallback dosyasini acikca vermek de mumkundur:

```powershell
.\start_zed_four_fusion_to_wsl.ps1 -Record -MinimumSources 3 `
  -Extrinsics ".\config\zed_four\active_distributed_body38_extrinsics.json"
```

Kameralar/tripodlar hareket etmediyse yeniden kalibrasyon gerekmez.

Kalibrasyonu yalniz dogrulamak icin:

```powershell
.\start_zed_four_fusion_to_wsl.ps1 -ValidateCalibrationOnly `
  -Extrinsics ".\config\zed_four\active_distributed_body38_extrinsics.json"
```

## Kapatma sirasi

Once fusion penceresinde `Q`, sonra Rerun ve Isaac'te `Ctrl+C`, en son dört
kaynak penceresinde `Ctrl+C`. Boylece JSONL ve Rerun oturum dosyalari kapanir.
