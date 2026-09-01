# ZED 2i BODY_38 → G1 23-DOF Skeleton Extractor

Windows 11 üzerindeki ZED 2i BODY_38 iskeletini WSL/GMR üzerinden G1 EDU
23-DOF referansına dönüştüren; Isaac Lab, MuJoCo ve ROS 2/RViz köprüleri
içeren araştırma projesidir.

Yeni bilgisayar kurulumu için önce [INSTALL_TR.md](INSTALL_TR.md) belgesini
izleyin. Otomatik kurulum ve doğrulama betikleri `install/` klasöründedir.
Windows/WSL, ZED SDK, Isaac Lab, tek/çift kamera, Rerun, ROS 2/RViz ve MuJoCo'yu
tek akışta kurmak için ayrıntılı
[yeni bilgisayar kurulum ve çalıştırma rehberini](docs/YENI_BILGISAYAR_KURULUM_VE_CALISTIRMA_TR.md)
kullanın.

Operatör kilidi, 4 saniyelik antropometrik kalibrasyon, pelvis-yerel
koordinatlar, G1 `raw_q/safe_q`, Rerun telemetrisi ve safety durumları için
[güvenli canlı taklit hattı](docs/ZED_G1_SAFE_MIMIC_PIPELINE_TR.md) belgesini
izleyin.

İki ZED 2i ile resmî Fusion BODY_38, güvenli tek-görünüm fallback'i ve
tek/çift kamera deney metrikleri için
[dual ZED Fusion rehberini](docs/DUAL_ZED_BODY38_FUSION_TR.md) izleyin. Dual
yol ayrıdır; mevcut tek-kamera başlatıcısı değiştirilmemiştir.

Dört ZED 2i'nin iki bilgisayara dağıtıldığı canlı BODY_38 Fusion, 2x2 kamera
arayüzü, Isaac/GMR, Rerun, JSONL kaydı ve yeniden kalibrasyon akışı için
[dört ZED BODY_38 çalıştırma kaydını](docs/FOUR_ZED_BODY38_RUNBOOK_TR.md)
izleyin. ZED360/native ağ Fusion tanı adı ayrı olarak
[zed_four_camera_test/README_TR.md](zed_four_camera_test/README_TR.md)
içindedir.

> Fiziksel robot güvenliği: Bu depo doğrudan gerçek G1 motor kontrolü için
> hazır değildir. Varsayılan akış simülasyon ve üst gövde takibidir. Sim2real
> öncesinde tork/hız limitleri, watchdog, self-collision, düşme engelleme,
> askı ve fiziksel acil durdurma doğrulanmalıdır.

Bu uygulama ZED 2i'nin resmî BODY_38 modelini kullanarak insan iskeletini
G1 23-DOF retargeting aşamasına uygun bir veri biçiminde çıkarır.

**Güvenlik:** Uygulama yalnızca algılama yapar. Unitree SDK'ye bağlanmaz ve
robota motor komutu göndermez.

## Resmî kaynaklar

- [Stereolabs Body Tracking](https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking)
- [Using the Body Tracking API](https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api)
- [Stereolabs Tutorial 8](https://github.com/stereolabs/zed-sdk/tree/master/tutorials/tutorial%208%20-%20body%20tracking/python)
- [Stereolabs SVO recording](https://www.stereolabs.com/docs/development/zed-sdk/modules/camera/recording)
- [ZED ROS 2 body tracking parameters](https://www.stereolabs.com/docs/integrations/ros-2/zed-stereo-node#body-tracking-parameters)
- [Unitree RL MjLab](https://github.com/unitreerobotics/unitree_rl_mjlab)
- [Unitree XR Teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)

## Çalıştırma

Önce ZED Depth Viewer, ZED Explorer veya kamerayı kullanan diğer programları
kapatın. ZED 2i'yi doğrudan bir USB 3.x porta bağlayın.

PowerShell:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
python .\zed_g1_skeleton.py --list-devices
python .\zed_g1_skeleton.py
```

Alternatif olarak `run_zed_g1_skeleton.bat` dosyasına çift tıklayın.

Tuşlar:

- `S`: JSONL kaydını aç/kapat
- `R`: takip edilen kişi kimliğini bırak ve en yakın geçerli kişiye yeniden kilitlen
- `D`: ham veri/tanı panelini açıp kapat
- `Q` veya `Esc`: güvenli çıkış

Hız öncelikli kullanım:

```powershell
python .\zed_g1_skeleton.py --model fast --fps 60
```

Kalite öncelikli kullanım:

```powershell
python .\zed_g1_skeleton.py --model accurate --fps 30
```

On saniyelik başsız test:

```powershell
python .\zed_g1_skeleton.py --headless --record --seconds 10
```

JSONL ile birlikte ZED'in tekrar işlenebilir yerel SVO2 kaydını almak:

```powershell
python .\zed_g1_skeleton.py --model accurate --fps 30 --record --record-svo2
```

SVO2; stereo görüntüyü ve ZED sensör zamanlamasını korur. BODY_38'in sınırlı
el noktaları Dex3-1'in el başına 7 motorunu çözmeye yetmediği için, daha sonra
21-landmark el takibini yeniden çalıştırabilmek açısından SVO2 önerilir.

Ekrandaki tanı paneli şunları canlı gösterir:

- insana Öklid 3B mesafesi ve kameradan ileri `X` mesafesi;
- pelvis/root `X Y Z`, body ID ve body confidence;
- confidence eşiğini geçen keypoint sayısı;
- ham sol/sağ bilek ve ayak bileği koordinatları;
- gövde/kol/bacak grup geçerlilikleri;
- ZED 2i IMU açısal hız ve doğrusal ivmesi;
- JSONL kare sayısı ve SVO2 kayıt durumu.

Varsayılan uygun mesafe 2–4 metredir. İstenirse değiştirilebilir:

```powershell
python .\zed_g1_skeleton.py --distance-min 2.5 --distance-max 4.5
```

## Çıktı

Kayıt açıldığında `recordings\zed_body38_*.jsonl` oluşturulur. Her satırda:

- zaman damgası ve kare sırası;
- kilitlenen kişinin ZED `body_id` değeri;
- ham ve filtreli 38 adet 3B keypoint;
- keypoint confidence değerleri;
- pelvis/root pozisyonu ve global rotasyonu;
- yerel eklem pozisyonları ve quaternion rotasyonları;
- pelvis koordinat sistemine dönüştürülmüş iskelet;
- omuz genişliğine göre normalize edilmiş iskelet;
- G1 için kol/bacak geçerlilik bayrakları, uzuv uzunlukları ve geometrik
  dirsek/diz açıları bulunur.

`geometric_angles` motor komutu değildir. G1'e bağlanmadan önce robot
kinematiği, eklem limitleri, self-collision ve denge policy'si kullanan ayrı bir
retargeting katmanı gereklidir.

Kayıt kapatılmadan önce ekrandaki `Kayıt=AÇIK N kare` sayacının arttığını
doğrulayın. `Q` veya `Esc` ile güvenli çıkış yapıldığında tamponlanan veriler
diske yazılır. Bir kaydı doğrulamak ve analiz çıktısı üretmek için:

```powershell
python .\analyze_body38_recording.py .\recordings\zed_body38_YYYYMMDD_HHMMSS.jsonl
```

Araç, `recordings\analysis` altında bir kalite özeti (`*_analysis.json`) ve
kare bazında tablo (`*_frames.csv`) oluşturur. Analiz; grup görünürlüğü,
zamanlama/jitter, keypoint hızı, kemik uzunluğu tutarlılığı, quaternion normu ve
Dex3 kaynak görünürlüğünü ölçer. `whole_body_candidate`, `upper_body_only` veya
`partial_or_retake` kararı üretir. Bunlar algılama tutarlılığı ölçüleridir;
motion-capture ground truth olmadan mutlak anatomik doğruluk anlamına gelmez.
Kayıt durdur/başlat araları ve 0,5 saniyeyi geçen takip boşlukları ayrı segment
sayılır; rapordaki `effective_fps` yalnızca aktif segmentlerden hesaplanır.

Tüm kayıtları tek tabloda karşılaştırmak için:

```powershell
python .\analyze_recording_collection.py .\recordings
```

Sonuçlar `recordings\analysis\collection_analysis.json` ve `.csv` dosyalarına
yazılır. Mevcut kayıt değerlendirmesi ve sonraki aşamalar
`BASELINE_AND_ROADMAP_TR.md` dosyasındadır.

G1 23-DOF + iki Dex3-1 için veri sözleşmesi
`config\g1_23dof_dex3.json`, aşamalı ROS/RViz/MuJoCo tasarımı ise
`SYSTEM_ARCHITECTURE.md` içindedir.

## Canlı MuJoCo taklidi

Tek ZED 2i'den Windows → WSL canlı BODY_38 aktarımı ve resmî G1-23DOF
modelinde sabit tabanlı IK taklidi hazırdır. Önce kayıtla oynatma, ardından
iki terminalli canlı kullanım ve güvenlik sınırları
`SIMULATION_LIVE_MIMIC_TR.md` dosyasında açıklanmıştır.

## Kamera yerleşimi

- Kamera sabit tripod üzerinde olmalı.
- İnsan tamamen kadraja girmeli; eller ve ayaklar kesilmemeli.
- Başlangıç mesafesi yaklaşık 2.5–4 metre olabilir.
- Zemin görünmeli ve yeterli, homojen aydınlatma bulunmalı.
- İlk testte kadrajda yalnızca bir kişi olmalı.

Program `RIGHT_HANDED_Z_UP_X_FWD` ve metre kullanır:

- `X`: kameradan ileri
- `Y`: sol
- `Z`: yukarı

Bu seçim ROS REP-103 ve sonraki G1 retargeting işlemleriyle uyumludur.

## Geliştirici doğrulaması

Kamera veya retargeting ayarlarını değiştirmeden önce ve sonra aynı kayıtla tam
regresyon paketini çalıştırın:

```powershell
powershell -ExecutionPolicy Bypass -File .\validate_motion_pipeline.ps1 `
  -Recording .\recordings\zed_body38_YYYYMMDD_HHMMSS.jsonl `
  -TargetFps 30
```

Komut; ZED şema testi, operatör/kalibrasyon/oklüzyon/safety testleri, Rerun kayıt
testi, capture benchmark'ı ve WSL içindeki gerçek GMR replay testini çalıştırır.
JSON/CSV sonuçları `reports` klasörüne yazılır. Ayrıntılı yöntem ve test kapıları
`docs/DEVELOPER_VALIDATION_PLAN_TR.md` içindedir.
