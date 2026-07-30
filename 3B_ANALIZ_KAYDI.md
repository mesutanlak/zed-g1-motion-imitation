# ZED BODY_38 3B analiz kaydı

Bu kayıt katmanı yalnızca algı verisini ölçer; G1'e motor veya eklem komutu
göndermez. ZED görüntü kaydı (`SVO2`), normal BODY_38 kaydı ve bu analiz kaydı
birbirinden bağımsızdır.

## Başlatma

Birinci PowerShell penceresinde paneli açın:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_g1_skeleton_panel_wsl.ps1
```

Dosya adı geçmiş uyumluluk nedeniyle `_wsl` içerir; panel varsayılan olarak
Windows üzerinde yerel Tkinter penceresinde çalışır. Böylece WSLg
`[WARN:COPY MODE]` sorunu analiz ekranını etkileyemez. GMR/Isaac ve ROS 2
verileri Ubuntu-22.04'e gitmeye devam eder.

Panelde `S` tuşu analizi kayda alır; tekrar `S` kaydı güvenli biçimde kapatır.
Panel açıldığı anda kayda başlamak için:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_skeleton_panel_wsl.ps1 -Record
```

Panel yanlış monitörde açılırsa ekranı açıkça seçebilirsiniz:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_skeleton_panel_wsl.ps1 `
  -Record -ScreenIndex 0
```

Başlatıcı WSLg'nin pencereyi görünmeyen sanal monitöre taşımasına yol açabilen
maksimize modunu kullanmaz; pencereyi seçilen ekranın görünür alanına sabitler.
Ayrıca WSLg pencereyi PowerShell'in arkasında oluşturursa Windows tarafındaki
yardımcı süreç paneli otomatik olarak öne getirir.

İkinci PowerShell penceresinde ZED yayınını başlatın:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_zed_live_smooth_to_wsl.ps1 `
  -Profile shared_gpu_safe
```

Dosyalar `C:\Users\Misafir\Desktop\ZED_G1_Projesi\analysis_recordings`
klasörüne yazılır.

## Üretilen dosyalar

- `body38_analysis_YYYYMMDD_HHMMSS.jsonl`: her BODY_38 karesinin eksiksiz
  kaynak paketi ve türetilmiş kinematik verileri.
- `body38_analysis_YYYYMMDD_HHMMSS_summary.csv`: Excel, pandas veya MATLAB ile
  hızlı grafik çizmek için kare başına özet.

JSONL kaydı ayrıca `NO_BODY`, `LOW_QUALITY` ve bozuk-kare durumlarını saklar.
Böylece örtülme anları başarılı kareler arasından kaybolmaz.

## Her karede saklanan ölçümler

Kaynak ZED alanları:

- 38 noktanın ham ve filtreli dünya koordinatları;
- 2B piksel koordinatları ve nokta güvenleri;
- pelvis konumu ve global kök quaternionu;
- yerel eklem konumları ve ZED yerel eklem quaternionları;
- köke göre ve omuz genişliğine göre normalize noktalar;
- IMU, mesafe, kaynak FPS ve capture-to-UDP gecikmesi.

Türetilmiş analiz alanları:

- pelvis merkezli 38 nokta ve her noktanın `x/y/z` hızı;
- omuz genişliği, kalça genişliği, üst/alt kol, uyluk ve baldır uzunlukları;
- dirsek, diz, kalça, ayak bileği ve bilek-proxy iç açıları;
- omuz ve kalçanın gövde eksenine göre fleksiyon/abduksiyon açıları;
- gövde öne/yan eğimi ile baş yaw/yan eğimi;
- ZED quaternionlarının yalnız analiz amaçlı Euler XYZ görünümü;
- eklem açısal hızları, ham-filtre farkı, eksik/düşük güvenli noktalar;
- elin gövde izdüşümüne girmesi ve karşı tarafa geçmesi için örtülme bayrakları.

`joint_angles_deg` insan iskeletinin geometrik/anatomik açılarıdır. Bunlar G1'in
23 motor açısı değildir. G1 motor hedefleri, bu kayıttan sonra GMR ve G1 eklem
limitleri kullanılarak ayrı retargeting aşamasında hesaplanmalıdır.

## Doğrulama

```powershell
wsl -d Ubuntu-22.04 -- bash -lc "cd /mnt/c/Users/Misafir/Desktop/ZED_G1_Projesi && python3 analysis_panel/test_skeleton_analysis_recorder.py"
```

Beklenen sonuç `ANALYSIS_RECORDER_OK` çıktısıdır.

## RViz görünümü

Özel 3B panel ROS 2 yayıncısı değildir. RViz için ayrı BODY_38 köprüsü kullanılır.
Üç ayrı PowerShell penceresindeki sıralama:

```powershell
# 1. ROS 2 köprüsü
powershell -ExecutionPolicy Bypass -File .\start_g1_body38_ros_bridge.ps1

# 2. ZED (ROS köprüsüne UDP 15054 kopyasını da yollar)
powershell -ExecutionPolicy Bypass -File .\start_zed_live_smooth_to_wsl.ps1 `
  -Profile shared_gpu_safe

# 3. RViz
powershell -ExecutionPolicy Bypass -File .\view_g1_body38_rviz.ps1
```

Yayınlanan topicler:

- `/zed/body38/markers`: kemikler, eklem noktaları ve temel etiketler;
- `/zed/body38/keypoints`: 38 noktanın `PoseArray` görünümü;
- `/zed/body38/diagnostics`: canlılık, güven ve geçerli nokta sayısı.

RViz yapılandırmasının Fixed Frame değeri `zed_camera` olmalıdır.
