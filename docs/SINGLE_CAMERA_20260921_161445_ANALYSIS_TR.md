# Tek kamera kaydı 20260921_161445 — BODY_38, GMR ve el takibi incelemesi

## Sonuç

Bu oturumda MediaPipe el takibi çalışmamıştır. Kod tabanı MediaPipe için ROI,
21 landmark, ZED depth örnekleme, çoklu kamera füzyonu ve Dex3 retargeting
bileşenlerini içerir; ancak kaydı başlatan Ubuntu komutu `--hand-tracking`
almamış, ZED ortamında MediaPipe kurulu olmamış ve resmi
`hand_landmarker.task` modeli bulunmamıştır.

Bu nedenle kayıtta görülen el noktaları MediaPipe'ın 21 el landmark'ı değildir.
Bunlar ZED BODY_38'in her el için sağladığı bilek ve dört kaba temsil
noktasıdır. BODY_38'teki `left_dex3_source` ve `right_dex3_source` oranları da
MediaPipe veya Dex3 yedi motor hedefi kapsamı anlamına gelmez.

## BODY_38 kaydı

- 1.683 geçerli kare, bozuk satır yok.
- Süre 56,34 saniye; etkin hız 29,86 FPS.
- Medyan kare aralığı 33,37 ms; tahmini sekiz kamera karesi kayıp.
- Bütün karelerde tracking durumu `OK`.
- Ortalama gövde güveni yüzde 96,88.
- Önerilen 2–4 m aralığında kalma yüzde 96,02.
- Üst gövde gözlenebilirliği yüzde 98,93.
- Tüm gövde çekirdek gözlenebilirliği yüzde 94,41.
- Sol/sağ kol gözlenebilirliği yüzde 99,29 / yüzde 98,93.
- BODY_38 kaba Dex3 kaynak gözlenebilirliği yüzde 90,79 / yüzde 95,07.
- 63.954 quaternion örneğinde ortalama norm hatası `7,53e-9`.
- 500 derece/s üstündeki açısal sıçrama 18; toplamın yüzde 0,028'i.

Bu kayıt önceki 15 FPS kaydından daha iyi kol ve el görünürlüğü sağlamıştır.
Yine de BODY_38 yalnız kaba parmak temsil noktaları verdiği için parmak eklem
açılarını güvenilir biçimde çıkarmaya yetmez.

## GMR ve Isaac izleme

Kalite özetindeki 605 senkron Isaac örneğinde:

- BODY izleme MPJPE ortalama 2,60 cm; p90 6,64 cm.
- GMR üst gövde göreli kalıntısı ortalama 4,02 cm; p90 9,13 cm.
- Robot eklem izleme RMSE ortalama 0,00171 rad.
- Toplam kontrol gecikmesi ortalama 84,15 ms; p90 91,29 ms; p99 98,90 ms.
- Isaac fizik döngüsü ortalama 53,38 Hz.
- Güvenlik: 403 GREEN, 158 YELLOW, 44 ORANGE.
- En sık nedenler: sol bilek örtülmesi 151, robot-gövde bariyeri 63,
  self-collision riski 47 ve orange safe return 17.

Sol kolun uç hata dağılımı sağ koldan daha kötü: sol el konum hatası ortalama
6,80 cm, sağ el 5,11 cm. Sol direct IK p90 değeri 5,48 cm iken sağda 1,75
cm'dir. Bu asimetri sol bilek örtülmesi ve gövde önündeki bariyer müdahaleleri
ile uyumludur.

## Rerun oturum kapanışı

Manifest 660 BODY karesi ve 605 imitation karesi gösterirken kayıp olmayan
JSONL dosyalarında sırasıyla 671 ve 612 satır vardır. Manifestte
`closed_unix_ns` bulunmadığı için oturum normal `close()` yolundan tamamlanmamış;
son periyodik snapshot'tan sonraki birkaç satır JSONL'ye yazılmıştır. Veri
kaybı işareti değildir, ancak bu oturumun manifest ve kalite özetinin son yedi
imitation satırını içermediğini gösterir. Yeni oturumda Rerun'ı terminalde
Ctrl+C ile bir kez kapatıp `Analiz oturumu:` mesajını beklemek gerekir.

## MediaPipe düzeltmesi

- Ubuntu ZED ortamına `mediapipe==0.10.35` kuruldu; `pip check` temiz.
- `install/setup_hand_tracking_ubuntu24.sh` eklendi.
- `start_zed_native_ubuntu.sh` el modeli, delegate ve çıkarım hızı seçeneklerini
  alıyor.
- Ham tek kamera el paketi ayrı el kanalının yanında Rerun analiz hedefine de
  ekleniyor; GMR BODY_38 kontrol paketi büyütülmüyor.
- Rerun 21 landmark yerel el şeklini gösteriyor ve kaynak şemasını açıkça
  kaydediyor.
- El benchmark aracı tek kamera `zed_operator_hand/v1` paketlerini ölçüyor.

Kalan harici gereksinim resmi `hand_landmarker.task` model dosyasıdır.
