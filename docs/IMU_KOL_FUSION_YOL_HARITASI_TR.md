# İki kol IMU + dört ZED BODY_38 yol haritası

Bu planın ilk hedefi iki dirseğin bükülme açısını güvenilir biçimde ölçmek, ikinci hedefi kameranın kolu kaybettiği anlarda bu ölçümü mevcut dört ZED 2i BODY_38 akışına yardımcı veri olarak katmaktır. İlk deneyler kayıt ve görüntüleme içindir; G1 hedeflerini değiştirme aşaması ayrı bir kabul kapısıdır.

## 1. Mevcut sisteme bağlanma noktası

Projede dört kamera iki Windows bilgisayarında çalışıyor. Ana PC'deki `zed_four_camera_test/distributed_body38_fusion.py` ortak dünya koordinatında `zed_body38_live/v1` üretir. Günlük çalışma sırası ve portlar `docs/FOUR_ZED_BODY38_RUNBOOK_TR.md` içindedir. Füzyon çıkışı yaklaşık 15 Hz; BODY_38'de sol/sağ omuz, dirsek, bilek sırasıyla 12/14/16 ve 13/15/17 indekslerindedir. Paket `RIGHT_HANDED_Z_UP_X_FWD` koordinatını, metre cinsinden 3B noktaları, güven skorlarını, eklem kuaternionlarını ve zaman damgasını taşır. `g1_reference_features.geometric_angles` içinde kamera kaynaklı `left_elbow_interior_deg` ve `right_elbow_interior_deg` zaten hesaplanır.

Önerilen veri yolu:

```text
Kollardaki 4 IMU → Raspberry Pi 4B → ayrı IMU UDP akışı + ham JSONL kaydı
                                             ↓
4 ZED → mevcut BODY_38 fusion → ana PC'de IMU eşleştirme/analiz katmanı
                                             ↓
                          kamera-only / IMU-only / birleşik karşılaştırması
                                             ↓
                         offline replay → RViz/Rerun → Isaac/MuJoCo
```

Başlangıçta kamera füzyon kodunun içine sensör sürücüsü koymayın: sensörler Pi üzerinde okunur; ana PC'de ayrı bir alıcı, IMU paketlerini birleşik BODY_38 ile eşleştirir. Analiz kayıtları çalışınca ve giriş sözleşmesi netleşince canlı kontrol için ayrı, sürümlü bir çıkış yapılır. Mevcut `zed_body38_live/v1` şemasının `imu` alanı dört giyilebilir IMU için hazır bir sözleşme değildir; mevcut `compact_live_packet()` alanları seçerek iletir. Bu alanı sessizce doldurmak veya kontrol paketini büyütmek yerine yeni şema/adaptör tasarlayın.

## 2. Sensörleri nereye takmalı?

Bir dirseğin açısı için **aynı kol üzerinde iki segment** gerekir: biri üst kolda, biri ön kolda. Sensörü eklem çizgisinin üzerine koymak yerine kemiğe yakın, sıkı ama rahat bir bantla sabitleyin. Her sensörün eksen işaretini ve seri/adresini fotoğraflayın; banda `L_UPPER`, `L_FOREARM`, `R_UPPER`, `R_FOREARM` etiketi koyun. Yumuşak doku kayması ve bant dönmesi ölçümü değiştirir.

| Aşama | Sol üst kol | Sol ön kol | Sağ üst kol | Sağ ön kol | Amaç |
|---|---|---|---|---|---|
| İlk doğrulama | BNO055 #1 | BNO055 #2 | Xsens | MPU6050 | Aynı tip BNO çiftiyle yöntemi doğrulama; sağ taraftaki karışık çifti kıyaslama |
| Gerekirse yeniden dağıtım | Xsens | BNO055 #1 | MPU6050 | BNO055 #2 | İki ön kolda yönelim sensörü, üst kollarda kamera desteği; kayıtlarla hangisinin iyi olduğunu karşılaştırma |

Sağ/sol ataması fiziksel kolaylığa göre ters çevrilebilir. Xsens'in **tam modeli ve bağlantısı** bilinmeden onun kuaternion, veri hızı, Pi sürücüsü veya protokolü varsayılmamalı. Xsens MTi, MTw Awinda ve DOT farklı kurulum yolları kullanır. İlk kurulumda cihaz üzerindeki model kodunu belirleyip kendi resmi yazılımında tek başına veri okuyun. Eğer cihaz yalnız ham ivme/jyro veriyorsa onu doğrudan BNO055 kuaternionuyla aynı ölçüm gibi kullanmayın.

MPU6050 6 eksenlidir; manyetometresi ve yerleşik mutlak yönelim çıkışı yoktur. İvme/jyro filtresi ve sıfır poz kalibrasyonu gerektirir. Özellikle baş yönü etrafındaki dönme zamanla sürüklenebilir. Bu nedenle sağ dirsek açısını uzun süreli, yalnız IMU kaynaklı kesin referans olarak kabul etmeyin. Kamera ile aralıklı düzeltme veya kinematik eklem kısıtı gerekir. İki koldan aynı kalitede, bağımsız uzun süreli açılar isteniyorsa daha sonra aynı sınıfta bir dördüncü yönelim sensörü edinmek işin en sade çözümüdür.

Sensörleri kolun dış/arka tarafındaki düz bir bölgeye, üst kolda dirsekten birkaç santimetre yukarıya, ön kolda dirsek ve bilek arasına koyun; kemik çıkıntısına, kasın en çok şiştiği yere ve dirsek kıvrımına koymayın. Sıfır poz ve fonksiyonel dirsek bükme kalibrasyonu, milimetrik yer seçiminden daha önemlidir. Raspberry Pi ve güç kaynağını bel/üst sırt tarafında taşıyın; kablo çekme kuvvetini doğrudan sensör kartına bindirmeyin. Uzun kol boyunca çıplak I²C kablosu sorun çıkarırsa kısa yerel kablo + uygun arayüz/ara denetleyici tasarımına geçin.

## 3. Raspberry Pi bağlantı ve ilk açılış

1. Raspberry Pi OS'ta I²C'yi açın (`sudo raspi-config` → Interface Options → I2C), yeniden başlatın ve `sudo apt install i2c-tools` ile tarama aracını kurun. Pi 4B'de fiziksel pin 3 SDA/GPIO2, pin 5 SCL/GPIO3, pin 1 3.3 V, pin 6 GND'dir. Pi GPIO girişleri 3.3 V toleranslıdır; kart üzerindeki VCC ve I²C pull-up devresini görmeden 5 V hattı bağlamayın.
2. **Sensörleri tek tek** bağlayıp `i2cdetect -y 1` ile adresi görün. İlk BNO055 varsayılan `0x28`; ikinci BNO055'in ADR/ADDR pinini kullanılan kartın şemasına göre değiştirip `0x29` yapın. MPU6050 tipik olarak `0x68` (AD0 yüksekse `0x69`) kullanır. İki BNO aynı adreste bırakılırsa birlikte okunamaz. Adres değiştirme yöntemi breakout karta bağlıdır.
3. `python3 -m venv .venv --system-site-packages` ile Pi'ye özgü ortam açın; kullanılan breakout'a uygun resmi BNO055/MPU6050 Python sürücülerini burada kurun. Önce her sensörün kimlik, ivme, açısal hız ve (varsa) kuaternionunu birkaç dakika terminalde okuyun; sonra birlikte deneyin.
4. BNO055'in Raspberry Pi I²C saat uzatma davranışı bazı kurulumlarda sorun çıkarır. Okumalar takılır veya hata verirse kablo/pull-up'ları kontrol edip Adafruit'in Pi için yazılımsal I²C çözümünü değerlendirin. I²C çoklayıcıyı varsayılan çözüm saymayın. USB/UART seçeneği yalnız kart ve toplam bağlantı planıyla uyumluysa kullanın.
5. Xsens'i modelinin desteklediği USB/seri/BLE yolu ile ayrı çalıştırın. MTi ailesi için resmi Linux SDK ve örnekleri var; giyilebilir Xsens modelleri aynı sürücüyü kullanmayabilir. Pi üzerinde sürücü desteği yoksa önce PC'den kayıt alma yolu da değerlendirilebilir.

İlk masa denemesi: dört cihazı aynı hareketsiz düzleme koyun; 2–5 dakika paket kaybı, kuaternion normu, jiroskop sıfırı ve ısınma kaynaklı kaymayı görün. Sonra tek tek 90° döndürün. BNO055 kalibrasyon durumunu (system/gyro/accel/mag) her kayıtta saklayın. Metal masa, güç kablosu veya robot gövdesi yakınında manyetik yönelim sıçramasını ayrıca deneyin.

## 4. Kodun yeri ve veri sözleşmesi

Yeni Pi kodunu `imu_capture/` altında tutun: `drivers/` (BNO055, MPU6050, modele özgü Xsens adaptörü), `capture.py` (tek örnekleme döngüsü), `calibration.py`, `publish.py`, `config.example.yaml`. Ana PC tarafı için `imu_fusion/receiver.py`, `alignment.py`, `elbow.py`, `quality.py`, `replay.py`; deney kayıtlarını `recordings/imu_*.jsonl`, kişiye/oturuma özel kalibrasyonları `config/imu/` altında tutun. Bağlantı ve kalibrasyon değerlerini kod içine gömmeyin.

Pi'den **ayrı UDP portunda** `arm_imu/v1` yayınlayın; portu mevcut ZED kaynak/önizleme/kontrol portlarıyla çakıştırmayın. Her örnekte en az şu alanlar olsun:

```json
{
  "schema": "arm_imu/v1",
  "session_id": "20260917_trial01",
  "sequence": 1234,
  "sensor_id": "bno_left_upper",
  "side": "left",
  "segment": "upper_arm",
  "pi_monotonic_ns": 123456789,
  "pi_utc_ns": 1780000000000000000,
  "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
  "gyro_rad_s": [0.0, 0.0, 0.0],
  "accel_m_s2": [0.0, 0.0, 9.81],
  "calibration": {"system": 3, "gyro": 3, "accel": 3, "mag": 3},
  "valid": true
}
```

Bu yalnız önerilen sözleşme örneğidir; sayısal değerler gerçek ölçüm değildir. BNO055 ve Xsens kuaternion sırası/koordinat tanımı sürücüde doğrulanıp `xyzw`'ye çevrilmeli. MPU6050 için güvenilir mutlak kuaternion yoksa alanı `null` bırakıp ham ölçümü yayınlayın. Her sensörün fiziksel konumu, cihaz modeli, yazılım sürümü, ölçek ayarı, örnekleme hızı ve kalibrasyon kimliği oturum üst bilgisinde yer alsın. Hatalı/eksik sensör verisini önceki geçerli örnekle sessizce değiştirmeyin.

Pi hedefi başlangıçta 50 Hz sensör örneklemesi ve her örneğe okuma anına yakın `time.monotonic_ns()` zaman damgasıdır; gerçek hız ve sırayla okuma gecikmesi kayıttan ölçülür. Kamera 15 Hz olduğu için her BODY_38 karesine en yakın IMU örneklerini ya da iki ölçüm arasındaki enterpolasyonu bağlayın. Üç bilgisayarın UTC saatlerini NTP/chrony ile yakın tutun; **UTC/monotonic saatleri doğrudan eşit varsaymayın**. Bilinen kol hareketiyle iki akışın gecikmesini çapraz korelasyonla ölçün, ofseti oturumda kaydedin. Paket geliş saati kamera çekim saati yerine kullanılmamalı.

## 5. Dirsek açısı hesabı ve kalibrasyon

Kamera baz çizgisi: aynı tarafın omuz `S`, dirsek `E`, bilek `W` noktaları ile iç açı `acos( dot(S−E, W−E) / (|S−E| |W−E|) )` bulunur. Düz kol yaklaşık 180° iç açı, dolayısıyla kullanılacak bükülme açısı yaklaşık `180° − iç açı` olur. Bu değer, görünürlük ve keypoint güveniyle birlikte kaydedilmeli. Projedeki `g1_reference_features.geometric_angles.*_elbow_interior_deg` iç açıdır; doğrudan G1 motor açısı değildir.

IMU tarafında önce her kartın sensör eksenini kola sabit segment eksenine dönüştürün (`q_world_segment = q_world_sensor ⊗ q_sensor_segment`). Üst kol ile ön kolun göreli yönelimini `q_rel = inverse(q_world_upper) ⊗ q_world_forearm` ile bulun. Düz kol referansını çıkarın; fonksiyonel bükme hareketiyle dirseğin bükülme eksenini tahmin edin. Göreli kuaternionun toplam dönme büyüklüğünü doğrudan dirsek fleksiyonu saymayın: ön kol pronasyon/supinasyonu ayrı bir eksendir.

Her takışta uygulanacak kısa protokol:

1. Sensörler sabitlenmişken 5–10 saniye hareketsiz durun; jiroskop ofseti ve BNO kalibrasyonunu kontrol edin.
2. Kollar düz, avuç yönü tanımlı A/T veya nötr pozda 3–5 saniye bekleyin; iki sensör arasındaki sıfır dönüşünü kaydedin.
3. Omuzu olabildiğince sabit tutarak her dirseği 5–10 kez yavaşça büküp açın; bükülme eksenini bu veriden çıkarın.
4. Ön kolu kendi ekseni etrafında döndürün; bunun bükülme ölçüsünü ne kadar bozduğunu görün. Sonra omuzu hareket ettirerek aynı testi tekrarlayın.
5. Bant yeniden takılır/dönerse sensör–segment kalibrasyonunu yenileyin. BNO manyetometre kalibrasyonu ortam değişince yeniden değerlendirilmeli.

## 6. Füzyon sırası ve kalite kapıları

**A. Sadece kayıt:** Kamera ve dört IMU aynı oturumda yazılsın. Birleştirilmiş açılar yalnız grafik/CSV olarak hesaplanıp `camera`, `imu`, `combined`, `difference_deg`, `age_ms`, `quality_reason` ayrı tutulsun.

**B. Çevrimdışı füzyon:** Kamera dirsek zinciri güvenilirken kamera açısı ve 3B konumu ana referans kalsın; IMU ile yüksek frekanslı hareket ve kısa örtülme aralıkları izlensin. Kamera güveni düşükken, kalibrasyonu iyi ve taze IMU ile kısa süreli tahmin yapılsın. MPU6050 tarafında sürüklenmeyi kamera yeniden görünür olduğunda düzeltin. IMU tek başına bileğin dünya konumunu vermez; omuz/dirsek kamera ve kol uzunluğu kısıtlarıyla birlikte kalmalıdır.

**C. Canlı gölge mod:** Ana PC alıcısı birleşik açıyı ve kalitesini ayrı yayınlasın; mevcut `15050` GMR/Isaac girişini henüz değiştirmesin. Rerun veya offline grafikte kamera akışıyla aynı anda izleyin. Paket bayat, kalibrasyon düşük, manyetik sıçrama veya iki kaynak arasında büyük fark varsa IMU katkısını sıfırlayın; aniden yeni hedefe sıçramayın.

**D. Simülasyon ve kontrol:** Yalnız A–C testleri geçince, sürümlü adaptör üzerinden retargeting girişine alın. Mevcut hız/ivme/eklem/self-collision ve watchdog sınırları korunmalı; önce Isaac/MuJoCo, sonra fiziksel robot değerlendirilmelidir.

Önerilen karşılaştırma deneyleri: sabit 0°/45°/90°/yaklaşık 120° pozlar (mekanik açıölçerle); yavaş ve hızlı tekrar; omuz sabit/omuz hareketli; ön kol pronasyonu; kamera örtülmesi; sensörün yeniden takılması; metal/elektrik yakınında BNO yönelim değişimi; 10 dakika uzun kayıt. Ölçümler: bağımsız açıölçere göre MAE ve p95, kamera–IMU farkı, hareketsiz açı dağılımı, 10 dakikalık sürüklenme, zaman hizası p95, paket kaybı, bayat örnek yüzdesi, kamera kaybı süresince toparlanma. Hedefleri ilk veriyle belirleyin; tek bir video veya yalnız kamera açısını mutlak gerçek kabul etmeyin.

## 7. İlk uygulama sırası

1. Xsens modelini ve breakout kartlarının fotoğraflarını/etiketlerini kaydedin; Pi üzerinde tek tek algılama ve kayıt yapın.
2. İki BNO055'i aynı kola takıp nötr + fonksiyonel kalibrasyonla yalnız dirsek bükülmesini çıkarın. Kamera `*_elbow_interior_deg` ve açıölçerle karşılaştırın.
3. Xsens + MPU6050 ile diğer kolu ekleyin; MPU sürüklenmesini özellikle uzun kayıtta ölçün.
4. Pi'den ayrı `arm_imu/v1` akışını ana PC'ye ve ham JSONL'ye verin. Kamera kayıtlarıyla zaman eşleştirmesini offline çözün.
5. Gölge modda kayıplı/örtülü deneyleri geçirin; ancak sonra simülasyon retargeting adaptörünü açın.

## Teknik kaynaklar

- [ZED BODY_38 keypoint ve orientation açıklaması](https://docs.stereolabs.com/docs/development/zed-sdk/modules/body-tracking)
- [Raspberry Pi GPIO ve 3.3 V sınırları](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
- [BNO055 adresi, quaternion ve kalibrasyon](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/faqs)
- [BNO055/Pi I²C saat uzatma notları](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching)
- [MPU6050 teknik özellikleri](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf)
- [Xsens yazılım ve modele göre SDK'lar](https://www.xsens.com/support/software-documentation)
- [İki IMU ile dirsek açısı ve sensör–segment kalibrasyonu araştırması](https://pmc.ncbi.nlm.nih.gov/articles/PMC9785932/)
