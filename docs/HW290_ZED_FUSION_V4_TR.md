# HW-290 sol kol → ZED BODY_38 füzyonu (v4)

## Neden segment açısı

`fourson.mp4` kaydındaki hareketlerde kol baş üstüne, yana, öne ve gövde önünde
çapraza taşınıyor. Ön kol ayrıca kendi uzun ekseni etrafında dönüyor. V4 ana
dirsek değerini iki fiziksel kol parçasının kalibre edilmiş uzun eksenleri
arasındaki açı olarak hesaplar:

- omuzun ve bütün kolun ortak 3B hareketi açıya eklenmez;
- ön kol pronasyon/supinasyonu fleksiyon sayılmaz;
- iki kasa arasındaki küçük sabit montaj farkı düz kol nötr pozunda çıkarılır;
- toplam bağıl 3B dönüş tanı alanında korunur.

IMU mutlak konum üretmez. Omuz, dirsek ve bilek konumları ZED BODY_38'den gelir.
IMU; dirsek açısı, açısal hız, kısa süreli yönelim ve kamera kapanması sırasında
kalite kontrollü yedek ölçüm sağlar.

## Raspberry Pi'ye kopyalanacak dosyalar

Windows PowerShell:

```powershell
scp "imu_capture/hw290_elbow_quaternion.py" mesut@10.42.0.10:/home/mesut/imu_test/
scp "imu_capture/hw290_six_face_calibration.py" mesut@10.42.0.10:/home/mesut/imu_test/
scp "imu_capture/dual_hw290_test.py" mesut@10.42.0.10:/home/mesut/imu_test/
scp "imu_capture/arm_imu_protocol.py" mesut@10.42.0.10:/home/mesut/imu_test/
```

Komutlar proje kökünden çalıştırılmalıdır. Pi adresi değiştiyse `10.42.0.10`
yerine `ip -4 addr show wlan0` sonucunu kullanın.

## Zorunlu altı-yüz ivme kalibrasyonu

Ön kol kartının masa kaydında yerçekimi büyüklüğü 1–18 m/s² arasında değişti.
Bu nedenle kasalar tamamlandıktan sonra bir kez şu dosyayı çalıştırın:

```bash
cd /home/mesut/imu_test
python3 hw290_six_face_calibration.py
```

Program her kart için +X, -X, +Y, -Y, +Z ve -Z yönlerini sırayla ister.
Kartı sert yüzeyde sabit tutup yalnız ekrandaki istem geldiğinde Enter'a basın.
Sonuç varsayılan olarak şuraya yazılır:

```text
/home/mesut/hw290_accel_calibration.json
```

## Önce yalnız kayıt testi

```bash
cd /home/mesut/imu_test
python3 hw290_elbow_quaternion.py \
  --accel-calibration /home/mesut/hw290_accel_calibration.json \
  --output /home/mesut/hw290_left_arm_test.jsonl
```

Kalibrasyon varsayılan `/home/mesut/hw290_accel_calibration.json` yolundaysa
Thonny'de `hw290_elbow_quaternion.py` dosyasına düz Run basıldığında otomatik
yüklenir. UDP adresi vermek için yukarıdaki terminal komutu kullanılmalıdır.

Başlangıç boyunca:

1. Sol kol düz olmalı.
2. Kasalar kol üzerinde oynamamalı.
3. Geri sayım, gyro kalibrasyonu, filtre yerleşmesi ve nötr alma bitene kadar
   yaklaşık 20 saniye hareket edilmemeli.
4. `SIFIR ALINDI` mesajından sonra kol her yöne taşınabilir.

## ZED bilgisayarına UDP gönderme

`ZED_PC_IP` Windows bilgisayarın Raspberry Pi ile aynı ağdaki IPv4 adresidir:

```bash
python3 hw290_elbow_quaternion.py \
  --side left \
  --hz 50 \
  --accel-calibration /home/mesut/hw290_accel_calibration.json \
  --udp-host ZED_PC_IP \
  --udp-port 15060 \
  --source-id pi4-hw290-left \
  --session-id zed-deneme-001 \
  --output /home/mesut/hw290_left_zed-deneme-001.jsonl
```

Sağ koldaki BNO055 işlemi aynı porta `--side right` ile yayın yapabilir. Paketler
`side` alanıyla ayrılır.

## Gönderilen temel alanlar

- `elbow.flexion_deg`: ana dirsek açısı, düz kol yaklaşık 0°;
- `elbow.velocity_deg_s`: 4 Hz alçak geçiren filtreli açı hızı;
- `elbow.velocity_gyro_deg_s`: gyrodan hesaplanan tanı hızı;
- `elbow.relative_3d_deg`: pronasyonu da içeren toplam bağıl dönüş;
- `upper_arm/forearm.quaternion_xyzw`;
- kalibre edilmiş ivme, gyro, ham pusula ve sensör zamanları;
- her kol parçasının dünya koordinatındaki uzun ekseni;
- `quality`, `valid`, `failure_codes` ve zamanlama ölçümleri.

Pusula verisi kaydedilir fakat kalibre edilmeden yönelim filtresine sokulmaz.
Mutlak kol yönelimi ve konum için ZED kullanılır.

## Füzyon kapıları

- `valid=false` IMU örneği robot hedefini değiştirmez.
- ZED ile IMU 12° içinde uyuşursa kalite ağırlıklı birleştirilir.
- 12–25° arasında zayıf kaynağın ağırlığı azaltılır.
- Fark 25° üzerindeyse iki kaynak güvenilir görünüyorsa çelişki kaydedilir.
- IMU örneği 100 ms'den eskiyse kullanılmaz.
- Kamera bileği veya dirseği kaybettiğinde kaliteli IMU geçici dirsek yedeği olur.

İlk canlı deneyde füzyon sonucu yalnız kayda ve analiz paneline eklenmelidir.
Tekrarlanabilir hata ölçülmeden G1 eklem hedefini doğrudan sürmemelidir.
