# BNO055 dirsek verisini BODY_38'e isteğe bağlı ekleme

Bu tasarım ZED kamera yakalama ve çoklu kamera füzyon kodunu değiştirmez. Raspberry Pi ayrı bir `arm_imu/v1` UDP akışı üretir. BODY_38 tüketicisi bu akışı zaman damgasıyla eşler ve yalnız kalite kapıları geçilirse dirsek kısıtı olarak kullanır. IMU kapanırsa mevcut ZED hattı çalışmaya devam eder.

## Veri sözleşmesi

Pi her kol için 20–50 Hz'de bir JSON UDP paketi yollar. Paket `imu_capture/arm_imu_protocol.py` içindeki `build_arm_imu_packet` ile oluşturulur.

Mevcut iki ayrı I2C hattı için Pi'de canlı çalıştırma örneği:

```bash
cd ~/zed-g1-motion-imitation
python3 imu_capture/bno055_elbow_stream.py \
  --upper-bus /dev/i2c-1 \
  --forearm-bus /dev/i2c-8 \
  --side left \
  --angle-mode segment \
  --segment-axis x \
  --flexion-sign 1 \
  --source-id pi4-arm-imu \
  --session-id deneme-001 \
  --hz 20 \
  --udp-host ZED_BILGISAYAR_IP \
  --udp-port 15060 \
  --output recordings/left_arm_imu.jsonl
```

İlk denemede `--udp-host` verilmeden yalnız JSONL kaydı alınabilir. Bu yeni dosya daha önce çalışan deneme dosyasını değiştirmez.

Kontrol için taşınan alanlar:

- `timestamp_ns`, `sequence`, `side`
- `source_id`, `session_id`: kaynak Raspberry Pi ve aynı ZED/IMU denemesinin kimliği
- `elbow.flexion_deg`: düz kol 0 derece, fleksiyon pozitif
- `elbow.velocity_deg_s`: quaternion açısından türetilmiş ve alçak geçiren filtreli hız
- `elbow.relative_3d_deg`: iki segment arasındaki en kısa toplam 3B dönüş
- `elbow.off_axis_deg`: menteşe ekseni dışındaki swing dönüşü
- Her sensörün `quaternion_xyzw`, `gyro_rad_s`, `linear_acceleration_m_s2`
- `calibration`: sistem, gyro, ivme, manyetometre seviyeleri
- İki sensörün ayrı örnek zamanları
- `timing.read_skew_ms`, `timing.sample_period_ms`
- `valid`, `quality`, `failure_codes`

Manyetometre IMUPLUS modunda kullanılmadığı için `mag=0` tek başına hata değildir.

`--angle-mode segment` varsayılandır ve eksen öğrenmez. Nötr konuma göre sensör kasasında kol boyunca uzanan sabit X/Y/Z ekseninin yön değişimini dirsek açısı olarak verir. `--segment-axis`, iki kasada da omuzdan bileğe paralel duran kart ekseni olarak bir kez belirlenir. Bu ölçüm ön kolun kendi uzun ekseni etrafındaki pronasyon/supinasyona karşı toplam 3B açıdan daha dayanıklıdır.

`--angle-mode 3d`, iki sensör arasındaki toplam 3B dönüşü verir; kolay bir tanı modudur fakat pronasyon/supinasyonu fleksiyona ekleyebilir. `--angle-mode hinge` hareketten öğrenilen menteşe eksenini kullanır ve eksen dışı hareketi ayrıca hesaplar.

## Bilgisayar tarafı

`motion_pipeline/imu_udp.py` UDP'yi bloklamadan alır. `motion_pipeline/imu_fusion.py` örnekleri BODY_38 zamanına interpolate eder, geometrik ZED açısını insan fleksiyonuna çevirir ve uyuşmazlığı denetler.

Örnek kullanım:

```python
from motion_pipeline.imu_fusion import elbow_interior_deg, fuse_elbow_flexion
from motion_pipeline.imu_udp import ArmImuUdpReceiver

imu_receiver = ArmImuUdpReceiver(port=15060)

# Her BODY_38 karesinde, filtrelenmiş omuz/dirsek/bilek noktalarından sonra:
imu = imu_receiver.align(timestamp_ns, side="left", max_age_ms=100.0)
interior = elbow_interior_deg(left_shoulder, left_elbow, left_wrist)
fusion = fuse_elbow_flexion(
    zed_interior_deg=interior,
    zed_confidence=min(left_shoulder_conf, left_elbow_conf, left_wrist_conf),
    imu=imu,
)
record["left_elbow_sensor_fusion"] = fusion.as_dict()
```

İlk aşamada sonuç yalnız JSONL kayda ve analiz paneline eklenir. `fusion.usable`, gecikme, uyuşmazlık ve hata kodları gerçek kayıtlarla doğrulanmadan robot hedefi değiştirilmez. İkinci aşamada güvenilir sonuç `AnatomicalElbowRegularizer` için düşük öncelikli hedef olur. BODY_38 omuz, dirsek ve bilek konumları değiştirilmez.

## Güvenli seçim mantığı

- İki kaynak 12 derece içinde uyuşuyorsa kalite ağırlıklı birleştirilir.
- 12–25 derece arasında zayıf kaynağın ağırlığı azaltılır ve `ZED_IMU_ELBOW_DISAGREEMENT` yazılır.
- 25 dereceden büyük farkta iki kaynak da güvenilir görünüyorsa çıktı `conflict` olur ve kontrol için kullanılmaz.
- ZED kol güveni yüzde 40'ın altındaysa ve IMU kalitesi en az 0,75 ise IMU geçici yedek olabilir.
- IMU 100 ms'den eskiyse kullanılmaz.
- Geçersiz anatomik açı, düşük kalibrasyon, okuma kayması ve eksen dışı hareket hata kodlarında korunur.

## Zaman eşleme

Pi ve ZED bilgisayarının UTC saatleri aynı NTP kaynağına bağlı olmalıdır. Ağ paketinin ulaşma zamanı ölçüm zamanı olarak kullanılmaz. Pi paketi sensör okumasının orta zamanını `timestamp_ns` olarak göndermeli, iki sensörün kendi zamanlarını da tanı alanında saklamalıdır.

20 Hz IMU ile 30/60 Hz BODY_38 arasında doğrusal açı interpolasyonu yapılır. Daha sonra 50 Hz kararlı çalıştırılabiliyorsa gecikme azalır. CSV yazma ve UDP gönderme, sensör okuma döngüsünü bekletmemelidir.

## Uygulama sırası

1. Pi kodundaki dinamik gyro işaret çevirme bölümünü kaldır.
2. Sarılmış fleksiyon, sürekli test açısı, quaternion hızı ve gyro hızını ayrı alanlarda tut.
3. Nötr referansı tek örnek yerine 1–2 saniyelik hareketsiz quaternion ortalamasından al.
4. Swing–twist ayrıştırmasıyla `off_axis_deg` üret.
5. `build_arm_imu_packet` ile UDP yayınla.
6. BODY_38 bilgisayarında `ArmImuUdpReceiver` ile yalnız kayıt amaçlı eşleştir.
7. En az on adet `0–45–90–120–90–45–0` deneyiyle hata dağılımını ölç.
8. Görsel kapanma deneylerinde IMU yedekleme davranışını doğrula.
9. Doğrulamadan sonra sonucu dirsek regularizer hedefine bağla.

## Montaj gereksinimi

Sensör kasası segment üzerinde dönmemelidir. Her kasada iki temas veya iki kayış, kaymaz ince ped ve kablo gerilim boşaltması kullanılmalıdır. Üst sensör distal üst kolun düz yüzeyine, ön kol sensörü dorsal orta ön kola yerleştirilir. Her iki kartın işaretli eksenleri aynı anatomik yöne bakmalı ve bu yön oturum metadata'sında saklanmalıdır.
