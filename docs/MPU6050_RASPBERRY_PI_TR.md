# MPU6050 / GY-521 Raspberry Pi bağlantı ve ilk test

## Donanım özeti

MPU6050; üç eksen ivmeölçer ve üç eksen jiroskop içeren 6 eksenli bir IMU'dur. Sıcaklık ölçümü bir hareket serbestlik derecesi değildir. Kart üzerinde manyetometre veya barometre bulunmadığı için bu modül tek başına 9/10 DOF değildir.

## Tek kart, `/dev/i2c-1`

| GY-521 | Raspberry Pi 4 fiziksel pin | BCM işlevi | İlk test |
|---|---:|---|---|
| VCC | 1 | 3V3 | Bağla |
| GND | 6 | GND | Bağla |
| SDA | 3 | GPIO2 / SDA1 | Bağla |
| SCL | 5 | GPIO3 / SCL1 | Bağla |
| XDA | — | Yardımcı I2C data | Boş bırak |
| XCL | — | Yardımcı I2C clock | Boş bırak |
| AD0 | 9 | GND | `0x68` için bağla |
| INT | — | Kesme çıkışı | İlk testte boş bırak |

Raspberry Pi GPIO girişleri 3,3 V seviyesindedir. GY-521 kartını 3V3 ile beslemek, bilinmeyen klon kartlarda SDA/SCL hatlarının 5 V'a çekilmesi riskini önler.

AD0 GND olduğunda adres `0x68`, 3V3 olduğunda `0x69` olur. Aynı I2C hattında en fazla iki MPU6050 doğrudan adreslenebilir.

## Gerçek donanım envanteri

- 2 adet BNO055: sol üst kol ve sol ön kol.
- 2 adet HW-290 10DOF: MPU6050 + HMC5883L/QMC5883 + BMP180; sağ üst kol ve sağ ön kol.
- 1 adet GY-521/MPU6050: 6 eksenli; göğüs.

HW-290 kartındaki 10 DOF ifadesi; ivme 3 + gyro 3 + manyetometre 3 + basınç 1 toplamıdır. Basınç verisi eklem açısına katılmaz. Kartın hareket yongası yine MPU6050'dir; 10 DOF, aynı karttaki ek pusula ve barometreden gelir.

### HW-290 Raspberry Pi bağlantısı

| HW-290 | Raspberry Pi 4 fiziksel pin | İlk test |
|---|---:|---|
| VCC_IN | 1 (3V3) | Bağla |
| 3.3V | — | Boş bırak; VCC_IN ile aynı anda bağlama |
| GND | 6 (GND) | Bağla |
| SCL | 5 (GPIO3/SCL1) | Bağla |
| SDA | 3 (GPIO2/SDA1) | Bağla |
| FSYNC | — | Boş bırak |
| INT | — | Boş bırak |
| DRDY | — | Boş bırak |

Kart 3,3–5 V giriş ve seviye dönüştürme devresiyle satılır. Raspberry Pi için VCC_IN'i 3V3 ile beslemek uygun ve temkinli seçimdir.

## İki BNO, iki HW-290 ve bir MPU6050 için adres planı

- `/dev/i2c-1`: BNO055 `0x28`, HW-290 sağ üst kol `0x68`, göğüs MPU6050 `0x69` (AD0 = 3V3).
- `/dev/i2c-8`: BNO055 `0x28`, HW-290 sağ ön kol `0x68`.

HW-290 kartların MPU6050 adres seçim pini dışarı çıkarılmamış görünüyor. Bu nedenle iki kart farklı I2C bus'larında tutulur. HW-290 üzerindeki HMC5883L genellikle `0x1E` (QMC5883 klonuysa `0x0D`), BMP180 ise `0x77` adresini kullanır.

GY-521 kartlarında çoğunlukla SDA/SCL pull-up dirençleri bulunur. Bir hatta çok sayıda breakout bağlandığında eşdeğer pull-up fazla küçülebilir. I2C hatası görülürse TCA9548A I2C çoklayıcı kullanmak veya fazla breakout pull-up dirençlerini devreden çıkarmak gerekir. Uzun gövde kablolarında SDA-GND ve SCL-GND çiftlerini birlikte taşımak ve kabloları kısa tutmak gerekir.

## İlk test

```bash
cd /home/mesut/imu_test
python3 mpu6050_test.py
```

Adres taraması:

```bash
sudo apt install -y i2c-tools
i2cdetect -y 1
```

İlk kart için tabloda `68`, ikinci kartın AD0 pini 3V3 olduğunda `69` görünmelidir.

MPU6050 hazır quaternion üretmez. Nihai sistemde ham ivme ve jiroskop verisinden 6 eksenli Mahony/Madgwick yönelim filtresi çalıştırılacak; manyetometre olmadığı için uzun süreli yaw sürüklenmesi ayrıca izlenecektir.

HW-290 kartını tanımak için:

```bash
python3 hw290_10dof_test.py
```

Beklenen kimlikler MPU6050 `0x68`, HMC5883L `0x1E` veya QMC5883 `0x0D`, BMP180 `0x77` adresleridir.
