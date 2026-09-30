# Raspberry Pi 4B beş IMU pin ve I2C adres planı

## Mevcut bus'lar

| Bus | SDA | SCL | Mevcut sensör |
|---|---|---|---|
| `/dev/i2c-1` | fiziksel pin 3 / GPIO2 | fiziksel pin 5 / GPIO3 | BNO055 sol üst kol, `0x28` |
| `/dev/i2c-8` | fiziksel pin 36 / GPIO16 | fiziksel pin 38 / GPIO20 | BNO055 sol ön kol, `0x28` |

`/dev/i2c-8` eşlemesi `/boot/firmware/config.txt` içinde `bus=8,i2c_gpio_sda=16,i2c_gpio_scl=20` olarak doğrulanmıştır.

## Ortak besleme

Pi fiziksel pin 1'den bir ortak 3V3 dağıtım rayı, fiziksel pin 6'dan bir ortak GND dağıtım rayı oluşturulur. Beş sensörün tamamı bu iki raya bağlanabilir. Pi pin 17 de aynı 3V3, pin 9 da aynı GND rayıdır; mevcut BNO kabloları pin 17 ve 9'da kalabilir. Dağıtım rayını Pi'ye hem pin 1 hem pin 17 ile ayrı ayrı beslemek gerekmez.

Temiz kurulumda, Pi kapalıyken iki BNO'nun VCC kabloları Pi pin 1/17'den alınıp ortak 3V3 raya; GND kabloları pin 6/9'dan alınıp ortak GND raya taşınır. Sonra Pi pin 1 yalnız 3V3 rayı, pin 6 yalnız GND rayı besler. Breadboard güç rayı ortadan kesikse iki yarı ayrı jumper ile birleştirilir.

Hiçbir sensör 5 V hattına bağlanmaz. HW-290 kartında `VCC_IN` kullanılır, kartın `3.3V` pini boş kalır.

## Beş sensörün tam dağılımı

| Sensör | Görev | Bus | Adresler | Besleme |
|---|---|---|---|---|
| BNO055 #1 | sol üst kol | i2c-1, pin 3/5 | `0x28` | mevcut pin 1/6 veya ortak ray |
| BNO055 #2 | sol ön kol | i2c-8, SDA pin 36 / SCL pin 38 | `0x28` | mevcut pin 17/9 veya ortak ray |
| HW-290 #1 | sağ üst kol | i2c-1, pin 3/5 | MPU `0x68`, pusula `0x1E` veya `0x0D`, BMP180 `0x77` | VCC_IN/GND ortak ray |
| HW-290 #2 | sağ ön kol | i2c-8, SDA pin 36 / SCL pin 38 | MPU `0x68`, pusula `0x1E` veya `0x0D`, BMP180 `0x77` | VCC_IN/GND ortak ray |
| GY-521 MPU6050 | göğüs | i2c-1, pin 3/5 | `0x69`; AD0 mutlaka 3V3 | VCC/GND ortak ray |

## Tek tek kablolama

### HW-290 #1, sağ üst kol

- `VCC_IN` -> ortak 3V3
- `GND` -> ortak GND
- `SDA` -> Pi fiziksel pin 3
- `SCL` -> Pi fiziksel pin 5
- `3.3V`, `FSYNC`, `INT`, `DRDY` -> boş

### HW-290 #2, sağ ön kol

- `VCC_IN` -> ortak 3V3
- `GND` -> ortak GND
- `SDA` -> Pi fiziksel pin 36
- `SCL` -> Pi fiziksel pin 38
- `3.3V`, `FSYNC`, `INT`, `DRDY` -> boş

### GY-521 MPU6050, göğüs

- `VCC` -> ortak 3V3
- `GND` -> ortak GND
- `SDA` -> Pi fiziksel pin 3
- `SCL` -> Pi fiziksel pin 5
- `AD0` -> ortak 3V3; bu bağlantı adresi `0x69` yapar
- `XDA`, `XCL`, `INT` -> boş

## Beklenen I2C taramaları

`i2cdetect -y 1`: BNO `28`, HW-290 MPU `68`, göğüs MPU `69`, BMP180 `77`, pusula `1e` veya `0d`.

`i2cdetect -y 8`: BNO `28`, HW-290 MPU `68`, BMP180 `77`, pusula `1e` veya `0d`.

Kartların üzerindeki pull-up dirençleri paralel bağlanır. Bus taramasında kaybolma veya `Remote I/O error` oluşursa yeni sensör eklemeye devam edilmez; pull-up direnci ölçülür veya TCA9548A ile bus'lar ayrılır.

## Kurulum sırası

1. Tüm programları durdur, Pi'yi kapat ve ortak besleme rayını kur.
2. Yalnız iki BNO bağlıyken `i2cdetect -y 1` ve `i2cdetect -y 8` ile her iki bus'ta `28` doğrula.
3. Pi'yi kapat, HW-290 #1'i i2c-1'e ekle, tekrar tara.
4. Pi'yi kapat, HW-290 #2'yi i2c-8'e ekle, tekrar tara.
5. Pi'yi kapat, göğüs GY-521'in AD0 pinini 3V3'e sabitleyip i2c-1'e ekle, `69` adresini doğrula.
6. Adresler kararlıysa sensör test kodlarını tek tek çalıştır.
