# Raspberry Pi 4B'de tek kol, iki Adafruit BNO055 ilk testi

Bu klasör iki sensörün aynı Raspberry Pi üzerinde okunması için başlangıç noktasıdır. `bno055_pair.py` üst kol ve ön kol kartlarının kuaternion, ivme, açısal hız ve kalibrasyon durumlarını gösterir. İstenirse JSONL kaydeder. Ekrandaki **göreli dönüş, dirsek bükülme açısı değildir**; ilk ortak pozdan sonraki toplam 3B yönelim değişimidir. Kolun anatomik eksen kalibrasyonu sonraki aşamadır.

## Gerekenler

- Raspberry Pi 4 Model B, güncel Raspberry Pi OS, sağlam USB-C güç kaynağı.
- İki Adafruit BNO055 breakout. Fotoğraftaki kart **Adafruit 4646 STEMMA QT** sürümüdür; eski breakout'ın delikli pin sırası farklı olabilir.
- Kısa jumper/STEMMA kabloları. İlk denemeyi masada yapın; kola takılan uzun kablolara hemen geçmeyin.
- Pi üzerinde terminal/SSH erişimi. Tam Adafruit sürücüsü kurulumu için internet erişimi gerekir; ilk donanım kontrolü için aşağıdaki paketsiz dosya kullanılabilir. Python 3.10 veya yenisi önerilir.

Pi'de henüz işletim sistemi yoksa Windows'ta [Raspberry Pi Imager](https://www.raspberrypi.com/software/) ile microSD karta **Raspberry Pi OS Desktop (64-bit)** yazın. Kurulumda kullanıcı adı/parola ve Wi-Fi ayarlayın; uzaktan erişecekseniz SSH'yi de açın. Pi 4B'yi USB-C üzerinden uygun **5 V / 3 A** güç kaynağıyla çalıştırın.

## 1. Pi kapalıyken kablolama

Fotoğrafınızın yönünde **USB-C güç girişi solda, USB/Ethernet soketleri altta, 40 pinli başlık sağda**. Pinleri iki dikey sütun olarak okuyun: **kartın içine yakın sol sütun tek sayılar, dış kenardaki sağ sütun çift sayılardır**. En üst çift, ekran soketi tarafındaki `1–2` çiftidir:

```text
Fotoğraftaki görünüm              Karta yakın     Dış kenar
ekran soketi / ÜST                1: 3V3         2: 5V  ← kullanma
                                  3: SDA/GPIO2   4: 5V  ← kullanma
                                  5: SCL/GPIO3   6: GND
                                  7: GPIO4       8: GPIO14
                                  9: GND        10: GPIO15
USB/Ethernet / ALT                ...            ...
```

**Önce yalnız bir sensör takın:** Pi kapalıyken kartın `VIN` hattını fiziksel **1**, `GND` hattını **6**, `SDA` hattını **3**, `SCL` hattını **5** numaralı pine bağlayın. `5V` yazan 2 ve 4 numaralı pinlere bağlamayın. Buradaki 1/3/5/6 sayıları **fiziksel pin numarasıdır**; GPIO2 ve GPIO3 ise yazılım adlarıdır.

Fotoğraftaki QT kablosunun renkleri Adafruit'in standart mavi SDA kablosuyla aynı görünmüyor. Siyahın GND, kırmızının VIN, sarının SCL, beyazın SDA olması olasıdır; **yalnız renge bakarak Pi'ye takmayın**. Kartın `VIN`, `GND`, `SDA`, `SCL` yazılı delikleriyle kablonun boştaki uçlarını multimetrenin süreklilik modunda eşleştirin veya pin sırası belgeli bir STEMMA QT–jumper kablo kullanın. Adafruit'in standart QT kablosunda siyah GND, kırmızı güç, mavi SDA, sarı SCL'dir. Paket içindeki siyah pin şeridi fotoğrafta lehimlenmemiştir; lehimlemeden deliklere jumper sıkıştırıp güvenilir bağlantı beklemeyin.

İkinci sensörü eklerken ilk kartın boş QT soketinden ikinci kartın QT soketine bir **QT–QT kablo** ile dört hattı birlikte taşıyabilirsiniz. Bu kartta iki QT soketi aynı I²C hattına bağlıdır. İkinci kartın `ADR` deliğini ayrıca 3,3 V'a bağlamanız gerekir; bunun için ADR padine kablo veya verilen pin şeridini **lehimlemek** gerekir. İki kart da aynı `SDA`/`SCL` hattını paylaşır. Alternatif olarak bağlantıları breadboard üzerinde dağıtabilirsiniz.

| Raspberry Pi fiziksel pin / hat | BNO055 #1 (üst kol) | BNO055 #2 (ön kol) |
|---|---|---|
| Pin 1: 3V3 | VIN | VIN, QT zinciriyle aynı hat |
| Pin 6: GND | GND | GND, QT zinciriyle aynı hat |
| Pin 3: GPIO2/SDA | SDA | SDA, QT zinciriyle aynı hat |
| Pin 5: GPIO3/SCL | SCL | SCL, QT zinciriyle aynı hat |
| Pin 17: 3V3 | Bağlanmaz | ADR için alternatif 3,3 V kaynağı |

İlk kartın `ADR` pini boş kalır ve adresi `0x28` olur. İkinci kartın `ADR` pinini **3,3 V seviyesine** bağlayın; adresi `0x29` olur. QT zinciri kullanıyorsanız ikinci kartın `ADR` ve `VIN` delikleri arasına kısa bir kablo lehimlemek de mümkündür; **yalnız VIN gerçekten Pi'nin 3,3 V pininden besleniyorsa** bunu yapın. Ayrı kabloyla Pi fiziksel pin 17'ye bağlamak diğer seçenektir; bu pin fotoğraf yönünde karta yakın sütunun üstten 9. pinidir. Adafruit, iki BNO055'i aynı hatta kullanmak için ADR pinini yüksek yapmayı tarif eder. `3VO` regülatör **çıkışıdır**, VIN yerine güç girişi olarak kullanmayın. `PS0/PS1` pinlerini I²C denemesinde boş bırakın. Pi GPIO hatlarına 5 V uygulamayın. Güç varken kabloları değiştirmeyin.

Başta yalnız #1'i takın. Onu doğrulayınca Pi'yi kapatıp #2'yi ekleyin. İki kartın SDA/SCL/GND/3V3 hatları ortaktır; ayrı Raspberry Pi I²C pinleri aranmaz.

## 2. Raspberry Pi OS hazırlığı

Pi terminalinde:

```bash
sudo apt update
sudo apt install -y i2c-tools python3-venv python3-pip
sudo raspi-config
```

Menüde **Interface Options → I2C → Yes** seçin. Ardından BNO055'in I²C saat uzatması için Adafruit'in önerdiği hız düşürmeyi uygulayın: yeni Raspberry Pi OS'ta `sudo nano /boot/firmware/config.txt` açın ve dosyada aynı satırdan zaten yoksa şu satırı ekleyin:

```text
dtparam=i2c_arm_baudrate=10000
```

Eski OS sürümünde dosya `/boot/config.txt` olabilir. Kaydedip `sudo reboot` ile yeniden başlatın. 10 kHz, ilk güvenilir bağlantı denemesi içindir; bu hızda yüksek örnekleme hedeflemeyin.

## 3. Adresleri kontrol et

Pi tekrar açılınca:

```bash
ls /dev/i2c-1
i2cdetect -y 1
```

Yalnız ilk kart bağlıyken `28` görünmeli. Pi'yi kapatıp ikinci kartı ekledikten sonra taramada `28` ve `29` birlikte görünmeli. İki kart bağlanınca hâlâ tek adres çıkarsa önce ikinci kartın ADR bağlantısını kontrol edin. Hiç adres çıkmazsa SDA/SCL'nin yerini, GND/VIN'i, I²C'nin etkinliğini ve breakout üzerindeki pin isimlerini kontrol edin.

### Pi internete çıkamıyorsa: paketsiz ilk okuma

`bno055_probe_no_deps.py`, Linux'un `/dev/i2c-1` arayüzünü kullanır; `pip` veya `i2c-tools` istemez. Önce yalnız ilk sensörü bağlayın. Windows'ta Thonny'yi **Remote Python 3 (SSH)** yorumlayıcısıyla Pi'ye bağlayın, **File → Open** ile bu dosyayı Windows'tan açın ve **F5**'e basın. Çıktıda `CHIP_ID=0xa0`, `cal=(...)`, `q_xyzw=(...)`, `acc_m_s2=(...)` ve `gyro_rad_s=(...)` alanlarını bekleyin. Kırmızı **Stop** düğmesiyle durdurun.

İlk sensör doğrulanınca Pi'yi kapatıp ikinci sensörü `ADR=3,3 V` ile ekleyin. Dosyanın başındaki `ADDRESSES = (0x28,)` satırını `ADDRESSES = (0x28, 0x29)` yapıp yeniden çalıştırın. `sensorler_arasi_3d_aci` yalnız iki kartın yönelim farkını gösteren tanı değeridir; kalibre edilmiş dirsek açısı değildir. Bu paketsiz dosya ilk donanım kontrolü içindir; Body_38 ile birleştirmek üzere kayıt almak için aşağıdaki tam sürücü ve JSONL akışı kullanılmalıdır.

## 4. Kodu Pi'ye taşı ve Python ortamını kur

Bu depodaki `imu_capture` klasörünü Pi'ye kopyalayın. Örneğin Windows'ta proje kökünden PowerShell ile (kullanıcı adı ve IP'yi kendinize göre değiştirin):

```powershell
ssh kullanici@PI_IP "mkdir -p ~/zed-g1-motion-imitation"
scp -r .\imu_capture kullanici@PI_IP:~/zed-g1-motion-imitation/
```

Pi terminalinde:

```bash
cd ~/zed-g1-motion-imitation
python3 -m venv --system-site-packages .venv-pi
source .venv-pi/bin/activate
python -m pip install -r imu_capture/requirements-pi.txt
```

Raspberry Pi OS Bookworm ve sonrası sistem Python'una doğrudan `sudo pip install` yapılmamalı; bu sanal ortamı sonraki oturumlarda yeniden etkinleştirin. `board` modülü `adafruit-blinka` paketinden gelir; farklı bir `board` paketi kurmayın.

### Hangi IDE, dosya nerede?

İlk deneme için **Thonny** önerilir; Raspberry Pi OS Desktop ile gelir. Pi masaüstünde **Programming → Thonny** açın. `File → Open` ile `/home/KULLANICI/zed-g1-motion-imitation/imu_capture/bno055_pair.py` dosyasını açın (`KULLANICI` yerine Pi hesabınızın adını yazın). Sağ alttaki yorumlayıcı menüsünden `Configure interpreter` ile `/home/KULLANICI/zed-g1-motion-imitation/.venv-pi/bin/python` yolunu seçin. Sonra **F5/Run** ile çalıştırın; sensörler hazır olunca Thonny'nin alt Shell bölümündeki Enter istemine cevap verin. `--output` gibi seçenekler vermek ve uzun kayıt almak için terminal komutu daha uygundur.

Windows bilgisayardan geliştirmek isterseniz VS Code'un **Remote-SSH** eklentisiyle Pi'deki aynı klasörü uzaktan açabilirsiniz. Başlangıç için Thonny daha az kurulum ister. Bu proje Python kodunu **Raspberry Pi OS üzerinde CPython ile** çalıştırır; Arduino IDE veya Pico/MicroPython yorumlayıcısı seçmeyin.

## 5. İki sensörü çalıştır

Pi terminalinde, proje kökünde ve `.venv-pi` etkin iken:

```bash
python imu_capture/bno055_pair.py
```

Ekranda her kart için `kal=sistem/jyro/ivme/manyetometre`, `hazir=True/False` ve `q_xyzw=(...)` görünür. İlk açılışta kalibrasyon değerlerinin sıfır olması beklenebilir; jiroskop için kartları bir süre hareketsiz tutun, diğer kalibrasyonlar için Adafruit yönergelerine göre farklı yönlerde hareket ettirin. Her iki kartta `hazir=True` görünce kolu düz ve sabit tutup **Enter** tuşuna basın. Bu poz sıfır kabul edilir. Sonra ön kolu yavaşça bükün. Göreli dönüş alanının değişmesi, iki sensörden birlikte veri geldiğini gösterir. Bu alan henüz tek eksenli dirsek açısı sayılmaz.

Kayıt almak için:

```bash
python imu_capture/bno055_pair.py --output recordings/imu_bno055_ilk_test.jsonl
```

`recordings/` klasörü otomatik açılır. Her satırda iki sensörün ayrı okuma zamanları vardır. Oturumu bitirmek için `Ctrl+C` kullanın. Program varsayılan olarak 5 Hz okur; daha hızlı okuma ancak iletişim kararlıysa denenmeli.

## Sorun giderme

| Belirti | İlk kontrol |
|---|---|
| `No module named board` | `.venv-pi` etkin mi, `requirements-pi.txt` kuruldu mu? |
| `No module named adafruit_bno055` | `python -m pip install -r imu_capture/requirements-pi.txt` komutunu etkin sanal ortamda tekrar çalıştırın. |
| Yalnız `0x28` görünüyor | İkinci BNO055'in ADR pini 3,3 V'a bağlı mı? İkinci kartı tek başına deneyin. |
| `Remote I/O error`, takılma veya atlayan değer | Besleme ve kısa kabloları kontrol edin; 10 kHz satırının etkin olduğunu doğrulayın. Devam ederse Adafruit'in yazılımsal I²C çözümü gerekir. |
| Kuaternion var, `hazir=False` | Sistem kalibrasyonu 0. Kalibrasyon tamamlanmadan göreli yönelimi değerlendirmeyin. |
| Kol sabitken büyük yönelim sıçraması | Mıknatıs/metal, gevşek bant, güç kablosu ve kalibrasyon durumunu kontrol edin. |

## Kaynaklar

- [Adafruit BNO055 pinleri ve ADR](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/pinouts)
- [Adafruit Raspberry Pi Python bağlantısı](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython)
- [Adafruit Raspberry Pi I²C hız düşürme](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching)
- [Adafruit BNO055 kalibrasyonu](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration)
