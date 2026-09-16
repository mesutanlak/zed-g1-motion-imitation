# ZED 2i – Tall Adjustable Tripod adaptörü

Bu parça, MakerWorld modelindeki **evrensel sekizgen yuvaya** geçer ve ZED 2i'yi kameranın altındaki standart **1/4-20 UNC** tripod dişinden tutar. Kameranın gövdesini sıkan bir kelepçe kullanılmadığı için lensler, USB-C kablosu ve havalandırma alanı açık kalır.

## Gerekli donanım

- 1 adet **1/4-20 UNC × 1/2 inç (12,7 mm) button-head** vida
- Vida başı çapı en fazla **11,5 mm**, baş yüksekliği en fazla **4,5 mm**
- Kamera ile plaka arasına 1 mm neopren, TPU veya ince kauçuk ped (önerilir)

1/2 inç vida bu tasarımdaki 4,8 mm derinlikli baş yuvasıyla kullanıldığında kameranın içine yaklaşık **5,7 mm** girer. Bu, datasheet'teki **en fazla 7 mm** sınırının altındadır. Daha uzun vida kullanmayın; rondela kullanırsanız kameraya giren gerçek boyu yeniden kontrol edin.

## Hangi dosya?

1. Önce `octagon_fit_test_standard.stl` dosyasını basın.
2. Sekizgen yuvaya rahatça girip boşluksuz oturuyorsa `zed2i_octagon_adapter_standard.stl` kullanın.
3. Test çok sıkıysa `loose` sürümüne geçin. Gevşek sürüm sekizgenin karşılıklı düz yüzleri arasında yaklaşık 0,28 mm daha küçüktür.

STL veya geometri-only 3MF dosyasını Bambu Studio'ya alın ve yazıcı olarak **X1E / 0,4 mm nozzle** seçin. Dosyada A1 yazıcı profili bulunmaz.

## X1E baskı ayarı

- Malzeme: PETG (tercih), ASA veya ABS; PLA yalnızca iç mekân ve düşük sıcaklık için
- Katman: 0,20 mm
- Duvar: 5–6
- Üst/alt katman: 6
- Dolgu: %40 gyroid veya cubic
- Yön: Geniş, düz kamera yüzeyi tabla üzerinde; sekizgen çıkıntı yukarı bakacak
- Destek: Gerekmez
- Brim: PETG'de genellikle gerekmez; ASA/ABS'de 5 mm kullanılabilir

Tabla yüzeyinde taşma/elephant foot oluşuyorsa Bambu Studio'da elephant-foot compensation uygulayın. Sekizgeni zımparalamadan önce `loose` test parçasını deneyin.

## Montaj

1. Button-head vidayı sekizgen tarafındaki 12 mm'lik baş yuvasından geçirin.
2. İnce kauçuk pedi plaka ile kamera arasına yerleştirin; merkezi deliği kapatmayın.
3. Kamerayı 1/4-20 dişine elle tutturun ve yalnızca dönmeyecek kadar sıkın.
4. Adaptörün sekizgen çıkıntısını tripodun üst yuvasına bastırarak takın.
5. USB-C kablosuna küçük bir servis payı bırakın; kablonun kamerayı çekmesine izin vermeyin.

## Tasarım ölçüleri

- Destek plakası: 80 × 42 × 6 mm, R6 köşeler
- Standart sekizgen: 17,90 mm çevrel çember yarıçapı, yaklaşık 33,08 mm karşılıklı düz yüz mesafesi
- Gevşek sekizgen: 17,75 mm çevrel çember yarıçapı, yaklaşık 32,80 mm karşılıklı düz yüz mesafesi
- Sekizgen geçme yüksekliği: 5,8 mm; giriş pahı: 0,7 mm
- Vida geçiş deliği: 6,8 mm
- Vida başı yuvası: 12,0 mm çap × 4,8 mm derinlik

Kamera yaklaşık 229 g olduğu için tripodun ayaklarını tam açın ve ağırlık merkezini ayakların içinde tutun. Hareketli robot veya titreşimli platform kullanımında ayrıca emniyet bağı ekleyin.
