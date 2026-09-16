# Dört Köşe ZED 2i Mount - Fusion 360 Tasarım Paketi

Bu tasarım tek bir dört-kamera barı değildir. Odanın/çalışma alanının dört
köşesine yerleştirilen **dört bağımsız ve aynı sehpa kafası** içindir. Her
kamera yükseklik, pan (sağa-sola) ve tilt (yukarı-aşağı) açısından ayrı
ayarlanabilir.

## Tasarım kararı

Uzun taşıyıcı kolonun tamamını plastik basmak güvenli ve rijit değildir. PETG
ve PLA uzun süreli yükte sürünür; eklemli uzun baskılar kameranın extrinsic
kalibrasyonunu bozar. Bu nedenle:

- Yükseklik: 1/4-20 üst vidalı, kilitli teleskopik fotoğraf ışık sehpası
  (önerilen çalışma aralığı yaklaşık 0.8-2.1 m).
- Basılan parçalar: kamera plakası, +/-45 derece tilt yoke'u ve kablo saddle'ı.
- Pan: yoke'u sehpanın 1/4-20 vidası üzerinde döndürüp metal insertte sıkma.
- Kamera bağlantısı: metal `1/4-20 x 3/8 in` vida. Kameraya giren diş boyu 7
  mm'yi kesinlikle aşmamalı.
- Kablo: kamerayla birlikte dönen saddle'da birinci strain relief; sehpa
  kolonunda 10-15 cm servis halkasından sonra cırt kelepçe ile ikinci relief.

## Kaynak ölçüler

Stereolabs teknik çizimine göre ZED 2i zarfı `175.3 x 43.1 x 30.3 mm`, kütlesi
229 g'dır. Alt bağlantıda `1/4-20 UNC` delik (maksimum vida giriş boyu 7 mm) ve
M3x0.5 seçenekleri bulunur. Bu ilk sürüm, farklı gövde revizyonlarıyla daha
uyumlu olduğu için merkez 1/4-20 deliğini kullanır; kamerayı plastikle
sıkıştırmaz ve havalandırma/optik yüzeyleri kapatmaz.

## Fusion 360'ta oluşturma

1. Fusion 360'ı açın ve `Utilities > Add-Ins > Scripts and Add-Ins` yoluna
   gidin.
2. `Scripts` sekmesinde `+` düğmesine basın ve
   `Fusion360_ZED2i_Corner_Mount` alt klasöründeki aynı adlı `.py` dosyasını
   seçin. Klasör adı, `.py` ve `.manifest` taban adları bilerek aynıdır.
3. Script'i seçip `Run` deyin. Yeni bir Design açılır.
4. Tarayıcıda `PRINT_` ile başlayan üç component basılacak parçalardır.
   `REF_ZED2i_ENVELOPE_DO_NOT_PRINT` yalnızca çarpışma kontrolü içindir.
5. Her `PRINT_` component için `Save As Mesh`; birim mm, format 3MF, refinement
   High seçin.

Scriptin başındaki `USER PARAMETERS` değerleri değiştirilebilir. Özellikle
gerçek kablo çapını kumpasla ölçüp `CABLE_DIAMETER`, satın alınan insertin
üretici tablosuna göre de `PIVOT_INSERT_D` ve `STAND_INSERT_D` değerlerini
değiştirin. Değişiklikten sonra script'i tekrar çalıştırın.

## Her köşe için donanım listesi

| Parça | Adet | Not |
|---|---:|---|
| Kilitli teleskopik ışık sehpası | 1 | Üstte 1/4-20 erkek vida, geniş ayak |
| Basılı tilt yoke | 1 | PETG/ASA önerilir |
| Basılı kamera plakası | 1 | Kamera altına ince kauçuk/TPU pad |
| Basılı kablo saddle | 1 | Kabloyu ezmeden sıkmalı |
| 1/4-20 x 3/8 in kamera vidası | 1 | Pul ile; kamera girişini ölçün |
| 1/4-20 pirinç heat-set insert | 1 | Yoke altı, dış çapı tasarıma uymalı |
| M5 heat-set insert | 2 | Kamera plakasının iki yanına |
| M5 x 16-20 mm civata | 2 | Pivot; yıldız kol veya başparmak vida |
| M5 geniş pul + fiber/nylon pul | 4 | Sürtünme ve plastik koruması |
| M3 civata + nyloc somun | 2+2 | Kablo saddle |
| 3 mm neopren/kauçuk pad | 1 | Yaklaşık 170 x 40 mm, lensleri kapatmaz |
| 15-20 mm cırt kablo bağı | 2 | Servis halkası ve kolon sabitleme |
| Emniyet halatı | 1 | Kamera gövdesinden değil plakadan stand'a |

Dört köşe için tabloyu dörtle çarpın. Basılı parçalardan önce **tek set prototip**
basılması önerilir.

## Baskı ayarları

- Malzeme: iç ortam için PETG; sıcak/güneş alan ortam için ASA. PLA'yı kalıcı
  kurulumda kullanmayın.
- Nozul: 0.4 veya 0.6 mm.
- Gerekli tabla alanı: yoke için en az yaklaşık 205 x 80 mm. Daha küçük tabla
  varsa yoke iki parçalı revizyona çevrilmelidir.
- Kamera plakası ile yoke arasında her yanda 1.5 mm montaj boşluğu bırakılmıştır;
  fiber pullar bu boşlukta merkezleme ve sürtünme yüzeyi sağlar.
- Katman: 0.20-0.24 mm.
- Duvar: en az 5 perimeter.
- Üst/alt: en az 6 katman.
- Dolgu: yoke %45 gyroid/cubic; plaka %35; cable saddle %40.
- Yoke: geniş arka yüzeyi tabla üzerinde olacak şekilde; pivot deliklerinde
  gerekirse yalnızca build-plate support.
- Plaka: düz yüzeyi tablaya; insert yuvaları yatay olduğundan deliği matkapla
  son ölçüsüne getirmek daha temiz sonuç verir.
- Kritik delikler baskıdan sonra rayba/matkapla temizlenmelidir. İlk prototipte
  insert yuvası için +/-0.2 mm test kuponu basın.

## Montaj

1. M5 insertleri kamera plakasının iki yanındaki yuvalara, insert üreticisinin
   sıcaklık önerisiyle ve ekseni kaçırmadan yerleştirin.
2. 1/4-20 inserti yoke altına yerleştirin.
3. Kameranın altında kaymayı önleyen ince bir neopren/TPU pad kullanın.
4. `1/4-20 x 3/8 in` vidayı alttan geçirip kamerayı plakaya bağlayın. Vidanın
   kameraya giren gerçek boyunu pul dahil ölçün; **7 mm'yi geçmesin**.
5. Plakayı iki M5 pivot civatası ve fiber pullarla yoke'a bağlayın. İki tarafı
   eşit sıkın; tilt ayarlanabilsin ama kendi kendine düşmesin.
6. USB-C locking konektörü kameraya vidalayın. Kabloyu keskin bükmeden saddle'a
   alın; saddle kablo kılıfını hafifçe tutsun, ezmesin.
7. Tilt boyunca kabloya 10-15 cm servis halkası bırakın. Halkadan sonra kabloyu
   standın sabit bölümüne cırtla bağlayın. Kabloyu teleskopik kilitlerin veya
   ayak menteşelerinin üzerinden geçirmeyin.
8. Plaka ile stand arasında gevşek fakat düşmeyi engelleyen emniyet halatı
   takın.

## Dört köşe yerleşimi ve devreye alma

- Kameraları köşelerde aynı yüksekliğe mecbur etmeyin; görüş kapanmasını
  azaltmak için karşılıklı iki kamerada 10-20 cm yükseklik farkı yararlı olabilir.
- Her tripodun bir ayağını çalışma alanının dışına bakacak şekilde çevirin ve
  ayaklara ağırlık/sandbag koyun. Yürüme hattına taşan ayakları işaretleyin.
- Kamera lens eksenlerini çalışma hacminin merkezinde, tercihen göğüs/pelvis
  yüksekliğinde kesiştirecek şekilde tilt edin. Aşırı aşağı açı, ayak ve bilek
  takibini zayıflatabilir.
- USB kablosu ile güç kablosunu gergin bırakmayın; zeminde kablo kanalı kullanın.
- Fiziksel yerleşim bittikten sonra dört kameranın extrinsic kalibrasyonunu
  yeniden alın. Pivot/stand kilitlerine tanık çizgisi atın. Çizgiler ayrılırsa
  kalibrasyonu geçersiz sayın.

## Basımdan önce ölçülmesi gereken üç şey

Bu parametrik başlangıç tasarımı üretilebilir durumdadır; nihai revizyon için
şunları kumpas/şerit metreyle doğrulayın:

1. Kullanılan USB-C locking kablonun dış çapı.
2. Satın alınan M5 ve 1/4-20 insertlerin dış çapı/uzunluğu.
3. İstenen minimum-maksimum lens yüksekliği ve sehpanın 1/4-20 üst bağlantısı.

Kamerayı takmadan önce basılı kafayı 1 kg cansız yükle 30 dakika test edin;
gevşeme, çatlama veya stand devrilme eğilimi varsa kullanmayın.
