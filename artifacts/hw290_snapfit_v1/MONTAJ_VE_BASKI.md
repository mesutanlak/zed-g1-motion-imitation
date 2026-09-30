# HW-290 vidasız geçme kasa

Bu sürümde metal vida, somun, silikon, köpük veya ayrı baskı çenesi kullanılmaz.
Kartın kendisinde vida deliği bulunması gerekmez.

## Bir IMU için basılacak parçalar

- `STL_BASKI/01_HW290_vidisiz_gecme_govde_2_adet.stl`: 1 adet
- `STL_BASKI/02_HW290_vidisiz_gecme_kapak_2_adet.stl`: 1 adet

Dosya adındaki `2_adet`, iki HW-290 bulunduğu için toplam baskı önerisidir.
Tek IMU denemesi için her dosyadan yalnızca birer adet basın.

## Baskı ayarları

- Malzeme: PETG önerilir; PLA ile tırnağı yavaşça esnetin.
- Katman: 0,20 mm
- Duvar: 4 çevre duvarı
- Dolgu: %25–35
- Destek: kapalı
- Gövde: STL dosyasındaki düz tabanıyla
- Kapak: STL dosyasında baskı yönüne çevrilmiştir; düz üst yüzeyi tabla üzerindedir

## Kartın takılması

1. Raspberry Pi gücünü kesin.
2. Jumper kabloları IMU'ya takılı bırakın.
3. Kabloları gövdenin 22 mm genişliğindeki arka ağzından dışarı geçirin.
4. Kartın arka kenarını önce kasaya indirin.
5. Kartı iki uzun kenar desteğinin üzerine yatırın. Elektronik parçalar tabana değmemelidir.
6. Kartı yavaşça arkaya kaydırın. Yan kenarlar dört kısa üst dudağın altına girmelidir.
7. Kart arka iki sabit dayamaya gelince ön kenarın ortasına bastırın.
8. Orta esnek tırnak öne açılıp kartın üzerinden geçerek kilitlenir.
9. Kartın sağa sola, öne arkaya ve yukarı hareket etmediğini kontrol edin.
10. Kabloyu arka iki küçük yuvadan ince kablo bağıyla yük alacak şekilde sabitleyin.
11. Koruyucu kapağı gövdenin üzerinden düz şekilde bastırarak geçirin.

## Kartın çıkarılması

1. Kapağı iki yandan eşit biçimde yukarı çekin.
2. Ön orta tırnağı dışarı, yani kasanın önüne doğru yalnızca gerektiği kadar esnetin.
3. Aynı anda kartın ön kenarını kaldırın.
4. Kartı öne kaydırıp yan dudaklardan çıkarın.

Tırnağı 90 derece bükmeyin. PETG tekrar sökme işlemleri için PLA'dan daha uygundur.

## Yön ve kalibrasyon

Kasa kola bağlandıktan sonra gerçek sensör `+X` yönünü kasanın dışına işaretleyin.
Üst kol ve ön kol sensörlerinin `+X` yönleri omuzdan ele doğru bakmalıdır.
Son montaj tamamlandıktan sonra altı yüz ivme kalibrasyonunu ve dirsek nötr başlangıcını yeniden alın.
