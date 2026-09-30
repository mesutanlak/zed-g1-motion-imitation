# HW-290 dirsek ölçümü v3 — kullanım

> Bu belge eski toplam-3B masa testini açıklar. Çok yönlü kol hareketleri ve
> ZED UDP çıkışı için güncel belge: `HW290_ZED_FUSION_V4_TR.md`.

## Değişikliğin amacı

`hw290_elbow_quaternion.py` artık ana sonuç olarak sensörlerin başlangıç pozuna göre **toplam bağıl 3B dönüş açısını** verir. Ekrandaki alan adı `dirsek=` ve JSONL alanı `elbow_deg`'dir.

Eski `dirsek_X` hesabı sensörlerin +X ekseninin omuzdan ele doğru çok iyi hizalanmasını gerektiriyordu. Bu değer `X_tani=` / `elbow_x_legacy_deg` adıyla yalnız tanı için korunmuştur. Ana kontrol ve sonraki ZED karşılaştırması `elbow_deg` ile yapılmalıdır.

Toplam bağıl açı, iki sensör farklı fakat sabit yönlerde takıldıktan sonra yeniden nötr alındığında bu sabit montaj dönüşlerinden etkilenmez. Saf dirsek bükme/açma testinde kullanışlıdır. Önkol pronasyon/supinasyonu veya başka eklem dönüşleri de toplam 3B açıya katılır; yalnız anatomik fleksiyonu ayırmak için ileride ZED segment eksenleri veya işlevsel dirsek ekseni kalibrasyonu kullanılmalıdır.

## Raspberry Pi'ye güncel dosyayı aktarma

Windows PowerShell'de, Pi Wi-Fi adresi `10.42.0.10` ise:

```powershell
scp "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation\imu_capture\hw290_elbow_quaternion.py" mesut@10.42.0.10:/home/mesut/imu_test/
```

Pi adresi değiştiyse `ip -4 addr show wlan0` sonucundaki adresi kullan. Hedefte `dual_hw290_test.py` aynı klasörde bulunmalıdır. Thonny uzak yorumlayıcısı ile `/home/mesut/imu_test/hw290_elbow_quaternion.py` dosyasını açıp Run çalıştırılabilir.

## Her testten önce

1. `/dev/i2c-1` kartını üst kola, `/dev/i2c-8` kartını ön kola tak.
2. Kasa içindeki kartlar ve kasalar kol üzerinde oynamamalı. Kablo çekişini kol/kasa üzerinde ayrıca sabitle.
3. Sensörlerin aynı yöne bakması artık ana toplam açı için zorunlu değildir. Kasa yönlerini yaklaşık aynı tutmak `X_tani` verisini ve kayıtların karşılaştırılmasını kolaylaştırır.
4. Programı başlatınca ilk geri sayım ve jiroskop kalibrasyonu boyunca tamamen hareketsiz kal.
5. Filtre yerleşirken kolu düz ve sabit tut.
6. Program ayrıca 2 saniyelik nötr quaternion ortalaması alır. `NOTR ALINDI` satırındaki `p95` tercihen 3° altında olmalıdır. 5° üstündeyse program uyarır; yeniden başlatıp daha hareketsiz ölçmek daha doğrudur.
7. `SIFIR ALINDI` mesajından sonra dirseği yavaşça büküp aç.

## Ekrandaki değerler

- `dirsek`: kullanılacak ana bağıl 3B açı. Düz başlangıçta yaklaşık 0°, 90° bükmede yaklaşık 90° beklenir.
- `X_tani`: eski +X segment açısı. Ana sonuç değildir. `dirsek` doğruyken bunun farklı olması, sensör eksenlerinin kola farklı yönlerde yerleştiğini gösterebilir.
- `hiz`: iki sensör arasındaki bağıl açısal hız, derece/saniye.
- `a_oran`: o anki ivme normunun başlangıç normuna oranı; 1'e yakın olması iyidir.
- `kalite`: `OK` veya kalite bayrakları.

Kalite bayrakları:

- `DT_GAP`: okuma döngüsünde yaklaşık 50 ms veya daha büyük boşluk.
- `GYRO_NEAR_SATURATION`: jiroskop ±250°/s aralığının sınırına yakın.
- `ACCEL_CORRECTION_REJECTED`: ivme normu güven aralığının dışında olduğu için o örnekte yerçekimi düzeltmesi kullanılmadı.
- `RAPID_RELATIVE_MOTION`: bağıl hız 500°/s üzerinde; çok hızlı hareket veya sensör/kablo oynaması olabilir.

## Sensörün konumu değişirse

- Program başlamadan önce konumu değişmişse sorun değildir: kolu düz tutarak yeniden kalibrasyon ve nötr referans al.
- Program çalışırken sensör kasa içinde veya kasa kol üzerinde dönerse ölçüm referansı bozulur. Durdur, mekanik bağlantıyı düzelt ve programı yeniden başlat.
- Sadece sensörün yerini değiştirip eski kayda devam etmek veya eski nötr quaternionu kullanmak doğru değildir.

## Kayıt

Kayıt `/home/mesut/hw290_elbow_YYYYMMDD_HHMMSS.jsonl` konumuna yazılır. Ana alanlar:

- `elbow_deg`: kullanılacak toplam bağıl açı;
- `relative_3d_deg`: `elbow_deg` ile aynı geriye uyumlu alan;
- `elbow_x_legacy_deg`: eski X tabanlı tanı açısı;
- `relative_delta_wxyz`: nötre göre bağıl quaternion;
- `relative_rate_dps`: bağıl açısal hız;
- `quality_ok`, `quality_flags`: örnek kalitesi;
- iki sensörün quaternion, ivme, gyro ve manyetometre ham verileri.

ZED entegrasyonunda ilk aşamada `elbow_deg`, `relative_delta_wxyz`, `relative_rate_dps`, zaman damgası ve kalite alanları kaydedilip BODY_38 dirsek açısıyla karşılaştırılmalıdır. Kalite bayrağı olan IMU örneği ZED sonucunu zorla değiştirmemelidir.
