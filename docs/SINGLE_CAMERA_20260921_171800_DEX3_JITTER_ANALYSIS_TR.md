# Tek kamera Dex3 titreşim analizi — 2026-09-21 17:18

## İncelenen oturum

- `recordings/zed_body38_20260921_171800.jsonl`
- `recordings/zed_body38_20260921_171800.svo2`
- `rerun_recordings/rerun_body38_20260921_171703.rrd`
- `rerun_recordings/rerun_body38_20260921_171704/`

## Gövde ve el algılama sonucu

- BODY_38: 3692 kare, 124.34 saniye, 29.68 FPS.
- Üst gövde gözlenebilirliği: `%97.5`.
- Bütün çekirdek gövde gözlenebilirliği: `%88.5`.
- El çıkarım paketi: 1079; ölçülen hız `8.70 FPS`.
- Sol el: `419/1079 = %38.83`.
- Sağ el: `308/1079 = %28.54`.
- İki el aynı anda: `%14.92`.
- En az bir el: `%52.46`.
- MediaPipe çıkarım p50/p95: `25.74/46.14 ms`.
- Sol odak retry: 687 deneme, 112 kurtarma.
- Sağ odak retry: 737 deneme, 51 kurtarma.
- Baskın kayıp: `MEDIAPIPE_NO_HAND=1004`; ardından
  `DETECTOR_TRACK_LOST=424`.

İkinci sıkı ROI denemesi toplam 163 el ölçümünü kurtardı. Ek deneme nedeniyle
p95 çıkarım önceki yaklaşık 24 ms seviyesinden 46 ms'ye çıktı; asenkron işçi
sayesinde BODY_38 yakalama hızı yine yaklaşık 30 FPS kaldı.

## Sürekli küçük hareketin kaynağı

Isaac hedefi yanlış izlemiyordu. 1388 Isaac telemetri örneğinde hedef ile gerçek
Dex3 eklem norm hatası medyanda solda `0.0000039 rad`, sağda `0.0000019 rad`
ölçüldü. Simülasyondaki hareket ZED tarafındaki değişken hedefin doğru biçimde
uygulanmasıydı.

Ham el ölçümleri sabit kabul edilen bir saniyelik pencerelerde bile:

- Sol ortalama eklem standart sapması: `0.0615 rad`.
- Sağ ortalama eklem standart sapması: `0.0654 rad`.
- Sol ardışık ham hedef adımı medyan/p90: `0.254/0.361 rad`.
- Sağ ardışık ham hedef adımı medyan/p90: `0.258/0.370 rad`.

Kayıt sırasındaki güvenli hedefte:

- Sol kare adımı medyan/p90: `0.0857/0.2082 rad`.
- Sağ kare adımı medyan/p90: `0.0581/0.1840 rad`.
- Sol hız normu medyan/p90: `2.55/6.16 rad/s`.
- Sağ hız normu medyan/p90: `1.74/5.42 rad/s`.
- İvme sınırlayıcı toplam 7449 eklem olayı üretti.

İki yazılım davranışı gürültüyü görünür yaptı:

1. `PalmSE3Filter` şekil kesim frekansı 12 Hz iken ölçüm hızı yalnız 8.7 Hz idi;
   filtre her yeni landmark şeklini neredeyse doğrudan geçiriyordu.
2. İvme sınırlayıcı hedefi geçtiğinde durmuyordu. Sıfır olmayan hızla hedefin
   iki tarafına geçip sürekli küçük salınım üretebiliyordu.

Ayrıca düşük tespit kapsamı yüzünden hedef sık sık HOLD ile FADE arasında gidip
geliyordu. Solda 74, sağda 61 ayrı HOLD → FADE olayı oluştu. Bu, parmakların
algılanan poza dönüp tekrar nötre açılması şeklinde görünüyordu.

## Uygulanan titreşim düzeltmesi

- Avuç pozu filtresi `3 Hz`, kanonik beş parmak şekil filtresi `2 Hz` yapıldı.
- Eklem ölçüm deadband'i `0.07 rad` olarak ayarlandı.
- Küçük değişikliklerde hedef alpha `0.06`, büyük jestlerde `0.82`; böylece
  yumruk geçişi hızlı, sabit el daha sakin kaldı.
- Azami eklem hızı `2.5 rad/s`, ivmesi `10 rad/s²` yapıldı.
- Normal el pozu HOLD süresi `0.65 s`, yumruk HOLD süresi `1.5 s` oldu.
- Nötre dönüş `1.0 s` içine yayıldı.
- Konum integratörü hedefi geçtiği anda konumu tam hedefte sabitliyor ve ilgili
  eklem hızını sıfırlıyor.

## Aynı kayıtla çevrim dışı karşılaştırma

Yeni ayar aynı ham MediaPipe verisine uygulandı:

| Metrik | Kayıt sırasındaki filtre | Yeni filtre | Değişim |
|---|---:|---:|---:|
| Sol medyan kare adımı | 0.0857 rad | 0.0249 rad | `%71` azalma |
| Sağ medyan kare adımı | 0.0581 rad | 0.0126 rad | `%78` azalma |
| Sol p90 kare adımı | 0.2082 rad | 0.1246 rad | `%40` azalma |
| Sağ p90 kare adımı | 0.1840 rad | 0.1051 rad | `%43` azalma |
| Sol FADE karesi | 1286 | 752 | `%42` azalma |
| Sağ FADE karesi | 1717 | 1259 | `%27` azalma |
| Sol tamamen sabit kare | `%1.35` | `%28.6` | iyileşti |
| Sağ tamamen sabit kare | `%1.63` | `%29.9` | iyileşti |

Güçlü el pozu genliği korundu; iki elin güvenli hedef normu p95 ortalaması
yaklaşık `2.86 rad` kaldı. Filtre yumruğu silmedi.

Ham kayıt ölçümü:
`recordings/analysis/zed_body38_20260921_171800_dex3_jitter.json`

Yeni filtreyle yeniden işlenmiş ölçüm:
`recordings/analysis/zed_body38_20260921_171800_dex3_jitter_filtered_v3.json`

## Kalan geliştirme alanları

### 1. Algılama kapsamı

En önemli sınır hâlâ tek kamera MediaPipe kapsamıdır. Sol `%38.8`, sağ `%28.5`
değerleri hedeflenen `%80` seviyesinin altındadır. Filtre görünümü sakinleştirir
ama hiç görülmeyen eli yeniden oluşturamaz.

Bir sonraki teknik adım, ROI kaybolduğunda düşük hızlı tam görüntü el yeniden
yakalama ve ardından bilek ROI'sine dönüş olmalıdır. Bu, sürekli iki ROI
çalıştırmaktan daha verimli olabilir.

### 2. Kamera pozu ve el görünümü

Sağ el kapsamı daha düşük ve en uzun kayıp `11.10 s`. Kamera açısından sağ
elin gövde veya diğer kol tarafından daha çok örtülmesi olasıdır. SVO2 üzerinde
zaman kodlu manuel etiketleme ile gerçek detector miss ile oklüzyon ayrılmalıdır.

### 3. Poz histerezisi

Yeni deadband sürekli hedefi azaltır. Daha sakin jest kontrolü gerekirse
`OPEN/PARTIAL/FIST` için Schmitt histerezisli ayrık poz katmanı eklenebilir.
Bu, parmak hassasiyetini azaltır fakat teleoperasyon jestlerini kararlı tutar.

### 4. Çoklu kamera

İki veya dört görünümde bir kameranın el kaybı diğer kamerayla telafi edilebilir.
Dört kamera hattında el füzyonu vardır. Çift kamera hattında Dex3 el füzyonu
henüz bulunmadığı için ayrıca taşınmalıdır.

### 5. Gecikme

Odak retry p95 çıkarımı 46 ms'ye yükseltti fakat 100 ms kabul sınırının altında
ve BODY yakalamayı bloke etmiyor. Retry kurtarma oranı ileride ölçülerek sağ el
için düşük verimli ikinci deneme yerine tam görüntü yeniden yakalama seçilebilir.

## Doğrulama

- El ve Dex3 testleri: `27 passed`.
- Rerun başsız regresyonu geçti.
- Python derleme ve shell sözdizimi kontrolleri geçti.
- Fiziksel robot DDS çıkışı kapalıdır.
