# Codex uygulama promptu — 4×ZED BODY_38 kilitli el takibi ve Dex3 entegrasyonu

Aşağıdaki görevi mevcut çalışma alanında uçtan uca uygula:

`C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation`

## Amaç

Mevcut dört ZED 2i + BODY_38 + G1 23-DOF sistemine, operatör kimliğine kilitli iki-el takibi ekle. Her fiziksel kamerada yalnızca kilitli BODY_38 operatörünün sol/sağ bileği çevresinde el görüntüsü kırpılmalı; her el için 21 landmark çıkarılmalı; dört kameranın ölçümleri zaman uyumlu ve robust biçimde 3B fusion uzayında birleştirilmeli; bileğe bağlı normalize edilmiş el şekli ayrı bir Dex3-1 retargeting çözücüsüne gönderilmeli; sonuçta sol 7 ve sağ 7 motor hedefi üretilip kayıt, Rerun ve uygun simülasyon adaptörlerine aktarılmalı.

Mevcut 23-DOF gövde retargeting davranışını değiştirme veya geriletme. El sistemi varsayılan olarak kapalı, CLI/config ile etkinleştirilebilen ayrı ve modüler bir katman olsun. Fiziksel robota komut göndermeyi etkinleştirme; bu görev yalnız algılama, fusion, retargeting sözleşmesi, görselleştirme, kayıt ve simülasyon tarafını kapsıyor.

Hedef veri akışı:

```text
4 × ZED 2i görüntüsü + depth + BODY_38
        ↓
Her kamerada kilitli body_id ve LEFT_WRIST / RIGHT_WRIST
        ↓
Bilek merkezli, dinamik boyutlu sol/sağ ROI
        ↓
Her ROI'de MediaPipe Hand Landmarker: 21 × 2B landmark + confidence
        ↓
Kamera intrinsics/extrinsics + depth/ray + ortak capture timestamp
        ↓
Landmark başına robust çoklu kamera 3B fusion
        ↓
Bileğe bağlı avuç koordinat sistemi ve ölçek normalizasyonu
        ↓
Ayrı Dex3 retargeting çözücüsü
        ↓
q_body[23], q_left_dex3[7], q_right_dex3[7]
        ↓
JSONL / Rerun / Isaac Lab veya mevcut simülasyon adaptörü
```

## Önce yapılacak inceleme

Kod değiştirmeden önce repository talimatlarını ve mevcut çalışma ağacını incele. Özellikle aşağıdaki dosyaları ve bunların çağırdığı modülleri oku:

- `SYSTEM_ARCHITECTURE.md`
- `config/g1_23dof_dex3.json`
- `zed_g1_skeleton.py`
- `zed_four_camera_test/distributed_body38_fusion.py`
- `zed_four_camera_test/start_distributed_source.ps1`
- `zed_four_camera_test/distributed_body38_source.py` veya gerçek kaynak süreci hangi dosyadaysa o dosya
- `zed_four_camera_test/start_distributed_receiver.ps1`
- `start_zed_four_sources.ps1`
- `isaaclab_bridge/` ve canlı 23-DOF köprüleri
- Rerun kayıt/görselleştirme kodu
- ilgili testler ve mevcut JSONL şemaları

Aktif dört-kamera yolunun ZED SDK Fusion publisher mı, uygulama düzeyi distributed BODY_38 yolu mu olduğunu kod ve başlatma scriptlerinden kanıtla. El verisini yalnız aktif/amaçlanan üretim yoluna ekle; aynı işi iki farklı protokolde kopyalama. Dört tam RGB görüntüyü ağ üzerinden merkeze taşımak yerine, mümkünse MediaPipe çıkarımını her kamerayı açan kaynak bilgisayarda çalıştır ve kompakt landmark/depth metadata’sını mevcut kaynak paketine veya ayrı sürümlenmiş el kanalına ekle. İki hosttaki GPU/CPU yükünü ölç ve bu seçimi dokümante et.

MediaPipe, Stereolabs projeksiyon/depth API’leri, Unitree `xr_teleoperate`, Dex3 ve `dex-retargeting` için güncel resmi dokümantasyonu ve resmi repository’leri kontrol et. Teknik kararları birincil kaynaklarla doğrula. Harici kod veya model dosyası ekleyeceksen lisansını, sürümünü/commit’ini ve kaynağını dokümante et; bilinmeyen kaynaktan URDF/YAML kopyalama. API isimlerini hafızadan tahmin etme.

## Mimari gereksinimler

El özelliğini tek büyük dosyaya ekleme. En az şu sorumlulukları ayrı, test edilebilir modüller halinde tasarla; isimleri mevcut proje stiline göre uyarlayabilirsin:

1. El veri sözleşmeleri ve şema doğrulama.
2. BODY_38 bileğinden ROI üretme ve görüntü sınırlarına kırpma.
3. MediaPipe backend/adaptörü.
4. Sol/sağ el kimlik ilişkilendirmesi ve temporal takip.
5. Kamera geometrisi, depth örnekleme ve çoklu-view 3B fusion.
6. Avuç koordinat sistemi/ölçek normalizasyonu.
7. Dex3 retargeting adaptörü ve güvenlik filtresi.
8. JSONL, Rerun ve simülasyon çıkış adaptörleri.

MediaPipe kurulmamışsa mevcut BODY_38 akışı çalışmaya devam etmeli. Özellik açıkken eksik model/dependency için anlaşılır ve eyleme dönük hata ver. Model dosyasının konumu config/CLI ile seçilebilsin; sessizce internetten indirme yapma.

## Operatör ve el kimliği kilidi

El algılayıcı tam görüntüde bağımsız el aramamalı. Her kamera için mevcut operatör seçimi/kilidi kullanılmalı:

1. Yalnız `operator_selection.state == LOCKED` olan BODY_38 örneğini kabul et.
2. Kilitli gövdenin `LEFT_WRIST` ve `RIGHT_WRIST` 3B noktalarını aynı kameranın görüntüsüne, o kameranın gerçek intrinsics/distortion modeliyle projekte et. Fusion dünya noktasını kamera görüntüsüne yansıtırken doğru world→camera dönüşümünü kullan; frame isimlerini ve matris yönlerini açıkça doğrula.
3. Bilek çevresinde dinamik ROI oluştur. ROI boyutu, mümkünse görüntüdeki önkol uzunluğu/depth ile ölçeklensin; sabit piksel varsayımı config fallback’i olsun. ROI’ye parmak uçları için bilekten ele doğru pay ve minimum/maksimum boyut sınırı ekle.
4. MediaPipe yalnız bu crop üzerinde çalışsın. Crop landmark’larını kayıpsız olarak tam görüntü piksel koordinatına geri dönüştür.
5. Sonucu `{camera_serial, locked_body_id/unique_operator_id, side, capture_timestamp_ns}` anahtarıyla sakla.

Eşleştirmede yalnız MediaPipe handedness sonucuna güvenme. Esas anatomik kimlik BODY_38’in sol/sağ bileği olsun. Aday skoru en az şu terimleri içersin ve ağırlıkları config’ten gelsin:

```text
score =
    w_wrist * projected_wrist_2d_distance
  + w_temporal * previous_hand_pose_distance
  + w_handedness * handedness_disagreement_penalty
  + w_time * temporal_discontinuity
```

El çaprazlandığında, avuç ters döndüğünde, tek el kaybolduğunda ve kadrajda ikinci kişi bulunduğunda kimlik değiştirmemesi için histerezis/gating uygula. Kilit kaybolursa başka kişinin eline otomatik atlama; el çıktısını geçersiz yap veya kontrollü hold/fade uygula. Mevcut `R` operatör kilidi sıfırlama davranışı el takip durumunu da sıfırlasın.

## MediaPipe 21-landmark çıkarımı

İlk backend MediaPipe Hand Landmarker olsun. 21 landmark, detection/presence/tracking confidence ve handedness skorunu sakla. MediaPipe “world landmarks” değerlerini ZED/metrik dünya koordinatı olarak kullanma; bunları yalnız yardımcı şekil/temporal özellik olarak değerlendir.

Video/live mode kullan ve aynı el izi için zaman durumunu koru. ROI koordinat dönüşümü, renk formatı ve timestamp birimi açıkça test edilsin. Her kamera/side için bağımsız takip durumu kullan; farklı kameraların tracker durumlarını karıştırma.

Kaynak paketine en az şunları ekleyen sürümlenmiş, geriye uyumlu bir sözleşme tasarla:

- kamera seri numarası ve host kimliği;
- sequence ve ham/normalize capture timestamp;
- kilitli body/operator kimliği ve side;
- ROI’nin tam görüntüdeki koordinatları;
- 21 adet tam görüntü piksel veya normalize 2B landmark;
- landmark confidence/visibility;
- MediaPipe handedness yalnız yardımcı skor olarak;
- görüntü çözünürlüğü ve kullanılan intrinsics kimliği/hash’i;
- landmark başına geçerli ZED depth/deprojection varsa kamera-yerel 3B nokta;
- inference süresi, frame yaşı ve hata/ret nedeni.

NaN/Inf üretme; JSON’a `allow_nan=False` ile yazılabilir olmalı. UDP datagram boyutunu ölç. Güvenli sınırı aşarsa fragmentation’a bel bağlama; el için ayrı küçük datagram, sıkıştırılmış binary sözleşme veya uygun yerel protokol tasarla ve test et. Mevcut BODY_38 kontrol paketini büyütüp canlı akışı bozma.

## Zaman hizalama ve dört-kamera fusion

“Her kameradan en son gelen kareyi” doğrudan birleştirme. Mevcut `sample_timeline_ns`, clock-offset düzeltmesi, `normalized_capture_timestamp_ns`, target capture zamanı ve kamera yayılım mantığını yeniden kullan veya ortak bir yardımcı modüle çıkar.

- Her landmark gözlemini ortak fusion capture timestamp’ine interpolate/predict et.
- Maksimum interpolation/prediction süresi config’te sınırlı olsun.
- Hızlı parmaklarda eski karelere düşük ağırlık ver; süre sınırı aşılırsa katkıyı reddet.
- Her landmark için kullanılan kamera sayısı, capture spread, reprojection error ve fusion confidence kaydedilsin.

3B ölçüm için iki tamamlayıcı yol destekle:

1. Geçerli ZED depth varsa 2B landmark çevresinde robust depth örnekleme yap, kamera-yerel noktayı deproject et ve kalibre edilmiş camera→fusion-world extrinsic ile dönüştür.
2. En az iki görünüm varsa kalibre edilmiş kamera ışınlarından triangulation yap.

Tek bir pikseldeki depth’e güvenme. Küçük median/trimmed patch, depth confidence ve foreground sürekliliği kullan; el arkasındaki duvar depth’ini reddetmek için BODY_38 wrist depth’i ve anatomik mesafe kapıları ekle.

Landmark başına robust fusion uygula: RANSAC veya pairwise triangulation + reprojection/inlier seçimi, ardından confidence/depth/time/reprojection ağırlıklı çözüm. Hatalı bir kamera ortalamayı bozmamalı. En az iki güvenilir view tercih edilsin; yalnız tek kamera kaldığında geçerli depth ile “single-view fallback” üretilebilir fakat confidence düşürülüp kaynak modu işaretlenmeli. Hiç güvenilir ölçüm yoksa önceki hedefi sınırsız tutma.

Fusion dünya koordinatının mevcut BODY_38 fusion uzayıyla aynı olduğunu sentetik ve gerçek kalibrasyon testleriyle doğrula. Sol/sağ bilek ankrajı ile landmark 0 arasında makul mesafe kapısı uygula; bilek için BODY_38 ve el landmark’ını körlemesine ortalama, güvenilirliklerini açıkça modelle.

## El normalizasyonu

Retargeting’e ham kamera/dünya koordinatı değil, bileğe bağlı normalize edilmiş el şekli ver:

- origin: landmark 0 / wrist; BODY_38 bilek dünya konumu global ankraj metadata’sı olarak ayrıca korunsun;
- palm X: index MCP (5) ile pinky MCP (17) arasındaki doğrultu;
- palm Y: wrist (0) → middle MCP (9), X’e karşı Gram–Schmidt ile ortogonalize;
- palm Z: X×Y avuç normali;
- scale: wrist–middle MCP uzunluğu ve/veya avuç genişliğinin robust birleşimi.

```text
p_normalized = R_palm^T @ (p_world - p_wrist) / palm_scale
```

Sağ ve sol el için açık bir canonical convention tanımla. Aynalama işaretlerini gizli sabitlerle değil test edilen dönüşümlerle yap. Dejeneratif/kollinear landmark, çok küçük scale, ters normal ve ani eksen flip durumlarını tespit et. Quaternion/rotation sürekliliği gerekiyorsa işaret sürekliliği uygula.

## Dex3-1 retargeting

İnsan parmak eklem açılarını doğrudan Dex3 motorlarına kopyalama. Ayrı bir görev-uzayı optimizer/adaptörü oluştur. Her elin motor sırası kesinlikle `config/g1_23dof_dex3.json` ile aynı olsun:

```text
thumb_0, thumb_1, thumb_2,
middle_0, middle_1,
index_0, index_1
```

Unitree’nin resmi Dex3 `dex-retargeting` yapılandırmasını ve URDF/kinematic modelini lisans/sürüm kontrolüyle yeniden kullan. Resmi giriş 25×3, MediaPipe 21×3 ise sahte dört landmark üretme. İnsan hedef vektörü oluşturma katmanını MediaPipe indeks çiftlerini doğrudan kabul edecek biçimde adapte et; upstream kütüphaneyi gereksiz fork etme.

Optimizer hedefleri en az şu normalize vektör/mesafeleri kapsasın:

- wrist→thumb tip (0→4);
- wrist→index tip (0→8);
- wrist→middle tip (0→12);
- thumb tip→index tip (4→8), pinch için yüksek ağırlık;
- thumb tip→middle tip (4→12);
- ilgili MCP→tip doğrultuları.

Genel amaç:

```text
q* = argmin_q Σ_i w_i ||robot_task_i(q) - human_task_i||²
              + λ_prev ||q - q_previous||²
              + λ_rest ||q - q_rest||²
```

Joint limitleri config’ten oku; koda ikinci bir limit tablosu gömme. Çıktıya position limit, hız limiti, ivme/slew limiti, confidence gate ve watchdog uygula. El kaybında tanımlı kısa hold süresinden sonra güvenli açık/nötr el pozuna yumuşak dön. Sol ve sağ el kaybı birbirinden bağımsız ele alınsın.

Üretilen sözleşme en az şunları içersin:

```text
q_body[23]
q_left_dex3[7]
q_right_dex3[7]
q_target[37]  # yalnız kayıt/analiz kolaylığı için türetilmiş birleşik görünüm
```

Simülasyon adaptöründe gövde ve elleri ayrı articulation/joint gruplarına yaz. Mevcut asset rubber/fixed hand içeriyorsa parmak hareket ediyor gibi sahte çıktı gösterme: Dex3 eklemli asset’i resmi kaynaktan entegre et veya özelliği açıkça “landmark/target visualization only” olarak işaretle.

## Kayıt, replay ve görselleştirme

Mevcut JSONL kayıtlarını geriye uyumlu tut. Yeni sürümlü kayıt alanları şunları içersin:

- per-camera ROI ve 2B landmark’lar;
- varsa kamera-yerel depth 3B noktaları;
- fused world 3B landmark’lar ve landmark başına kalite;
- normalized hand landmarks/palm frame;
- raw ve safe Dex3 hedefleri;
- solver residual, iterations/time, saturation ve watchdog durumu;
- eksik el/kamera için açık rejection reason.

SVO2 ve JSONL eşleştirmesini timestamp/serial üzerinden doğrula. Eski yalnız BODY_38 JSONL veya ekran kaydından 21-landmark 3B verisi varmış gibi davranma. SVO2 mevcutsa offline yeniden işleme komutu ekle; yalnız ekran videosu varsa bunun metrik depth/kalibre edilmiş multi-view fusion sağlayamayacağını raporla.

Rerun’da şunları ayrı yollar altında göster:

- her kamerada full frame veya gizlilik/performans için isteğe bağlı ROI görüntüsü;
- sol/sağ ROI kutuları ve 21 adet 2B landmark;
- fusion world’de dört kameranın ışın/depth adayları, inlier/outlier durumu;
- fused 21×3 el iskeleti, palm frame eksenleri;
- Dex3 raw/safe 7+7 hedefleri, confidence, solver residual ve latency grafikleri.

Görselleştirme canlı kontrol yolunu bloke etmesin; bounded queue ve drop metriği kullan.

## CLI, config ve çalışma kolaylığı

Mevcut başlatma scriptlerini bozmadan şu kabiliyetleri ekle:

- el takibini aç/kapatma;
- MediaPipe model yolu ve delegate seçimi;
- ROI ölçek/min/max değerleri;
- el inference FPS limiti;
- timestamp/fusion eşikleri;
- single-view depth fallback politikası;
- Dex3 retargeting aç/kapatma;
- yalnız landmark kaydı, yalnız replay ve simülasyon modu;
- fiziksel robot çıkışı daima kapalı varsayılan.

İki bilgisayarlı dört kamera çalıştırma komutlarını Türkçe runbook’a birebir, kopyalanabilir PowerShell komutlarıyla ekle. Hangi dependency’nin ana PC’ye, hangisinin laptopa kurulacağını belirt. Mevcut sanal ortamları keyfi olarak birleştirme; pyzed ve MediaPipe’ın desteklediği Python sürümü/delegate uyumluluğunu kontrol et.

## Test ve doğrulama

Donanım gerektirmeyen deterministik testler yaz. En az şu senaryolar kapsansın:

1. BODY_38 bilek projeksiyonu ve ROI sınır kırpma.
2. Crop→full-image landmark dönüşümünün round-trip doğruluğu.
3. İkinci kişi, çapraz eller ve yanlış MediaPipe handedness altında kimlik sürekliliği.
4. Timestamp interpolation/prediction ve stale-frame reddi.
5. Bilinen kameralar/3B noktalarla sentetik triangulation; gürültü ve bir outlier kamera altında hata sınırı.
6. Depth patch’inde arka plan outlier’ı reddi.
7. camera→world extrinsic yönü ve birim dönüşümü.
8. Sol/sağ canonical palm frame, ölçek invariance ve eksen flip önleme.
9. Dex3 joint order, limit, velocity/acceleration gate ve bağımsız hand-loss watchdog.
10. JSON şemasında NaN/Inf reddi, eski BODY_38 paketleriyle geriye uyumluluk ve UDP boyut kapısı.
11. MediaPipe kurulu değilken el özelliği kapalı sistemin eksiksiz çalışması.
12. Mock solver/backend ile uçtan uca iki-el paket akışı.

Varsa kısa SVO2 örneğiyle offline integration testi ekle fakat CI testlerini donanım, lisanslı SDK veya büyük model dosyasına zorunlu bağlama. Gerçek dört-kamera kabul testinde şu metrikleri raporlayan bir benchmark aracı oluştur:

- kamera başına inference FPS ve p50/p95 süre;
- end-to-end p50/p95 latency;
- landmark başına ortalama kamera/inlier sayısı;
- reprojection error p50/p95;
- capture spread ve stale rejection oranı;
- sol/sağ ID switch sayısı;
- hand valid coverage;
- Dex3 solver p50/p95 süre, residual ve limit saturation oranı.

Başlangıç kabul hedefleri config/dokümanda açıkça tanımlansın. Ölçülmemiş performansı başarılı ilan etme. En az sentetik testte 3B triangulation hatası 10 mm altında olmalı; gerçek veri hedeflerini ayrı “ölçülecek” eşikler olarak belirt. Hızlı parmak hareketleri için 40–70 ms kamera yayılımının kabul edilmediğini ve zaman hizalama metriğinin görünür olduğunu doğrula.

## Uygulama sırası

Çalışmayı güvenli ve doğrulanabilir aşamalara böl:

1. Mimari audit, resmi kaynak doğrulaması ve veri sözleşmesi.
2. Tek kamera/offline ROI + MediaPipe + depth 3B.
3. Distributed kaynak paketleri ve zaman hizalı dört-kamera fusion.
4. Palm normalization ve Dex3 optimizer.
5. Kayıt/replay/Rerun.
6. Dex3 eklemli simülasyon adaptörü.
7. Benchmark, runbook ve regresyon testi.

Her aşamada ilgili testleri çalıştır. Mevcut kullanıcı değişikliklerini koru; ilgisiz dosyaları biçimlendirme veya geri alma. Büyük harici dependency/asset eklemeden önce repository’nin mevcut yaklaşımını izle. Donanım yoksa mock/sentetik testlerle ilerle ve hangi gerçek kamera doğrulamalarının kaldığını açıkça yaz.

## Tamamlanma ölçütü ve teslim raporu

Görev yalnız tasarım belgesiyle tamamlanmış sayılmaz. Çalışan kod, config, dependency kurulumu, testler, replay/benchmark aracı ve Türkçe runbook üret. Son yanıtta:

1. Uygulanan veri akışını kısa özetle.
2. Değişen/eklenen dosyaları amaçlarıyla listele.
3. Çalıştırılan testleri ve gerçek çıktılarını bildir.
4. Donanım olmadan doğrulanamayan noktaları dürüstçe belirt.
5. Ana PC ve laptop için kesin çalıştırma komutlarını ver.
6. İlk gerçek dört-kamera testinde izlenecek metrikleri ve geçme/kalma koşullarını yaz.
7. Fiziksel G1/Dex3 komut çıkışının kapalı kaldığını doğrula.

Belirsizlikte varsayım yapabilirsin fakat koordinat sistemi, timestamp, joint order, robot limitleri veya harici API söz konusuysa varsayımı kod içine gömmek yerine kaynak ve test ile doğrula.
