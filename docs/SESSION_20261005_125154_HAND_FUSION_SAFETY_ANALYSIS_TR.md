# 2026-10-05 12:51 oturumu: el takibi, fusion ve G1/Dex3 güvenlik analizi

## Karar özeti

Bu oturum fiziksel robot denemesine hazır kabul edilmemelidir. BODY_38 ve kol IK
zinciri çalışmaktadır; ancak dört temel kabul kapısı geçilmemiştir:

1. `31571870` kamerasının dünya kalibrasyonu yaklaşık 0.90 m kayıktır ve fusion
   tarafından her görüldüğü karede `CALIBRATION_DRIFT` ile reddedilmiştir.
2. Kamera süreçlerinde el detektörü çalışmış olsa da ayrı el UDP paketleri fusion
   sürecine hiç ulaşmamıştır; bu nedenle birleşik kayıtta el coverage sıfırdır.
3. GMR/Isaac akışında uzun veri boşlukları ve güvenlik müdahaleleri vardır.
4. Eski gövde bariyeri Dex3 el geometrisini değil, kısa lastik el uç noktasını
   kullanıyordu; simülasyondaki el-gövde geçişi bu nedenle kaçabiliyordu.

El taşımasını BODY datagramına yedekleyen çift yol, Dex3'e özgü 0.299 m
uç-geometri profili, Isaac tarafında profil uyuşmazlığında fail-closed ret ve SVO
üzerinde doğrulanmış daha sıkı ikinci ROI bu çalışma kapsamında eklendi. Fiziksel
robot çıkışı kapalı tutulmuştur.

## İncelenen veri

- `recordings/four_body38_fusion_20261005_125154.jsonl`
- `recordings/zed_body38_33773329_20261005_125320.{jsonl,svo2}`
- `recordings/zed_body38_31571870_20261005_125320.{jsonl,svo2}`
- Asıl eşleşen Rerun oturumu:
  `rerun_recordings/rerun_body38_20261005_125206/`
- Kullanıcının ilettiği `...123711` oturumu da kontrol edildi; zaman olarak
  verilen 12:51 fusion/SVO kaydından öncedir.

## 1. Dört kamera fusion bulguları

| Ölçüm | Sonuç | Yorum |
|---|---:|---|
| Toplam fusion karesi | 2855 | 227.28 s duvar süresi, 192.33 s aktif |
| Aktif hız | 14.83 FPS | 15 FPS hedefiyle uyumlu |
| 4/4 kaynaklı kare | %0 | Kabul edilmez |
| 3/4 kaynaklı kare | %100 | Tüm oturum eksik kaynakla çalışmış |
| `31571870` kabul oranı | %0 | 2353/2353 görünüm `CALIBRATION_DRIFT` |
| Diğer üç kamera | %100 | Her fusion karesine katkı vermiş |
| Kaynak sync p50 / p95 | 25.93 / 26.33 ms | Kullanılabilir |
| Ham cross-view MPJPE p50 / p95 | 0.25 / 0.33 m | Kalibrasyon kayması nedeniyle yüksek |
| Pose hizalama sonrası p50 / p95 | 0.05 / 0.07 m | İskelet şekli temelde tutarlı |
| Capture-to-send p50 / p95 | 84 / 94 ms | Sonraki el+GMR gecikmesine az pay bırakıyor |
| Kayıt drop | 0 | Dosya yazımı darboğaz değil |

`31571870` pelvisinin fused gövdeye göre medyan ofseti 0.903 m, p95 değeri
1.019 m'dir. Medyan vektör yaklaşık `[-0.682, +0.585, -0.159] m`'dir. Buna
karşın pelvis çıkarılarak karşılaştırılan iskelet şekli hatası yalnızca yaklaşık
0.066 m'dir. Bu desen, yanlış kişiden çok yanlış kamera dış parametresini
gösterir. Kamera/tripod yerinden oynadıysa mevcut kalibrasyon kullanılmamalıdır.

## 2. El takibi neden görünmedi?

Fusion kaydındaki her el paketi boş `per_camera` listesi ve
`INSUFFICIENT_RELIABLE_VIEWS` taşımaktadır. Bu, detektörün çalışmadığı anlamına
gelmiyor. İki kaynak JSONL'sindeki el paketleri şema ve boyut açısından geçerli:

| Kamera | El paketi | Sol | Sağ | İki el | En az bir el | Hız | p50 / p95 inference |
|---|---:|---:|---:|---:|---:|---:|---:|
| 33773329 | 1350 | %44.59 | %34.59 | %19.26 | %59.93 | 7.49 Hz | 26.1 / 46.6 ms |
| 31571870 | 1338 | %31.61 | %32.66 | %10.99 | %53.29 | 7.50 Hz | benzer sınıf |

Fusion zamanlarıyla çevrimdışı eşleştirmede `33773329` için 2562/2562 karede,
`31571870` için 2324/2569 karede 120 ms'den genç bir el paketi bulunmuştur.
JSON paketleri de 60 kB UDP sınırının çok altındadır. Sonuç: model ve zamanlama
veri üretmiş; ayrı el UDP yolu paketleri fusion alıcısına taşımamıştır. Olası
nedenler host firewall'ı, yanlış port/yön veya ayrı kanalın sessizce kaybolmasıdır.

### Uygulanan taşıma düzeltmesi

- Ayrı UDP el kanalı korunmuştur.
- Aynı el paketi BODY datagramına yedek olarak eklenmiştir.
- Fusion alıcısı iki yolu doğrular, zamanını düzeltir ve tekrarları ayıklar.
- `separate`, `embedded`, `duplicate`, `invalid` sayaçları ve beş saniyelik
  “BODY var, el paketi yok” uyarısı eklenmiştir.
- Kaynak kapanışında detektör işi drop sayısı ayrıca raporlanır.
- BODY+el birleşimi nadiren 60 kB sınırını aşarsa BODY akışı kesilmez; yalnız
  gömülü el yedeği o hedef için çıkarılır ve olay sayacı raporlanır.

Bu tasarım ağdaki ayrı el portları kapalı kalsa bile el verisini BODY ile aynı
çalışan yoldan taşır; ayrı kanal çalışıyorsa gereksiz paketi deduplikasyon atar.

## 3. SVO2 ile ROI A/B sonucu

İki SVO2 de BODY_38 ve MediaPipe ile baştan sona tekrar oynatıldı. Her iki dosyanın
sonunda bulunan bozuk kuyruk SDK tarafından otomatik onarıldı. Sonraki okumada
`33773329` için 2861/2861, `31571870` için 2864/2864 frame hatasız okundu;
sırasıyla 2707 ve 2683 BODY_38 kaydı üretildi.

| Profil | Sol | Sağ | İki el | En az bir el | Ortalama el-geçerli | p50 / p95 |
|---|---:|---:|---:|---:|---:|---:|
| Eski ikinci ROI `0.78` | %48.81 | %37.83 | %21.22 | %65.43 | %43.32 | 27.06 / 47.22 ms |
| Seçilen ikinci ROI `0.65` | **%52.04** | %37.81 | **%21.50** | **%68.35** | **%44.92** | 27.67 / 47.45 ms |

`0.65` profili sol eli +3.23 puan ve “en az bir el” ölçümünü +2.92 puan
artırdığı için varsayılan yapıldı. İlk denenen BODY_38 yönlendirmeli kaba ROI
%59.94 “en az bir el” sonucu vererek geriledi; geri alındı. Minimum ROI'yi
96 piksele düşürmek de iyileşme sağlamadı. Algılama confidence eşikleri, etiketli
ground-truth olmadan yanlış-pozitif riskini artırmamak için 0.20'de tutuldu.

Seçilen profil `31571870` tekrarında sol %32.76, sağ %31.94, iki el %10.25,
en az bir el %54.45 ve ortalama el-geçerli %32.35 verdi. Bu kamerada kazanç
sınırlıdır; sağ el için 35.34 saniyelik en uzun kayıp görülmesi görüş açısı,
oklüzyon veya kişinin o bölümde kameraya dönük olmaması gibi kamera-özel bir
sorunu gösterir. Etiketli video olmadan bunlar birbirinden kesin ayrılamaz.

Bu iyileştirme tek kamera detector coverage'ıdır; çoklu görünüm fused el kalitesi
ancak yeni canlı kayıtta ölçülebilir. Mevcut %44.92 ortalama geçerlilik, %80 kabul
hedefinin altındadır. Mesafeye göre `33773329` temel profilinde 0–2 m'de sol/sağ
yaklaşık %65.8/%52.0, 2–2.5 m'de %77.3/%47.4, 2.5–3 m'de %48.1/%32.1 ve
3–3.5 m'de %39.2/%35.7 görülmüştür. Yaklaşmak yardımcı olmuş, özellikle sağ eli
tek başına çözmemiştir.

Üretilen ölçümler:

- `recordings/reprocessed_20261005_roi_ab/zed337_baseline_benchmark.json`
- `recordings/reprocessed_20261005_roi_ab/zed337_focus65_benchmark.json`
- `recordings/reprocessed_20261005_roi_ab/zed315_focus65_benchmark.json`

## 4. Sistem neden bazen durup stabil poza dönüyor?

Eşleşen Rerun oturumunda 1648 imitation karesi vardır: 1153 GREEN, 195 YELLOW,
300 ORANGE. Stabil poza dönüşlerin çoğu rastgele duruş değil, güvenlik
makinesinin tasarlanmış davranışıdır.

| Neden | Kare sayısı |
|---|---:|
| `self_collision_risk` | 303 |
| `orange_safe_return` | 205 |
| `robot_body_barrier_projection` | 100 |
| `left_wrist_occluded` | 88 |
| `gmr_high_residual` | 68 |
| `right_wrist_occluded` | 28 |
| `self_collision_proximity` | 15 |

Ayrıca paket akışında 29.947 s, 5.667 s ve 0.467 s boşluklar vardır. Watchdog bu
boşlukları doğru biçimde güvenli dönüş olarak ele almıştır. Güvenlik eşiklerini
gevşetmek görünür duraklamayı azaltabilir, fakat fiziksel robot riskini artırır;
bu nedenle yapılmadı. Öncelik veri kesintisini ve yanlış bariyer geometrisini
düzeltmektir.

GMR residual p50/p90/p99 değerleri 0.0149/0.110/0.301 m, maksimum 0.575 m'dir.
Kol IK medyanı iyidir: sol 0.008 m, sağ 0.012 m. Kuyruk hataları ve oklüzyonlarda
bozulma büyüdüğü için “IK iyi görünüyor” gözlemi ortalama davranış için doğru,
uç durum güvenliği için yeterli değildir.

## 5. Dex3 elinin gövde içinden geçmesi

Önceki bariyer wrist-roll sonrasında 0.107946 m'lik G1 lastik el ucu kullanıyordu.
Resmi G1-29 + Dex3 zincirindeki wrist pitch, wrist yaw, palm mount, finger base,
proximal ve distal uzunluklarının açık-el zarfı yaklaşık 0.299 m'dir. Bariyer
Dex3'ün büyük bölümünü hesaba katmadığı için parmaklar gövdeye girebiliyordu.

Uygulanan değişiklikler:

- `g1_23dof` ve `g1_29dof_dex3` uç profilleri ayrıldı.
- Dex3 profili tüm bariyer ileri-kinematik kontrollerinde 0.299 m muhafazakâr
  erişim kullanır.
- Launcher seçilen asset profilini GMR'ye açıkça geçirir.
- Isaac Dex3 asset ile gelen GMR güvenlik profilini kontrol eder; profil eksik
  veya farklıysa komutu `REJECTED_END_EFFECTOR_PROFILE` ile reddeder.
- Fiziksel Dex3 DDS çıkışı halen kapalıdır.

Bu, görülen ana bilek-parmak zarfı açığını kapatır. Yine de parmak/link meshlerinin
tam çarpışma kanıtı değildir. Dinamik simülasyonda ayrıntılı link geometrisi,
temas sensörü ve minimum mesafe testi geçmeden fiziksel robot serbest
bırakılmamalıdır.

## 6. Resmi Unitree aktarım uyumu

`config/unitree_official_sources.lock.json` artık yerel resmi kaynakların
commitlerini sabitler: `unitree_sim_isaaclab`, `xr_teleoperate`, `unitree_ros` ve
`unitree_sdk2_python`. Doğrulama komutu mevcut kurulumda başarıyla tamamlandı:

```text
UNITREE_DEX3_VERIFY_OK pinned_sources=4 asset=official
retarget=normalized_21_task_fallback dds=disabled
```

Önemli sınırlama: resmi izole DexPilot solver ortamı henüz kurulu değil; sistem
halen `normalized_21_task_fallback` kullanıyor. Ayrıca bu oturum
`kinematic_debug` modudur; denge, temas ve gerçek motor geri beslemesi
doğrulanmamıştır.

## 7. Yeni canlı kabul testi

1. `31571870` kamera montajını sabitleyin; tüm kameralar hareketsizken yeniden
   extrinsic kalibrasyon alın. İlk 30 s testte 4/4 katkı ve sıfır
   `CALIBRATION_DRIFT` görülmeden devam etmeyin.
2. Rerun, Isaac/GMR, fusion, laptop kaynakları ve ana-PC kaynaklarını
   `docs/UBUNTU_4ZED_ISAAC_RERUN_HAND_SVO2_CALISTIRMA_TR.md` sırasıyla başlatın.
3. Kaynak loglarında `El paketleri`, fusion logunda `el_rx=separate+embedded`
   sayaçlarının arttığını doğrulayın. Rerun el `per_camera` listesi boşsa testi
   geçersiz sayın.
4. 1.5, 2.0, 2.5 ve 3.0 m'de açık el, yumruk, pinch, el rotasyonu, gövde önünde
   çaprazlama ve kısa oklüzyonları iki el için kaydedin. Dört SVO2 açık kalsın.
5. Kabul kapıları: kaynak katkısı 4/4 >%95, kamera başı el inference >=10 Hz,
   fused geçerli el coverage >=%80, capture-spread p95 <=40 ms, uçtan uca el
   p95 <=100 ms, 350 ms'den uzun beklenmeyen boşluk yok, sıfır kimlik değişimi
   ve sıfır gövde penetrasyonu.
6. Sonra sırasıyla kinematik replay, dinamik simülasyon, hardware dry-run ve
   askıda/düşük hızlı fiziksel test uygulayın. Fiziksel testte state feedback,
   CRC, joint position/velocity/acceleration limitleri, dead-man ve bağımsız
   emergency-stop zorunludur.

## Son durum

- El paketinin tamamen kaybolmasına yol açan taşıma mimarisi düzeltildi.
- SVO ölçümüyle faydası gösterilen ROI adayı varsayılan yapıldı.
- Dex3 asset/safety profil eşleşmesi fail-closed hale getirildi.
- Kötü kalibrasyon ve veri boşlukları teşhis edildi; güvenlik dönüşleri
  gevşetilmedi.
- Fiziksel robot etkinleştirilmedi. Yeni dört-kamera kabul testi, resmi DexPilot,
  dinamik self-collision/denge testi ve donanım geri-bildirim güvenlik katmanı
  tamamlanana kadar durum **HOLD**'dur.
