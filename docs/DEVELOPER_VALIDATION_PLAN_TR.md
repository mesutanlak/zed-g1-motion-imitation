# ZED 2i → G1 23-DOF geliştirme ve doğrulama planı

## Temel ilke

Algı, retargeting, fiziksel kontrol ve öğrenme ayrı katmanlardır. Bir katmanın
hatası sonraki katmanda filtre veya policy ile gizlenmemelidir. Her değişiklik
aynı ham kayıtla deterministik replay edilip önceki baseline ile karşılaştırılır.
Gerçek robot çıkışı; simülasyon, HIL ve askı testleri tamamlanana kadar kapalıdır.

## Önerilen repository yapısı

```text
capture/            ZED SDK, kimlik kilidi, kalibrasyon, ham kayıt
perception/         pelvis-local iskelet, güven, oklüzyon ve zincir kestirimi
retargeting/        BODY_38 → GMR → yalnız G1-23DOF raw_q
feasibility/        joint/velocity/acceleration/workspace/collision safe_q
simulation/         Isaac Lab tüketicisi, denge policy, telemetri
training/           offline dataset, reference-state builder, RL ortamı
visualization/      Rerun ve ROS 2/RViz adaptörleri
tests/              birim, sentetik oklüzyon, replay, simülasyon ve HIL
reports/            benchmark çıktıları
config/             şemalar, robot limitleri ve deney profilleri
```

Mevcut kod bu ayrıma doğru ilerliyor. Taşıma tek seferde yapılmamalı; önce
arayüzler (JSON schema ve Python dataclass), sonra modüller ayrılmalıdır.

## Veri sözleşmesi

Her frame aşağıdaki zaman damgalarını ve kaynak durumunu korumalıdır:

- T0 kamera capture;
- T1 ZED body tracking tamamlandı;
- T2 Windows UDP gönderimi;
- T3 WSL alımı;
- T4/T5 GMR başlangıç/bitiş;
- T6/T7 Isaac alım/komut uygulama;
- T8 fizik adımı sonrası gözlem.

Kayıtlar ham keypoint, ZED filtreli keypoint, pelvis-local keypoint, confidence,
tracking state, body ID, yerel quaternion, operatör/kalibrasyon state ve quality
reason taşımalıdır. `raw_q`, `filtered_q`, `safe_q` birbirinin üzerine
yazılmamalıdır.

## Test piramidi

1. Birim test: koordinat dönüşümü, kimlik kilidi, kemik kısıtı, limit ve filtre.
2. Sentetik test: bilek-torso örtüşmesi, 1–5 frame dropout, sol/sağ karışması,
   bozuk timestamp ve paket sırası.
3. Offline replay: aynı JSONL/SVO2 ile capture ve GMR metriklerinin tekrar üretimi.
4. Isaac headless: hedef hareketi, joint RMSE, base roll/pitch, ayak kayması,
   tork/enerji, düşme ve self-collision.
5. Isaac GUI canlı: GPU paylaşımı, effective FPS ve p95/p99 gecikme.
6. HIL: motor çıkışı kapalıyken gerçek DDS state ve watchdog.
7. Fiziksel robot: askı + E-stop + düşük hız + tek kol; kapsam kademeli artar.

`validate_motion_pipeline.ps1` ilk üç kapının çekirdeğidir.

## Deney tasarımı

Her deney tek değişkenli olmalıdır.

| Deney | Değişken | Sabitler | Kabul ölçütü |
|---|---|---|---|
| ZED profil | 15/30 FPS, model | aynı SVO2/poz | core visibility, jitter, p95 latency |
| smoothing | ZED smoothing | aynı GMR | jerk azalırken latency artışı sınırlı |
| GMR weight | bilek/dirsek weight | aynı capture | end-effector hata ve limit saturation |
| memory | 0.10/0.20/0.30 s | aynı oklüzyon klibi | recovery ve false-hold |
| actuator | Kp/Kd | aynı safe_q | RMSE, torque RMS, overshoot |

Kamera verisi için mutlak “milimetrik” iddia ancak marker tabanlı motion-capture
ground truth ile yapılabilir. ZED keypoint'leri anatomik eklem merkezleri değil,
öğrenilmiş ve tutarlı kinematik landmark'lardır.

## Smooth takip için öncelik sırası

1. Kimlik ve calibration kilidi; başka kişiye handover yok.
2. 30 Hz gerçek capture. Tek GPU'da ZED+Isaac bunu koruyamıyorsa ZED ayrı GPU/
   bilgisayara alınır; 15 Hz veriyi aşırı filtrelemek 30 Hz yerine geçmez.
3. Bozuk kareyi at; son iyi zinciri kısa süre tut. Hatalı noktaya doğru interpolate
   etme.
4. XYZ'yi bağımsız filtrelemek yerine rijit kol/bacak zinciri ve pelvis-local
   geometriyi optimize et.
5. GMR sonrasında joint-space One Euro ve hız/ivme projection uygula.
6. Isaac'ta gerçek actuator limitleri ve PD dinamiği kullan; yalnız soft position
   clamp yeterli değildir.
7. Gecikme telafisini ancak T0…T8 ölçüldükten sonra, sınırlı horizon ile dene.

## Öğrenme aşaması

GMR canlı teleoperation solver'ıdır; tek başına denge policy'si değildir. Üst
beden sabit çift destek aşaması deterministik IK ile tamamlanmalıdır. Daha sonra:

1. Kabul edilmiş `safe_q`, q/qd, root state, contact ve quality flag episode olarak
   kaydedilir.
2. Operatör bazında train/validation/test ayrılır; aynı kişinin komşu frameleri
   farklı split'e konmaz.
3. AMASS/LAFAN1 gibi motion kaynakları GMR ile G1'e retarget edilir; ZED gürültü ve
   oklüzyon modeli ayrı domain randomization olarak eklenir.
4. Privileged teacher tam robot state ve gelecek reference alır; student gerçek
   robotta bulunacak proprioception ve seyrek pose hedefini alır.
5. Reward: local body pose/velocity, end-effector, contact, root stability,
   action-rate, torque/energy ve collision terimlerinden oluşur.
6. Policy yalnız yeni, görülmemiş operatör ve hareketlerde benchmark'ı geçerse
   sim-to-real kapısına girer.

## Kaynak kararları

- Stereolabs BODY_38 + body fitting: kayıp noktalar ve local rotations için ana
  algı kaynağı; confidence/minimum keypoint/prediction timeout sahneye göre ölçülür.
- GMR: morfoloji farkı için task-space optimizasyon ve resmi Unitree model zinciri.
- Unitree Isaac Lab: simülasyon/gerçek robotla aynı DDS semantiğine yaklaşmak.
  Resmi `unitree_sim_isaaclab` görevleri şu anda G1-29DOF (`g129`) yayımlar;
  projenin 23-DOF Isaac asset'i bu nedenle resmi Unitree URDF/mesh'ten üretilmiş
  ayrı bir uyarlamadır ve 29-DOF policy ile eşdeğer kabul edilmemelidir. Resmi
  23-DOF RL baseline bugün `unitree_rl_mjlab` tarafında bulunur.
- H2O/OmniH2O: kinematik referanstan physics-aware tracking policy'ye geçişte
  teacher-student ve domain randomization modeli.
- CBF tabanlı safe imitation: IK'den sonra, robot komutundan önce ayrı feasibility
  filtresi; safety katmanı retargeting solver'ının içine gizlenmez.
