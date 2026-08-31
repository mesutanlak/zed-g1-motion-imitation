# G1 23-DOF reference-motion tracking policy

Bu yapı canlı BODY_38/IK zincirinin yerine geçmez. Canlı ya da kayıtlı algı
zincirinin ürettiği temiz G1 referansını fiziksel olarak uygulanabilir biçimde
takip etmeyi öğrenir:

```text
SVO2 veya BODY_38 JSONL
  -> offline GMR + 23-DOF arm IK
  -> güven/zaman boşluğu temizliği
  -> 50 Hz fiziksel reference clip
  -> G1 FK body hedefleri
  -> Isaac Lab + RSL-RL PPO
  -> q_des = q_ref + sınırlı residual
```

## Politika sözleşmesi

Policy observation sırası sabittir ve toplam 88 değerdir. Eğitim kökü sabittir;
12 bacak eklemi resmi nominal double-support pozunda kalır. Policy yalnız bel ve
kollar için GMR/IK referansının çevresinde küçük bir düzeltme öğrenir:

| Gözlem | Boyut | Açıklama |
|---|---:|---|
| `projected_gravity` | 3 | Robot gövde frame'inde yerçekimi |
| `base_ang_vel` | 3 | Gövde açısal hızı |
| `q - q_default` | 11 | Bel + iki kolun nominal poza göre eklemleri |
| `qd` | 11 | Üst-gövde eklem hızları |
| `q_ref - q` | 11 | BODY_38/GMR/IK tracking hatası |
| `qd_ref - qd` | 11 | Üst-gövde hız tracking hatası |
| `body_reference_error` | 24 | Pelvis, torso, omuz, dirsek ve el konum hatası |
| `foot_contact_state` | 2 | Sabit double-support durumu |
| `previous_action` | 11 | Residual eylem sürekliliği |
| `reference_confidence` | 1 | 0–1 temiz referans güveni |

Eylem 11 boyutlu ve eklem başına en fazla 0,15 rad küçük residual'dır. Referans güveni 0.70 altında azalarak
etki eder; 0.25 altında yeni güvensiz referans kabul edilmez ve son güvenli
hedef tutulur. Eylemden sonra G1'in soft joint limitleri zorunlu uygulanır.

Reward yapısı `config/reference_motion_tracking.yaml` ile aynıdır:

- joint tracking, el/dirsek konumu, normalize üst-kol/ön-kol yönü ve body orientation pozitif reward;
- üst-gövde torku, eklem ivmesi, action rate/magnitude, joint limit ve
  self-collision yakınlığı negatif penalty;
- denge, yürüme, itme recovery ve düşme öğrenimi yoktur; episode yalnız süre
  sonunda biter.

Self-collision reward'u dört kritik arm-arm/arm-torso link çifti için yoğun
yakınlık cezasıdır. Isaac sürümünün external contact sensörü bütün self-contact
olaylarını güvenilir vermediği için bu, PhysX self-collision ile birlikte erken
uyarı görevi görür.

## Kayıttan eğitim verisi hazırlama

PowerShell'de proje dizininde:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force

# BODY_38 JSONL
.\prepare_g1_reference_dataset.ps1 `
  -InputPath .\recordings\zed_dual_body38_20260819_164136.jsonl `
  -OutputDir .\datasets\reference_motion\164136 `
  -Mode upper_body

# SVO2: aynı komut, yalnız InputPath .svo2 olur.
```

SVO2 önce gerçek kayıt temposunda yeniden oynatılır ve BODY_38 JSONL çıkarılır.
Klip hazırlayıcı yalnız `LOCKED + READY` kareleri kullanır, kamera/iskelet
güvenini birleştirir, 200 ms'den büyük zaman boşluklarında klibi böler, 50 Hz'e
yeniden örnekler ve 5 rad/s ile 30 rad/s² reference sınırlarını uygular.
`clips/clips_manifest.json` içindeki her `reference_npz` bağımsız eğitim
klibidir. Veri FK'sı resmi Unitree `g1_23dof.xml` ile MuJoCo'da hesaplandığı için
veri hazırlama Isaac GUI/GPU sürücüsüne bağlı değildir.

## Multi-clip eğitme ve doğrulama

Hazırlama komutu klipleri birleştirmez. Güven ortalaması ve güven tabanı ile
hesaplanan kalite skoruna göre en temiz bağımsız klibi `validation`, kalanları
`train` olarak `clips_manifest.json` içine yazar. Isaac'te her environment
reset sırasında dengeli biçimde bir train klibi ve o klip içinde bir başlangıç
frame'i örnekler. Kısa klibin padding bölgesi hiçbir zaman örneklenmez.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\start_g1_reference_policy_training.ps1 `
  -ClipsManifest ".\datasets\reference_motion\114253\clips\clips_manifest.json" `
  -NumEnvs 512 `
  -MaxIterations 2000 `
  -ValidationSteps 500 `
  -AcceptNvidiaEula
```

Eğitim varsayılan olarak headless çalışır. İlk smoke test için `-NumEnvs 64
-MaxIterations 10`; RTX 5090 üzerinde başlangıç için 512 environment uygundur.
GUI ile eğitim yalnız görsel tanı içindir ve `-Gui` ile açılır. Checkpoint'ler
`logs/rsl_rl/g1_23dof_upper_body_residual/` altına yazılır. Eğitim sonunda
held-out validation klibi aynı başlangıçlarla iki kez oynatılır: önce
`NORMAL IK / zero residual`, sonra `Policy + IK`. Joint, el, dirsek, limb
direction, orientation, ivme, tork, limit saturation, self-collision proxy ve
foot-slip metrikleri karşılaştırılır. Aday takip hatasını %0,5'ten fazla
kötüleştirirse mevcut canlı policy korunur. Yalnız kabul edilen policy
normalizer ile birlikte atomik olarak şuraya yayımlanır:

```text
policies/g1_reference_upper_body/policy.onnx
policies/g1_reference_upper_body/policy_metadata.json
policies/g1_reference_upper_body/policy_evaluation.json
policies/g1_reference_upper_body/policy_evaluation.md
```

Her adayın raporu, kabul edilmese bile kendi run klasöründe kalır:
`logs/rsl_rl/g1_23dof_upper_body_residual/<run>/exported/`.

Eski `g1_23dof_reference_motion` checkpoint'leri 23 action/168 observation
sözleşmesine aittir ve bu yeni canlı policy ile uyumlu değildir.

```powershell
.\start_g1_reference_policy_play.ps1 `
  -MotionNpz ".\datasets\reference_motion\164136\clips\...clip_000.npz" `
  -Checkpoint ".\logs\rsl_rl\g1_23dof_reference_motion\...\model_....pt" `
  -AcceptNvidiaEula
```

610.88 sürücüsünde Warp'ın `cuDeviceGetUuid / API not supported` ön uyarısı
görülebilir; bu makinede D3D12 + CUDA PhysX eğitim ve canlı smoke testleri
başarıyla tamamlanmıştır. Başarı ölçütü uyarı satırı değil, `Learning iteration`
ve canlıda `G1 scene initialization complete` satırlarının görülmesidir.

## Canlı Normal IK / Policy Powered kullanımı

Dual ZED ve analiz panelini normal biçimde açın. Isaac komutu policy dosyasını
otomatik bulur ve her zaman `NORMAL IK` ile başlar. Varsayılan canlı açılış
`kinematic_debug + fixed_double_support` biçimindedir: robot kökü ve alt beden
nominal pozda sabit kalır; bu aşamada denge/yürüme policy'si çalıştırılmaz:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 `
  -Mode upper_body `
  -AcceptNvidiaEula `
  -AllowUnvalidatedDriver
```

PhysX altında bağımsız denge testi ancak açıkça
`-ImitationMode dynamic -StanceMode balance_policy` verilirse etkinleşir.

Tek kamera ZED, Dual ZED fusion veya Isaac penceresi odaktayken `P`:

- `NORMAL IK`: yalnız BODY_38 -> fusion -> GMR/IK sonucu;
- `POLICY POWERED`: aynı IK hedefi + güvene bağlı, limitli 11-eklem residual.

Policy hiçbir modda bacak komutu üretemez. Başlangıçta doğrudan policy modunu
istemek için `-PolicyPowered` eklenebilir. Model henüz eğitimden çıkmadıysa
normal IK çalışmaya devam eder ve `P` bilinçli olarak devre dışı kalır.
Isaac eğitimden önce açılmış olsa bile `P` basıldığında kabul edilmiş yeni
deployment'ın dosya imzası kontrol edilir ve policy hot-reload edilir; Isaac'i
yeniden başlatmak gerekmez.

## Eğitim sırası

1. `upper_body`: sabit kök/alt beden, kısa ve temiz arm/torso motion klipleri.
2. Aynı hareketlerle hold-out değerlendirme; eğitim klibini değerlendirmede
   kullanmayın.
3. Cross-body, hand-on-chest, iki kol eşzamanlı ve hızlı/yavaş gesture klipleri.
4. `clips_manifest.json` ile environment başına dengeli örneklenen multi-clip
   generalist policy; validation klibi hiçbir train environment'ına girmez.
5. Canlı `q_ref/qd_ref/confidence` adaptörü ve 30 Hz -> 50 Hz latest-valid
   resampler ile A/B testi. Canlı kamera PPO eğitim loop'una bağlanmaz.

Kabul metrikleri: üst-gövde joint RMSE p50 ≤ 0.05 rad, el/dirsek MPJPE p50 ≤ 3
cm, faz gecikmesi p50 ≤ 120 ms, limit saturation ≤ %1 ve self-collision ≤ %0.5.
Normal IK ile Policy Powered aynı kayıt üzerinde karşılaştırılmalıdır.

## Harici hareket veri setleri

Öncelik kendi kalibre edilmiş SVO2 kayıtlarınızdır; kamera ve operatör dağılımı
canlı sisteme en yakın veridir. Harici veri doğrudan G1 joint'i değildir ve her
klip BODY/SMPL/BVH -> G1 23-DOF retargeting, workspace projection, joint-limit
ve temas temizliğinden geçmelidir.

- **LAFAN1**: 5 kişi, 77 BVH sequence, 496.672 frame, 30 FPS ve yaklaşık 4,6
  saat. Walk, obstacle, dance, fall/get-up gibi yararlı sınıflar içerir.
  Lisansı CC BY-NC-ND 4.0'dır; ticari/derivative kullanım şartlarını dikkatle
  inceleyin. https://github.com/ubisoft/ubisoft-laforge-animation-dataset
- **AMASS**: 300'den fazla kişi, 11.000'den fazla motion ve 40+ saat standardize
  SMPL/SMPL-H hareket. Dataset/model lisansına kayıt olmak gerekir; araştırma
  şartlarını doğrulayın. https://github.com/nghorbani/amass
- **CMU Motion Capture Database**: 2.605 trial; locomotion, interaction, sport
  ve scenario kategorileri. Resmi site kullanım koşullarını açıklar ve toe/hand
  verilerinin gürültülü olabileceği uyarısını yapar. https://mocap.cs.cmu.edu/
- **BeyondMimic/LAFAN1 pipeline**: motion tracking reward ve evaluation tasarımı
  için en yakın açık referanstır. https://beyondmimic.github.io/ ve
  https://github.com/MinusModulo/Beyondmimic
- **Unitree RL MjLab G1-23DOF**: resmi 23-DOF CSV->NPZ ve tracking config'i joint
  order/self-collision karşılaştırması için referanstır.
  https://github.com/unitreerobotics/unitree_rl_mjlab
- **NVIDIA ProtoMotions**: AMASS ölçeğinde motion library, retargeting ve
  Isaac Lab değerlendirme yaklaşımı için yararlıdır.
  https://github.com/NVLabs/ProtoMotions

İlk dengeli veri karışımı için öneri: %35 stand/upper-body gesture, %20
cross-body/hand-on-chest/occlusion, %20 walk/turn, %10 squat/weight shift, %10
recovery ve %5 zor hareket. Fall veya cartwheel gibi hareketler ilk policy'ye
eklenmemelidir; önce ayakta tracking metriği oturmalıdır.
