# ZED 2i → G1 EDU 23-DOF güvenli canlı taklit hattı

Bu sürüm yalnız simülasyonda çalışır. `physical_robot_output=false` korunur;
Unitree DDS motor topic'i açılmaz. Gerçek robota geçiş; askı, fiziksel E-stop,
operatör güvenlik alanı ve bağımsız tork/çarpışma denetimi doğrulanmadan
yapılmamalıdır.

## Kaynak temeli

- [Stereolabs BODY Tracking API](https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api): BODY_38, tracking ID, body fitting, yerel eklem rotasyonları ve positional tracking.
- [Stereolabs Body Tracking](https://docs.stereolabs.com/docs/development/zed-sdk/modules/body-tracking): stereo derinlik + yapay sinir ağı iskelet çıkarımı.
- [GMR](https://github.com/YanjieZe/GMR): gerçek zamanlı, optimizasyon tabanlı genel motion retargeting.
- [Unitree Isaac Lab simülasyonu](https://github.com/unitreerobotics/unitree_sim_isaaclab): simülasyon/gerçek robot DDS ayrımı ve G1 simülasyon yapısı.
- [Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2): resmî G1 tanımları ve ROS 2 entegrasyonu.
- [Isaac Lab actuator sınırları](https://isaac-sim.github.io/IsaacLab/develop/source/api/lab/isaaclab.actuators.html): eklem konum, hız ve efor sınırlarının ayrımı.
- [One Euro Filter](https://gery.casiez.net/1euro/): hıza bağlı kesim frekansı ile jitter/gecikme dengesi.
- [Safe Human–Humanoid Motion Imitation](https://github.com/RISC-NYUAD/Safe_Human_Humanoid_Motion_Imitation): algı, retargeting, kontrol ve güvenlik filtresini ayrı katmanlar hâlinde kurma yaklaşımı.

## Uygulanan veri akışı

1. `zed_g1_skeleton.py` ZED BODY_38 çıkarır.
2. `OperatorSelector`, aynı adayın 10 kare kararlı olmasını bekler; `body_id`
   ve `unique_object_id` kilitlenir. Operatör kaybolunca `LOST` olur ve başka
   kişiye otomatik geçmez. Yalnız `R` tuşu yeni seçim izni verir.
3. İlk kilitte 4 saniyelik nötr kalibrasyon yapılır. Omuz/pelvis genişliği,
   torso, iki taraflı üst kol, ön kol, uyluk, kaval, ayak açıklığı, zemin ve
   nötr pelvis/omuz dönüşü robust medyan ile kaydedilir.
4. Keypoint'ler her karede `PELVIS_LOCAL_X_FWD_Y_LEFT_Z_UP` koordinatına
   alınır. Kamera konumu ve kişinin kameraya göre birkaç santimetrelik
   ötelemesi robot hedefini taşımaz.
5. Omuzlar ve kalçalar 2B torso poligonunu oluşturur. Dirsek/bilek poligon
   içine girerse tek nokta tutmak yerine omuz–dirsek–bilek zinciri kalibre
   edilmiş iki kemik uzunluğuyla birlikte çözülür.
6. `Body38ToGMR`, kalibre insan yönlerini resmî G1 link merkezleri ve erişim
   yarıçapına ölçekler. Ulaşılamayan bilek hedefi en yakın erişilebilir noktaya
   yumuşak projekte edilir.
7. GMR yalnız resmî 23 eklemi açık bırakır. Yeni
   `zed_body38_to_g1_23dof.json` görev ağırlıkları üst beden takibine göre
   ayarlanmıştır. BODY_38 yerel quaternion'ları doğrudan kullanılmaz:
   Stereolabs ebeveyn zinciri pelvis'ten ilgili ekleme kadar bileştirilerek
   global yönelim üretilir, sonra aynı pelvis-local koordinat sistemine alınır.
   Dirsek IK'sinde insan anatomisine aykırı geriye katlanma dalı solver içinde
   engellenir; fiziksel motor limitleri değiştirilmez.
8. One Euro filtre XYZ keypoint'lerine bağımsız uygulanmaz. GMR sonrasındaki
   23 eklem uzayında uygulanır: kalça/diz 3–6 Hz, ayak bileği 2–5 Hz, bel
   3–5 Hz, omuz 5–8 Hz, dirsek 6–10 Hz, bilek 4–7 Hz. 30 FPS'de Nyquist
   güvenlik payı otomatik korunur.
9. `G1FeasibilityFilter`, resmî G1-23DOF eklem limitleri ile konservatif hız
   ve ivme limitlerini uygular. Isaac yalnız `safe_q` alır; `raw_q` yalnız
   analiz içindir.

## Safety durumu

- `GREEN`: normal hedef, blend 1.0.
- `YELLOW`: uygulanabilir fakat kalite/limit uyarısı vardır, blend 0.55.
- `ORANGE`: son güvenilir hedef tutulur.
- `RED`: sayısal olmayan/bozuk hedef; güvenli nominal poz.

Sebep alanı tek bir `LOW_QUALITY` yazısı değildir. Şunlardan birini veya
birkaçını taşır: `right_wrist_occluded`, `left_wrist_occluded`,
`left_right_swap_risk`, `bone_length_violation`, `stale_packet`,
`gmr_high_residual`, `joint_limit_saturation`, `self_collision_risk`.

## Rerun görünümü ve metrikler

Rerun'da insan BODY_38 iskeleti gerçek operatör ölçeğinde; G1 `raw_q`
turuncu, `safe_q` yeşil iskelet olarak ayrı gösterilir. Eklem seçildiğinde
pozisyon, hız, quaternion, Euler açıları, ilişkili geometrik açılar ve filtre
durumu görülebilir. RRD/JSONL/CSV kaydı zaman çizelgesinde tekrar oynatılabilir.

Canlı karşılaştırma görünümü aynı karede `HUMAN PRE-GMR` (mavi), `G1 RAW`
(turuncu) ve `G1 SAFE` (yeşil) iskeletlerini yan yana kaydeder. Böylece GMR
öncesi hedef, IK sonucu ve safety sonrası komut birbirinden ayrılabilir.

Taşınan metrikler:

- algı: visible keypoint ratio, kemik uzunluğu bağıl hatası/CV, sağ-sol swap,
  oklüzyon kurtarma süresi;
- retargeting: IK residual, workspace projection, limit saturation, solver
  süresi/iterasyonu ve self-collision;
- Isaac: joint tracking RMSE, base roll/pitch, foot slip, torque RMS, adım
  enerjisi, düşme sayısı/oranı ve episode success;
- sistem: capture/effective FPS, paket kaybı, jitter, watchdog ve p50/p95/p99.

Zaman damgaları `T0…T8` olarak korunur: ZED işleme `T1-T0`, Windows→WSL
`T3-T2`, GMR `T5-T4`, Isaac komut `T7-T6`, toplam kontrol `T8-T0`.

## Çalıştırma sırası

Üç ayrı PowerShell kullanın:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_g1_rerun.ps1
```

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 `
  -Mode upper_body -InputFps 60 -AcceptNvidiaEula
```

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_zed_live_smooth_to_wsl.ps1 `
  -Profile realtime_60
```

Operatör nötr biçimde, kollar gövdeden biraz açık ve iki ayak yerde 4 saniye
bekler. Ekranda `LOCKED CAL=READY 100%` görülmeden Isaac hedefi üretilmez.

## Kayıt bulgusu ve eğitim sınırı

Son yerel kayıtlarda etkin hız 15 FPS, görünür keypoint oranı yaklaşık
%93.7–%94.5 ve kol kemiği CV değerleri yaklaşık %0.7–%2.1 bulundu. Bu değerler
kalibrasyon/retargeting başlangıcı için uygundur; ground-truth motion-capture
olmadığı için milimetrik doğruluğu kanıtlamaz.

On binlerce policy bu değişiklikle “üretilmiş” sayılmaz. Policy eğitimi için
önce kabul edilmiş `safe_q`, gözlem, tracking error ve safety state eşleşmeleri
episode olarak kaydedilmeli; train/validation kişileri ayrılmalı; Isaac Lab'de
domain randomization ve düşme/çarpışma cezalarıyla eğitim yapılmalı; ardından
SVO2 kayıtları deterministik replay testinde doğrulanmalıdır. Mevcut aşamada
deterministik GMR + feasibility hattı, RL policy'den önce güvenilir veri üretir.
