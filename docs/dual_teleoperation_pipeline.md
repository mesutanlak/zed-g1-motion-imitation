# Dual ZED → G1 teleoperation pipeline

## Ölçülen 2026-08-19 başlangıç durumu

Kaynak kayıt: `recordings/zed_dual_body38_20260819_122000.jsonl`.

| Aşama | Mevcut implementasyon | Girdi → çıktı | Frekans | Frame | Timestamp |
|---|---|---|---:|---|---|
| Kamera yakalama | `zed_dual_body38_fusion.py`, iki yerel `sl.Camera` | ZED görüntü/depth → kamera-local BODY_38 | 30 Hz istek; kayıtta iki kamera da ortalama ≈24.2 BODY_38/s | `RIGHT_HANDED_Z_UP_X_FWD`, metre | `TIME_REFERENCE.IMAGE`; ayrıca host receive ns |
| Statik extrinsic | ZED360 JSON + `transform_body_to_fusion_frame` | `p_camera` → `p_world = R p + t` | Her iskelet | Ortak Fusion/world | Capture timestamp korunur |
| SDK Fusion | `sl.Fusion` publish/subscribe | İki BODY_38 → SDK fused BODY_38 | SDK process döngüsü | Ortak Fusion/world | SDK görüntü zaman eşleme |
| Yeni human estimator | `motion_pipeline/human_state.py` | Kalibre kamera iskeletleri → temiz, eklem-bazlı human state | Latest-valid, en fazla kaynak 30 Hz | Ortak Fusion/world | capture/receive/target; ≤50 ms reject gate ve kısa velocity prediction |
| Pelvis dönüşümü | `motion_pipeline/calibration.py` | World BODY_38 → pelvis-local iskelet | Her kabul edilen frame | `X_FORWARD/Y_LEFT/Z_UP` pelvis frame | Kaynak timestamp |
| Retarget | `isaaclab_bridge/gmr_live_bridge.py`, GMR + 23-DOF direct arm refinement | Temiz human state → G1 23-DOF `q_raw`, `q_safe` | 30 Hz input ayarı, newest-only UDP | Human pelvis → GMR/G1 base | Kaynak timestamp tüm latency zincirinde taşınır |
| Feasibility | `motion_pipeline/safety.py` + `collision_geometry.py` | `q_raw` → limit/velocity/accel/collision kontrollü `q_safe` | Her GMR çözümü | G1 official 23-DOF order | Solve/apply timestamp |
| Isaac | `isaac_g1_23dof_live.py` | 30 Hz `q_safe` → cubic-Hermite 200 Hz `q_des` → PhysX | Physics 200 Hz (`dt=0.005`) | G1 base/world | receive/apply/observed timestamps |

Eski kayıtta iki kameranın timestamp farkı p50 1.28 ms, p90 32.11 ms idi; FPS farkı yoktu. Asıl hata, global MPJPE eşiğinin bilek/parmak aykırı değerleri yüzünden bütün iskeleti reddetmesiydi: 78.6 saniyede 107 mode geçişi oluştu.

## Refactor sonrası veri akışı

```text
Cam1 BODY_38 ─┐
              ├─ capture-time synchronization + short prediction
Cam2 BODY_38 ─┘
                         ↓
             locked-body association (camera-local IDs)
                         ↓
          per-joint confidence/view/bone/temporal quality
                         ↓
       outlier-gated weighted joint fusion (no skeleton average)
                         ↓
       bone constraint + L/R guard + elbow-plane continuity
                         ↓
         clean human state + per-segment confidence/age/source
                         ↓
       pelvis-local direction-based GMR / official G1 23-DOF
                         ↓
       joint/velocity/acceleration/collision feasibility filter
                         ↓
          30→200 Hz cubic-Hermite latest-reference resampler
                         ↓
                     Isaac PhysX
```

Tüm eşikler `config/dual_teleoperation.json` içindedir. Bir core calibration hatası varsa iki frame sessizce birleştirilmez ve sabit fallback kamera kullanılır. Tek bir distal eklem aykırıysa yalnız o eklem en güvenilir kameradan veya kısa temporal prediction'dan gelir.

## Yeni kayıt için offline regresyon

```powershell
& .\.venv-zed\Scripts\python.exe .\tools\replay_dual_fusion.py `
  .\recordings\zed_dual_body38_20260819_122000.jsonl
```

Bu kayıtta sonuç:

- Eski: 649 fusion / 540 single, 107 mode geçişi.
- Yeni: 1162 joint-fusion / 27 calibration fallback, 13 mode geçişi.
- Frame-to-frame eklem displacement p95: 0.132 m → 0.078 m (yaklaşık %41 daha düşük). Aynı ölçüm Camera-1-only için 0.087 m, Camera-2-only için 0.100 m'dir; bu kayıtta joint-fusion ikisinden de daha süreklidir.
- Core camera disagreement p50 0.095 m, p95 0.111 m; bu nedenle global el/parmak hatası core calibration'ı artık yanlış biçimde düşürmüyor.

## Safety ayrımı

Üst-kol/gövde kapsülleri omuz bağlantısı ve arms-down pozunda geometrik olarak doğal biçimde çakıştığı için yalnız soft telemetry'dir. Önkol/el–gövde de cross-body hareketlere izin veren soft telemetry'dir. Kollar arası çarpışma ve el–baş teması hard safety olarak korunur. Düşük fusion güveni bütün gövdeyi durdurmaz; yalnız ilgili kolun GMR katkısını azaltır.

## Rerun tanıları

Canlı paket ve CSV/Rerun artık şunları taşır:

- per-joint `quality`, `source`, `state`, `prediction_age_ms`;
- pelvis/torso/sol kol/sağ kol/bacak segment quality;
- core, pelvis, bilek ve bütün-joint camera disagreement;
- timestamp delta ve fusion failure reason code;
- dirsek state (`VALID`, `NEAR_SINGULAR`, `OCCLUDED`, `RECOVERING`);
- GMR raw/safe q, residual/solve time ve Isaac target/actual tracking ölçümleri.

## Çalışma modları ve kabul sırası

İlk canlı doğrulama `upper_body + fixed_double_support` ile yapılır. Sıra: tek omuz eksenleri, dirsek, iki kol, cross-body, el göğüste, kollar çapraz, waist yaw. Fixed-base/direct replay sayısal olarak geçmeden dynamic whole-body moda geçilmemelidir. `tools/replay_dual_fusion.py` her değişiklikte aynı kaydı yeniden ölçmek için regresyon girişidir.

## 2026-08-19 13:24 kaydı: hareket zamanlaması ve fusion kilidi

Kaynaklar: `recordings/zed_dual_body38_20260819_132651.jsonl` ve
`rerun_recordings/rerun_body38_20260819_132423`.

- 2.155 dual frame'in 2.114'ü joint fusion (%98,1), 41'i kısa single fallback idi.
- İki kamera capture timestamp farkı p50 18,22 ms, p90 18,47 ms ve maksimum 18,68 ms idi. İki kamera FPS'i birbirine yakın olduğundan sorun saat kayması değil, eşik çevresindeki kısa evidence/calibration geçişleridir.
- Isaac joint RMSE p50 0,0134 rad ve body MPJPE p50 0,0097 m: mevcut IK/retargeting doğru çalışmaktadır.
- Buna karşılık uygulanan hedef jerk p50 yaklaşık 946.529 rad/s³ idi. Görülen aşırı hızlı tepkinin ana nedeni budur.

Fusion artık üç temiz dual frame sonunda kilitlenir. Operatör kilitliyken kısa tek-görüş veya kalibrasyon aykırı değerleri bütün modu `single_fallback` durumuna düşürmez; ilgili joint en güvenilir kameradan ya da kısa temporal prediction'dan gelir. Telemetri gerçek durumu `LOCKED_DUAL`, `LOCKED_PARTIALLY_VISIBLE` veya `LOCKED_CALIBRATION_GUARD` olarak ayrıca bildirir. Operatör gerçekten `LOST` olursa 1,5 saniye sonra kilit bırakılır.

13:24 kaydındaki yüksek jerk sorununu çözmek için eklenen eski kritik sönümlü,
üçüncü dereceden governor hedefi fazla geciktirdi. Bu profil
`smooth_bounded` adıyla yalnız tanı/çok yumuşak playback için korunur.

## 2026-08-19 14:28 kaydı: düşük gecikmeli canlı referans

Kaynaklar: `recordings/zed_dual_body38_20260819_142810.jsonl` ve
`rerun_recordings/rerun_body38_20260819_142207`.

- Kamera iskeleti çoğunlukla geçerliydi; iki kameranın normal dual capture farkı
  p50 6,27 ms, alternatif USB fazı ise p50 39,63 ms idi.
- Eski smooth reference profilinde güvenli IK hedefi → Isaac referansı mutlak
  eklem hatası p50 0,223 rad, p90 1,028 rad idi. Isaac actual → uygulanmış
  referans hatası p50 yalnız 0,041 rad olduğundan görünür gecikme IK/PhysX değil,
  son reference governor kaynaklıydı.
- Aynı 648 örnek offline replay edildiğinde yeni `low_latency` profil güvenli
  hedef → referans hatasını p50 0,040 rad ve p90 0,240 rad değerine indirdi.
  Bu yaklaşık sırasıyla %82 ve %77 azalmadır.
- 33 ms reject eşiği donanımın 39,6 ms USB fazındaki geçerli ikinci görünüşü
  reddediyordu. Eşik 50 ms yapıldı; eski frame hedef timestamp'e velocity
  prediction ile taşındığı için stale skeleton körlemesine ortalanmıyor.
  Kalibrasyon/core disagreement ve joint outlier kapıları aynen korunuyor.

Varsayılan Isaac referans yolu GMR'nin 30 Hz güvenli hedefini Hermite ile
yeniden örnekler ve son G1 komutuna Unitree'nin resmi yüksek-seviye G1 örneğini
temel alan fiziksel hız zarfı uygular. Kinematik görünümde zarf gerçek duvar
zamanıyla ilerler; GUI'nin 200 Hz'in altında çizmesi hızı değiştirmez:

- mode: `low_latency`;
- response/catch-up: 5,0 Hz;
- joint velocity: 0,65 rad/s;
- joint acceleration: 2,5 rad/s²;
- joint jerk: 25,0 rad/s³.

Bu katman IK açısını değiştirmez. PowerShell parametreleri
`-ReferenceTrackingMode`, `-ReferenceResponseHz`, `-ReferenceMaxVelocity`,
`-ReferenceMaxAcceleration` ve `-ReferenceMaxJerk` ile
ayarlanabilir. Canlı CSV/Rerun ayrıca `reference_target_error_rms_rad` ve
`reference_target_error_max_rad`, gerçek `reference_step_dt_s` ve limitlerin
kaynağını kaydeder.

Gelecek reference-motion tracking policy için veri sözleşmesi `config/reference_motion_tracking.yaml` dosyasındadır. Canlı telemetri artık `q_ref`, `qd_ref`, `qdd_ref`, segment/reference confidence ve istenen observation bileşenlerini kaydeder. `foot_contact_state` kamera verisinden tahmin edilmez; eğitim ortamında PhysX contact sensor kullanılmalıdır.
