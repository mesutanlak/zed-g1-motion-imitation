# Resmi Unitree Dex3 + BODY_38 çalıştırma ve doğrulama

Bu yol mevcut BODY_38 → GMR → G1 23-eklem IK’sini değiştirmez. G1-29DOF
asset’inde 23 mevcut hedef aynı adlı eklemlere yazılır; `waist_roll`,
`waist_pitch` ve iki bilekteki pitch/yaw eklemleri resmi asset nötründe kalır.
Yalnız sol/sağ yedi Dex3 eklemi ayrıca sürülür.

Fiziksel güvenlik kuralı sabittir: bu entegrasyon Unitree SDK/DDS import etmez,
DDS topic’i açmaz ve her el sözleşmesinde
`physical_robot_output_enabled=false` zorunludur.

## Resmi ve sabitlenmiş kaynaklar

- Unitree Isaac Lab: <https://github.com/unitreerobotics/unitree_sim_isaaclab>
- Unitree G1-29 + Dex3 config: <https://github.com/unitreerobotics/unitree_sim_isaaclab/blob/main/robots/unitree.py>
- Unitree XR teleoperation: <https://github.com/unitreerobotics/xr_teleoperate>
- Kullanılan commit’ler: `config/unitree_official_sources.lock.json`

Repository’lerin kodu değiştirilmeden kullanılır. Isaac asset/config doğrudan
`G129_CFG_WITH_DEX3_BASE_FIX` üzerinden yüklenir. Resmi simülatörün DDS action
provider’ı kullanılmaz; hedefler yalnız mevcut Isaac sürecindeki articulation’a
yazılır.

## Bir defalık kurulum — ana PC

```powershell
powershell -ExecutionPolicy Bypass -File .\install\install_unitree_dex3_sim.ps1 -FetchAssets
```

Resmi asset paketi 1 GB’den büyüktür. Script pinned Unitree repository’lerini
`C:\g1il\repos` altına kurar, `xr_teleoperate` submodule’lerini alır ve resmi
`fetch_assets.sh` ile USD paketini açar. Resmi DexPilot’ın NumPy/Torch
sürümlerini ZED/MediaPipe ortamından ayırmak için `C:\g1il\envs\dex3` oluşturur.
Fiziksel DDS kurulumu/başlatması yapmaz.

Kurulumdan sonra kaynak/commit/asset/joint sözleşmesini Isaac açmadan doğrulayın:

```powershell
.\.venv-zed\Scripts\python.exe .\tools\verify_unitree_dex3_install.py
```

MediaPipe modeli iki bilgisayarda şu konumda bulunmalıdır:

```text
C:\ZED_G1\models\hand_landmarker.task
```

Model dosyası otomatik indirilmez.

## Çalıştırma sırası

Ana PC — Isaac/GMR (resmi Dex3 asset, DDS kapalı):

```powershell
.\start_g1_isaaclab_live.ps1 -Mode upper_body -ImitationMode kinematic_debug -Dex3 -AcceptNvidiaEula
```

Ana PC — dört kamera receiver + Rerun/ROS + araştırma kaydı:

```powershell
.\start_zed_four_fusion_to_wsl.ps1 -HandTracking -Record
```

Laptop:

```powershell
.\start_zed_four_sources.ps1 -Role Laptop -MainPcHost 192.168.50.10 -HandTracking -HandModel "C:\ZED_G1\models\hand_landmarker.task" -RecordSvo2
```

Ana PC — kameralar:

```powershell
.\start_zed_four_sources.ps1 -Role MainPc -HandTracking -HandModel "C:\ZED_G1\models\hand_landmarker.task" -RecordSvo2
```

Varsayılan el profili üç metre için 160 px minimum ROI, kamera başına 8 Hz
asenkron çıkarım, tek ROI’de tek el ve 0,30 güven eşiğidir. BODY_38 kamera
capture döngüsü MediaPipe’ı beklemez. Tam RGB veya tam depth görüntüsü el
worker’ına/ağa taşınmaz.

Paylaşımlı GPU profili Isaac fiziğini 200 Hz hedefinde bırakır, yalnız GUI
render’ını 25 Hz’e (`RenderInterval=8`) düşürür. Rerun kaynak ve GMR kayıt
örneklemesi varsayılan 10 Hz’dir; kontrol kanallarının hızını sınırlamaz.

## Kontrol ve kayıt sözleşmesi

`15050` BODY_38 kontrol paketinde yalnız küçük alan bulunur:

```text
dex3_control/v1 = timestamp + q_left[7] + q_right[7]
                  + confidence/watchdog + physical=false
```

GMR alanı yorumlamadan Isaac’e geçirir. Isaac aynı paketteki 23 gövde hedefini
eski yolla uygular, 14 eli resmi Dex3 joint adlarına yazar. Paket kesilirse
eller 0,20 saniye hold, ardından 0,55 saniyede nötre fade eder.

Fusion kaydı varsayılan `research` ayrıntı düzeyindedir: fused BODY_38,
fused 21-el noktaları, kalite/residual/timing ölçümleri ve kamera başına
detector özeti saklanır; aynı ham 21×2/21×3 dizileri her body karesinde dört kez
tekrarlanmaz. Sorun ayıklamak için receiver’a `-RecordDetail full` verilebilir.

## Kayıt testi

```powershell
.\.venv-zed\Scripts\python.exe .\tools\benchmark_hand_tracking.py `
  .\recordings\four_body38_fusion_YYYYMMDD_HHMMSS.jsonl `
  --output .\recordings\hand_benchmark_latest.json
```

Eski tam kaydı güncel geometri eşikleriyle yeniden birleştirmek için:

```powershell
.\.venv-zed\Scripts\python.exe .\tools\refuse_hand_recording.py `
  .\recordings\four_body38_fusion_YYYYMMDD_HHMMSS.jsonl `
  .\recordings\four_body38_fusion_refused_v2.jsonl
```

Benchmark v2 sol/sağ ve iki-el kapsamasını, en uzun takip/kayıp süresini,
kamera-detector başarısını, depth completeness’i, 0/1/2/3/4 kamera landmark
histogramını, yalnız çoklu-görüş capture spread’ini ve joint-limit/hız/ivme
sınır olaylarını ayrı raporlar. ID-switch yalnız `annotated_identity` içeren
etiketlenmiş replay’de sayılır; etiketsiz kayıtta sayı uydurulmaz.

## Kabul kontrolü

Başlangıçta şu satırlar görülmelidir:

```text
Using official unitreerobotics G1-29DOF + Dex3 USD (DDS disabled)
Dex3 local articulation controller ready: 14 joints, no DDS
```

Isaac telemetrisinde `asset_profile=g1_29dof_dex3`,
`dex3_dds_enabled=false` ve iki ayrı Dex3 watchdog bulunmalıdır. El kaybolurken
gövde hareketinin sürmesi beklenir; BODY_38/IK regresyonu sayılır ve test hatası
kabul edilmez.
