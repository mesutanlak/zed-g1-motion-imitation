# G1 retargeting, Isaac Lab ve çoklu ZED 2i değerlendirmesi

## 1. Son kayıtların kararı

Analiz, kayıt kapatma/açma aralarını ayrı segment kabul eder. Böylece duvar
saati yerine gerçek aktif klip süresi ve FPS ölçülür.

| Kayıt | Aktif süre | Segment | Aktif FPS | Tüm vücut | Dex3 L/R | Karar |
|---|---:|---:|---:|---:|---:|---|
| `134332` | 176,3 s | 3 | 29,86 | %90,8 | %84,7 / %87,5 | Ana başlangıç |
| `124005` | 44,9 s | 1 | 30,00 | %84,9 | %85,3 / %89,7 | Doğrulama |
| `152359` | 15,9 s | 4 | 29,50 | %40,9 | %28,2 / %55,5 | Reddedilecek/tekrar |

`134332` ek özellikleri:

- body confidence `%96,9`;
- önerilen 2–4 m mesafede `%97,7`;
- IMU kapsaması `%100`;
- quaternion büyük sıçrama oranı `%0,048`;
- sol/sağ dirsek hareket aralığı `172,1° / 170,5°`;
- sol/sağ diz hareket aralığı `124,9° / 121,5°`;
- temel uzuv kemik uzunluğu CV değerleri çoğunlukla `%1–3`;
- sol/sağ foot-support görünürlüğü `%70,2 / %80,2`.

Ayak hedefleri, özellikle sol ayak, gövde ve kollardan daha zayıftır. Ayrı,
yavaş ayak bileği pitch/roll ve topuk-parmak teması kaydı gerekir.

## 2. G1 23-DOF retargeting yeterliliği

Mevcut JSONL'ler şunları içerir:

- BODY_38 ham/filtreli 3B noktalar ve 2B pikseller;
- her nokta için confidence;
- pelvis/root pozisyonu ve global quaternion;
- 38 yerel pozisyon ve yerel quaternion;
- kamera koordinatında ve pelvis-relative iskelet;
- kamera intrinsics/stereo extrinsics;
- görüntü-zamanlı ZED 2i IMU;
- SVO2 tekrar işleme kaynağı.

Bu veri **G1 23 gövde eklemi için çevrimdışı IK referansı oluşturmaya
yeterlidir**, fakat doğrudan 23 motor açısı değildir.

| G1 grubu | İnsan kaynağı | Durum |
|---|---|---|
| Hip pitch/roll/yaw | pelvis, hip, knee, yerel rotasyon | Yeterli |
| Knee | hip–knee–ankle yönü | Güçlü |
| Ankle pitch/roll | ankle, heel, big/small toe | Kullanılabilir; sol ayak güçlendirilmeli |
| Waist yaw | pelvis–spine root/local rotasyon | Yeterli |
| Shoulder 3 eksen | clavicle–shoulder–elbow ve local rotasyon | Güçlü |
| Elbow | shoulder–elbow–wrist | Güçlü |
| Wrist roll | forearm rotasyonu + el temsil noktaları | Orta; düşük ağırlık/regularization |
| Dex3-1, el başına 7 motor | BODY_38 parmak temsil noktaları | Yetersiz |

Eksik olan şey yeni kamera verisinden çok retargeting katmanıdır:

1. İnsan nötr A/T pozundan robot nötr pozuna kalibrasyon.
2. Kamera/world → robot base koordinat dönüşümü.
3. İnsan kemik ölçeğinden G1 link ölçeğine yön/poz hedefleri.
4. G1 23-DOF URDF/USD üzerinde Pinocchio veya Isaac Lab IK.
5. Eklem limitleri, hız/ivme, self-collision ve temporal continuity.
6. Ayak temas hedefleri ve denge/locomotion policy'si.
7. Dex3 için SVO2 üzerinde 21-landmark el modeli ve ayrı dex-retargeting.

## 3. Isaac Lab kararı

Bilgisayar yeterlidir:

- RTX 5090, 32 GB VRAM;
- Intel Core Ultra 9 285K;
- yaklaşık 64 GB RAM;
- WSL GPU erişimi çalışıyor.

Isaac Lab, paralel RL, domain randomization, çoklu sensör ve karmaşık sahne
üretimi için MuJoCo'dan daha uygundur. Ancak mevcut Unitree resmî destek
durumu önemlidir:

- `unitree_rl_lab` şu anda doğrudan `G1-29dof` görevlerini listeliyor;
- `unitree_sim_isaaclab` G1-29dof + Dex3 sahneleri sağlıyor;
- bizim robot `G1-23dof`, dolayısıyla asset, actuator/joint mapping, observation
  ve action boyutları için 23-DOF görev türetilmesi gerekir;
- `unitree_rl_mjlab` ise `Unitree-G1-23Dof-Tracking-No-State-Estimation`
  görevini doğrudan destekliyor.

Bu nedenle MuJoCo'yu bırakmak doğru değildir. Önerilen hibrit akış:

```text
ZED/Fusion → offline retargeting → q_ref(23)
                 │
                 ├─ Isaac Lab: motion tracking eğitimi, domain randomization
                 └─ MuJoCo: resmî 23-DOF sim2sim güvenlik doğrulaması
                                      ↓
                                fiziksel G1
```

Unitree'nin kendi `unitree_rl_lab` deposu da eğitimden sonra MuJoCo sim2sim ve
ondan sonra sim2real önerir. RTX 50 serisi için Unitree simülasyon deposundaki
Isaac Sim 5.0 notu esas alınmalı; sürümler rastgele karıştırılmamalıdır.

## 4. On iki ZED 2i ile çoklu kamera

Stereolabs Fusion API çoklu body tracking için doğru çözümdür. Her kamera
BODY_38 yayıncısı olur; Fusion subscriber aynı kişiyi ortak WORLD
koordinatlarında birleştirir. Kamera extrinsics değerleri ZED360 ile
kalibre edilir.

### Tek bilgisayar sınırı

ZED 2i USB 3 cihazıdır. USB kameralar donanımsal trigger sunmaz; görüntü
timestamp'leriyle yazılımsal eşleştirilir. Resmî dokümanda tek ZED'in
1080p30 ham trafiğinin yaklaşık `250 MB/s`, USB 3.0 pratik üst sınırının
yaklaşık `620 MB/s` olduğu belirtilir. Powered hub güç sağlar ama bant
genişliğini artırmaz.

On iki kamerayı tek anakart USB denetleyicisine bağlamak uygun değildir.
Gerekirse:

- bağımsız controller'lı PCIe USB 3.x kartları;
- her controller'a en fazla 1–2 yüksek çözünürlüklü kamera;
- aktif ve kaliteli USB 3 kabloları;
- seri numarasına göre sabit kamera kimliği;
- Windows Device Manager'da kameraların farklı USB root controller'lara
  dağıldığının doğrulanması

gerekir. Buna rağmen 12 adet `HUMAN_BODY_ACCURATE + NEURAL depth` örneğini tek
GPU'da gerçek zamanlı çalıştırmak risklidir.

### Önerilen kurulum

İlk prototipte 12 yerine 4 kamera:

- odanın dört köşesi;
- yaklaşık 1,6–2,0 m yükseklik;
- merkeze 10–20° aşağı bakış;
- kişinin çevresinde geniş ortak görüş alanı;
- doğrudan güçlü ışık ve karşılıklı lens parlamasından kaçınma.

Sonra 6–12 kameraya ölçeklemek gerekirse:

- 3–4 capture bilgisayarı, her birinde 3–4 kamera;
- her capture bilgisayarında BODY_38 publisher;
- kablolu ağ ve PTP ile host saat senkronizasyonu;
- merkezi Fusion bilgisayarı;
- her publisher için benzersiz port;
- ZED360 ile ortak WORLD extrinsic kalibrasyonu.

USB ZED 2i'de gerçek donanımsal frame trigger olmadığı için hızlı el/parmak
hareketlerinde tam kare eşzamanlılığı garanti edilemez. Vücut taklidi için
timestamp/Fusion yaklaşımı uygulanabilir; hassas parmak çalışmasında daha
yüksek hız, kısa pozlama ve ek el kameraları değerlendirilmelidir.

## 5. Fusion'dan G1'e veri akışı

```text
ZED publishers (4→12)
    ↓ BODY_38 + timestamp + confidence
Stereolabs Fusion
    ↓ fused body ID + fused 3D skeleton in WORLD
Quality gate
    ↓ confidence, temporal continuity, limb consistency
Human→G1 retargeter
    ↓ q_ref[23], qd_ref[23], foot contacts
Isaac Lab playback/training
    ↓ policy
MuJoCo sim2sim safety gate
    ↓ only after approval
Physical G1 controller
```

Dex3 yolu ayrıdır:

```text
calibrated camera images → per-view 21 hand landmarks
→ multi-view association/triangulation → dex-retargeting → 14 Dex3 targets
```

BODY_38 Fusion, görüş kaybını ve gövde pozunu iyileştirir; tek başına parmak
falanks açılarını oluşturmaz.

## 6. Uygulama planı

1. `134332` segmentlerini hareket etiketlerine ayır.
2. JSONL → G1 23-DOF `q_ref` çevrimdışı retargeter yaz.
3. `q_ref` dosyasını RViz ve mevcut MuJoCo'da kinematik oynat.
4. Dört ZED 2i ile ZED360 kalibrasyonu ve Fusion örneğini doğrula.
5. Tek-kamera ve dört-kamera iskelet hatasını aynı hareketlerde karşılaştır.
6. G1-23 asset'ini resmî Unitree modelinden Isaac Lab'e aktar.
7. `unitree_rl_lab` motion tracking görevini 23 action'a türet.
8. Isaac Lab policy'sini MuJoCo sim2sim'de sınırla/doğrula.
9. Dex3 multi-view el retargeting'i ayrı modül olarak ekle.
10. Fiziksel G1 yalnızca tüm güvenlik kapıları geçince etkinleştir.

## Resmî referanslar

- <https://github.com/unitreerobotics/unitree_rl_lab>
- <https://github.com/unitreerobotics/unitree_sim_isaaclab>
- <https://github.com/unitreerobotics/unitree_rl_mjlab>
- <https://isaac-sim.github.io/IsaacLab/>
- <https://docs.stereolabs.com/docs/development/zed-sdk/modules/fusion>
- <https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/multi-camera>
- <https://www.stereolabs.com/docs/fusion/zed360>
- <https://github.com/stereolabs/zed-examples/tree/master/body%20tracking/multi-camera>
