# ZED 2i → G1 23-DOF + Dex3-1 mimic mimarisi

Bu proje algılama, retargeting, simülasyon ve fiziksel robot katmanlarını ayrı
tutar. Mevcut aşama yalnızca **algılama ve çevrimdışı analizdir**.

## Kontrollü eksenler

- G1 gövde: 23 eklem
- Sol Dex3-1: 7 motor
- Sağ Dex3-1: 7 motor
- Toplam hedef vektörü: 37 eksen

BODY_38, gövde retargeting'i için iyi bir başlangıçtır; fakat her elde yalnızca
bilek ve parmak temsil noktaları sağlar. Dex3-1'in yedi motor açısını benzersiz
olarak çözmek için yeterli değildir. JSONL ile birlikte SVO2 saklanmalı ve daha
sonra her el için 21-landmark el takipçisi çalıştırılmalıdır.

## Veri akışı

```text
ZED 2i
  ├─ BODY_38: 3B keypoint + confidence + local quaternion
  ├─ IMU: orientation + angular velocity + acceleration
  └─ SVO2: stereo görüntü + depth için tekrar işlenebilir kaynak
          │
          ▼
Çevrimdışı kalite kapısı
  ├─ görünürlük/confidence
  ├─ kare aralığı ve takip sürekliliği
  ├─ kemik uzunluğu tutarlılığı
  └─ quaternion normu ve açısal sıçrama
          │
          ▼
Retargeting
  ├─ G1 23-DOF: ölçekten bağımsız yönler + Pinocchio/IK
  ├─ Dex3-1: 21-landmark el modeli + dex-retargeting
  └─ eklem/ hız / ivme / self-collision / denge sınırları
          │
          ▼
Önce RViz → sonra MuJoCo → en son fiziksel G1
```

### Opsiyonel 4×ZED el katmanı

El özelliği varsayılan kapalıdır ve mevcut BODY_38 kontrol datagramını
değiştirmez. Her `zed_g1_skeleton.py` kaynağı yalnız LOCKED operatörün bilek
ROI'lerinde 21-landmark çıkarıp ayrı `zed_operator_hand/v1` UDP kanalına yollar.
`distributed_body38_fusion.py` saat-ofseti düzeltilmiş capture zamanında depth ve
ışın adaylarını ortak BODY_38 fusion dünyasında birleştirir; palm-normalize şekli
Dex3 analiz/simülasyon adaptörüne verir. Fiziksel DDS publisher yoktur. Kurulum,
replay ve kabul kapıları `docs/DEX3_HAND_TRACKING_RUNBOOK_TR.md` içindedir.

## ROS 2 sözleşmesi (sonraki aşama)

Algılama düğümü:

- `/mimic/human/body38`: zaman damgalı 38 nokta, confidence ve quaternion
- `/mimic/human/imu`: ZED 2i IMU
- `/mimic/quality`: kabul/ret ve grup görünürlükleri

Retargeting düğümü:

- giriş: yukarıdaki algılama konuları
- çıkış: `/mimic/g1/joint_targets` içinde 23+14 konum hedefi
- confidence düşerse yeni hedef üretmez; güvenli nötr hedefe yumuşak geçiş ister

RViz adaptörü yalnızca `sensor_msgs/JointState` ve TF yayınlar. MuJoCo adaptörü
aynı hedef vektörünü simülasyon modeline uygular. Fiziksel Unitree DDS adaptörü
ayrı paket olur ve simülasyon doğrulanmadan etkinleştirilmez.

## Kayıt protokolü

1. İnsan 2,5–4 m uzakta, tüm el ve ayaklar kadrajda olmalı.
2. İlk 5 saniye A/T pozu ve hareketsiz duruş kaydedilmeli.
3. Her hareket 30–60 saniye, yavaş başlayıp normal hıza çıkmalı.
4. Önden kayda ek olarak sağ/sol yönelim ve self-occlusion testleri yapılmalı.
5. Her oturum JSONL + SVO2 olarak kaydedilmeli.
6. `partial_or_retake` kayıtlar retargeting eğitimine alınmamalı.

## Fiziksel robot güvenlik kapısı

Fiziksel G1'e doğrudan kamera keypoint'i gönderilmez. Eklem konum, hız ve ivme
sınırları; self-collision; destek poligonu/denge; watchdog; dead-man ve operatör
acil durdurma birlikte bulunmalıdır. İlk fiziksel test askıda ve düşük hızda
yapılmalıdır.
