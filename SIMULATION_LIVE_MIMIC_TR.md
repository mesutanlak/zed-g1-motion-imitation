# Tek ZED 2i → G1-23DOF canlı MuJoCo ve RViz taklidi

## Güncel mimari

- ZED 2i, Windows 11 üzerinde Stereolabs BODY_38 iskeleti üretir.
- Kompakt paket UDP ile WSL Ubuntu 22.04'e gider.
- WSL, Unitree'nin resmi G1-23DOF MuJoCo modelini yükler.
- Retargeting ayrı bir IK durumunda yapılır; fizik durumuna doğrudan `qpos`
  yazılmaz.
- MuJoCo serbest taban, yerçekimi, çarpışma ve zemin temasını hesaplar.
- Eklem hedefleri resmi Unitree `FixStand` PD kazançlarıyla, modeldeki gerçek
  tork sınırları içinde uygulanır.
- Fizik motorundaki aynı pozlar ROS 2 ile RViz'e yayınlanır.
- Fiziksel robot DDS/motor komutu yoktur.

## Neden artık havada kalmaz?

Önceki kinematik sürüm her karede `qpos` yazıp yalnızca `mj_forward` çağırdığı
için yerçekimi ve temas çözücüsü çalışmıyordu. Varsayılan `physics` modu artık
yalnızca kuvvet/tork uygular ve `mj_step` ile zamanı ilerletir.

Güncel yerinde tam-vücut aşamasında:

- kollar, waist yaw, kalçalar, dizler ve ayak bileği pitch eksenleri canlı
  insandan taklit edilir;
- `grounded` modu düşük insan ayağını destek ayağı olarak zeminde tutar ve
  yalnız yükselen ayağı salınım ayağı yapar;
- pelviste açıkça modellenmiş yumuşak bir güvenlik askısı vardır;
- askı ağırlığın varsayılan `%55` kısmını taşır, ayaklar yine zemine basar;
- askı yatay kaçışı ve aşırı gövde devrilmesini yay/sönümleyici kuvvetlerle
  sınırlar; tabanı ışınlamaz veya sabitlemez;
- düşme eşiği aşılırsa yalnızca simülasyon güvenlik sıfırlaması yapılır.

`grounded` varsayılan simülasyon seçeneğidir. Serbest `--lower-body mimic`
iki ayağın temasını korumaz ve yalnız araştırma/hata ayıklama içindir. Fiziksel
G1 üzerinde bacak taklidi için hâlâ eğitimli denge/tracking policy gerekir.

## Canlı MuJoCo

PowerShell terminal 1:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_mujoco_mimic.ps1
```

PowerShell terminal 2:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_zed_live_smooth_to_wsl.ps1
```

## Engeller + MuJoCo fizik + RViz + sensörler

Önce eski klavye simülasyonu açıksa kapatın:

```bash
~/robot_tools/stop_g1_keyboard_sim.sh
```

WSL terminal 1:

```bash
~/robot_tools/start_g1_zed_obstacle_sim.sh
```

WSL terminal 2:

```bash
~/robot_tools/view_g1_sensors_rviz.sh
```

Windows tarafında ZED canlı yayını:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_zed_live_to_wsl.ps1
```

Bu mod RViz'de şunları birlikte gösterir:

- `/g1/robot_mesh`: MuJoCo'daki gerçek fizik pozundan G1 STL meshleri;
- `/joint_states`: 29 model eklemi; 23-DOF dışındaki altı eklem resmi modelde
  kilitlidir;
- `/livox/lidar`: simüle Mid-360 nokta bulutu;
- `/camera/camera/depth/image_rect_raw`: simüle D435i depth görüntüsü;
- `/camera/camera/depth/color/points`: depth nokta bulutu;
- `/tf`: `odom`, torso, lidar ve kamera çerçeveleri.

Canlı taklit gecikmesini azaltmak için robot meshleri, `/joint_states` ve TF
20 Hz; daha pahalı MuJoCo LiDAR/depth ray-cast sensörleri 5 Hz yayınlanır.
Terminalde `input=LIVE`, anlık `rx`/`accepted` hızları ve veri yaşı gösterilir.
`input=NO_BODY`, kamera/UDP çalışırken geçerli insan bulunmadığını gösterir.
Kamera veya ağ tamamen kesilirse `input=STALE` görünür. Her iki durumda da
robot güvenli FixStand hedefine döner.

Topic kontrolü:

```bash
~/robot_tools/inspect_g1_ros_topics.sh
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
ros2 topic hz /livox/lidar
ros2 topic info /camera/camera/depth/color/points -v
```

Durdurma:

```bash
~/robot_tools/stop_g1_zed_obstacle_sim.sh
```

Eski `start_g1_obstacle_sim.sh` klavye + yürüyüş policy hattıdır. ZED mimic ile
aynı anda açılmaz; iki ayrı MuJoCo süreci aynı robot/sensör topiclerini
yayınlamamalıdır.

## Kayıttan fizik testi

```powershell
wsl -d Ubuntu-22.04 -- bash `
  /mnt/c/Users/Misafir/Desktop/ZED_G1_Projesi/sim_mujoco/run_g1_zed_mimic.sh `
  --replay-jsonl /mnt/c/Users/Misafir/Desktop/ZED_G1_Projesi/recordings/zed_body38_20260727_134332.jsonl `
  --replay-fps 30 --mode physics --lower-body grounded
```

`--mode kinematic` yalnızca IK hata ayıklaması içindir ve fizik doğrulaması
sayılmaz.

## Doğrulama sonucu

- Boş akışta iki ayak da sürekli zemin temasında, pelvis yaklaşık `0,779 m`.
- 28 Temmuz kaydının belirgin bacak hareketli 500 karelik bölümünde 488 kare
  kabul edildi (`%97,6`).
- Ortalama IK uç-nokta hatası `0,0361 m`, p95 `0,0539 m`.
- Eklem limit olayı `0`, düşme/reset `0`.
- Fizik sonucunda sağ diz `1,065 rad`, sağ kalça pitch `0,748 rad`; kol
  shoulder-roll eksenleri yaklaşık `0,77–0,85 rad` aralıkta hareket etti.
- Destek teması test boyunca solda `%99,2`, sağda `%95,5` korundu.
- ROS 2'de mesh, TF, joint states, LiDAR ve depth topicleri canlı doğrulandı.

### 28 Temmuz 17:47 tek kamera deneyi

- 2087 geçerli BODY_38 karesi, aktif bölümlerde `29,97 Hz`.
- Temel iki kol ve iki bacak için gereken 12 nokta karelerin `%84,8`'inde
  aynı anda güven eşiğinin üzerinde.
- Dirsek hareket aralığı solda `77,35°`, sağda `144,23°`; diz hareket aralığı
  solda `124,13°`, sağda `115,48°`.
- Kısa örtülmeler için son geçerli uzuv yönü `0,18 s` korunur.
- Retargeting diz ve dirsek hiper-ekstansiyonunu kullanmaz.
- Salınım ayağı eşiği düşürüldü; salınım hacmi ileri/yan yönde `0,32 m`,
  düşey yönde `0,32 m` ile sınırlandı. Destek ayağı zemin hedefini korur.
- İlk 600 kare fizik testinde ortalama IK hatası `0,0412 m` değerinden
  `0,0263 m` değerine, p95 hata `0,0559 m` değerinden `0,0371 m` değerine
  düştü. Eklem limit olayı ve fizik reseti görülmedi.

İnsan omuz hareket alanı G1-23DOF omuz hareket alanından daha geniştir. Tam
başüstü kol pozlarında çözücü resmî modelin omuz sınırına ulaşır; bunu aşmak
fiziksel robotta mümkün değildir. G1-29DOF/GMR sanal referansı daha fazla waist
ve wrist ekseni kullanabilir, fakat 23-DOF fiziksel hedefe projeksiyonda bu altı
eksen yine kilitlenmelidir.

## Resmi kaynaklar

- <https://github.com/unitreerobotics/unitree_mujoco>
- <https://github.com/unitreerobotics/unitree_rl_mjlab>
- <https://github.com/unitreerobotics/unitree_rl_lab>
- <https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api>
