# ZED 2i → GMR → Unitree G1 EDU 23-DOF → Isaac Lab

## Kurulan ayrık yapı

- Windows kamera/proje: `C:\Users\Misafir\Desktop\ZED_G1_Projesi`
- Yerel Isaac ortamı: `C:\g1il\env` (Python 3.11)
- Isaac Sim: 5.0.0
- Isaac Lab: v2.2
- Unitree model ve görevleri: resmî `unitree_rl_lab`, `unitree_ros`,
  `unitree_sim_isaaclab`
- WSL GMR ortamı: `/home/misafir/g1_isaaclab_project/envs/gmr_zed`
- Retargeting: upstream `YanjieZe/GMR`

Isaac Sim WSL2 içinde çalıştırılmaz. ZED Windows'tan WSL/GMR köprüsüne,
GMR de Windows'taki Isaac Lab'e UDP yollar. MuJoCo çalışma alanı bu kurulumdan
ayrıdır. Bu zincir Unitree DDS açmaz ve fiziksel robota komut gönderemez.

## Sürücü durumu

RTX 5090 ve CUDA testi geçti. Bilgisayardaki NVIDIA `610.62` sürücüsü, Isaac
Sim 5.x için doğrulanmamıştır:

- Vulkan başlangıçta `rtx.scenedb` içinde çöküyor.
- D3D12 ekransız modda çalışıyor.
- D3D12 GUI testinde `GPU device lost` oluşabiliyor.

GUI için RTX 5090'ı destekleyen NVIDIA `581.42 WHQL` sürücüsünü temiz kurup
Windows'u yeniden başlatın:

<https://www.nvidia.com/en-us/drivers/details/257264/>

Sürücü değiştirilene kadar ekransız uçtan uca test:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 `
  -Mode upper_body -AcceptNvidiaEula -Headless
```

## GUI ile canlı kullanım

Üç süreç ayrı tutulur. Önce Isaac Lab + WSL/GMR köprüsünü birinci
PowerShell'de başlatın:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 `
  -Mode upper_body -AcceptNvidiaEula
```

İlk D3D12/RTX GUI açılışı sırasında Windows yaklaşık 3–4 dakika
`Yanıt Vermiyor` gösterebilir. Terminalde `G1 scene initialization complete`
satırı görülene kadar pencereyi kapatmayın. Sonraki açılışlarda önceden
dönüştürülmüş `C:\g1il\cache\g1_23dof\g1_23dof_rev_1_0.usd` kullanılır.

Canlı 3B BODY_38 ön-görünüm panelini ikinci PowerShell'de başlatın:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_g1_skeleton_panel_wsl.ps1
```

ZED yakalama ve UDP yayınını üçüncü PowerShell'de başlatın:

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
powershell -ExecutionPolicy Bypass -File .\start_zed_live_smooth_to_wsl.ps1
```

Canlı profil varsayılan olarak kayıt yapmaz; böylece gecikme ve görüntü
takılması azalır. JSON kaydı için `-Record`, JSON+SVO2 için `-RecordSvo2`
ekleyin. `-RecordSvo2` kullanıldığında kayıt başlangıçta açıktır; `S` tuşu
kaydı kapatır.

ZED ham BODY_38 verisini WSL'de iki ayrı porta kopyalar:

- `15050`: GMR retargeting köprüsü;
- `15052`: yalnızca 3B analiz paneli.

Bu ayrım analiz panelinin GMR paketlerini tüketmesini veya geciktirmesini
önler.

`upper_body` güvenli başlangıçtır: bacaklar resmî nominal duruşta kalır,
gövde ve kollar GMR hedeflerini izler. `whole_body`, bacak hedeflerini de
uygular; denge takip politikası eğitilmeden robot simülasyonda düşebilir.

## Doğrulanan veri hattı

1. ZED SDK BODY_38 konumları ve güven değerleri 30 Hz alınır.
2. Kısa süreli kayıp noktalar bellekle tamamlanır; kişi pelvis merkezine
   alınır ve ayak zemini `z=0` yapılır.
3. Upstream GMR, G1-29DOF IK çözümünü üretir.
4. Fiziksel G1 EDU'da bulunmayan altı eksen isimle kaldırılır ve resmî
   G1 23-DOF sırası oluşturulur.
5. 7 Hz filtre, eklem hız sınırları ve eski veri zaman aşımı uygulanır.
6. Isaac Lab resmî Unitree URDF'sini, aktüatör tork/hız sınırlarını,
   yerçekimini ve temas fiziğini kullanır.

Kayıt testinde GMR 12 geçerli hedefle 23 eklem paketi üretti. Isaac Lab
ekransız testte GPU PhysX ile 100 adımı ve tam PowerShell/WSL başlatıcı
zincirini tamamladı.
