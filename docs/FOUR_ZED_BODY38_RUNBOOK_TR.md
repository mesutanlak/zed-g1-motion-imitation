# 4 ZED 2i BODY_38 + Isaac + Rerun calistirma kaydi

Bu yol iki Windows bilgisayarinda dört ZED 2i kullanir. ZED360'in ag uzerinde
verdigi `WRONG BODY FORMAT` hatasina bagli degildir. Her ZED kendi hostunda
BODY_38 ve dusuk hizli JPEG onizleme yayinlar; ana PC extrinsic kalibrasyonu
uygulayip tek BODY_38 akisi uretir.

Sabit esleme:

| Host | ZED seri | BODY UDP | Onizleme UDP |
|---|---:|---:|---:|
| Laptop `192.168.50.11` | 39504762 | 16000 | 16100 |
| Laptop `192.168.50.11` | 34760587 | 16006 | 16106 |
| Ana PC `192.168.50.10` | 31571870 | 16002 | 16102 |
| Ana PC `192.168.50.10` | 33773329 | 16004 | 16104 |

## Gunluk calistirma

ZED Explorer, ZED360 ve eski publisher sureclerini kapat. Iki bilgisayarda da
proje klasorunde guncel kodu al:

```powershell
git pull --ff-only origin main
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
```

Laptopta iki kaynak penceresini otomatik ac:

```powershell
cd "C:\Users\MSI\Desktop\zed-g1-motion-imitation"
.\start_zed_four_sources.ps1 -Role Laptop -MainPcHost 192.168.50.10
```

Ana PC'de iki yerel kaynak penceresini otomatik ac:

```powershell
cd "C:\Users\mesut\OneDrive\Masaüstü\ZED_G1\zed-g1-motion-imitation"
.\start_zed_four_sources.ps1 -Role MainPc
```

Ana PC'de ayri bir PowerShell'de Rerun'i ac. Bu surec canli analizi gosterir ve
her oturumu otomatik olarak `rerun_recordings` altina `.rrd`, `.jsonl`, `.csv`
ve oturum ozeti olarak yazar:

```powershell
.\start_g1_rerun.ps1 -Mode live -ListenPort 15052 -LiveMaxHz 15
```

Ana PC'de ayri bir PowerShell'de Isaac/GMR'i ac:

```powershell
.\start_g1_isaaclab_live.ps1 -Mode upper_body -ImitationMode kinematic_debug -AcceptNvidiaEula -InputFps 15
```

Son olarak ana PC'de 4-ZED fusion ve 2x2 arayuzu ac:

```powershell
.\start_zed_four_fusion_to_wsl.ps1 -Record -MinimumSources 3 -FusionHz 15 -PreviewHz 10
```

`-MinimumSources 3`, tek gorusun anlik BODY kaybinda Isaac akisini kesmez;
ekranda ve kayitta her karenin gercek katkisi `fusion_katki=3/4` veya `4/4`
olarak kalir. Yalniz dort goruslu kare uretmek icin `-MinimumSources 4` kullan.

Fusion penceresi tuslari: `S` JSONL kaydini acip kapatir; `Q`/`Esc` guvenli
cikis yapar. Kaynak penceresindeki `R` yalniz o kameranin operator kilidini ve
yerel notr kalibrasyonunu sifirlar. Olculen kayit sirasinda gereksiz `R`
kullanma.

## Kayitlar ve olculecek alanlar

- `recordings/four_body38_fusion_YYYYMMDD_HHMMSS.jsonl`: tam fusion paketleri,
  dört kameranin fusion-world BODY_38 noktaları, confidence, kamera pozlari,
  senkron yayilimi, cross-view MPJPE/p95, kaynak/fusion FPS ve kuyruk kaybi.
- `rerun_recordings/`: Rerun `.rrd`, ham analiz JSONL/CSV ve oturum ozeti.
- Isaac/GMR `15050`: kontrol icin oncelikli ve kompakt fused BODY_38.
- Rerun `15052`: dört kamera ham iskeleti dahil tam analiz paketi.
- WSL/ROS `15054`: kompakt ek kopya.

Saglikli bir kayitta `bagli=4/4`, `gecersiz=0`, `drop=0`, fusion FPS yaklasik
14-15 ve mumkun oldugunca cok `fusion_katki=4/4` beklenir. `cross_view_mpjpe_m`
dusuk olmalidir; 0.10 m uzeri kalibrasyon/ortak gorus kontrolu gerektirir.

## Tripod hareket ederse yeniden kalibrasyon

Kaynaklar acik, Isaac/Rerun/fusion alicisi kapali olsun. Ana PC'de 25-30 saniye
ham kalibrasyon kaydi al:

```powershell
.\zed_four_camera_test\start_distributed_receiver.ps1 `
  -Source "39504762:16000","31571870:16002","33773329:16004","34760587:16006" `
  -CalibrationRecord ".\recordings\four_body38_static_calibration_NEW.jsonl" `
  -Fps 15 -MinimumSources 4
```

Ortak gorus alaninda tek kisi, ayaklar sabit ve kollar acik T-poza yakin dursun.
`ham_kayit` en az 150-240 olunca `Ctrl+C` yap. Sonra extrinsic ve world-pose
dosyalarini uretip dogrudan aktif et:

```powershell
.\zed_four_camera_test\start_distributed_calibration.ps1 `
  -CapturePath ".\recordings\four_body38_static_calibration_NEW.jsonl" `
  -OutputPath ".\config\zed_four\distributed_body38_extrinsics_NEW.json" `
  -WorldPosesJsonl ".\config\zed_four\four_camera_world_poses_NEW.jsonl" `
  -ReferenceSerial 33773329 -Activate
```

Aktif kopyalar:

```text
config/zed_four/active_distributed_body38_extrinsics.json
config/zed_four/active_four_camera_world_poses.jsonl
```

`start_zed_four_fusion_to_wsl.ps1` sonraki gun bu aktif dosyayi otomatik okur,
dört seri numarasini ve semayi dogrular. Kameralar/tripodlar hareket etmediyse
yeniden kalibrasyon gerekmez.

Kalibrasyonu yalniz dogrulamak icin:

```powershell
.\start_zed_four_fusion_to_wsl.ps1 -ValidateCalibrationOnly
```

## Kapatma sirasi

Once fusion penceresinde `Q`, sonra Rerun ve Isaac'te `Ctrl+C`, en son dört
kaynak penceresinde `Ctrl+C`. Boylece JSONL ve Rerun oturum dosyalari kapanir.
