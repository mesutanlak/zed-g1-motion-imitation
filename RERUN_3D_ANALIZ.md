# Rerun ile ayrı BODY_38 3B analiz sistemi

Bu uygulama mevcut `analysis_panel/` uygulamasını **değiştirmez**. ZED 2i
BODY_38 UDP paketlerini ayrı dinler veya JSONL kaydını oynatır. Yalnızca algı
analizi yapar; Unitree motor komutu üretmez.

## Kurulum

```powershell
cd C:\Users\Misafir\Desktop\ZED_G1_Projesi
python -m pip install -r .\requirements-rerun.txt
```

## Canlı kullanım

Önce Rerun analizini açın:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_rerun.ps1
```

Sonra ayrı PowerShell'de mevcut ZED yayınını açın:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_zed_live_smooth_to_wsl.ps1 `
  -Profile shared_gpu_safe
```

Rerun, mevcut analiz UDP kopyasını `15052` portundan alır. Aynı anda eski
3B paneli de `15052` üzerinde açılamaz; ikisinden birini seçin.

## Kayıt oynatma

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_rerun.ps1 `
  -Mode playback `
  -InputFile .\recordings\zed_body38_20260730_130903.jsonl
```

Oynatma hızı kontrol penceresinden 0.1x–4x değiştirilebilir. Rerun zaman
paneliyle duraklatma, kare kare ilerleme, tekrar döngüsü ve zaman çizelgesinde
gezme yapılabilir.

Önceden üretilmiş bir `.rrd` kaydını doğrudan açmak:

```powershell
powershell -ExecutionPolicy Bypass -File .\open_g1_rerun_recording.ps1 `
  -Recording .\rerun_recordings\rerun_body38_YYYYMMDD_HHMMSS.rrd
```

## Eklem inceleme ve düzenleme

- Rerun 3B görünümünde bir eklem noktasına tıklayın.
- Selection panelinde konum, ham konum, güven, takip kaynağı, hız, filtre
  hatası, quaternion, Euler XYZ ve ilgili anatomik/geometrik açılar görünür.
- Ayrı kontrol penceresindeki **Eklem düzenleme** sekmesinde eklemi seçerek
  X/Y/Z ofsetini değiştirebilirsiniz.
- **Filtre ve oynatma** sekmesinde güven eşiği, EMA alpha, iskelet ölçeği,
  global ofset, azami eklem hızı, örtülme tutma süresi ve oynatma hızı
  değiştirilebilir.
- Değişiklikler yalnızca analiz akışına uygulanır; kaynak paket korunur.

## Üretilen dosyalar

Her oturum `rerun_recordings/` altında şunları üretir:

- `rerun_body38_*.rrd`: Rerun 3B sahne, seçilebilir eklem bileşenleri,
  açı/kalite zaman serileri ve oynatma zaman çizelgesi.
- `skeleton_analysis.jsonl`: kaynak paketin kayıpsız kopyası, işlenmiş
  noktalar, türetilmiş kinematik ve o karedeki etkin parametreler.
- `frames.csv`: kare, gövde ve taşıma/kalite özeti.
- `joints.csv`: her karede her eklem için konum, ham konum, köke göre konum,
  güven, takip durumu, hız, filtre hatası, quaternion, Euler ve ilgili açılar.
- `angles.csv`: tüm açı ve açısal hızların uzun format zaman serisi.
- `session_manifest.json`: koordinat sistemi, birimler, kaynak, dosya
  bağlantıları, ilk parametreler ve isteğe bağlı SVO2 yolu.

## SVO2 hakkında

SVO2 iskelet verisinden üretilemez; ZED SDK'nin stereo görüntü, derinlik ve
sensör verisini içeren özgün kamera kaydıdır. SVO2'yi ZED başlatıcısında
`-RecordSvo2` ile kaydedin ve analiz oturumuna bağlayın:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_rerun.ps1 `
  -Svo2Path .\recordings\ornek.svo2
```

Bu yol `session_manifest.json` içine yazılır; böylece RRD, JSONL/CSV ve özgün
SVO2 aynı deney oturumunda izlenebilir.

## Doğrulama

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_rerun.ps1 `
  -Mode demo

python .\rerun_analysis\test_rerun_analysis.py
```
