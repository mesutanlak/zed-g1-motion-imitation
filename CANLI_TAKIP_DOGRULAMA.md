# Canlı takip doğrulaması

## Yapılan düzeltmeler

- GMR 29-DOF çözümünden sonradan eklem atmak yerine, G1 EDU 23-DOF modelinde
  bulunmayan bel roll/pitch ve iki bileğin pitch/yaw eklemleri IK sırasında
  sıfıra kilitlenir.
- ZED filtreli noktası kısa süre kaybolduğunda, yalnızca önceki güvenilir
  noktaya süreklilik gösteren ham stereo noktası geçici yedek olarak kullanılır.
- GMR logu artık `targets=doğrudan+ham_yedek+hafıza`, `reject_reasons` ve
  `ik_pos=ortalama/maksimum m` alanlarını gösterir.
- Isaac üst gövde PD kazançları yalnız simülasyonda 2.0 katına çıkarılmıştır.
  Alt bedenin resmî nominal çift-temas duruşu değişmez.
- Analiz panelinde Ctrl+C artık Qt olay döngüsünü düzenli kapatır; aktif
  `QPainter` içinde `KeyboardInterrupt` oluşmaz.

## Ölçülen sonuç

Kayıt replay testinde üst-gövde eklem takip hatasının medyanı 0.311 rad'dan
0.134 rad'a, yüzde 90 değeri 0.611 rad'dan 0.186 rad'a düştü. Her iki ayak
yüksekliği 0.035 m kaldı ve reset oluşmadı.

23-DOF kilitli GMR testinde kilitli altı eklemin en büyük mutlak hareketi
1.27e-6 rad oldu. Aynı kayıtta bilek hedeflerinin medyan Kartezyen IK hatası
sol/sağ için yaklaşık 0.030/0.040 m, yüzde 90 hatası 0.042/0.083 m ölçüldü.
Bu değerler tek kamera ve mevcut antropometrik ölçekle milimetrik doğruluk
iddiasının gerçekçi olmadığını gösterir.

## Kalibrasyon

Başlatırken `-HumanHeightM` değerini kameradaki kişinin gerçek boyuna ayarlayın:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 `
  -Mode upper_body `
  -StanceMode fixed_double_support `
  -HumanHeightM 1.80 `
  -UpperStiffnessScale 2.0 `
  -UpperDampingScale 2.0 `
  -AcceptNvidiaEula
```

Doğruluk üç ayrı katmanda izlenmelidir:

1. ZED algısı: geçerli FPS, `Capture->UDP`, ham/filtre RMS.
2. Retargeting: `ik_pos`, hedef sayısı ve red nedenleri.
3. Isaac fiziği: `tracking_err`, iki ayak yüksekliği ve reset sayısı.

Tek ZED 2i ve farklı insan/robot uzuv oranlarıyla ilk gerçekçi hedef santimetre
mertebesidir. Milimetreye yaklaşmak için kamera-robot extrinsic kalibrasyonu,
kişinin boy ve uzuv ölçüleri, ek bir görüş/marker referansı ve robot uç-efektör
geri beslemesi gerekir.

## Eğitim kararı

Canlı üst-gövde gecikmesini gidermek için ilk adım LLM eğitmek değildir.
Retargeting deterministik geometri/IK, fizik takibi ise PD veya bir motion
tracking politikası problemidir. Mevcut JSONL kayıtları regresyon ve kişiye özel
kalibrasyon için değerlidir; genel bir politika eğitmek için hareket çeşitliliği
yetersizdir.

Sonraki eğitim aşamasında AMASS veya LAFAN1 hareketleri GMR ile G1 23-DOF'a
kilitli biçimde retarget edilmeli, ayak teması ve eklem limitleriyle
temizlenmeli, ardından Isaac Lab'de referans takip politikası eğitilmelidir.
AMASS yalnız lisans koşullarına uygun, ticari olmayan araştırma kapsamında
kullanılmalıdır. LLM, düşük seviyeli eklem komutu üretmek için kullanılmamalıdır.

## Örtülme ve yeniden yakalama

`zed_body38_20260730_102827.jsonl` kaydında sağ bilek 33 kare (2,20 s)
kaybolmuş ve yeniden göründüğü ham karede 1,53 m sıçramıştır. Bu nedenle
algı katmanı aşağıdaki durum makinesini uygular:

- Güvenilir nokta: normal GMR hedefi olarak kullanılır.
- Kısa kayıp: en fazla iki kare süreklilik gösteren ham nokta, ardından kısa
  süreli son güvenilir değer kullanılır.
- Uzun kayıp veya üst üste binme: eklem hedefi geçersiz sayılır; hatalı ham
  nokta güvenilir belleğe yazılmaz.
- Yeniden görünme: nokta ancak üç ardışık ve birbiriyle tutarlı kareden sonra
  yeniden etkinleştirilir.
- Tüm canlı giriş bayatlarsa Isaac önce kısa süre hedefi tutar, sonra yalnızca
  üst gövdeyi resmi nominal poza yumuşakça döndürür. Alt beden ve fizik kontrolü
  aktif kalır.

Isaac günlüğündeki `upper_state` alanı bu akışı gösterir:

- `LIVE`: güvenilir canlı hedef uygulanıyor.
- `HOLD`: çok kısa paket/kare boşluğu tolere ediliyor.
- `RETURN`: uzun kayıpta üst gövde güvenli nominal poza dönüyor.

Regresyon testi:

```powershell
python .\isaaclab_bridge\test_occlusion_resilience.py `
  .\recordings\zed_body38_20260730_102827.jsonl
```

Beklenen çıktı `OCCLUSION_RESILIENCE_OK` olmalıdır.
