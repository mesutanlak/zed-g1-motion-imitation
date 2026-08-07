# G1-23 BODY_38: koşullu AKC ve MIRROR kurtarma katmanı

## Güvenli geri dönüş noktası

Bu geliştirme öncesindeki çalışan sistem değişmez bir Git referansıyla
korunur:

- dal: `codex/pre-mirror-akc-baseline-20260807`
- etiket: `baseline/zed-g1-stable-20260807`
- commit: `f7c2141`

Geri dönmek için çalışan süreçleri kapattıktan sonra:

```powershell
git switch codex/pre-mirror-akc-baseline-20260807
```

Yeni entegrasyon ayrı `codex/mirror-akc-g1-23dof` dalındadır. ZED SDK'nın
BODY_38 algılama/model/çekim yolu değiştirilmemiştir; eklenen mantık yalnız
BODY_38 sonrası iskelet düzeltme ve GMR sonrası komut güvenliği katmanındadır.

## Çalışma sırası

1. ZED 2i BODY_38 ham iskeleti üretir.
2. Normal ve güvenilir kare doğrudan mevcut akıştan geçer.
3. `torso_overlap`, düşük güven veya kemik uzunluğu ihlalinde koşullu AKC kol
   zinciri kurtarması dört omuz-dirsek adayı arasından seçim yapar.
4. GMR mevcut nominal `q_nom` çözümünü üretir.
5. Yönlü omuz-dirsek-bilek düzlem normali dirsek dalını korur. Dal değişimi
   için dört güvenilir ardışık kare gerekir; düz kola yakın tekillikte son
   güvenilir pole normal kullanılır.
6. MIRROR-esinli continuation yalnız yüksek residual, düşük sert çarpışma
   marjı, limite doğru yaklaşma, gerçek kareler-arası büyük `q_nom` adımı veya
   doğrulanmakta olan dirsek dalı olayında çalışır. Alt beden, bel ve wrist
   roll çözüm değişkeni değildir.
7. Sürekli kapsül mesafeleri ve mevcut feasibility filtresi sonucu son kez
   denetler. Isaac Lab'e yalnız `safe_q` gönderilir.

Buradaki MIRROR bileşeni makaledeki tam GPU uygulamasının kopyası değildir.
Mevcut GMR'ı bozmadan aynı entegrasyon sözleşmesini sağlayan, olay tetiklemeli
ve yakın-komşulukta çalışan DLS/QP continuation kurtarmasıdır.

## Çarpışma sınıfları

Sert güvenlik çiftleri; üst kol-gövde (omuz montaj toleranslı), el-baş ve iki
kol arasındaki çiftlerdir. Önkol/el-gövde mesafeleri yine sürekli ölçülüp
telemetriye yazılır, ancak göğüs önünde kol çaprazlama gibi geçerli hareketler
robotu dondurmasın diye yumuşak temas olarak sınıflanır. Tehlike oluşursa
yalnız ilgili kolun referans blend'i azaltılır; diğer kol ve gövde canlı kalır.

## Çalıştırma ve geri alma bayrakları

Varsayılan başlatıcı yeni korumaları açar:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 -Mode upper_body -AcceptNvidiaEula
```

Tanı/karşılaştırma için:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 -Mode upper_body -AcceptNvidiaEula -NoMirrorRescue
powershell -ExecutionPolicy Bypass -File .\start_g1_isaaclab_live.ps1 -Mode upper_body -AcceptNvidiaEula -NoAnatomicalBranchContinuity
```

ZED başlatıcısındaki `-ArmRecoveryMode legacy` yalnız A/B regresyon testi
içindir; varsayılan `akc` olarak kalır.

## Replay kabul sonuçları

`zed_body38_20260804_154534.jsonl`, örtüşme aralığı (160 kare):

- kabul edilen MIRROR düzeltmesi: 12/44 tetiklenen olay
- ortalama task-space kazanımı: 4,09 mm
- ortalama sert çarpışma marjı kazanımı: 11,05 mm
- ORANGE kare: 2 (MIRROR kapalı eski karşılaştırmada 8)
- relative residual p50/p95: 0,0473/0,1569 m

`zed_body38_20260805_141624.jsonl`, 60 FPS bölümü (180 kare):

- olay: 8; kabul edilen gereksiz düzeltme: 0
- relative residual p50/p95: 0,0295/0,0529 m
- sert çarpışma çifti: yok

Bu sonuçlar dosya replay regresyonudur; fiziksel G1'e komut gönderilmemiştir.
Canlı Isaac Lab kabulü için önce `upper_body` modunda kayıt alınmalı ve
`mirror_rescue_reasons`, `arm_reference_blend`, sert/yumuşak temas çiftleri ile
uçtan uca gecikme telemetrisi incelenmelidir.
