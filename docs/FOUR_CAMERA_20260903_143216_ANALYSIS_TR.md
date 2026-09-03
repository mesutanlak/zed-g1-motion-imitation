# 4-ZED kisa kayit incelemesi — 2026-09-03 14:32:16

Incelenen kaynaklar `recordings/four_body38_fusion_20260903_143216.jsonl` ve
`rerun_recordings/rerun_body38_20260903_143116` oturumudur.

## Sonuc

Bu kayitta algi/fusion hatti saglamdir. Rerun'a yazilan 510 BODY karesinin
504'u `FOUR_FUSED`, yalniz 6'si `PARTIAL_FUSED` olmustur; gercek dort-kamera
orani `%98.82`, cikis hizi `14.87 Hz`dir. Isaac'ta gorulen capraz-govde
takipsizliginin ana nedeni kamera kaybi degil, insan bilegi govdeyi gecerken
daha kisa G1 kol hedefinin gogus geometrisinin icinden gecmesi ve guvenlik
katmaninin bu gercek temasi engellemesidir.

## Kayittan olculenler

- Kamera zaman yayilimi ortalama `34.84 ms`.
- En gec kamera p50 `29.1 ms`, p90 `40.8 ms`, en kotu `58.1 ms`.
- Kontrol toplam gecikmesi p50 yaklasik `130 ms`.
- Cross-view MPJPE ortalama `9.6 cm`; sol/sag bilek uyusmazligi ortalama
  `11.6/12.5 cm`.
- Referans hiz `1.2 rad/s`, ivme `6 rad/s2` limitlerine sikca dayanmistir.
  Bu, kullanicinin gozledigi gereksiz hizli/sert hareketle uyumludur.
- Eski kodla sol kolun govdeyi caprazladigi 35/35 Isaac karesinde ham model
  temasi ve sol-kol safe-return gorulmustur.

## Uygulanan duzeltmeler

1. BODY_38 bilegi karsi omuz tarafina gectiginde, kol on taraftaysa G1 hedefi
   robotun resmi govde geometrisinin onunden dolastirilir. Lateral ve dikey
   insan niyeti ile resmi ust-kol/onkol uzunluklari korunur. Bilek omuzdan
   2 cm'den fazla arkadaysa arkaya uzanma niyeti sayilir ve bu donusum
   uygulanmaz.
2. Yaklasik dairesel torso kapsulunun resmi G1 govdesinin on koselerini fazla
   buyutmesi icin 4 cm model payi eklendi. Kesin MuJoCo model temasi halen sert
   sinirdir; kafa, kollar-arasi veya gercek govde penetrasyonu gecirilmez.
3. Canli simulasyon varsayilani dengeli profile alindi: `15 Hz` girdi,
   `5.5 Hz` cevap, `0.85 rad/s` hiz, `4 rad/s2` ivme ve `35 rad/s3` jerk.
4. Yeni kayitlar `front_clearance_blend`, `front_clearance_shift_m` ve robot
   govde bariyeri alfa/marjini telemetrilerini CSV/JSONL kalite ozetine yazar.

## Yeniden oynatim kabul sonucu

Yeni retarget koduyla tum ham kayit yeniden oynatildi. Sol kolun govdeyi
caprazladigi 107 karenin `107/107` tanesi on-clearance yolunu kullandi,
`0` ham MuJoCo temasi ve `0` ORANGE safe-return uretti. Sag capraz hareketin
25/25 karesinde de temas yoktur. Iki kolun ayni yana gittigi sentetik resmi G1
model testi de temassiz gecti.

Kaydin daha sonraki bir bolumunde sag elin resmi robot kafa geometrisine gercek
temasi devam etmektedir. Bu kareler bilerek engellenir; fiziksel sinirlar
icinde kosulsuz takip, robotun katilarinin birbirinin icinden gecmesi anlamina
gelmez.

## Sonraki canli kabul

- Capraz-govde hareketinde `front_clearance_blend > 0`, ORANGE oraninda belirgin
  dusus ve fiziksel temas sayisinda sifir beklenir.
- Sabit pozda jerk ve hedef duzeltmesi eski kayittan dusuk olmali; robot belirgin
  titrememeli.
- Iki kol ayni yana hareket ederken ikisi de yonu takip etmeli. Kollar birbirine
  veya kafaya fiziksel olarak girecek hedefte bariyer devreye girmelidir.
- Fusion kabul hedefi `FOUR_FUSED >= %95`, cikis `>=13 Hz` olarak korunur.
