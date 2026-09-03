# 4-ZED kayit incelemesi — 2026-09-03 13:06:26

Incelenen kaynaklar `recordings/four_body38_fusion_20260903_130626.jsonl`,
`rerun_recordings/rerun_body38_20260903_130555`,
`rerun_body38_20260903_130552.rrd` ve `fourkayit309.mp4` ekran kaydidir.

## Sonuc

Bu oturum gercek bir dort-kamera oturumudur. Rerun'a ulasan 690 BODY karesinin
tamaminda `FOUR_FUSED`, `evidence_views=4` kaydedilmistir. Bir ekleme ortalama
katki `3.07` kameradir. Bu, dorduncu kameranin sistemden cikarildigi anlamina
gelmez: dort kaynak da karenin adayidir; guveni dusuk, okluzyonlu veya diger
kameralarla mekansal olarak uyusmayan tekil eklemler robust fusion tarafindan
bilerek dislanir.

Video, genel govde ve buyuk kol hareketlerinin Isaac'ta izlendigini; en belirgin
bozulmanin kollar govde onunde/arkasindayken ve ozellikle sag onkol gorusu
kayboldugunda olustugunu dogrular. Rerun'daki cok kisa onkol goruntusunun ayrica
bir cizim/kinematik temsil hatasi oldugu bulundu: cizgi dirsekten fiziksel el
ucuna degil, `wrist_roll_rubber_hand` govde orijinine kadar gidiyordu.

## Olculen performans

- Dort gorus: `690/690 FOUR_FUSED`, kare-basi `4/4 evidence_views`.
- Fusion cikisi: ortalama `14.60 Hz`, medyan `14.97 Hz`.
- Kamera capture yayilimi: ortalama `43.67 ms`, p90 `51.32 ms`, p99
  `69.03 ms`.
- En yuksek kamera gecikmesi: ortalama `29.70 ms`, p90 `39.97 ms`.
- En yuksek ag kuyrugu: ortalama `13.42 ms`, p90 `27.73 ms`.
- Windows capture-to-send: ortalama `40.07 ms`, p90 `55.20 ms`.
- GMR/Isaac toplam kontrol gecikmesi: ortalama `122.05 ms`, p90 `142.80 ms`.
- Ham kamera uyumu: cross-view MPJPE ortalama `6.27 cm`; eklem p95 ortalamasi
  `14.68 cm`. Sol bilek uyusmazligi `8.91 cm`, sag bilek `10.40 cm`.
- Dogrudan IK onkol yon hatasi: sol ortalama `10.37°`, sag `15.83°`. Sag kolda
  hic temiz gorus kalmayan bolumlerde bu hata `65.91°`'ye cikmistir.
- G1 referans hiz limiti karelerin buyuk bolumunde tam `0.65 rad/s`, ivme limiti
  tam `2.5 rad/s²` olmustur. Gecikmenin onemli bir bolumu bu eski limitlerde
  biriken hedef hatasidir; referans RMS hatasi ortalama `0.205 rad`dir.
- Isaac eklem takip RMSE ortalamasi `0.00088 rad`dir. Dolayisiyla ana gecikme
  fizik motorunun hedefi izleyememesi degil, hedefin Isaac'a gec ulasmasi ve
  referans rate-limitidir.
- Rerun BODY tablosu yalniz `690` kare (`~2.35 Hz`) yazarken GMR tarafinda
  `3764` benzersiz fusion sequence'i (`~12.8 Hz`) vardir. Eski Rerun'da her
  eklem icin yuzden fazla ayri log cagrisi ve 64 kB civari analiz paketi canli
  gorsellestirmeyi geriden getirmistir; kontrol JSONL akisi kaybolmamistir.
- Guvenlik dagilimi: `GREEN=3148`, `YELLOW=1026`, `ORANGE=2304`. En cok
  nedenler `self_collision_risk`, `self_collision_proximity`,
  `robot_body_barrier_projection` ve sol/sag kol safe-return'dur. Bu bariyerler
  kolun govdenin icinden gecmesini engelledigi icin kaldirilmamistir.

## Uygulanan duzeltmeler

1. Kaynak kameranin ag onizlemesinden buyuk yerel baslik/tani paneli cikarildi.
   Dortlu ekranda her kutunun sol ustunde `355x55 px`, kucuk ve transparan uc
   satirlik konsol kaldi; kafa ve omuzlar gorunur.
2. G1 RAW/SAFE/actual ve HUMAN_PRE_GMR cizimine resmi rubber-hand geometrisinden
   fiziksel el ucu eklendi. Resmi modelde omuz-dirsek `19.29 cm`, eski
   dirsek-wrist-origin `10.05 cm`, yeni dirsek-el-ucu `20.81 cm`dir. El uzantisi
   `10.80 cm`dir. Kontrol hedefi degismeden goruntu, hata metrigi ve carpisma
   kapsulu fiziksel ele kadar uzar.
3. IK'nin `PREVIOUS_HOLD` adayi, BODY_38 kol hedefi 4 mm'den fazla hareket
   ettiginde artik tercih avantaji kazanamaz. Sabit operatorde hold/deadband
   titremeyi bastirmaya devam eder; gercek kol hareketi eski poza takilmaz.
4. Dort-kamera eklem inlier yaricapi calistirma profilinde `30 cm -> 18 cm`
   dusuruldu. Govde arkasindan gelen celiskili kol/bilek tahmini, temiz on/yan
   gorusleri bozmak yerine yalniz uyusuyorsa katkida bulunur.
5. Rerun paketi allow-list ile `64,208 -> 28,629 byte`, GMR kontrol paketi
   `15,122 byte` oldu. Kontrol paketi her zaman analiz/disk isinden once gider.
   UDP backlog'u birikirse Rerun en yeni tam pakete atlar.
6. Rerun'da 38 ayri eklem entity'si yerine varsayilan olarak tek batched
   `Points3D` yazilir. Tum sayisal eklem/aci verisi CSV ve JSONL'de korunur;
   ayrintili entity modu yalniz opt-in'dir.
7. Isaac varsayilan girdi hizi gercek fusion hiziyle `15 Hz` eslendi. Simulasyon
   referans cevap hizi `5 -> 7 Hz`, hiz `0.65 -> 1.2 rad/s`, ivme
   `2.5 -> 6 rad/s²`, jerk `25 -> 60 rad/s³` oldu. Bunlar fiziksel robota DDS
   komutu gondermeyen simulasyon yolundadir.

> Not: Bu ilk hizlandirma profili 14:32 kisa kaydinda fazla sert bulundu.
> Guncel dengeli varsayilanlar `5.5 Hz`, `0.85 rad/s`, `4 rad/s²` ve
> `35 rad/s³` olarak ayarlanmistir. Guncel kabul analizi
> `FOUR_CAMERA_20260903_143216_ANALYSIS_TR.md` dosyasindadir.

## 23/29 DOF siniri

Mevcut canli sozlesme ve Isaac artikulasyonu G1 23-DOF modelidir. Bu duzeltme
29-DOF olmayan modele sahte wrist pitch/yaw eklemez; fiziksel rubber-hand el
ucunu dogru gosterir ve mevcut guvenli kontrolu korur. EDU U3 29-DOF motor
kontrolune gecis; 29-DOF USD/URDF, motor sirasi, limitler, collision modeli,
referans policy boyutu ve fiziksel robot guvenlik kabulunu birlikte degistiren
ayri bir model migrasyonu olarak yapilmalidir. 23 elemanli paketi 29 motorlu
diye etiketlemek guvenli degildir.

## Sonraki kayit kabul hedefi

- `bagli=4/4`, `body_taze=4/4`, frame-level `FOUR_FUSED`, `drop=0`.
- Fusion `>=13 Hz`; yeni Rerun kaydi `>=10 Hz` ve canli goruntu kuyruksuz.
- `mean_camera_fused >=3.0`; her eklemde zorla 4/4 yerine temiz gorus secimi.
- Cross-view MPJPE ortalama `<6 cm`, bilek p90 `<12 cm`. Bu esikler asilirsa
  yazilimla zorla ortalamak yerine statik extrinsic yeniden alinmalidir.
- Sag/sol `direct_ik_forearm_error_deg` p90 `<25°`; temiz gorus sifir oldugunda
  robot ani dal secmek yerine onceki guvenli pozu korumali.
- Yeni hiz/ivme limitleriyle `reference_target_error_rms_rad` p90 `<0.30 rad`
  ve `total_control_ms` p90 `<120 ms` hedeflenir.
- G1 SAFE kol/govde penetration sayisi sifir kalmalidir.
