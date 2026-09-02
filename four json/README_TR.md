# Four kamera kalibrasyon gelen kutusu

Tercihen ZED360 içinde **Finish Calibration** sonrasında kaydettiğiniz dört
kameralı Fusion JSON dosyasını bu klasöre koyun. ZED360 ağ kalibrasyonu bu
kurulumda çalışmazsa kalite kapısını geçen
`zed_body38_distributed_extrinsics/v1` uygulama kalibrasyonu da doğrudan buraya
konabilir. Klasörde aynı anda yalnızca **bir** `.json` dosyası bulunmalıdır.

`start_zed_four_fusion_to_wsl.ps1` dosya tipini otomatik seçer. ZED360 dosyasını
ZED SDK ile `RIGHT_HANDED_Z_UP_X_FWD`/metre sistemine okuyup seçilen referans
kameraya göre yeniden tabanlar; BODY_38 extrinsic dosyasını ise doğrudan ama
aynı katı kalite kapısından geçirerek kullanır.

Seed veya kalibrasyon tamamlanmadan kaydedilmiş, bütün kamera pozları sıfır olan
bir dosya kabul edilmez. Tripodlardan biri hareket ederse ZED360 kalibrasyonunu
yeniden yapıp buradaki JSON'u değiştirin.

Rig'e özel `.json` Git'e gönderilmez; laptopun bu dosyaya ihtiyacı yoktur.
Dosya yalnız Fusion alıcısının çalıştığı ana PC'de bu klasörde bulunmalıdır.
