# Raspberry Pi 4 ve breadboard göğüs taşıyıcıları

Bu pakette basılması gereken iki parça vardır:

| STL | Baskı adedi | Kullanım |
|---|---:|---|
| `01_Raspberry_Pi4_acik_gecmeli_gogus_tabani_1_adet.stl` | 1 | Raspberry Pi 4 Model B |
| `02_Breadboard_160x52_5_gecmeli_gogus_tabani_1_adet.stl` | 1 | 160 × 52,5 × 9 mm breadboard |

Üst kapak ve ayrı kelepçe parçası yoktur. Her STL tek parça basılır.

## Ölçü kaynağı

Raspberry Pi tabanı, kullanıcının verdiği `RP-008343-DS-1` resmi mekanik
çizimine göre hazırlanmıştır:

- PCB: 85 × 56 mm
- Montaj deliği düzeni: 58 × 49 mm
- PCB delikleri: 2,7 mm
- Köşe yarıçapı: 3 mm

PCB delikleri normalde M2.5 bağlantıya uygundur. Bu sürüm, kullanıcının mevcut
M2 donanımı için 2,30 mm taban delikleri ve altta M2 somun yuvaları kullanır.
PCB deliğindeki boşluğu almak için vida başının altında M2 pul kullanılır.
M3 vida PCB deliğinden geçirilmemelidir.

## Raspberry Pi için gerekenler

- 1 adet Raspberry Pi tabanı
- 4 adet M2 × 12 mm vida
- 4 adet M2 pul
- 4 adet M2 somun
- En fazla 25 mm genişlikte cırt/kayış veya plastik kelepçe

Pi yalnızca vidalarla tutulmaz. Üst ve alt PCB kenarlarında toplam dört esnek
tırnak vardır. İki uçtaki alçak dayamalar da sağa-sola kaymayı engeller.
Vidalar giyilebilir kullanımda ikinci güvenlik katmanıdır.

### Raspberry Pi montajı

1. Dört M2 somunu tabanın altındaki altıgen ceplere yerleştirin.
2. Gerekirse somunları montaj boyunca küçük bir bant parçasıyla tutun.
3. Kartın bir uzun kenarını iki tırnağın altına hafif açıyla sokun.
4. Karşı kenarı eşit biçimde aşağı bastırıp diğer iki tırnağa kilitleyin.
5. Dört kart deliğini yükselticilerle hizalayın.
6. Her M2 vidaya bir pul takın ve üstten vidalayın.
7. Vidaları kartı eğmeyecek kadar sıkın; zorlayarak sıkmayın.
8. Kayışı iki yandaki 28 × 4,2 mm yuvalardan geçirin.

USB, Ethernet, Type-C, micro-HDMI ve GPIO tarafları açık bırakılmıştır. Kart
tabandan 6 mm yüksekte durur ve tabanda üç büyük havalandırma penceresi vardır.

## Breadboard montajı

Breadboard için vida veya yapıştırıcı kullanılmaz. Ölçülen gerçek breadboard
boyutu 160 × 52,5 × 9 mm kabul edilmiştir.

1. Breadboardun bir uzun kenarını aynı taraftaki üç tırnağın altına sokun.
2. İki kısa ucun dayamalar arasında kaldığını kontrol edin.
3. Karşı uzun kenara eşit kuvvetle bastırın.
4. Üç karşı tırnağın sırayla breadboard üst kenarına geçtiğini kontrol edin.
5. Kayışı uçlardaki 28 × 4,2 mm yuvalardan geçirin.
6. İstenirse dört küçük yuvadan ayrıca plastik kelepçe geçirilebilir.

Sökmek için karşı taraftaki üç tırnağı ince plastik bir kartla dışarı doğru
esnetip breadboardu kaldırın. Tornavidayla sertçe kanırtmayın.

## Baskı ayarları

- Baskı yönü: geniş ve düz alt yüzey tabla üzerinde
- Nozul: 0,4 mm
- Katman: 0,20 mm
- Duvar: 4 çevre
- Üst/alt katman: en az 5
- Dolgu: %25-35 gyroid veya grid
- Destek: kapalı
- Ölçek: %100
- Malzeme: PETG önerilir; iyi kalite PLA+ da kullanılabilir
- Breadboard parçası 193 mm uzundur; 220 mm tablaya gerekirse çapraz yerleştirin
- Uzun breadboard parçasında 5-8 mm brim kullanılabilir

Tırnaklar için standart kırılgan PLA yerine PETG daha güvenlidir. Dilimleyicide
fil ayağı telafisi varsa yaklaşık 0,15-0,20 mm kullanılabilir.

## Dosyalar ve doğrulama

- `mesh_validation.json`: iki STL'nin boyut, hacim, watertight ve tek gövde sonucu
- `01_Raspberry_Pi4_onizleme.png`: boş ve montajlı Pi görünümü
- `02_Breadboard_onizleme.png`: boş ve montajlı breadboard görünümü
- `build_chest_carriers.py`: ölçüleri değiştirip STL'leri yeniden üretmek için kaynak
- `render_previews.py`: gerçek ağ geometrisinden önizlemeleri üretir

Otomatik kontrolde hem Pi hem breadboard temsili hacimleriyle taşıyıcılar
arasında istenmeyen katı çakışma `0,0 mm³` bulunmuştur. İki STL de watertight,
tek gövdeli ve baskı tabanında Z=0 konumundadır.
