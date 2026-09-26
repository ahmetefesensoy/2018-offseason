# Takım Otonom Planları ve Rezervasyonlar — Faz 3

Bu faz, maç öncesinde bilinen takım arkadaşı rotalarını tek bir doğrulanmış plana
dönüştürür. Jetson bu planı zaman bağımlı güvenlik koridorları olarak yayınlar,
algılanan takım robotları rotadan ayrıldığında güveni düşürür ve RViz'de rotayı,
çakışmaları ve sapmayı gösterir.

Bu paket robotu sürmez. `frc_alliance_playbook` hiçbir koşulda motor komutu
yayınlamaz; çıktıları sonraki strateji ve navigasyon fazlarının girdisidir.

## Veri akışı

```text
Strategy Studio / CSV / PathPlanner 2025.0
                    |
                    v
       canonical schema + SHA-256
                    |
                    v
          alliance_playbook node <----- /world/state
             |              |            dost robot gözlemleri
             |              +---- güven azalması
             v
   /alliance/plan + /alliance/reservations
                    |
                    v
        alliance_visualizer -> /alliance/markers -> RViz
```

Her plan tamamen kabul edilir veya tamamen reddedilir. Bilinmeyen sürüm, alan
dışı konum, `NaN`/sonsuz değer, artmayan zaman, hız/ivme ihlali, segment zaman
çakışması, eksik fallback veya fallback döngüsü kısmi bir plan üretmez.

## Kanonik koordinat ve zaman

- `field_version` yalnızca `2018-power-up-v1` olabilir.
- Alan `16.46 × 8.23 m` kabul edilir.
- Koordinatlar ittifaktan bağımsız olarak mavi kökenli `map` çerçevesindedir.
- Başlık radyan, mesafe metre, tüm zaman alanları mikro-saniyedir.
- Plan bir veya iki takım arkadaşı içerir; kendi robotumuz bu dosyaya yazılmaz.
- SHA-256; `content_sha256` alanı hariç, UTF-8, sıralı anahtarlar ve boşluksuz
  ayraçlarla kanonik JSON üzerinden hesaplanır.

Kırmızı ittifak görünümü yalnız bir önizleme dönüşümüdür; kaydedilen dosyanın
koordinat sistemini değiştirmez.

## Strategy Studio

Depo kökünde yerel bir sunucu açın:

```bash
python -m http.server 8080
```

`http://localhost:8080/strategy_studio/` adresinde:

- en fazla iki takım robotu ve birden çok görev segmenti düzenlenir;
- alana tıklayarak nokta eklenir, sürükleyerek veya ok tuşuyla taşınır;
- hız, ivme, alan sınırı ve zaman hataları anında gösterilir;
- gövde + koridor tüpleri 100 ms örneklenir; örnekler arasındaki doğrusal hareket
  analitik olarak çözülerek aradaki çarpışmalar da bulunur;
- kanonik JSON, doğrulayıcıyla uyumlu SHA-256 ve PNG sunum görseli indirilir;
- dosyalar yalnız tarayıcıda işlenir ve dışarı yüklenmez.

Ayrıntılı kullanım ve klavye kısayolları `strategy_studio/README.md` içindedir.

## Desteklenen içe aktarma biçimleri

### Kanonik JSON

Depodaki `config/alliance/example-plan.json` doğrudan yüklenebilir. Dosyada hash
varsa içerikle eşleşmek zorundadır.

### CSV

Başlık tam olarak şöyledir:

```text
time,x,y,heading,vx,vy,omega,event
```

`time` saniye, `heading` derece, konum metre ve hız metre/saniyedir. Olay alanı
`SCORE_SWITCH:left-switch` gibi `TASK:target` biçiminde yeni segment başlatır.
Konum farkından türetilen hızla yazılı hız arasında `0.25 m/s` üzerinde fark
varsa dosya reddedilir.

### PathPlanner

İçe aktarıcı güncel `2025.0` `.path` dosyalarındaki kübik Bézier waypoint'lerini
span başına deterministik 20 adımla örnekler. `.auto` içinde sıralı `path`, `wait`
ve `named` komutları desteklenir. Aynı anda birden çok sürüş komutu doğurabilecek
`parallel`, `race` ve `deadline` yapıları güvenli biçimde reddedilir.

Biçim referansı: [PathPlanner paths and autos](https://pathplanner.dev/gui-editing-paths-and-autos.html).

## CLI

ROS workspace kurulduktan sonra örnek planı doğrulayın:

```bash
playbook validate config/alliance/example-plan.json
```

CSV örneği:

```bash
playbook import-csv ally.csv ally.json \
  --plan-id q12-blue --alliance blue --team 254 --label ally-left
```

PathPlanner dosyaları:

```bash
playbook import-path Pickup.path pickup.json \
  --plan-id q12-blue --alliance blue --team 254 --label ally-left

playbook import-auto Example.auto auto.json --paths ./paths \
  --plan-id q12-blue --alliance blue --team 254 --label ally-left
```

Tek takım dosyaları Studio'da açılıp ikinci takım eklenerek birleşik plan olarak
dışa aktarılabilir.

## ROS 2 ve RViz demosu

Varsayılan launch, paketle gelen hash'li örnek planı açar:

```bash
ros2 launch frc_bringup foundation.launch.py \
  use_fake_roborio:=true \
  use_fake_autonomy:=false \
  use_synthetic_perception:=true
```

Özel plan ve beklenen hash:

```bash
ros2 launch frc_bringup foundation.launch.py \
  alliance_plan_path:=/data/q12-blue.json \
  alliance_plan_sha256:=86d0e7a307ee5b44b93344a47a8758384a2c3c4379269808639bc13dc23e5be2
```

Önemli topic'ler:

```bash
ros2 topic echo /alliance/plan
ros2 topic echo /alliance/reservations
ros2 topic echo /diagnostics
ros2 topic echo /alliance/markers
```

`Alliance Reservations` RViz katmanı şunları gösterir:

- her takımın merkez rotası ve yarı saydam gövde + koridor tüpü;
- takım numarası ve anlık plan güveni;
- aynı zamanda üst üste gelen kırmızı rezervasyon bölgeleri;
- algılanan dost robottan en yakın plan koridoruna sapma çizgisi.

`/autonomy/status.mode` değerinin `DISABLED/TELEOP` durumundan `AUTONOMOUS`
durumuna geçişi plan zamanının sıfırını başlatır; simülasyonda
`AUTONOMOUS_SIM` aynı sözleşmeyi kullanır. Mode otonomdan çıktığında füzyon
durumu temizlenir. ROS saati geriye sarılırsa (örneğin rosbag tekrarında) yeni
bir epoch açılır. Algılanan dost robot ilk anda deterministik en-yakın rotaya
eşlenir, ardından sabit `track_id` bağı korunur; robotlar kesiştiğinde kimlikler
yer değiştirmez. Koridor dışı sapma üstel olarak güveni düşürür; kayıp gözlem
her yapılandırılmış yarı ömürde güveni yarılar. Güven hiçbir gözlemle
kendiliğinden yükselmez.

## Rosbag

Standart kayıt listesine plan, rezervasyon ve marker topic'leri eklendi:

```bash
./jetson/scripts/record_autonomy.sh /bags/alliance_demo
```

Plan JSON'u ve kayıt başlangıcındaki `content_sha256` aynı demo artifact'iyle
birlikte saklanmalıdır. Böylece RViz sunumu ve sonraki kararların hangi takım
varsayımıyla üretildiği tekrar kanıtlanabilir.

## Güven sınırı ve sonraki faz

Bu faz takım niyetini doğrulanmış bir dünya-modeli girdisine dönüştürür; en yüksek
skoru seçen görev puanlayıcısı, rakip risk modeli, zaman-genişletilmiş global
planlayıcı ve yerel obstacle avoidance henüz bu paketin görevi değildir. Sonraki
faz bu rezervasyonları ve `/world/state` tahminlerini kullanarak aday görevleri
puanlayacak; gerçek hareket yine roboRIO güvenlik kapısından geçecektir.
