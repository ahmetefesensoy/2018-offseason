# Alliance Strategy Studio

Strategy Studio, takım arkadaşlarının önceden bilinen otonom rotalarını 2018
alanı üzerinde düzenleyen, tamamen yerel çalışan bir tarayıcı aracıdır. Dosyalar
herhangi bir sunucuya yüklenmez.

## Çalıştırma

Depo kökünde:

```bash
python -m http.server 8080
```

Ardından `http://localhost:8080/strategy_studio/` adresini açın. Yerel HTTP,
SHA-256 için gereken Web Crypto API'nin bütün güncel tarayıcılarda çalışmasını
sağlar.

## İş akışı

1. En fazla iki takım robotunun numarasını, gövde ölçüsünü ve hareket
   limitlerini girin.
2. Aktif robot ve segmenti seçip alan üzerine yol noktaları ekleyin. Noktalar
   sürüklenebilir; ok tuşlarıyla hassas hareket ettirilebilir.
3. Görev, başlangıç zamanı, süre ve güvenlik koridorunu ayarlayın. İçe aktarılan
   başlangıç pencereleri ve fallback bağlantıları arayüzde değiştirilmeseler de
   JSON round-trip sırasında kayıpsız korunur.
4. Sağ panelde hareket limiti hatalarını ve zaman bağımlı koridor
   çakışmalarını giderin.
5. `JSON indir` ile Jetson'ın doğrudan doğruladığı schema-1 planı, `PNG indir`
   ile sunum görselini alın.

`JSON / CSV aç` düğmesi kanonik planları veya şu kesin başlığa sahip rotaları
kabul eder:

```text
time,x,y,heading,vx,vy,omega,event
```

CSV zamanı saniye, başlığı derece ve konumu metre cinsindedir. `event` değeri
`SCORE_SWITCH:left-switch` biçiminde görev ve hedef belirtebilir.

Kırmızı açı önizlemesi çizimi 180 derece çevirir; kaydedilen koordinatlar her
zaman mavi ittifak kökenli kanonik `map` çerçevesinde kalır.

## Klavye

- `Delete`: seçili noktayı siler.
- `Ctrl/Command + Z`: son çizim işlemini geri alır.
- Ok tuşları: seçili noktayı 5 cm taşır; `Shift` ile 25 cm taşır.
- `Escape`: nokta seçimini kaldırır.

İndirilen `content_sha256`, Python doğrulayıcısıyla aynı anahtar sıralaması ve
sayı gösterimi üzerinden hesaplanır. Yine de yarışma öncesi son dosyayı CLI ile
doğrulayın:

```bash
playbook validate config/alliance/example-plan.json
```
