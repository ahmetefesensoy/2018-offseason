# NEO/SparkMax Swerve Commissioning Rehberi

Bu rehber dört NEO drive, dört NEO steer, dört REV Through Bore absolute
encoder ve MXP SPI NavX kullanan 2018 offseason robotu içindir. Yazılım masaüstü
testlerinden geçmiş olsa da aşağıdaki fiziksel ölçümler tamamlanmadan robot yerde
tam hızda sürülmemelidir.

## Güvenli Başlangıç Durumu

Kod bilinçli olarak aşağıdaki muhafazakâr değerlerle gelir:

- azami doğrusal hız: `1.0 m/s`;
- azami açısal hız: `2.0 rad/s`;
- wheelbase ve track width: `0.60 m` varsayımı;
- drive reduction: `6.75:1` varsayımı;
- dört absolute offset: `0 rad`;
- tüm drive, turn ve absolute-encoder inversion bayrakları: `false`.

Bu değerlerin hiçbiri fiziksel robot ölçümü yerine geçmez. Robot ilk kez enable
edilirken tekerlekler yerden kesilmiş, lift aşağıda, intake boş ve bir operatör
Driver Station acil durdurma kontrolünün yanında olmalıdır.

## Donanım ve CAN Kontrol Listesi

| Modül | Drive SparkMax | Steer SparkMax |
|---|---:|---:|
| Front-left (`FL`) | 10 | 11 |
| Front-right (`FR`) | 12 | 13 |
| Back-left (`BL`) | 14 | 15 |
| Back-right (`BR`) | 16 | 17 |

1. SparkMax firmware sürümünü REVLib 2025 ile uyumlu sürüme yükseltin.
2. Her steer motorundaki REV Through Bore Encoder'ı absolute-duty-cycle
   çıkışıyla Absolute Encoder Adapter (`REV-11-3326`) üzerinden steer
   SparkMax'ın Data Port'una bağlayın.
3. NEO sensör kablosunun kendi SparkMax Encoder Port'una bağlı olduğunu
   doğrulayın. NEO sensör kablosu çıkarılmış halde brushless motor sürmeyin.
4. CAN hattının iki ucunda terminasyon bulunduğunu ve robot kapalıyken CAN-H ile
   CAN-L arasında yaklaşık `60 ohm` ölçüldüğünü doğrulayın.
5. REV Hardware Client'ta yalnız bir controller'a enerji vererek CAN ID'lerini
   sırayla `10–17` yapın. Aynı anda aynı ID'yi taşıyan iki cihaz bırakmayın.
6. Intake/elevator/climb ID'lerinin `1`, `2`, `3`, `9` olduğunu ve çakışmadığını
   doğrulayın.

## Absolute Encoder Yönü ve Offset Kalibrasyonu

SmartDashboard üzerinde her modül için şu alanlar yayınlanır:

```text
Swerve/<MODUL>/RawAbsoluteRotations
Swerve/<MODUL>/MeasuredAngleDegrees
Swerve/<MODUL>/DesiredAngleDegrees
```

Her modülü ayrı ayrı kalibre edin:

1. Robot disabled iken tekerleği fiziksel olarak tam ileriye hizalayın.
2. `RawAbsoluteRotations` değerini kaydedin.
3. Modülü üstten bakıldığında saat yönünün tersine çevirin. `MeasuredAngleDegrees`
   artmıyorsa yalnız ilgili `absoluteEncoderInverted` alanını `true` yapın,
   deploy edin ve testi tekrarlayın.
4. Tekerleği yeniden ileri hizalayın ve yeni raw değeri kaydedin.
5. Offset'i aşağıdaki bağıntıyla hesaplayın:

   ```text
   absoluteOffsetRadians = RawAbsoluteRotations × 2π
   ```

6. Değeri `SwerveConstants.MODULE_CONFIGS` içindeki doğru modüle yazın.
7. Robotu kapatıp açın. Tekerlek ileri bakarken `MeasuredAngleDegrees` değerinin
   `0°` civarında olduğunu doğrulayın.

Offset ölçümü, encoder inversion kararı verildikten sonra yapılmalıdır. Encoder
yönü sonradan değiştirilirse offset yeniden ölçülmelidir.

## Steer Motor Yönü

1. Robot bloklar üzerindeyken küçük bir pozitif açı hedefi verin.
2. Modül üstten bakıldığında saat yönünün tersine gitmeli ve ölçülen açı hedefe
   yaklaşmalıdır.
3. Ölçülen açı hedeften uzaklaşıyorsa robotu hemen disable edin ve yalnız o
   modülün `turnInverted` bayrağını değiştirin.
4. Dört modülü `+30°`, `-30°` ve wrap-around yakınında `+170°/-170°` hedefleriyle
   ayrı ayrı doğrulayın.

`turnInverted` motor yönünü, `absoluteEncoderInverted` sensör işaretini düzeltir;
iki alan birbirinin yerine kullanılmamalıdır.

## Drive Motor Yönü ve Mesafe Ölçeği

1. Tüm modüller `0°` ileri bakarken robot bloklar üzerinde `+x = 0.2 m/s`
   komutu verin.
2. Her tekerlek robotu ileri itecek yönde dönmelidir. Yanlış olan tek modülün
   `driveInverted` bayrağını değiştirin.
3. Robotu yere indirin ve düz, boş alanda `3.0 m` düşük hızlı sürün.
4. Dashboard/pose mesafesi ile şerit metre ölçümünü karşılaştırın.
5. Sistematik oran hatası varsa önce gerçek teker çapını ve gerçek mekanik
   redüksiyonu ölçün; `WHEEL_DIAMETER_METERS` ve `DRIVE_REDUCTION` değerlerini
   fiziksel ölçüme göre güncelleyin. Yazılımı rastgele bir katsayıyla düzeltmeyin.

## Şase Geometrisi

`WHEEL_BASE_METERS` ön ve arka modül dönme eksenleri arasındaki mesafedir.
`TRACK_WIDTH_METERS` sol ve sağ modül dönme eksenleri arasındaki mesafedir.
CAD'den ve fiziksel şaseden iki ölçümü de alın. Mevcut `0.60 m` değerlerini gerçek
ölçülerle değiştirdikten sonra saf dönme komutunda dört modülün doğru teğet
açılara yöneldiğini doğrulayın.

## NavX Kontrolü

1. NavX'i roboRIO MXP SPI bağlantısına takın.
2. Robot hareketsizken startup calibration'ın bitmesini bekleyin.
3. SmartDashboard'da `Swerve/GyroConnected=true` ve
   `Swerve/GyroCalibrating=false` görülmelidir.
4. Joystick button `4` ile heading'i sıfırlayın.
5. Robotu üstten bakıldığında saat yönünün tersine döndürün;
   `Swerve/HeadingDegrees` artmalıdır.
6. Gyro çıkarıldığında veya kalibrasyondayken kodun field-relative yerine
   robot-relative sürüşe düştüğünü düşük hızda doğrulayın.

## Sürüş Doğrulama Sırası

1. Disabled sensör/telemetri kontrolü.
2. Blok üzerinde tek modül steer kontrolü.
3. Blok üzerinde tek modül drive yönü.
4. Blok üzerinde dört modül robot-relative sürüş.
5. Yerde `0.2 m/s` düz sürüş ve mesafe kalibrasyonu.
6. Yerde düşük hızlı strafe.
7. Yerde düşük hızlı saf dönüş.
8. Robot-relative birleşik x/y/omega komutu.
9. NavX sıfırlandıktan sonra field-relative sürüş.
10. Driver Station disable, NavX bağlantı kaybı ve CAN fault denemeleri.

Her adım önceki adım sorunsuz tamamlandıktan sonra uygulanır. Ölçülen wheelbase,
track width, reduction, inversion ve offset değerleri commit edilmeden
`MAX_SPEED_MPS` veya `MAX_ANGULAR_SPEED_RAD_PER_SEC` yükseltilmez.

## Yazılım Doğrulaması

Windows PowerShell'de Java 17 ile:

```powershell
$env:JAVA_HOME='C:\Program Files\Eclipse Adoptium\jdk-17.0.20.101-hotspot'
.\gradlew.bat clean test build --console=plain
```

Bu komutun exit code'u `0` olmalı ve deploy edilebilir robot JAR'ı
`build/libs/` altında oluşmalıdır. Test başarısı yalnız yazılım davranışını
kanıtlar; yukarıdaki fiziksel commissioning adımlarının yerine geçmez.

REV'in güncel bağlantı referansı:
[Using Encoders with the SPARK MAX](https://docs.revrobotics.com/brushless/spark-max/encoders).
