# 2018 Offseason Robotu — Mekanizma Subsystem'leri Tasarımı

## Amaç ve Kapsam

Bu robot, 2018 FRC "Power Up" oyununu offseason'da tekrar oynamak için
yapılıyor. Nihai hedef **otonom mod** (Jetson Orin + görüntü işleme ile
dinamik engelden kaçınma ve en verimli hedefe gitme) ama otonom, sağlam
subsystem'lere ihtiyaç duyuyor. Bu spec sadece **mekanizma
subsystem'lerini** (Elevator, Intake, Climb) ve bunları çalıştırabilecek
minimum WPILib proje altyapısını kapsar. Swerve drive ve Jetson tabanlı
otonom mantığı, ayrı bir sonraki spec'te ele alınacak.

## Arka Plan: Mevcut Durum

Proje klasöründe üç bağımsız `.java` dosyası var (`Constant.java`,
`Intake.java`, `Robot.java`) ve bir CAD dosyası (`Assembly 1.step`).
Kod şu an **derlenmiyor**:

- `Constant.java` sınıf adı `constant` (küçük c), alanları `static`
  değil; `Robot.java` ise `Constants.sol1MotorDrivetrainPortu` gibi
  static erişim bekliyor.
- `Intake.java` paketi `frc.robot`, ama `Robot.java` onu
  `frc.robot.subsystems.IntakeSubsystem` olarak import ediyor.
- `Robot.java`'nın çağırdığı metod adları (`auto_intake()`,
  `intake_açma()` vb.) `Intake.java`'daki gerçek adlarla
  (`autoIntake()`, `intakeAcma()` vb.) eşleşmiyor.
- `build.gradle`, `vendordeps/` gibi hiçbir WPILib proje dosyası yok —
  bu dosyalar şu an bağımsız metin dosyaları, gerçek bir GradleRIO
  projesi değil.

CAD dosyası (`Assembly 1.step`) incelendiğinde (ekran görüntüleri +
STEP metin taraması ile):

- 4× swerve modül, taban köşelerinde.
- Yükseklik mekanizması bir **pivot arm değil, 3 katlı scissor lift**
  (X-linkaj). Sabit açıyla monte edilmiş, tek serbestlik derecesi
  (sadece extend/retract).
- Scissor'ın ucunda **6 roller'lı bir tutucu** (2 grup × 3, V açılı) —
  iki taraf birbirine kapanarak Power Cube'u sıkıştırıp tutuyor, ters
  yönde açılınca kutu bırakılıyor. Mevcut `Intake.java`'daki
  açma/kapama mantığı bu davranışla tutarlı.
- Ayrı bir **hook (kanca)** mekanizması, tırmanma (climb, tower
  rung'a asılma) için.

## Oyun Bağlamı (2018 Power Up) — Neden Bu Hedefler

- Switch plate yüksekliği: 9 inç (~23 cm). Scale plate yüksekliği: 5 ft
  (~152 cm, maç başında). Tower rung: 7 ft (~213 cm).
- Scale, switch'ten çok daha değerli (saniye başına sahiplik puanı
  aynı, ama scale'e ulaşmak daha zor/nadir, rakiplerle rekabetli).
- Kullanıcı kararı: Elevator **Scale yüksekliğine (152 cm) çıkabilmeli**.

## Subsystem Tasarımları

### 1. `ElevatorSubsystem` (yeni)

**Donanım:** 1× NEO motor + SparkMax + dahili relative encoder. Ek
limit switch yok (şimdilik) — soft limit encoder tabanlı olacak.

**Sorumluluk:** Scissor lift'i motor pozisyonuna göre hedef yüksekliğe
götürmek.

**Arabirim:**
```java
public class ElevatorSubsystem extends SubsystemBase {
    public void setTargetHeight(double heightMeters);
    public double getCurrentHeight();
    public boolean atTarget();
    public void stop();
    // periodic(): PID/feedforward hesaplayıp motora uygular, SmartDashboard'a yükseklik basar
}
```

**Kontrol:** WPILib `ProfiledPIDController` + `ElevatorFeedforward`
(motion-profiled, ani hareket yerine yumuşak ivmelenme — hem
mekanizmayı hem de Power Cube'u korur).

**Preset yükseklikler** (`Constants.java` içinde):
`GROUND_METERS`, `SWITCH_METERS` (0.23), `SCALE_METERS` (1.52).

**Bilinmeyen/varsayım:** Encoder tick → yükseklik dönüşüm katsayısı
(scissor linkaj geometrisine bağlı) gerçek robotta kalibre edilene
kadar placeholder bir sabit olarak bırakılacak, kodda açıkça
`// TODO: kalibre et` ile işaretlenecek. Bu, spec'in bilinen tek
belirsizliği ve robotu gerçekten çalıştırmadan kapatılamaz.

### 2. `IntakeSubsystem` (mevcut kodun düzeltilmesi)

Mevcut mantık (iki taraflı roller motorların birbirine kapanıp
kutuyu sıkıştırması, ultrasonik sensörle kutu varlığı tespiti) **doğru
ve korunacak**. Yapılacak düzeltmeler:

- Paket adı `frc.robot.subsystems` olacak (dosya buna göre taşınacak).
- Sınıf adı zaten `IntakeSubsystem` — korunuyor.
- Metod adları İngilizceye ve `Robot`/`RobotContainer`'ın çağıracağı
  adlarla tutarlı hale getirilecek: `autoIntake()`, `open()`,
  `close()`, `stop()`, `hasCube()` (mevcut Türkçe adlar yerine —
  kodun geri kalanıyla tutarlılık ve olası derleyici/IDE sorunlarını
  önlemek için, davranış aynı kalacak).
- `RelativeEncoder` tabanlı açı sınırlaması (`sagenco`/`solenco` ile
  ±0.20 tur sınırı) aynen korunacak.

### 3. `ClimbSubsystem` (yeni, iskelet)

CAD'de hook mekanizmasını tahrik eden motor/aktüatör net görünmüyor.
Bu subsystem **iskelet** olarak yazılacak: tek bir motor varsayımıyla
(`ClimbSubsystem` bir `SparkMax` + basit `extend()`/`retract()`/`stop()`
metodlarıyla), gerçek donanım netleşince genişletilecek. Otonom
kapsamına girmiyor (climb teleop'un son saniyelerinde elle tetiklenir).

## Proje Altyapısı

Şu an eksik olan, standart bir GradleRIO WPILib command-based proje
iskeleti eklenecek:

- `build.gradle`, `settings.gradle`, `.wpilib/wpilib_preferences.json`
- `vendordeps/REVLib.json` (NEO/SparkMax için)
- `RobotContainer.java` (command-based pattern: subsystem'ler +
  buton bağlamaları burada toplanacak, `Robot.java` sadece
  `TimedRobot` yaşam döngüsünü yönetecek)
- `Constants.java` (mevcut `Constant.java`'nın yerini alacak, doğru
  isimlendirme ve `public static final` alanlarla)

Swerve drivetrain kodu bu spec'in kapsamı **dışında** — mevcut
`Robot.java`'daki basit tank-drive mantığı (4 motor, arcade drive)
şimdilik yerinde bırakılacak, swerve'e geçiş ayrı bir iş.

## Test Stratejisi

Gerçek robot donanımı olmadan test edilebilecek kısımlar:
- `ElevatorSubsystem`'in PID/feedforward hesaplama mantığı, mock/simüle
  encoder değerleriyle birim testi (WPILib `SimHooks` /
  `HAL.initialize` simülasyon desteğiyle).
- `IntakeSubsystem`'in açma/kapama karar mantığı (encoder pozisyonuna
  göre motor komutu üretme) — saf fonksiyon olarak test edilebilir.

Gerçek donanım gerektiren kısımlar (encoder kalibrasyonu, gerçek
yükseklik doğrulama) robotta elle test edilecek; kod bunun için net
`TODO` işaretleri taşıyacak.

## Kapsam Dışı (sonraki spec'ler)

- Swerve drive kodu (mevcut kod tank-drive, offseason'da swerve'e
  geçiş ayrı bir mimari iş).
- Jetson Orin ↔ RoboRIO NetworkTables entegrasyonu, YOLO tabanlı engel
  tespiti, AprilTag pipeline, dinamik path planning (PathPlanner) —
  önceki konuşmada karara bağlanan otonom mimarisi, bu subsystem'ler
  tamamlandıktan sonra ayrı bir spec'te ele alınacak.
- Climb mekanizmasının gerçek donanım detayları (motor tipi, aktüasyon
  yöntemi) netleşmedi; `ClimbSubsystem` iskelet kalacak.
