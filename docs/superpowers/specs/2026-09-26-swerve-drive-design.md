# 2018 Offseason Robotu — Swerve Drive Tasarımı

## Amaç ve Kapsam

Bu robotun mevcut sürüş sistemi basit bir tank-drive (4 düz motor,
arcade mixing) — CAD'de görülen gerçek şase ise 4 köşeli bir **swerve
drive**. Bu spec, mevcut `RobotContainer`'daki tank-drive kodunu tamamen
kaldırıp, gerçek swerve donanımını süren bir subsystem yapısı kurmayı
kapsıyor. Bu, otonom modun (Jetson Orin + görüntü işleme tabanlı
dinamik yol planlama) temel bağımlılığı — otonom, swerve olmadan
anlamlı bir hareket kabiliyeti kazanamaz. Jetson/vision entegrasyonu bu
spec'in **dışında**, swerve tamamlandıktan sonra ayrı bir spec'te ele
alınacak.

## Donanım

- **4 modül**: FL (front-left), FR (front-right), BL (back-left), BR
  (back-right).
- **Motorlar**: her modülde 2× NEO (SparkMax controller) — biri drive
  (tekerleği döndürür), biri steer/azimuth (tekerleğin yönünü çevirir).
  Mevcut Intake/Elevator/Climb subsystem'leriyle aynı motor/controller
  ailesi (REVLib), ekstra vendordep gerekmiyor.
- **Steer açısı ölçümü**: REV Through Bore Encoder, her modülün steer
  motoruna bağlı, SparkMax'ın data port'u (duty-cycle) üzerinden okunur
  — mutlak açı verir, NEO'nun kendi relative enkoderi mekanik redüksiyon
  nedeniyle bu iş için güvenilir değil.
- **Gyro**: NavX (Kauai Labs), SPI/I2C üzerinden RoboRIO'ya bağlı — saha
  üzerindeki robot yönünü (heading) verir, field-relative sürüş ve
  odometry için gerekli. Yeni bir vendordep (`studica`/NavX JSON)
  eklenmesi gerekiyor.
- **CAN ID planı** (mevcut 1-9 aralığıyla çakışmasın diye 10'dan
  başlıyor):

  | Modül | Drive motor | Steer motor |
  |---|---|---|
  | Front-Left (FL) | 10 | 11 |
  | Front-Right (FR) | 12 | 13 |
  | Back-Left (BL) | 14 | 15 |
  | Back-Right (BR) | 16 | 17 |

## Yazılacak Yapı

### 1. `SwerveModule` (yeni)

Tek bir modülün donanımını (2 SparkMax + Through Bore Encoder) sarmalar.

**Sorumluluk:** Hedef bir `SwerveModuleState` (hız + açı) verildiğinde,
drive motorunu o hıza, steer motorunu o açıya getirir.

**Arabirim:**
```java
public class SwerveModule {
    public SwerveModule(int driveMotorPort, int steerMotorPort, /* ... */);
    public void setDesiredState(SwerveModuleState desiredState);
    public SwerveModuleState getState();      // mevcut hız+açı
    public SwerveModulePosition getPosition(); // mevcut mesafe+açı (odometry için)
    public void stop();
}
```

- Steer kontrolü: `ProfiledPIDController` (açısal, wrap-around/-180°..180°
  farkı hesaba katan `MathUtil.inputModulus` ile), hedef açıya en kısa
  yoldan döner.
- Drive kontrolü: basit `PIDController` + `SimpleMotorFeedforward`,
  hedef hıza (m/s) yakınsar.
- **Optimizasyon**: `SwerveModuleState.optimize()` (WPILib standart) ile,
  modül 180°'den fazla dönmek yerine motor yönünü tersine çevirip daha
  az dönerek aynı harekete ulaşır.

### 2. `SwerveDriveSubsystem` (yeni)

4 modülü ve NavX'i yönetir, odometry tutar.

**Arabirim:**
```java
public class SwerveDriveSubsystem extends SubsystemBase {
    public void drive(double xSpeedMps, double ySpeedMps, double rotRadPerSec, boolean fieldRelative);
    public Pose2d getPose();
    public void resetOdometry(Pose2d pose);
    public void stop();
    // periodic(): odometry güncellemesi (SwerveDrivePoseEstimator)
}
```

- `WPILib SwerveDriveKinematics` ile 4 modülün fiziksel konumlarından
  (chassis merkezine göre x/y offsetleri — CAD'den gerçek ölçüler
  alınacak, şimdilik placeholder kare düzen varsayımıyla) `ChassisSpeeds`
  → 4× `SwerveModuleState` dönüşümü yapılır.
- `SwerveDrivePoseEstimator` ile odometry: 4 modülün `SwerveModulePosition`
  verisi + NavX heading'i füzyonlanır. (AprilTag entegrasyonu, önceki
  otonom mimarisi tartışmasında karara bağlandığı gibi, Jetson/otonom
  spec'inde eklenecek — bu spec'te odometry sadece encoder+gyro'ya
  dayanıyor.)

### 3. `RobotContainer` Değişiklikleri

- Mevcut 4 düz `SparkMax` drivetrain alanı (`driveLeft1/2`,
  `driveRight1/2`) ve `driveWithJoystick()` metodu **tamamen
  kaldırılacak**.
- Yeni `SwerveDriveSubsystem` alanı eklenecek.
- Joystick eksenleri field-relative swerve'e bağlanacak: eksen 1
  (ileri/geri) → x hızı, eksen 0 (sağa/sola) → y hızı, eksen 4
  (döndürme) → açısal hız. Değerler `SwerveDriveSubsystem.drive(...)`'a
  `fieldRelative=true` ile geçirilecek.
- `Constants.java`'ya yeni swerve CAN ID'leri, modül fiziksel offsetleri
  (placeholder), max hız/açısal hız sabitleri eklenecek.

## Test Stratejisi

Gerçek robot donanımı olmadan test edilebilecek kısımlar:
- `SwerveModule.setDesiredState()`'in optimizasyon mantığı (hedef açı
  180°'den fazla farksa motor yönünü tersine çevirip daha kısa yoldan
  gitmesi) — saf hesaplama, HAL simülasyonuyla test edilebilir.
- `SwerveDriveSubsystem.drive()`'ın `ChassisSpeeds` → 4×
  `SwerveModuleState` dönüşümü — kinematik hesaplama, encoder/gyro
  donanımı gerektirmez.
- Field-relative dönüşüm: verilen bir heading açısıyla x/y hız
  vektörünün doğru döndürüldüğü.

Gerçek donanım gerektiren kısımlar (motor CAN ID doğrulaması, encoder
kalibrasyonu, NavX bağlantısı, gerçek modül fiziksel offsetleri) robotta
elle test edilecek; kod bunun için net `TODO` işaretleri taşıyacak.

## Kapsam Dışı (sonraki spec)

- Jetson Orin ↔ RoboRIO NetworkTables entegrasyonu, YOLO tabanlı engel
  tespiti, AprilTag pipeline, PathPlanner ile dinamik path planning —
  bu swerve subsystem'i tamamlandıktan sonra ayrı bir spec'te ele
  alınacak.
- Modüllerin gerçek fiziksel offsetleri (chassis merkezine göre x/y
  mesafeleri) CAD'den kesin ölçülerle doğrulanana kadar placeholder
  kalacak.
- Steer/drive PID gain'leri ve max hız sabitleri gerçek robotta
  kalibre edilecek, şimdilik makul varsayılan değerlerle başlanacak.
