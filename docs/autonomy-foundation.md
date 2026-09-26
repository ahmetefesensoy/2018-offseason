# ROS 2 Otonom Temeli — Kurulum ve Demo Rehberi

Bu faz, Jetson Orin'in güvenilmeyen/yüksek seviye planlayıcı; roboRIO'nun ise tek
motor otoritesi olduğu güvenli kontrol zincirini kurar. Gerçek kamera inference,
VSLAM, Nav2 ve skor stratejisi sonraki fazlardır. Bu temel tamamlanmadan onların
hiçbiri gerçek robotu süremez.

Perception ve takip çekirdeği artık `docs/perception-world-model.md` içindeki Faz
2 demosuyla eklenmiştir. Gerçek kamera/YOLO/VSLAM adaptörleri henüz bağlı değildir.

Maç öncesi takım arkadaşı rotaları, zaman bağımlı rezervasyonlar, RViz çakışma
görselleştirmesi ve yerel Strategy Studio artık `docs/alliance-playbook.md`
içindeki Faz 3 akışıyla eklenmiştir. Bu katman henüz hareket komutu üretmez.

## Şu anda çalışan zincir

```text
/autonomy/command (ROS 2)
        |
        v
frc_nt_bridge -- NT4 primitive frame, commit_sequence en son
        |
        v
roboRIO AutonomySafetyGate
  DS enabled + autonomous/test mode
  boot session + artan sequence
  <=100 ms yaş + <=150 ms geçerlilik
  hız/NaN/NavX/swerve kontrolleri
        |
        v
120 ms watchdog + ivme sınırlayıcı + robot-relative swerve
        |
        v
/autonomy/status -> TF/Odometry/Path/Marker -> RViz + rosbag
```

Jetson motor CAN hattına bağlanmaz. NetworkTables koparsa veya yeni geçerli
çerçeve 120 ms içinde gelmezse roboRIO swerve'i durdurur. Driver Station disable,
teleop geçişi, test başlangıcı ve komut iptali de latched hızı temizler.

## Gerçek robottan önce doldurulacak değerler

1. `build.gradle` içindeki `team = 0` gerçek FRC takım numarasıyla değiştirilmelidir.
2. `jetson/ros2_ws/src/frc_bringup/config/autonomy.yaml` içindeki `team_number: 0`
   aynı takım numarası yapılmalıdır. Bu durumda köprü `setServerTeam()` kullanır.
3. SparkMax CAN ID, motor yönü, absolute encoder yönü ve offset'leri
   `docs/swerve-commissioning.md` akışıyla fiziksel robotta doğrulanmalıdır.
4. Xacro'daki `camera_mount` dönüşümü kameranın ölçülen montaj konumuna göre
   güncellenmelidir; mevcut konum yalnız görselleştirme başlangıç noktasıdır.
5. CAD incelemesinde bumper zarfı `899.46 × 899.46 mm`, alt tabla yaklaşık
   `703.48 × 700.94 mm`, swerve alt-grup yerleşim farkı yaklaşık `524.7 mm`
   bulundu. Kontrol kinematiğindeki değer fiziksel pivot-pivot ölçümü yapılmadan
   sadece CAD bounding-box sonucuyla değiştirilmemelidir.

CAD ölçülerini tekrar üretmek için FreeCAD 1.1 ile:

```bash
FreeCADCmd tools/inspect_step.py --pass "Assembly 1.step"
```

Türkçe/boşluklu Windows yolunda STEP eklentisi sorun çıkarırsa dosyaları kısa,
ASCII bir geçici dizine kopyalayıp aynı komutu orada çalıştırın.

## roboRIO derleme ve deploy

Windows PowerShell:

```powershell
$env:JAVA_HOME='C:\Program Files\Eclipse Adoptium\jdk-17.0.20.101-hotspot'
.\gradlew.bat clean test build --console=plain
.\gradlew.bat deploy
```

Deploy öncesinde robot bloklar üzerinde olmalı, mekanizmalar boş olmalı ve bir
operatör Driver Station disable/E-stop kontrolünün başında bulunmalıdır.

## Jetson Orin ortamı

Faz 1 container'ı resmi ARM64 ROS 2 Lyrical `ros-base` imajının doğrulanmış
digest'ine sabitlenmiştir. NVIDIA'nın güncel Isaac ROS 5.0 yolu Jetson Orin için
JetPack 7.2 ve en az 128 GB NVMe önerir; GPU perception paketleri Faz 2'de Isaac
ROS CLI ortamına eklenecektir.

Jetson'da repository kökünden:

```bash
cd jetson
xhost +local:docker
docker compose build
docker compose up
```

GUI'siz gerçek robot çalıştırması için `compose.yaml` içindeki launch çağrısına
`use_rviz:=false` eklenebilir. Host network kullanıldığı için Jetson ile roboRIO
aynı robot ağında olmalı; NT4 TCP/UDP trafiği VLAN veya firewall tarafından
engellenmemelidir.

Container kullanmadan geliştirme:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r jetson/requirements.txt
source /opt/ros/lyrical/setup.bash
cd jetson/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch frc_bringup foundation.launch.py
```

## Donanımsız tam demo

Bu mod gerçek roboRIO yerine localhost'ta bir NT4 sunucusu, planar robot modeli
ve açıkça arm edilmesi gereken yavaş bir figure-eight komut kaynağı başlatır:

```bash
ros2 launch frc_bringup foundation.launch.py \
  use_fake_roborio:=true \
  use_fake_autonomy:=true \
  arm_fake_autonomy:=true \
  use_synthetic_perception:=true
```

RViz'de şunlar görünür:

- CAD ölçülü taban/bumper ve dört swerve tekeri;
- `map -> base_footprint -> base_link` TF zinciri;
- roboRIO/sim pose odometrisi ve yürütülen iz;
- aktif/pasif komut, sequence ve reject reason metni;
- sonraki fazlar için global/local plan, point cloud ve karar marker katmanları.
- takım arkadaşı rotaları, güvenlik koridorları, çakışma ve sapma marker'ları.

`arm_fake_autonomy` yalnız simülasyonda `true` yapılmalıdır. Gerçek robot launch'ı:

```bash
ros2 launch frc_bringup foundation.launch.py \
  use_fake_roborio:=false \
  use_fake_autonomy:=false
```

Gerçek planlayıcı `/autonomy/command` mesajında `armed=true` üretmeden robot
hareket etmez. Köprü roboRIO'nun yayınladığı boot-session kimliğini kendisi ekler.

## İzlenecek ROS ve NT4 alanları

ROS 2:

```bash
ros2 topic echo /autonomy/status
ros2 topic hz /autonomy/status
ros2 topic echo /autonomy/command
ros2 run tf2_ros tf2_echo map base_link
```

Kritik durum alanları:

- `command_active`: roboRIO'nun o anda geçerli hız tuttuğunu gösterir;
- `accepted_sequence`: roboRIO'nun kabul ettiği son monoton sıra;
- `command_age_us`: Jetson komutunun roboRIO saatindeki yaşı;
- `reject_reason`: `DISARMED`, `STALE_COMMAND`, `SESSION_MISMATCH`,
  `GYRO_UNHEALTHY`, `WATCHDOG_EXPIRED` gibi sabit kod;
- `gyro_healthy` ve `drivetrain_healthy`: hareket interlock'ları.

## Rosbag

Workspace source edildikten sonra:

```bash
./jetson/scripts/record_autonomy.sh /bags/foundation_demo
```

Kayıt listesi `frc_bringup/config/record_topics.txt` içindedir. Kamera, point
cloud ve plan topic'leri henüz yayınlanmasa da listeye şimdiden eklenmiştir; Faz
2–5 geldiğinde demo/tekrar üretim komutu değişmez.

## Fail-safe kabul denemeleri

Her deneme önce blok üzerinde, sonra boş ve bariyerli düşük hızlı alanda yapılır:

1. Robot autonomous enabled ve geçerli akıştayken Jetson ethernetini çıkarın;
   teker komutu en geç 150 ms içinde sıfıra gitmelidir.
2. `armed=false` gönderin; `reject_reason=DISARMED` ve `command_active=false`
   görülmelidir.
3. Driver Station'dan disable ve teleop'a geçin; hareket anında kesilmelidir.
4. Eski session veya tekrar sequence çerçevesi enjekte edin; kabul edilmemelidir.
5. NavX bağlantısını yalnız blok üzerindeki kontrollü testte kesin;
   `GYRO_UNHEALTHY` hareketi engellemelidir.
6. NaN, 0.75 m/s üzeri toplam translation veya 1.5 rad/s üzeri rotation komutu
   reddedilmelidir.
7. Rosbag'i tekrar oynatırken motor çıkışı kullanılmamalı; replay yalnız sim veya
   shadow modunda yapılmalıdır.

Bu testler geçmeden perception veya strateji düğümüne gerçek robotu arm etme
yetkisi verilmez.
