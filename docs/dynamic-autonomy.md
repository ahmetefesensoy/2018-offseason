# Dinamik Otonom — Hızlı Çalıştırma

Bu profil strateji seçimi, hareketli engel rotası, bağımsız çarpışma durdurma,
intake/elevator komutu, NT4 watchdog ve RViz açıklamasını tek akışta çalıştırır.
Motor yetkisi her zaman roboRIO'dadır; Jetson yalnız süreli ve sınırlandırılmış
`AutonomyCommand` gönderir.

## En hızlı doğrulama

ROS kurulumu olmadan karar, rota, detour ve acil durdurma çekirdeğini çalıştır:

```bash
cd jetson/ros2_ws
source install/setup.bash
ros2 run frc_bringup offline_autonomy_demo
```

Başarılı çıktı `pickup:cube-demo`, `detour: true`, `emergency_stop: true` ve
`deterministic_replay: true` içerir.

## Sentetik uçtan uca demo

Jetson veya ROS 2 bilgisayarında:

```bash
cd jetson/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch frc_bringup dynamic_autonomy.launch.py
```

Bu komut sahte roboRIO, sentetik kamera algısı, dünya modeli, takım rezervasyonu,
strateji motoru, dinamik navigator ve RViz'i açar. RViz'de `Dynamic Plan`,
`Tracked Perception World`, `Alliance Reservations` ve karar yazısı görünür.

Docker ile aynı profil:

```bash
docker compose -f jetson/compose.yaml build
docker compose -f jetson/compose.yaml up
```

## Gerçek Jetson modeli

1. ONNX veya TensorRT engine dosyasını Jetson'da `/models/power_up.engine` olarak
   yerleştir.
2. JetPack'in CUDA uyumlu PyTorch/TensorRT kurulumu hazırken
   `python3 -m pip install -r jetson/requirements-detector.txt` çalıştır.
3. `model.example.json` dosyasını `/models/power_up.json` adına kopyala.
4. Engine SHA-256 değerini metadata içindeki `artifact_sha256` alanına yaz.
5. Sınıf sırasını değiştirme: `power_cube`, `robot`, `switch_plate`,
   `scale_plate`, `vault_opening`.
6. Kamera topic adlarını `frc_perception/config/detector.yaml` içinde doğrula.

Gerçek kamera ve gerçek roboRIO ile:

```bash
ros2 launch frc_bringup dynamic_autonomy.launch.py \
  use_fake_roborio:=false \
  use_synthetic_perception:=false \
  use_detector:=true
```

`frc_bringup/config/autonomy.yaml` içindeki `team_number` gerçek FRC takım
numarasına ayarlanmalıdır. Model dosyası, metadata hash'i veya TF dönüşümü yanlışsa
detector başlamaz; bu bilinçli fail-closed davranıştır.

## Güvenli saha sırası

İlk çalıştırmayı robot blok üzerindeyken yap. Önce `/strategy/decision`,
`/navigation/path` ve `/navigation/state` topic'lerini gözle; ardından Driver
Station autonomous/test iznini ver. Kamera/world/goal 300–500 ms bayatlarsa
navigator disarm komutu yollar. Hareketli robot güvenlik zarfına girerse planner'dan
bağımsız Collision Monitor sıfır hız ve disarm üretir. RoboRIO ayrıca 150 ms
watchdog, NavX, drivetrain, sıra numarası ve zaman kontrollerini uygular.

## Önemli kalibrasyonlar

- Gerçek swerve encoder offsetleri ve NavX işareti
- `base_link -> camera_link` extrinsic ve kamera intrinsics
- Robot ayak izi/radius
- Elevator metre/encoder dönüşümü
- Intake açma-kapama yönleri
- Switch/scale yaklaşma pozları

Bu değerler doğrulanmadan robot yerde yüksek hızda çalıştırılmamalıdır.
