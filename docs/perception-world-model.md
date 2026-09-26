# Perception ve Dünya Modeli — Faz 2

Bu faz, kamera veya model markasına bağlı olmadan semantik gözlemleri takip edilen
oyun nesnelerine dönüştürür. Çıktı yalnız ROS ve RViz'e gider; hiçbir Faz 2 düğümü
`/autonomy/command` yayınlamaz ve robotu hareket ettiremez.

## Veri akışı

```text
Kamera + detector + depth/VSLAM adaptörü
              |
              v
/perception/observations  (tek atomik frame, map koordinatı)
              |
              v
frc_world_model
  doğrulama -> association -> velocity -> covariance -> 2 s prediction
              |
              +----> /world/state
              |
              +----> world_visualizer -> /world/markers -> RViz
```

Tracker aynı sınıftaki en yakın uyumlu izi deterministik olarak eşler. Robotlarda
`ALLY`, `OPPONENT` ve `UNKNOWN` ayrımı korunur. Robot tahminleri 100 ms aralıkla
iki saniye ileri gider; her adımda kovaryans büyür. Bir perception frame'i
bozuksa snapshot kısmen güncellenmez ve version artırılmaz.

Kaynak 250 ms boyunca güncellenmezse `/world/state.perception_fresh=false` olur
ve RViz marker'ları griye döner. Track'ler 750 ms sonunda silinir. Sonraki
strateji ve navigasyon fazları `perception_fresh=false` durumunu hareket izni
olarak kullanamaz.

## Donanımsız RViz demosu

Jetson/container workspace'i derlendikten sonra:

```bash
ros2 launch frc_bringup foundation.launch.py \
  use_fake_roborio:=true \
  use_fake_autonomy:=false \
  use_synthetic_perception:=true
```

Bu senaryo üç sabit Power Cube, düz hatta ilerleyen bir takım robotu ve koridoru
kesen bir rakip üretir. RViz'deki `Tracked Perception World` katmanı şunları
gösterir:

- turuncu Power Cube kutuları ve confidence etiketleri;
- mavi takım, kırmızı rakip ve sarı bilinmeyen robot ayak izleri;
- hız okları;
- iki-sigma kovaryans alanları;
- iki saniyelik tahmin çizgileri.

Ham ve işlenmiş topic'leri izlemek için:

```bash
ros2 topic hz /perception/observations
ros2 topic echo /world/state
ros2 topic echo /world/markers
```

Freshness davranışını göstermek için ana launch'ta
`use_synthetic_perception:=false` kullanın ve sentetik kaynağı ayrı terminalde
başlatın:

```bash
ros2 run frc_world_model synthetic_perception
```

Bu süreci durdurduktan yaklaşık 250 ms sonra world state stale olmalı, yaklaşık
750 ms sonra sentetik track'ler kaldırılmalıdır.

## Gerçek kamera ve model adaptörü sözleşmesi

Gerçek perception adaptörü aşağıdaki koşulların tamamını sağlamalıdır:

1. Kamera `CameraInfo` ile doğrulanmış intrinsics/distortion yayınlar.
2. Ölçülmüş `base_link -> camera_link -> camera_optical_frame` dönüşümü URDF'e
   yazılır; mevcut kamera transformu yalnız placeholder'dır.
3. Renk ve depth/stereo frame'leri donanım timestamp'iyle senkronize edilir.
4. Detector çıktısı depth median ve kamera modeliyle 3B konuma çevrilir.
5. VSLAM/field-localization sonucu kullanılarak her konum `map` koordinatına
   dönüştürülür; world model kamera-frame gözlem kabul etmez.
6. Her frame tek `SemanticObservationArray` olarak ve kesin artan timestamp ile
   yayınlanır. Frame içindeki tüm detection'lar aynı timestamp'i taşır.
7. `pose.covariance[0]` ve `[7]` metre-kare cinsinden doldurulur; bilinmeyen
   kovaryans sıfır yazılarak güvenliymiş gibi gösterilmez.
8. `class_name` yalnız sürümlü model metadata'sındaki sınıflardan biri olur:
   `power_cube`, `robot`, `switch_plate`, `scale_plate`, `vault_opening`.
9. Robot bumper rengi/roster kararsızsa `AFFILIATION_UNKNOWN` kullanılır.
10. Model metadata; dataset sürümü, sınıf sırası, giriş boyutu, ONNX SHA-256 ve
    TensorRT engine SHA-256 değerlerini taşır. Eşleşme yoksa node aktive olmaz.

Önerilen metadata örneği:

```yaml
schema_version: 1
dataset_version: frc-power-up-2018-v1
classes: [power_cube, robot, switch_plate, scale_plate, vault_opening]
input_shape: [1, 3, 640, 640]
precision: fp16
onnx_sha256: "64-character lowercase hash"
engine_sha256: "64-character lowercase hash"
```

Isaac ROS 5.0 YOLOv8 adaptörü ONNX dosyasını Jetson'da `trtexec` ile TensorRT
engine'e dönüştürebilir. Nvblox adaptörü depth görüntüsü ve pose girdisinden mesh,
ESDF ve sonraki Nav2 fazında kullanılacak 2B costmap üretir. Bu iki GPU hattı,
kamera modeli ve doğrulanmış engine artifact'i seçildikten sonra ayrı lifecycle
node'ları olarak eklenmelidir:

- YOLOv8: https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_object_detection/isaac_ros_yolov8/index.html
- Nvblox: https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_nvblox/index.html

## Rosbag ve tekrar üretilebilirlik

Standart kayıt komutu perception girdisini, atomik world snapshot'ını ve RViz
marker'larını birlikte kaydeder:

```bash
./jetson/scripts/record_autonomy.sh /bags/perception_demo
```

Aynı zaman sıralı observation frame'leri aynı config ile tekrar verildiğinde
track ID sırası ve snapshot sırası deterministiktir. Model inference sonucu
değişebileceği için acceptance rosbag'iyle detector çıktısı ayrıca altın veri
olarak saklanmalıdır.

## Bu fazın bilinçli sınırı

Repository şu anda gerçek YOLO engine, eğitim veri kümesi, kamera sürücüsü veya
VSLAM kalibrasyonu içermez. Bu artifact'ler kamera ve Jetson modeli belli olmadan
uydurulamaz. Mevcut kod onların güvenli ve test edilebilir giriş sözleşmesini,
tracking/world-state çekirdeğini ve sunulabilir RViz demonstrasyonunu sağlar.
