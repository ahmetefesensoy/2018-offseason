# Hardware target / Donanım hedefi

Selected development target: **Jetson Orin Nano Super 8 GB + USB 3 RealSense D455 + NVMe**. This is a design target, not a purchase or a claim of bench validation. The desktop simulator does not require this hardware.

Geliştirme hedefi Orin Nano Super 8 GB, USB 3 D455 ve NVMe olarak seçildi. Donanım henüz bağlanmadı. Satın alma işlemi yapılmadı; saha/robot güç ve mekanik yerleşimi ayrıca ölçülmelidir.

NVIDIA's [Orin Nano specs](https://www.nvidia.com/en-gb/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/) describe the 8 GB module, Gigabit Ethernet, USB 3 and NVMe support. [JetPack downloads](https://developer.nvidia.com/embedded/jetpack/downloads) list JetPack 7.2.1 / Ubuntu 24.04 and Orin-family support as of this review. The [RealSense ROS wrapper](https://github.com/realsenseai/realsense-ros) lists Jazzy support; exact D455 firmware/librealsense/JetPack compatibility has not been tested here. Pin those versions after the first hardware acceptance run. Isaac ROS is not required by the HSV backend.

## Camera integration / Kamera bağlantısı

Install the camera manufacturer's ROS wrapper following its current instructions. Enable aligned depth with explicit namespace/name:

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera \
  align_depth.enable:=true enable_sync:=true
```

The detector subscribes to:

| ROS topic | Meaning |
|---|---|
| `/camera/camera/color/image_raw` | BGR-compatible color image |
| `/camera/camera/aligned_depth_to_color/image_raw` | Depth aligned to color pixels |
| `/camera/camera/color/camera_info` | Intrinsics for those pixels |

`16UC1` uses the configured integer depth scale (default 0.001 m/unit); `32FC1` is already in metres. The camera optical frame must have a calibrated TF chain to `map`. The URDF's existing camera mounting transform is a placeholder. Never use unaligned depth with color-image intrinsics.

Varsayılan `backend: hsv` ağırlık dosyası istemez. Işık değişimi ve sarı başka nesneler yanlış tespit üretebilir; gerçek görüntülerde doğruluk ölçülmedi. `backend: yolo` seçimi için metadata sözleşmesine uyan, lisansı uygun bir model eğitilmeli ve Jetson üzerinde TensorRT çıktısı sınanmalıdır. Başka bir veri setinin veya modelin lisansı varsayılmadı.

For a real launch, set matching team numbers, calibrate offsets and camera TF, then use `use_fake_roborio:=false use_synthetic_perception:=false use_detector:=true`. The current bridge is NT4 for the preserved 2025 robot code; it does not provide a tested WPILib 2018/NT3 connection.

## Boot service / Açılış servisi

`jetson/systemd/frc-autonomy.service` is a target-side template, not an installed service on this PC. It expects a dedicated `frcrobot` user, source at `/opt/frc-autonomy/repo`, and a built workspace/venv at `/opt/frc-autonomy/ws`. Create/configure those paths on the Jetson, install the camera driver, verify a manual foreground launch, then install the unit:

```bash
sudo install -m 644 jetson/systemd/frc-autonomy.service /etc/systemd/system/frc-autonomy.service
sudo systemctl daemon-reload
sudo systemctl enable --now frc-autonomy
journalctl -u frc-autonomy -f
```

`/etc/frc-autonomy.env` may set `FRC_AUTONOMY_WS`, `FRC_ROS_DISTRO` and `ROS_DOMAIN_ID`. Robot/team configuration and camera calibration must be completed before enabling this service. The service has not been executed on a Jetson during this review.

Bu servis yalnız hedef Jetson içindir; WSL veya bu bilgisayara kurulmadı. İlk elle kamera/robot doğrulaması yapılmadan açılışa eklenmemelidir. Yapılandırma dosyaları robot örneğinden gerçek takım ve ölçülen sensör değerlerine uyarlanmalıdır.
