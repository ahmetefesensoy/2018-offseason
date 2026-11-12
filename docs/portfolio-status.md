# Verified state / Doğrulanmış durum — 2026-11-12

The working tree began at `0e756be` on `feature/ros2-autonomy-foundation`. The remote repository was already public; no new robot repository was needed. The owner describes the source as personal experimental work never used in competition.

| Verified item | Result |
|---|---|
| Language/API | Java, WPILib/GradleRIO 2025.1.1, Java 17, NT4 |
| Historical WPILib 2018 source | Not found in available Git refs |
| FMS `getGameSpecificMessage()` | Not present in current source |
| Field-side selection | ROS configuration parameters |
| Windows Gradle `test build` | Passed |
| Python core | 72 tests passed on WSL Ubuntu 24.04 |
| ROS distribution actually tested | Jazzy |
| ROS packages built | Nine |
| Live movement | 7.6163 m observed over a 20-second capture |
| Robot/camera/Jetson bench test | Not performed; hardware not connected |
| Third-party CAD 3028 | Not downloaded; reference still needs identification/license |

Fixes made during real launch validation: explicitly typed ROS string-array parameters, a tiny post-rotation velocity margin before the strict bridge gate, sensor-data QoS for camera subscriptions, aligned depth topic configuration, and automatic metre/millimetre depth decoding.

Sürüm kararı: kullanıcı 2018 POWER UP oyununu koruyup mevcut WPILib 2025 uygulamasıyla devam etmeyi seçti. Geçmişte bulunmayan bir WPILib sürümü için `legacy-2018` etiketi oluşturulmaz. Değişiklik öncesi commit `0e756be` korunur.

Older Docker/design files refer to ROS 2 Lyrical/Isaac ROS. They were not executed during this review. The tested WSL scripts explicitly target Jazzy; no claim of validated JetPack, Isaac ROS or camera compatibility is made from the desktop run.

Full-history Gitleaks 8.30.1 scan covered all available refs (53 commits at the baseline) with no findings. The pre-existing STEP file's license is unknown and is excluded from the new source-code license. No model weights or datasets were added.
