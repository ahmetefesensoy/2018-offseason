# 2018 FRC Robotu — ROS 2 Stratejik Otonom Tasarımı

## 1. Amaç

Bu sistem 2018 FIRST Power Up sahasında yalnız bir hedefe gitmek yerine maç
durumunu anlayan, takım arkadaşlarının önceden paylaşılmış stratejilerini hesaba
katan, rakiplerin muhtemel hareketlerinden kaçınan ve beklenen ittifak skorunu
sürekli eniyileyen deneysel bir tam otonom yığın olacaktır.

Jetson Orin algılama, dünya modeli, strateji ve hareket planlamasını çalıştırır.
RoboRIO; swerve, mekanizma kontrolü ve son güvenlik yetkisini elinde tutar. ROS 2
hiçbir zaman motor kontrolcülerine doğrudan erişmez. Sistem, neden belirli bir
görevi ve yolu seçtiğini RViz üzerinde canlı ve kayıt tekrarında açıklayabilir.

Bu tasarım araştırma/demonstrasyon amaçlı tam maç otonomunu destekler. Resmî
2018 maç ağı kuralları nedeniyle takım robotları arasında canlı kablosuz ROS
haberleşmesi yapılmaz. Takım arkadaşlarının niyetleri maçtan önce görsel editör,
PathPlanner dosyası veya log içe aktarma yoluyla yüklenir; sahadaki gerçek
sapmalar yalnız robot üstü algılamayla güncellenir.

## 2. Başarı Tanımı

Sistem tamamlandığında aşağıdaki gösterim tek kayıt oturumunda yapılabilmelidir:

1. İki takım arkadaşının planı görsel editör veya dosya içe aktarma ile yüklenir.
2. Robot; Power Cube, ittifak robotu, rakip robot ve saha engellerini algılar.
3. Aday görevler için beklenen puan, süre, risk ve takım çakışması hesaplanır.
4. Seçilen görev ve seçilmeyen en iyi iki alternatif RViz panelinde açıklanır.
5. Takım robotu planından sapar veya rakip koridora girerse robot 250 ms içinde
   stratejiyi yeniden değerlendirir; yerel hareket planlayıcı daha kısa sürede
   yavaşlatır, durur veya kaçış yörüngesi üretir.
6. Jetson bağlantısı, komut akışı veya kritik sensör kaybolursa RoboRIO 150 ms
   içinde sürüş komutunu sıfırlar.
7. Aynı rosbag ve aynı rastgelelik tohumu tekrar oynatıldığında aynı stratejik
   karar dizisi üretilir.

## 3. Kapsam ve Fazlar

Tek bir uygulama planına sığmayacak olan çalışma, aşağıdaki bağımsız ve
gösterilebilir alt projelere bölünür:

### Faz 0 — Hareket Temeli

Mevcut `2026-09-26-swerve-drive-design.md` ve
`2026-09-26-neo-sparkmax-swerve.md` planındaki NEO/SparkMax/Through Bore/NavX
swerve tamamlanır. Bu faz otonom kod içermez.

### Faz 1 — ROS Temeli ve Güvenli RoboRIO Köprüsü

Robot modeli, mesaj paketleri, NT4 köprüsü, heartbeat/watchdog, sahte sensörler,
rosbag kayıt ve hafif simülasyon kurulur. Çıktısı, RViz’den verilen sınırlı hız
komutunu simülasyonda ve blok üzerindeki gerçek robotta güvenli biçimde yürüten
bir sistemdir.

### Faz 2 — Lokalizasyon, Algılama ve Dünya Modeli

Kamera sözleşmesi, VSLAM, derinlik haritası, YOLOv8 TensorRT, çoklu nesne takibi
ve oyun nesnesi dünya modeli eklenir. Çıktısı, sürüş yapmadan canlı sahayı RViz’de
doğru koordinatlarda gösteren bir perception-only demonstrasyonudur.

### Faz 3 — Alliance Strategy Studio ve Niyet Tahmini

Görsel editör, plan/log dönüştürücüleri, şema doğrulama ve takım rotası
rezervasyonları eklenir. Çıktısı, iki takım planını bir sahneye yükleyip gerçek
takip verisiyle plan sapmasını gösterebilen bir demonstrasyondur.

### Faz 4 — Stratejik Karar Motoru

Görev üretici, skor modeli, rolling-horizon değerlendirici, Monte Carlo rollout,
Behavior Tree ve açıklama kaydı eklenir. Çıktısı, hareket etmeden kayıtlı bir
maçta doğru görev seçimlerini ve karşı-olgusal açıklamaları üreten sistemdir.

### Faz 5 — Dinamik Navigasyon

Nav2 costmap katmanları, State Lattice global planlayıcı, MPPI Omni yerel
kontrolcü, Collision Monitor ve mekanizma görevleri birleştirilir. Çıktısı,
simülasyonda ve kontrollü sahada uçtan uca dinamik otonomdur.

### Faz 6 — Sunum, Dayanıklılık ve Ölçüm

Özel RViz paneli, senaryo koşucusu, performans raporları, hata enjeksiyonu,
donanım-döngü testi ve sunum profili tamamlanır.

## 4. Hedef Platform

### Jetson çalışma zamanı

- NVIDIA Jetson Orin.
- JetPack 7.2.
- ROS 2 Lyrical.
- Isaac ROS 5.0, TensorRT 10.16 ve CUDA 13.2.
- En az 128 GB NVMe; model, container katmanları ve rosbag kayıtları NVMe’de.
- `full` profil: AGX Orin veya en az 16 GB belleğe sahip Orin; YOLOv8s FP16,
  VSLAM, nvblox ve MPPI eşzamanlı.
- `constrained` profil: 8 GB Orin; YOLOv8n FP16, daha kaba voxel/costmap,
  nvblox dynamic katmanı kapalı ve kayıt görüntüsü sıkıştırılmış.

Isaac ROS container imajı sürüm digest’iyle sabitlenir. Container dışındaki tek
kalıcı veriler model dosyaları, kalibrasyon, konfigürasyon ve kayıt dizinidir.

### Simülasyon

- CI ve geliştirici bilgisayarında hızlı testler için Gazebo tabanlı 2B/3B saha.
- Fotogerçekçi sentetik veri ve sunum görüntüsü için isteğe bağlı Isaac Sim,
  ayrı RTX iş istasyonunda çalışır.
- Isaac Sim Jetson üzerinde çalıştırılmaz.
- Aynı ROS mesajları simülasyon, rosbag replay ve gerçek donanımda kullanılır.

## 5. Depo Yapısı

Mevcut GradleRIO Java projesi kökte kalır. Jetson kodu ayrı çalışma alanında
tutulur:

```text
src/main/java/frc/robot/                 # RoboRIO, swerve ve mekanizmalar
src/test/java/frc/robot/                 # RoboRIO testleri
jetson/
  compose.yaml                           # Sabitlenmiş çalışma servisleri
  ros_ws/src/
    frc_autonomy_msgs/                   # Mesaj, servis ve action tanımları
    frc_robot_description/               # URDF, CAD meshleri, 2018 saha modeli
    frc_nt_bridge/                       # ROS 2 <-> NT4 ve zaman senkronizasyonu
    frc_world_model/                     # İzler, oyun durumu ve TF otoritesi
    frc_perception/                      # Kamera, YOLO, depth, VSLAM adaptörleri
    frc_alliance_playbook/               # Şema, import ve rezervasyon üretimi
    frc_strategy/                        # Görev üretimi, rollout ve BT
    frc_navigation/                      # Nav2 config ve özel costmap katmanları
    frc_rviz_plugins/                    # Karar ve maç paneli
    frc_bringup/                         # Launch, lifecycle ve profil seçimi
    frc_sim/                             # Saha, rakip ve senaryo simülasyonu
    frc_evaluation/                      # Replay, metrik ve rapor üretimi
strategy_studio/                         # Yerel web tabanlı görsel plan editörü
models/                                  # Model metadata; büyük engine dosyaları Git LFS/artifact
config/                                  # Saha, skor, güvenlik ve donanım profilleri
scenarios/                               # Tekrarlanabilir test senaryoları
```

Her ROS paketi tek sorumluluğa sahiptir. Paketler birbirinin iç Python/C++
sınıflarını çağırmaz; yalnız sürümlü mesajlar, action’lar ve servislerle iletişim
kurar.

`config/robot_capabilities.yaml`; intake, elevator, vault teslimi, park ve climb
yeteneğini, erişilebilir yükseklikleri ve mekanizma sürelerini tanımlar. Görev
üretici yalnız etkin ve sağlık durumu iyi yeteneklerden görev oluşturur. Böylece
aynı yazılım, fiziksel climber bulunmayan bu robotta `CLIMB` üretmez; sonradan
eklenen bir mekanizma ise strateji kodunu değiştirmeden profile eklenebilir.

## 6. Koordinat Çerçeveleri ve Zaman

TF ağacı:

```text
map -> odom -> base_link -> camera_link -> camera_optical_frame
                         -> intake_link
                         -> elevator_link
```

- `map`: 2018 sahasının sabit koordinat sistemi; mavi ittifak köşesi tek kanonik
  orijindir. Kırmızı taraf girdileri içe aktarılırken dönüştürülür.
- `odom`: kısa vadede sürekli, sıçramayan yerel çerçeve.
- `base_link`: CAD’den çıkarılmış robot merkezi ve gerçek dikdörtgen ayak izi.
- RoboRIO odometrisi `/rio/odom_raw`, NavX verisi `/rio/imu` olarak yayınlanır.
- VSLAM `/vision/odom` üretir. Füzyon düğümü `odom -> base_link` otoritesidir.
- Mutlak saha düzeltmesi varsa yalnız `map -> odom` dönüşümünü günceller.

Jetson monotonic clock, RoboRIO FPGA timestamp’iyle köprü başlatılırken ve her
5 saniyede tekrar eşleştirilir. Her komut ve ölçüm kaynak zamanı, alınma zamanı,
sıra numarası ve yaş bilgisi taşır. Negatif veya geriye giden zaman damgası
paketi geçersiz kılar.

## 7. Mesaj Sözleşmeleri

`frc_autonomy_msgs` aşağıdaki sürümlü tipleri tanımlar:

### Dünya modeli

- `GameObject`: UUID, sınıf, poz, hız, kovaryans, güven, son görülme zamanı.
- `RobotTrack`: iz kimliği, `ALLY/OPPONENT/UNKNOWN`, ayak izi, poz/hız,
  kovaryans, niyet olasılıkları ve zamanlı tahmin pozları.
- `GameState`: maç fazı/süresi, ittifak rengi, switch/scale sahipliği, tahmini
  küp/vault durumu, skor, FMS game-specific field configuration ve veri tazelik
  bayrakları.
- `WorldState`: yukarıdakilerin tek atomik snapshot’ı ve artan sürüm numarası.

### Takım planı

- `AlliancePlan`: şema sürümü, saha sürümü, ittifak, plan kimliği ve robotlar.
- `RobotPlan`: takım/robot etiketi, başlangıç pozu, ayak izi, hız/ivme sınırları,
  başlangıç güveni ve `PlanSegment[]`.
- `PlanSegment`: `PICKUP`, `SCORE_SWITCH`, `SCORE_SCALE`, `DELIVER_VAULT`,
  `CROSS_LINE`, `PARK`, `CLIMB`, `WAIT`; hedef kimliği, en erken/geç başlama,
  beklenen süre, zamanlı polyline, koridor yarıçapı, fallback kimliği.
- `ReservationTube`: robot, başlangıç/bitiş zamanı, zamanlı polygon ve güven.

### Strateji

- `CandidateTask`: görev kimliği, önkoşullar, tahmini süre, başarı olasılığı,
  hedef ve gerekli mekanizma durumu.
- `UtilityBreakdown`: doğrudan puan, sahiplik puanı, ranking-point değeri,
  gelecek değeri, süre, çarpışma, başarısızlık, takım çakışması, kural/ceza ve
  belirsizlik bileşenleri.
- `DecisionTrace`: kullanılan `WorldState` sürümü, adaylar, seçilen görev,
  rastgelelik tohumu, rollout özeti ve insan-okunur karşı-olgusal açıklama.
- `ExecuteTask.action`: seçilen görevin navigasyon ve mekanizma yürütümü.

### RoboRIO komut köprüsü

- `/Autonomy/Command`: `vx`, `vy`, `omega`, sıra numarası, Jetson zamanı,
  geçerlilik süresi ve arm-session kimliği.
- `/Autonomy/Status`: kabul edilen son sıra, RoboRIO zamanı, robot modu,
  heartbeat yaşı, pose, ölçülen hız, mekanizma durumu ve hata bitleri.

NT4 üzerinde bu değerler ayrı primitive topic’ler olarak yayınlanır; ROS mesajı
köprü içinde atomik snapshot’a çevrilir. Komut hızları m/s ve rad/s’dir.

RoboRIO, Driver Station game-specific message değerini ayrıştırır ve ham değerle
birlikte doğrulanmış `our_switch_side`, `scale_side` ve `opponent_switch_side`
alanlarını yayınlar. Mesaj eksik veya geçersizse saha tarafına bağlı görevler
üretilmez; sistem güvenli `CROSS_LINE`, `WAIT` veya capability profilinin izin
verdiği tarafsız görevlere düşer.

## 8. Alliance Strategy Studio

Studio maçtan önce yerel bilgisayarda çalışan bir web uygulamasıdır; maçta ağ
bağlantısına ihtiyaç duymaz.

### Girdi yöntemleri

1. 2018 saha görseli üzerinde görev bloğu ve yol çizme.
2. PathPlanner `.auto` ve `.path` dosyaları.
3. WPILog/CSV: `time,x,y,heading,vx,vy,omega,event` sütunları.
4. rosbag2: `nav_msgs/Odometry`, `nav_msgs/Path` ve görev event topic’leri.
5. Doğrudan kanonik `AlliancePlan` JSON.

### Doğrulama

İçe aktarıcı şu hatalarda planı reddeder:

- bilinmeyen şema/saha sürümü;
- eksik başlangıç pozu;
- monoton olmayan zaman;
- hız/ivme sınırı aşımı;
- saha dışı veya statik geometriyle kesişen yol;
- fallback döngüsü;
- aynı robotta zaman olarak çakışan segmentler.

İki takım robotunun koridorları çakışırsa plan yüklenir ancak sarı uyarı ve
çakışma aralığı gösterilir. Studio kanonik JSON, önizleme PNG ve plan hash’i
üretir. Jetson yalnız hash’i doğrulanmış planı etkinleştirir.

## 9. Algılama ve Lokalizasyon

### Sensör sözleşmesi

Kamera markası çekirdek paketlere gömülmez. Seçilen sürücü aşağıdaki ROS
sözleşmesini karşılamalıdır:

- senkronize renk ve depth/stereo görüntü;
- `CameraInfo` ile sabitlenmiş intrinsics/distortion;
- en az 30 Hz hedef hız;
- kamera içi senkron farkı en fazla 100 µs;
- URDF içinde ölçülmüş `base_link -> camera_link` extrinsic;
- frame timestamp jitter ölçümü ve diagnostics topic’i.

`recorded`, `stereo` ve `depth` launch profilleri aynı downstream topic adlarına
remap edilir. Böylece bir sürücü sorunu çekirdek otonomu değiştirmez.

### Semantik algılama

- Özel veri kümesi sınıfları: `power_cube`, `robot`, `switch_plate`,
  `scale_plate`, `vault_opening`.
- Robot ittifak etiketi bumper rengi ve önceden yüklenen roster ile belirlenir;
  kararsız durumda `UNKNOWN` kalır ve en muhafazakâr ayak izi kullanılır.
- YOLOv8 modelinin ONNX çıktısı Jetson üzerinde FP16 TensorRT engine’e çevrilir.
- 2B detection, depth median ve kamera modeliyle 3B poz/kovaryansa çevrilir.
- Engine metadata; veri kümesi sürümü, sınıf sırası, input boyutu ve SHA-256
  hash’ini taşır. Metadata eşleşmezse perception lifecycle düğümü aktive olmaz.

### Geometri ve takip

- Nvblox depth ve pose girdisinden yerel 3B mesh/ESDF ve Nav2 costmap üretir.
- Robot izleri için kovaryanslı Kalman/IMM takipçisi sabit-hız ve dönen-hız
  hipotezlerini birleştirir.
- Rakip niyeti; hız yönü, en yakın cube, switch/scale/vault yaklaşma koridoru ve
  önceki hareketten `P(PICKUP|SCORE|TRANSIT|DEFEND)` olarak çıkarılır.
- Tahmin ufku 2 saniye, adım 100 ms’dir. Ayak izi kovaryansla şişirilerek
  zamanlı occupancy tube üretilir.

### Takım niyet füzyonu

Önceden yüklenmiş plan başlangıç prior’ıdır. Gözlenen robot plan koridoruna
uyuyorsa güven korunur. Uzamsal veya zamansal sapma büyüdükçe güven üstel
azalır; düşük güvende plan rezervasyonu yumuşak maliyete dönüşür ve algılanan
robot izi asıl kaynak olur. Takım robotu görünmüyorsa belirsizlik zamanla büyür,
koridor sonsuza kadar sert engel olarak tutulmaz.

## 10. Strateji Motoru

### Katmanlar

1. **Behavior Tree executive:** maç fazı, arm/disarm, lokalizasyon bekleme,
   görev yürütme, timeout, recovery ve güvenli duruş.
2. **Görev üretici:** dünya durumundan geçerli pickup/score/vault/park/climb ve
   yield görevleri oluşturur; FMS saha konfigürasyonu, robot capability profili,
   mekanizma sağlığı ve kalan süreyi sert önkoşul olarak uygular.
3. **Rolling-horizon evaluator:** aday görev dizilerini puanlar.
4. **Yürütücü:** seçilen görevi Nav2 action ve mekanizma action’larına böler.

### Fayda fonksiyonu

Her görev dizisi aşağıdaki açıklanabilir bileşenlerle değerlendirilir:

```text
U = E[direct_score]
  + E[ownership_score_over_remaining_time]
  + w_rp * E[ranking_point_value]
  + gamma * E[next_state_value]
  - w_time * E[completion_seconds]
  - w_collision * P(collision)
  - w_failure * P(task_failure)
  - w_overlap * teammate_reservation_cost
  - w_penalty * P(rule_penalty)
  - w_uncertainty * state_uncertainty
```

Ağırlıklar `config/strategy/power_up.yaml` içinde sürümlenir. Maç fazına göre
profil değişir: autonomous, early-match, mid-match ve endgame. Doğrudan skor
yerine **marjinal ittifak skoru** kullanılır; takım robotunun zaten yapacağı
işi tekrar etmek değer kazandırmaz.

2018 skor modeli konfigürasyonda tablo güdümlüdür: autonomous line geçişi yalnız
bir kez, switch/scale sahipliği zaman integraliyle, vault küpleri adet ve güç
kapasitesiyle, park/climb ise capability ve kalan süreyle hesaplanır. FORCE,
BOOST ve LEVITATE etkileri ayrı state transition’larıdır; aynı küp veya aynı
zaman aralığı iki kez puanlanamaz. Qualification ranking-point değeri yalnız
maç türü qualification olduğunda etkinleşir; practice/playoff profilinde
`w_rp = 0` olur.

### Arama

- Strateji 5 Hz’de veya önemli dünya olayı geldiğinde yeniden değerlendirilir.
- Ufuk 6 saniye ve en fazla üç ardışık görevdir; endgame’de kalan maç süresi
  ufku sınırlar.
- Her aday dizi için 64 deterministik Monte Carlo rollout çalışır.
- Örneklenen değişkenler: yol süresi, pickup/score başarısı, robot tahminleri,
  algı kovaryansı ve mekanizma gecikmesi.
- Aynı skor için daha düşük risk, sonra daha kısa süre, sonra kararlı görev
  kimliği tercih edilerek karar titreşimi engellenir.
- Mevcut görev ancak yeni aday faydası hysteresis eşiğini aşarsa preempt edilir;
  acil çarpışma ve güvenlik durumu bu kurala tabi değildir.

Araştırma profili aynı `StrategyEvaluator` arayüzü arkasında sınırlı MCTS
uygulaması çalıştırabilir. MCTS gerçek robot komutu üretemez; önce replay ve
simülasyonda production rolling-horizon değerlendiriciyi geçmesi gerekir.

## 11. Hareket Planlama

### Costmap katmanları

Global ve local costmap aşağıdaki birleşik katmanları kullanır:

- 2018 saha statik geometrisi;
- robotun gerçek CAD ayak izi için inflation;
- nvblox depth obstacle katmanı;
- rakip prediction tube katmanı;
- takım rezervasyon katmanı;
- oyun kuralı keepout bölgeleri;
- dar ve riskli bölgeler için speed filter.

Planlanan takım yolu yumuşak maliyet, algılanan yakın robot ise sert dinamik
engel olur. Rakip prediction tube maliyeti zaman ve kovaryansla değişir.

### Planlayıcılar

- Global: `SmacPlannerLattice`, dikdörtgen/omnidirectional robot ayak izi ve
  swerve hareket setiyle.
- Yerel: Nav2 MPPI `OmniMotionModel`, 30 Hz; 2 saniyelik tahmin ufku.
- MPPI critics: path align/follow, goal, obstacle, near-collision, velocity,
  teammate-reservation ve opponent-risk.
- Debug profilinde aday yörüngeler ve critic maliyetleri yayınlanır; production
  profilinde yalnız optimal yörünge ve özet istatistik yayınlanır.
- Collision Monitor, planlayıcıdan bağımsız stop ve slow polygon’ları uygular.

Nav2 yalnız `cmd_vel_safe` üretir. NT4 köprüsü bunu sınırlı ve süreli RoboRIO
komutuna çevirir.

## 12. RoboRIO Güvenlik Sözleşmesi

RoboRIO komutu ancak tüm koşullar doğruysa kabul eder:

- Driver Station enabled ve izin verilen autonomous/experimental mode;
- arm-session kimliği o boot için geçerli;
- sıra numarası son kabul edilenden büyük;
- komut yaşı 100 ms’den küçük;
- heartbeat yaşı 100 ms’den küçük;
- değerler finite ve hız/ivme limitlerinde;
- swerve, NavX ve kritik mekanizma fault bayrakları temiz.

Bir koşul bozulursa hızlar sıfırlanır, aktif command iptal edilir ve neden status
topic’ine yazılır. Jetson motor CAN ağına bağlı değildir. RoboRIO tarafı 20 ms
döngüsünü, slew/current/brownout limitlerini ve mekanik soft-limitleri korur.
Operator joystick hareketi veya explicit cancel otonomu anında disarm eder.

## 13. RViz ve Sunum

Standart RViz ekranları:

- CAD’den dönüştürülmüş URDF RobotModel ve mekanizma eklemleri;
- 2018 saha mesh’i, TF ve robot izi;
- renk/depth görüntü, detection kutuları ve PointCloud2;
- nvblox mesh/ESDF ile global/local costmap;
- Power Cube kimlikleri ve güven değerleri;
- takım rezervasyon koridorları;
- rakip geçmiş izleri, kovaryans elipsleri ve tahmin tüpleri;
- tüm aday global yollar, MPPI aday yörüngeleri ve seçilen yol;
- stop/slow/keepout alanları.

Özel `frc_rviz_plugins` paneli şunları gösterir:

- maç saati, faz, skor ve switch/scale tahmini;
- Behavior Tree’nin aktif düğümü;
- seçilen görev ve en iyi iki alternatif;
- her adayın `UtilityBreakdown` çubukları;
- “neden bu?” karşı-olgusal cümlesi;
- perception, bridge, GPU, VSLAM ve lifecycle health;
- kayıt başlat/durdur, replay işareti ve acil disarm.

Sunum profili aday MPPI yollarını critic maliyetine göre yeşil-sarı-kırmızı
renklendirir. Performans profili bu yoğun debug yayınlarını kapatır.

## 14. Kayıt, Replay ve Açıklanabilirlik

Her koşuda rosbag2 şu grupları kaydeder:

- ham/işlenmiş sensör topic’leri;
- TF, odometry ve object tracks;
- canonical alliance plan ve hash;
- world-state snapshot’ları;
- candidate tasks ve tam decision trace;
- Nav2 planları/cmd_vel;
- NT4 bridge komut/status;
- diagnostics ve kaynak kullanımı.

Her `DecisionTrace`, girdideki world-state sürümünü ve deterministik tohumu
taşır. Offline replay runner canlı node graph yerine rosbag clock kullanır ve
karar hash’lerini beklenen altın dosyayla karşılaştırır. Karşı-olgusal açıklama
en iyi kaybeden adayla bileşen farklarını raporlar; üretken dil modeli kritik
karar zincirinde kullanılmaz.

## 15. Hata ve Degrade Davranışı

| Hata | Davranış |
|---|---|
| NT4/Jetson heartbeat kaybı | RoboRIO 150 ms içinde durur ve disarm olur |
| Kamera akışı kaybı | Yeni pickup görevi üretilmez; güvenli park/duruş |
| Depth/nvblox kaybı | Hız 0.5 m/s ile sınırlandırılır; Collision Monitor varsa sürer |
| VSLAM kaybı | Kısa süre wheel/NavX odometry; kovaryans eşiğinde dur/relocalize |
| NavX fault | Field-relative ve autonomous kapatılır |
| GPU sıcaklık/latency | Constrained profil, düşük inference hızı; eşik aşılırsa dur |
| Alliance plan geçersiz | Plan reddedilir; tek-robot muhafazakâr strateji |
| Takım robotu plandan sapar | Rezervasyon güveni düşer, algılanan iz öncelik kazanır |
| Rakip hızla yaklaşır | MPPI near-collision critic + Collision Monitor slow/stop |
| Elevator/intake fault | İlgili görevler üretilmez; park/yield fallback |

Hiçbir degrade modu bilinen engeli yok sayarak hızı artırmaz.

## 16. Test Stratejisi

### Birim ve özellik testleri

- Mesaj/JSON şema round-trip ve sürüm reddi.
- Koordinat/alliance dönüşümleri.
- Driver Station game-specific message ayrıştırma, eksik/geçersiz veri fallback’i.
- Capability profilinin desteklenmeyen görevleri üretmemesi.
- Fayda bileşenleri ve marjinal skor örnekleri.
- Deterministik rollout ve tie-break.
- Rezervasyon confidence decay ve covariance inflation.
- RoboRIO sıra, timeout, NaN, limit ve mode gate testleri.
- Costmap layer rasterizasyonu ve footprint çarpışması.

### Bileşen testleri

- Kayıtlı görüntüden detection → 3B nesne → track.
- RoboRIO sim ile NT4 round-trip ve bağlantı koparma.
- Nav2 lifecycle bringup/shutdown.
- Strategy Studio import/export ve hash doğrulama.
- rosbag replay’de karar hash’i karşılaştırma.

### Senaryo matrisi

En az şu senaryolar sürümlü dosya olarak tutulur:

- açık saha tek küp;
- takım rotasıyla başlangıç çakışması;
- takım robotunun 1 saniye gecikmesi;
- rakibin seçilen koridoru kesmesi;
- cube algısının kaybolması/yeniden bulunması;
- scale yerine switch’in marjinal olarak değerli olması;
- endgame’e geç kalma riski;
- VSLAM reset ve NT4 heartbeat kaybı;
- iki eşit faydalı görevde kararlı tie-break;
- yasak saha bölgesi ve penalty-risk görevi.

### Kabul eşikleri

- NT4 komut yaşı p95 < 40 ms; heartbeat kesildikten sonra fiziksel komut sıfırı
  en geç 150 ms.
- Strategy tick p95 < 200 ms, hiçbir tick 500 ms’yi aşmaz.
- Local controller 30 Hz hedefinin %99’unu karşılar.
- Kontrollü test sahasında pose RMS < 0.15 m ve heading RMS < 3 derece.
- Onaylı perception test setinde Power Cube recall ≥ 0.90 ve precision ≥ 0.90.
- 1.000 seed’li rastgele simülasyonda robot-robot temas sıfır; sistemin güvenli
  durduğu koşular başarısız görev sayılır fakat çarpışma sayılmaz.
- Rolling-horizon strateji, aynı senaryolarda sabit playbook baseline’ından
  ortalama en az %10 daha fazla ittifak skoru üretir ve p5 skoru düşürmez.
- Replay decision hash uyumu %100.

## 17. Kalibrasyon ve Devreye Alma Sırası

1. Swerve ve mekanizmalar manuel/kapalı çevrim doğrulanır.
2. RoboRIO watchdog blok üzerindeki robotta bağlantı koparılarak kanıtlanır.
3. CAD → URDF, footprint ve sensör extrinsics ölçülür.
4. Kamera intrinsics, stereo/depth hizası ve timestamp diagnostics doğrulanır.
5. Lokalizasyon robot elle gezdirilerek ground-truth işaretleriyle ölçülür.
6. Perception-only kayıtları toplanır; model acceptance setinden geçer.
7. Strategy replay, sonra simülasyon, sonra düşük hızlı boş saha çalışır.
8. Statik engel, hareketli maket, takım robotu ve rakip robot sırayla eklenir.
9. Bring-up hız sınırları ancak ilgili güvenlik ve hata enjeksiyon testleri
   geçtikten sonra artırılır.

## 18. Bilinçli Kapsam Dışı Konular

- Canlı robotlar arası kablosuz ROS ağı.
- İnsan veya rakip robota kasıtlı temas/engelleme stratejisi.
- Jetson’dan doğrudan CAN/motor kontrolü.
- İlk sürümde uçtan uca reinforcement learning ile kontrol.
- Kritik karar açıklamasında üretken LLM.
- Jetson üzerinde Isaac Sim.
- Sensör modeli belli olmadan markaya özel sürücünün çekirdek mimariye gömülmesi.

## 19. Kaynaklar ve Tasarım Gerekçesi

- FIRST 2018 kılavuzu skorları, maç fazlarını ve robot haberleşme sınırlarını
  tanımlar: https://firstfrc.blob.core.windows.net/frc2018/Manual/HTML/2018FRCGameSeasonManual.htm
- Isaac ROS 5.0 YOLOv8, ONNX → TensorRT engine akışını destekler:
  https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_object_detection/isaac_ros_yolov8/index.html
- Nvblox depth/pose’dan 3B sahne ve Nav2 costmap üretir:
  https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_nvblox/index.html
- Nav2 State Lattice, dikdörtgen omnidirectional robotlar için uygun global
  planlayıcıdır:
  https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/
- Nav2 MPPI Omni motion modelini, optimal trajectory ve critic görselleştirmeyi
  destekler:
  https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/
- Collision Monitor planlayıcıdan bağımsız stop/slow güvenlik katmanıdır:
  https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/
- WPILib/RobotPy NT4 coprocessor client bağlantısını destekler:
  https://robotpy.readthedocs.io/en/latest/troubleshooting.html
