# Phase 6 — Wildfire world + X500-1 thermal camera

Tarih: 2026-10-24. Ortam: Ubuntu 24.04 / WSL2, Gazebo Harmonic 8.15.0, Gazebo Sensors 8.2.2, ROS 2 Jazzy, PX4 v1.17.0. Branch: `refactor/ros2-architecture`.

Güncelleme: Paket içindeki eski dünya denetimi ve ağaçların yanlışlıkla muaf tutulması sonraki [safety fix](PHASE6_SAFETY_FIX.md) çalışmasında düzeltildi. Aşağıdaki ilk Phase 6 testleri ROS `--execute` ön kontrolünü veya wildfire uçuşunu doğrulamıyordu. Sonraki düzeltme mevcut oturumda salt-okunur test edildi; o oturum kapatılmadı.

Sonraki canlı kabul: Kullanıcının kendi başlattığı uçuşta gerçek sıcak bölge PNG/JSON kanıtı alındı: [HOTSPOT DETECTED PASS raporu](PHASE6_HOTSPOT_ACCEPTANCE.md). Aşağıdaki uçuş yapılmadığına ilişkin ifadeler ilk geliştirme testlerinin kapsamını anlatır; sonraki görüntü kanıtı bütün 15 waypoint görevinin tamamlandığı anlamına gelmez.

## Sonuç ve kapsam

Gerçek Gazebo termal sensörü ve ROS köprüsü çalışıyor. Wildfire üçlü SITL başlangıcı, doğru araç isimleri/konumları, sensörün X500-1 altında bulunması, gerçek Gazebo/ROS görüntü mesajları, AUDIT 3/3 ve güvenli kapanış doğrulandı. Varsayılan dünyanın start/AUDIT/stop regresyonu da geçti.

**Yeni wildfire dünyasında 15 waypoint uçuşu yapılmadı.** Bu çalışma hiçbir ARM, Offboard veya uçuş komutu göndermedi. Kullanıcının daha önce doğruladığı default-world launcher uçuşu referans alındı; bunun yeni senaryoda tekrarlanmış olduğu iddia edilmiyor. Sıcak bölgenin görüş testi gerçek OGRE2 render ile, fakat PX4'süz, 5 m'de sabitlenmiş X500 test modeli üzerinde yapıldı.

## Dünya ve kamera

`worlds/thermoswarm_wildfire.sdf`:

- 60×60 m düz zemin; zemin yüzeyi z=0 ve 300 K.
- Altı basit silindir/küre ağaç. Kalkış noktaları ve üç 2 m kare rotasını çevreleyen x=[0,12], y=[0,2] dikdörtgeninden en az 3 m uzakta; gerçek minimum daha yüksek.
- Merkez (1,1), boyut 1.2×1.2 m sıcak yüzey: 600 K. Merkez (-1,1) karşılaştırma yüzeyi: 300 K. İki yüzey yalnızca visual içerir, kalkış/iniş alanına collision eklemez.
- Sıcaklıklar her visual üzerinde `gz-sim-thermal-system` ile atanır. Kırmızı malzeme yalnızca normal görünüm içindir; termal ölçümün kaynağı sıcaklık component'idir. Yangın yayılımı, radyasyon modeli, duman veya AI yoktur.
- Orijinal manyetik alan, ENU dünya koordinatları, PX4 başlangıç GPS konumu ve 0.004 s fizik adımı korunur.

`scripts/wildfire_assets.py`, mevcut PX4 X500 SDF'sini SDFormat ile genişletir ve `models/x500_thermal/thermal_sensor.sdf` fragmanını mevcut `base_link` içine ekler. Üretilen model `.runtime/wildfire/models/x500_thermal/model.sdf` içindedir. Kaynak PX4 checkout'una yazılmaz. Model adı ve yeni sensör çıkarıldığında SDF'nin geri kalanı özgün genişletilmiş modelle eşittir; bu eşitlik test edilir. Ek kütle, atalet, joint, collision veya motor değişikliği yoktur.

Kamera: `type="thermal"`, 320×240 L8, 8 Hz, yatay FOV 90°, yakın/uzak düzlem 0.03/40 m. `base_link` altında z=-0.12 m ve +π/2 pitch ile +X görüş ekseni aşağı bakar. 5 m civarında görüş alanı yaklaşık 10×7.5 m'dir; sıcak alanın tümü düz araç varsayımıyla X500-1'in kare rotasında görünür. Gerçek uçuş eğimleri bu alanı değiştirir.

## Launcher ve ROS entegrasyonu

- `sim.sh start`: varsayılan PX4 tarafından başlatılan dünya/spawn yolu.
- `sim.sh start --world wildfire`: ayrı, sahipliği takip edilen Gazebo grubu; dünya içinde `x500_1`, `x500_2`, `x500_3` önceden yerleştirilir. PX4, `PX4_GZ_MODEL_NAME=x500_N` ve `PX4_GZ_STANDALONE=1` ile doğru modele bağlanır. `PX4_GZ_WORLD=thermoswarm_wildfire` ve proje/PX4 resource path'leri açıkça ayarlanır.
- `fleet.yaml` SYSID 2/3/4, namespace, spawn XY ve 5/8/11 m irtifaları değişmedi.
- `status` seçilen dünyayı gösterir; `check` manifestteki dünyayı `check.sh --world ...` üzerinden denetler. Wildfire dekorları isim bazlı dar bir izin listesine sahiptir; ekstra `x500_0` veya çakışan araçlar hâlâ reddedilir. Yanlış dünya sorgusu da başarısız olur.
- Köprü tek yönlüdür: Gazebo → ROS. Sensör mesajı yoksa placeholder üretilmez. Wildfire başlangıcı başarılı sayılmadan önce gerçek ROS image mesajları kontrol edilir.

| Alan | Değer |
| --- | --- |
| Gazebo topic | `/thermoswarm/x500_1/thermal/image` |
| Gazebo type / format | `gz.msgs.Image` / `L_INT8` |
| ROS topic | `/thermoswarm/x500_1/thermal/image` |
| ROS type / encoding | `sensor_msgs/msg/Image` / `mono8` |
| frame_id | `x500_1/thermal_camera_optical` |
| Timestamp | Gazebo simülasyon zamanı; ölçümlerde pozitif ve artan |
| Kalibrasyon | 3 K/gri seviye, bu sensör konfigürasyonuna özgü |

Saat köprüsü, TF yayıncısı, rosbag2 kaydı ve latency ölçümü eklenmedi. Image topic daha sonra standart ROS kayıt araçlarıyla kullanılabilir.

## Gerçek test kanıtları

| Test | Sonuç |
| --- | --- |
| Wildfire world + 3 PX4 başlangıcı | PASS, iki başarılı disarmed oturum |
| Araç konumları | Yaklaşık (0,0), (5,0), (10,0), ground temasında model z≈-0.01 |
| Sensör bağlantısı | `gz model -m x500_1 -l base_link -s thermal_camera`: thermal_camera, parent x500_1, local z=-0.12 |
| Gazebo gerçek image | PASS, 320×240, mono8 karşılığı, ~8 Hz |
| ROS gerçek image | PASS, 320×240 mono8, beklenen frame_id, artan stamp, ~8 Hz |
| Wildfire AUDIT | PASS 3/3; armed=False |
| Wildfire stop | PASS, taze 3/3 disarmed + landed kontrolü sonrası sahipli gruplar kapatıldı |
| Default world start/AUDIT/stop | PASS, son kodla tekrarlandı |
| Wildfire 15 waypoint + uçuşta termal görüntü | **Çalıştırılmadı; kullanıcı kabulü bekliyor** |

Yerdeki aşağı bakan kameranın gerçek görüntüsü tamamen 300 K zemindir (min=max=100). Bu ölçüm image pipeline kanıtıdır; yangın kontrastı kanıtı olarak kullanılmadı.

Sıcak bölge optik testi (`thermal_render_test.py`): ayrı `GZ_PARTITION`, ROS domain 88, üç statik model, **PX4 yok**. Aynı X500-1 termal modelinin referans pozu (0,0,5), ardından (0,2,5) olarak ayarlandı. Yalnızca testin özel Gazebo partition'ındaki statik model taşındı; canlı SITL'e poz/mode komutu gönderilmedi.

| Ölçüm | Gazebo | ROS |
| --- | --- | --- |
| Gerçek alınan frame sayısı | 16 | 16 |
| Boyut / encoding | 320×240 / mono8 karşılığı | 320×240 / mono8 |
| Min / max gri | 100 / 200 | 100 / 200 |
| Kontrast | 100 | 100 |
| Sıcak piksel | 1369 (%1.7826) | 1369 (%1.7826) |
| Yaklaşık sıcaklık aralığı | 300–600 K | 300–600 K |
| Varış hızı | 8.021 Hz | 7.988 Hz |
| Simülasyon timestamp hızı | 8.030 Hz | 7.996 Hz |

X500-1 statik modeli 2 m taşındığında sıcak bölge merkezi (128,88) pikselden (191,88) piksele kaydı: **63 piksel**. Bu, kameranın modele bağlı olduğunu destekleyen gerçek render ölçümüdür; uçuş dinamiği testi değildir.

![Gerçek ROS termal görüntüsü, 5 m statik test](evidence/phase6/elevated-ros.png)

![Aynı statik X500 2 m taşındıktan sonra gerçek Gazebo görüntüsü](evidence/phase6/translated-gz.png)

[Ham ölçüm raporu](evidence/phase6/render-test.json). PNG'ler alınan sensör byte'larından kayıpsız gri olarak yazıldı; normalize edilmedi, renklendirilmedi veya sentetik üretilmedi. Unit testlerdeki sentetik byte fixture'ları bu kanıtlara karıştırılmadı.

Yerel ayrıntılı loglar (Git dışında): `artifacts/phase6/wildfire-live.log`, `default-live.log`, `sensor-attachment.log`, `ground-gz.json`, `ground-ros.json`, `v3-tests.log` ve `artifacts/phase6/optics/`.

## Offline / statik testler

| Kontrol | Sonuç |
| --- | --- |
| XML parse ve `gz sdf -k` tam wildfire dünyası | PASS / `Valid.` |
| Kamera geometrisi, değişmeyen fizik, dünya izin listesi, image metadata/kontrast/PNG, supervisor | 16/16 PASS |
| Mevcut launcher testleri | 9/9 PASS |
| Kurulum regresyonları | 24/24 PASS |
| `thermoswarm_control/test` | 21/21 PASS |
| Orijinal v3 regresyonları (domain 87, taklit yayıncılar) | 10/10 PASS |
| ShellCheck, bash -n, Python sözdizimi, git diff --check | PASS |

Toplam **80 test**. Offline test sonuçları gerçek Gazebo sonuçlarından yukarıda ayrı tutulmuştur.

## Karşılaşılan sorunlar ve sınırlar

1. `ros_gz_bridge` başlangıçta kurulu değildi. Kullanıcı 12 yeni paketi açıkça onayladı; `sudo -n` parola istediği için kullanıcı kendi terminalinde kurulumu tamamladı. Paket 1.0.24 kurulu olarak doğrulandı. `setup.sh --plan` yeni bağımlılıkları içerir.
2. ROS vendor hook'ları `gz sim/model/sdf` sistem komutlarını gölgeledi. `/usr/share/gz`, `GZ_CONFIG_PATH` içine eklendi. Varsayılan dünya da bu ortamla tekrar test edildi.
3. Gazebo kendi argv'sini başlangıçta değiştiriyor; ilk denemede launcher bunu süreç kaybı sandı. Yalnızca bu denemede başlatılan, PID/başlangıç zamanı/komutu doğrulanan grup temizlendi. Yeni `sim_process.py` sabit sahiplik kimliği sağlar. Sonraki başlangıç ve kapanışlar geçti.
4. PX4 etkileşimli kabuğu kapalı stdin ile prompt döngüsüne girip log/CPU tüketiyordu. Launcher artık `px4 -d` kullanır; bu yalnızca pxh konsolunu kapatır, startup/mission/safety mantığını değiştirmez. Varsayılan ve wildfire disarmed akışları son kodla geçti. Eski büyük loglar silinmedi; `logs --tail` okuması sınırlandı.
5. OGRE2 + EGL headless gerçek görüntü üretti. Bu ortamda çözülmemiş render hatası yok. WSLg GUI görüntü penceresi ve `--render-mode x11` alternatifi test edilmedi. Başka GPU/sürücü için aynı sonuç garanti edilmez.
6. SDF, PX4'ün `gz_frame_id` uzantılarına ilişkin uyarılar verir; geçerli dünya ve gerçek frame ID doğrulandı. Uyarılar render hatası olarak gizlenmedi.
7. Yeni wildfire uçuş regresyonu bilinmiyor: 15 waypoint, uçuşta sıcak bölge görünürlüğü ve iniş/disarm kullanıcı tarafından ayrıca doğrulanmalı. Fizik eşitliği ve AUDIT, uçuş testi yerine geçmez.

## Kullanıcının WSL terminalindeki kesin adımlar

```bash
cd ~/projects/thermal-swarm-sitl-poc
source scripts/env.sh
./scripts/setup.sh --plan
./scripts/setup.sh --check
./scripts/sim.sh start --world wildfire --dry-run
./scripts/sim.sh start --world wildfire
./scripts/sim.sh status
./scripts/sim.sh check

gz model -m x500_1 -l base_link -s thermal_camera
python3 scripts/thermal_check.py --transport gz --frames 16
python3 scripts/thermal_check.py --transport ros --frames 16
```

Salt görüntü kontrastı/montaj kabulü, uçuş gerektirmeyen özel test:

```bash
python3 scripts/thermal_render_test.py --output artifacts/phase6/my-optical-test
```

Kullanıcı ayrıca uçuş yapmak isterse, yalnızca AUDIT/GZ CHECK başarılı olduktan sonra Terminal A'da:

```bash
ros2 run thermoswarm_control thermoswarm_three_x500_node --execute
```

X500-1 yükseldikten sonra ikinci sourced terminalde:

```bash
python3 scripts/thermal_check.py --transport ros --frames 32 --require-hot --min-hz 1 \
  --output artifacts/phase6/in-flight-thermal.png
ros2 topic hz /thermoswarm/x500_1/thermal/image
# Rate monitor için Ctrl-C; uçuşa müdahale etmez.
```

15/15 waypoint ile 3/3 iniş ve disarm sonrasında:

```bash
./scripts/sim.sh stop
./scripts/sim.sh status
# İsteğe bağlı varsayılan dünya kabulü:
./scripts/sim.sh start
./scripts/sim.sh check
./scripts/sim.sh stop
```

## Dosyalar ve kaynaklar

Yeni: `worlds/thermoswarm_wildfire.sdf`, `models/x500_thermal/thermal_sensor.sdf`, `config/thermal_bridge.yaml`, `scripts/wildfire_assets.py`, `scripts/thermal_check.py`, `scripts/thermal_render_test.py`, `scripts/sim_process.py`, `scripts/test_wildfire.py`, bu rapor ve `docs/evidence/phase6/` altındaki iki gerçek PNG + JSON.

Değişen: `scripts/sim.sh`, `scripts/sim_launcher.py`, `scripts/check.sh`, `scripts/thermoswarm_gz_world_check.py`, `scripts/setup.sh`, `scripts/env.sh`, `README.md`.

Orijinal v3, mission/safety, canonical `fleet.yaml`, PX4 kaynakları ve `main` değiştirilmedi. Commit/push, Docker, AI, rosbag2 veya latency özelliği eklenmedi. Test sonunda yönetilen simülasyon süreçleri kapatıldı.

Resmî kaynaklar: [Gazebo Sensors 8 termal sensör API](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1ThermalCameraSensor.html), [Gazebo termal kamera öğreticisi](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/tutorials/thermal_camera.md), [Harmonic Thermal sistemi](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/thermal/Thermal.cc), [Gazebo örnek termal dünyası](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/thermal_camera.sdf), [Jazzy ros_gz_bridge](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md).
