# Phase 6 — Wildfire dünya güvenlik kontrolü düzeltmesi

Tarih: 2026-10-24. Branch: `refactor/ros2-architecture`.

## Sorun ve düzeltme

ROS paketindeki `thermoswarm_gz_world_check.py`, yalnızca default dünyayı tanıyan eski bir kopyaydı. Bu nedenle wildfire dekorları yetim model sayılıyor ve iki görsel yüzey 1.41 m uzaklıkla genel 1.5 m eşiğine takılıyordu. `scripts/` kopyası ise tüm izinli statik nesneleri çarpışma hesabından çıkarıyor, fiziksel ağaçları da muaf tutuyordu.

Artık tek uygulama `ros2_ws/src/thermoswarm_control/thermoswarm_control/thermoswarm_gz_world_check.py` içinde. `scripts/thermoswarm_gz_world_check.py` buna yönlendiren uyumluluk girişidir; orijinal v3 dosyası değişmedi. Launcher başlangıcı, `sim.sh check`, doğrudan `check.sh` ve ROS `--execute` ön kontrolü aynı uygulamayı kullanır.

Dünya seçimi, launcher'ın kullandığı gerçek Gazebo adından yapılır: `default` veya `thermoswarm_wildfire`. `/gazebo/worlds` salt-okunur sorgusu tam olarak bir desteklenen dünya döndürmelidir. Launcher açık dünya seçimini iletir; bunun gerçek dünya ile uyuşmaması FAIL olur. Paket doğrudan çalıştırıldığında otomatik keşif kullanır; çalışma dizinine, dekor isimlerinden tahmine veya eski manifest'e dayanmaz. Model listesi ve poz sorgularının dünya adları da eşleşmelidir.

- Default: `ground_plane` ve yalnızca `x500_1`, `x500_2`, `x500_3` kabul edilir.
- Wildfire: bunlara ek olarak `fire_patch`, `ambient_patch`, `tree_1`–`tree_6` gereklidir. Ekstra, yinelenen, eksik veya izinsiz modeller FAIL olur.
- Wildfire için `/world/thermoswarm_wildfire/generate_world_sdf` üzerinden genişletilmiş **canlı SDF** alınır. Diskteki dosyaya veya yalnızca model adına bakarak muafiyet verilmez. SDF/model listesi farklıysa veya sorgu başarısızsa FAIL olur.
- İki yüzey ancak statik, yerde, beklenen ince kutu geometrisinde, collision/include/iç içe model içermeyen ve gerçek Thermal eklentisiyle 600 K / 300 K olarak tanımlanmışsa muaf tutulur. Kaynak SDF ve çalışan Gazebo bu koşulları zaten sağlıyordu; dünya dosyası değiştirilmedi.
- Ağaçlar fiziksel engeldir. 1.5 m yatay güvenlik mesafesine collision geometrisinin yatay yarıçapı ve yerel ofsetleri eklenir (mevcut ağaçlarda 1 m taç yarıçapı). Desteklenmeyen geometri/çerçeve, eksik veya sonlu olmayan poz FAIL olur. Araçlar arası eşik 1.5 m olarak korunur.
- `mission_node.py` yalnızca CLI seçenek kısaltmalarını kapatır. `--executete` ve `--exec` kullanım hatasıdır; doğru seçenek `--execute`. Mission, Safety, v3 uçuş kodu, PX4 kaynakları ve YAML kimlik/irtifaları değiştirilmedi.

Bu, uçuş öncesi yatay yerleşim kontrolüdür; dinamik engelden kaçınma sistemi değildir. Kontrol sırasında dünya geometrisini başka bir süreçle değiştirmeyin.

Gazebo'nun [SDFormat kaynak çözümleme API'si](https://gazebosim.org/api/sim/8/namespacegz_1_1sim.html) ve sistemdeki `/usr/share/gz/gz-msgs10/protos/gz/msgs/sdf_generator_config.proto`, kaynakların genişletilmesi için referans alındı. SDF export sorgusu dünya durumunu değiştirmez.

## Bu düzeltmede doğrulananlar

### Gerçek mevcut WSL/Gazebo oturumu — salt-okunur

| Kontrol | Sonuç |
| --- | --- |
| Canlı wildfire SDF: 3 X500 + 8 dekor + ground | PASS |
| Fire / ambient collision yok, sıcaklıklar 600 / 300 K | PASS |
| X500 XY yaklaşık (0,0), (5,0), (10,0); altı ağaç pozu | PASS |
| CLI dünya kontrolü `--world wildfire` | GZ CHECK PASS |
| `./scripts/sim.sh check` | GZ CHECK PASS + AUDIT PASS 3/3 |
| Kurulu ROS modülü, otomatik dünya keşfi | GZ CHECK PASS |
| `ros2 run ...` (`--execute` olmadan) | AUDIT PASS 3/3 |
| `ros2 run ... --executete` | Beklenen kullanım hatası, exit 2 |
| `setup.sh --check` | PASS; sürüm pinleri, paketler ve kurulu fleet doğrulandı |
| `thermoswarm_control` colcon derlemesi | PASS |

Yerel ham kayıtlar `artifacts/phase6-safety-fix/` altında: `live-world-response.txt`, `sim-check.log`, `installed-world-check.log`, `ros-audit.log`, `cli-typo.log`, `setup-check.log`. Bu dizin Git tarafından ignore edilir. Testler mevcut kullanıcı oturumunu başlatmadı/durdurmadı ve ARM/Offboard/uçuş komutu göndermedi.

### Offline regresyonlar

**109 test geçti:** 26 yeni dünya güvenliği testi + 16 wildfire + 9 launcher + 24 setup + 24 ROS paketi + 10 orijinal v3 testi. Ek olarak ShellCheck, `bash -n`, Python AST, `gz sdf -k` ve `git diff --check` geçti.

Yeni testler default üçlü PASS, wildfire üçlü + sekiz dekor PASS, sıcaklıkların korunması, ekstra `x500_0`, eksik/yinelenen modeller, bilinmeyen engel, spawn'daki ağaç, taç mesafesi, araç çakışması, yanlış/belirsiz dünya, bozuk SDF, sorgu hatası/zaman aşımı, çakışmalı/yerden yükseltilmiş/dinamik termal yüzey ve sonlu olmayan pozları kapsar. Launcher'ın dünya seçimini ortak denetime iletmesi test edilir. ROS testleri geçerli `--execute` yolunun aynı denetime uğradığını ve FAIL durumunda ROS düğümü bile başlatılmadığını mock ile doğrular; gerçek uçuş yerine geçmez.

**Bu düzeltmede yapılmayanlar:** default dünyada yeniden gerçek start/check/stop, gerçek dünyaya ekstra X500/engel ekleyerek olumsuz test, wildfire 15 waypoint uçuşu ve iniş/disarm. Aktif oturum korundu. Önceki Phase 6 render/default başlangıç testleri ayrı [rapordadır](PHASE6_VALIDATION.md); bu düzeltmenin yeni uçuş kanıtı olarak sunulmaz. Termal piksel/render testi bu düzeltmede yeniden yapılmadı; termal tanımların korunduğu doğrulandı.

## WSL terminalinde doğrulama

Kaynakları aldıktan sonra kurulu ROS paketini güncelleyin (bu makinede zaten derlendi):

```bash
cd ~/projects/thermal-swarm-sitl-poc
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/local_setup.bash
(cd ros2_ws && colcon build --packages-select thermoswarm_control --symlink-install)
source scripts/env.sh
```

Mevcut wildfire oturumunda uçuşsuz kontroller:

```bash
./scripts/sim.sh status
./scripts/sim.sh check
python3 -m thermoswarm_control.thermoswarm_gz_world_check
ros2 run thermoswarm_control thermoswarm_three_x500_node
python3 scripts/thermal_check.py --transport ros --frames 16
```

Oturum yoksa önce `./scripts/sim.sh start --world wildfire` çalıştırın. Çalışan oturum varken yeniden başlatmayın. Hatalı argüman kontrolü (beklenen exit 2):

```bash
ros2 run thermoswarm_control thermoswarm_three_x500_node --executete
echo "$?"
```

Yalnızca kullanıcı uçuşu başlatmak istediğinde ve GZ CHECK/AUDIT başarılı olduğunda:

```bash
ros2 run thermoswarm_control thermoswarm_three_x500_node --execute
```

15/15 waypoint, iniş ve disarm sonrasında kullanıcı isterse default regresyonunu ayrı oturumda çalıştırabilir:

```bash
./scripts/sim.sh stop
./scripts/sim.sh start
./scripts/sim.sh check
./scripts/sim.sh stop
```

Offline regresyonları tekrar çalıştırmak için (ROS testleri izole domain ve sahte komut yayıncıları kullanır):

```bash
source scripts/env.sh
PYTHONPATH=scripts python3 -m unittest scripts/test_world_safety.py scripts/test_wildfire.py scripts/test_sim_launcher.py scripts/test_setup.py -v
ROS_DOMAIN_ID=87 ROS_LOG_DIR=/tmp/thermoswarm-test-logs python3 -m unittest discover -s ros2_ws/src/thermoswarm_control/test -v
ROS_DOMAIN_ID=87 ROS_LOG_DIR=/tmp/thermoswarm-test-logs python3 scripts/test_thermoswarm_three_x500_v3.py -v
shellcheck scripts/setup.sh scripts/check.sh scripts/sim.sh scripts/env.sh
bash -n scripts/setup.sh scripts/check.sh scripts/sim.sh scripts/env.sh
```

## Bu düzeltmede değişen dosyalar

- `scripts/thermoswarm_gz_world_check.py`: tek paket uygulamasına yönlendirme.
- `ros2_ws/src/thermoswarm_control/thermoswarm_control/thermoswarm_gz_world_check.py`: ortak dünya/çarpışma politikası ve canlı SDF doğrulaması.
- `scripts/check.sh`: açık seçenek verilmezse otomatik canlı dünya keşfi.
- `ros2_ws/src/thermoswarm_control/thermoswarm_control/mission_node.py`: yalnızca CLI kısaltma davranışı.
- `scripts/test_world_safety.py` (yeni), `scripts/test_wildfire.py`, `ros2_ws/src/thermoswarm_control/test/test_world_preflight.py` (yeni): regresyon kapsamı.
- `README.md`, `docs/PHASE6_VALIDATION.md`, bu rapor (yeni): doğru ROS komutu, rebuild ve test kapsamı.

`sim_launcher.py` ve wildfire SDF incelendi; bu düzeltmede değiştirilmeleri gerekmedi. Çalışma ağacında önceki Phase 6'dan kalan diğer değişiklikler korundu. Commit/push veya `main` değişikliği yapılmadı.
