# Phase 5 — Kurulum ve doğrulama kaydı

Tarih: 2026-10-24. Branch: `refactor/ros2-architecture`.

Bu teslimin kapsamı kurulum betiğidir; Docker, yeni mission/safety davranışı ve canlı uçuş kapsam dışıdır. Commit ve push yapılmadı. Phase 4'ten kalan `.gitignore`, README ve izlenmeyen `sim.sh` / launcher / safety / launcher test dosyaları başlangıçta zaten mevcuttu; bu çalışma launcher kodunu değiştirmedi.

## İncelenen referans ortam

README, `docs/ARCHITECTURE.md`, paket manifestleri, Python ortamı, PX4 kurulum/CMake kaynakları, Agent superbuild bağımlılıkları ve Phase 4 launcher incelendi. Repoda `SETUP_ENV` adlı ayrı bir belge bulunmadı.

| Bileşen | Yerel kanıt |
| --- | --- |
| OS | Ubuntu 24.04.1, amd64 / mevcut WSL2 ortamı |
| Python | 3.12.3, `.venv-ros` system-site-packages açık |
| ROS | `/opt/ros/jazzy`, `ros-jazzy-ros-base` 0.11.0-1noble.20261006.035407 |
| Gazebo | `gz sim --versions`: 8.15.0; `gz-harmonic`: 1.0.0-1~noble |
| PX4 | v1.17.0, `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, SITL binary mevcut |
| px4_msgs | v1.17.0 / release/1.17, `86d8239e962f6939e05c3737784f60c02fa884db` |
| Agent | v2.4.3, `73622810d984349b80bbac0ef55fc0b694d62222`; `/usr/local/bin/MicroXRCEAgent` mevcut |
| Agent çalışma zamanı | `ldd` ile çözülen gerçek dosya `/usr/local/lib/libmicroxrcedds_agent.so.2.4.3` |
| ROS paketleri | `ros2_ws/install` içindeki `px4_msgs` ve `thermoswarm_control` bulunuyor/import ediliyor |
| Fleet | Kurulu `share/thermoswarm_control/config/fleet.yaml` okunuyor, şema doğrulanıyor ve canonical config ile aynı |

Agent 2.4.3 `--version` seçeneğini desteklemediği için bu seçenek sürüm kanıtı olarak kullanılmadı. Kütüphane adı ve kaynak checkout'u doğrulandı; mevcut binary'nin kaynakla bit düzeyinde eşleştiği iddia edilmiyor. PX4 binary yalnızca dosya/çalıştırılabilirlik açısından kontrol edildi, çalıştırılmadı.

## Dosyalar ve davranış

Yeni dosyalar:

- `scripts/setup.sh`: salt-okunur plan/check, açık kurulum onayı, sistem işlem kapısı, kaynak/derleme korumaları, yerel Agent ve ROS derlemesi.
- `scripts/setup-versions.env`: ana ve Agent bağımlılık depolarının commit pinleri; ROS apt-source sürümü.
- `scripts/requirements-px4-constraints.txt`: çalışan Python ortamından alınan PX4 bağımlılık sürüm kısıtları.
- `scripts/env.sh`: Jazzy, yerel ROS overlay, `.venv-ros` ve varsa Agent wrapper ortamı.
- `scripts/setup_verify.py`: ROS düğümü oluşturmadan kurulu paket/fleet kontrolü; başka overlay veya kaynak fallback'ini reddeder, standart colcon symlink build dizinini kabul eder.
- `scripts/test_setup.py`: izole fixture'larla kurulum regresyonları.
- Bu doğrulama kaydı.

`README.md` güncellendi. Orijinal v3, `thermoswarm_control` mission/safety kodu ve canonical `fleet.yaml` değiştirilmedi. `main` branch'inde işlem yapılmadı.

Kurulum sistem Python'una pip yazmaz; PX4 için ayrı `.venv-px4` kullanır. Yeni Agent, sabitlenmiş Fast-CDR/Fast-DDS/memory/spdlog ile proje altına kurulur. Agent'a ait `LD_LIBRARY_PATH` yalnızca wrapper sürecine verilir; ROS kütüphaneleri gölgelenmez. Mevcut uyumlu Agent ve PX4 binary korunur. Kaynak değişiklikleri, yanlış commit, eksik submodule, uyumsuz Agent/venv veya yarım PX4 build dizini otomatik silinmez/düzeltilmez; açıklama ile durulur. Çalışan PX4 varken kurulum reddedilir.

## Otomatik sonuçlar

| Kontrol | Sonuç |
| --- | --- |
| `bash -n scripts/setup.sh scripts/env.sh scripts/sim.sh` | PASS |
| `shellcheck scripts/setup.sh scripts/env.sh scripts/sim.sh` | PASS |
| Yeni Python dosyaları AST/sözdizimi | PASS |
| `test_setup.py` | 24/24 PASS |
| Phase 4 `test_sim_launcher.py` | 9/9 PASS |
| `thermoswarm_control/test` pytest | 21/21 PASS |
| Orijinal v3 regresyonları | 10/10 PASS |
| `scripts/setup.sh --help`, varsayılan plan, `--check` | PASS |
| `source scripts/env.sh` + `sim.sh start --dry-run` | PASS: 1/2/3 instance, 0/5/10 m spawn komutları |
| `sim.sh status` | Yönetilen simülasyon yok; canlı başlatma yapılmadı |
| `git diff --check` | PASS |

Toplam **64 otomatik test** geçti. Kurulum testleri sistemde apt/sudo/network çalıştırmaz; geçici Git depoları ve komut taklitleri kullanır. Eksik bağımlılık, onaysız kurulum, apt transitive upgrade/removal, değişmiş commit/tag, yerel değişiklikler, rootfs korunması, başarısız derleme işaretçisi ve tekrar çalıştırma durumlarını kapsar.

V3 testlerinin ilk denemesi sandbox'ın `~/.ros/log` yazma kısıtında durdu. İzinli tekrar çalıştırmada `ROS_DOMAIN_ID=87` ve `ROS_LOG_DIR=/tmp/thermoswarm-phase5-ros-logs` kullanıldı; 10/10 geçti. Test yayıncıları taklittir. Test loglarındaki ARM/FLIGHT metinleri simüle edilmiş durum makinesi geçişleridir; PX4/Gazebo başlatılmadı ve canlı uçuş komutu gönderilmedi.

## İzole gerçek kurulum ve yeniden çalıştırma

Mevcut çalışma alanının build/install dizinlerine dokunmadan `/tmp/thermoswarm-phase5-install-28dpim6y` altında gerçek `setup.sh --install --yes --jobs 2` çalıştırıldı. Sistem bağımlılıkları, mevcut PX4/Agent ve `.venv-ros` kullanıldı; kaynak pinleri kontrol edildi. `thermoswarm_control` ve config geçici alana kopyalandı; harici checkout'lar salt-okunur kullanım için symlink ile referanslandı.

- İlk çalıştırma **PASS**: `px4_msgs` sıfırdan derlendi (9 dakika 12 saniye), `thermoswarm_control` derlendi (1.84 saniye); toplam iki paket 9 dakika 14 saniyede tamamlandı.
- Geçici workspace'in kurulu iki paketi ve kurulu `fleet.yaml` doğrulandı; kurulum sonu tüm kontroller **PASS**.
- Aynı komutun ikinci çalıştırması **PASS**: `KEEP ROS workspace: source fingerprint and installed package checks match.` Hiç `Starting >>>` satırı yok; yeniden derleme yapılmadı.
- Apt/sudo, yeni dış kaynak indirme, PX4/Agent yeniden derleme veya canlı simülasyon çalışmadı.

Log kopyaları (Git dışında): `.runtime/setup-validation/first-install.log` ve `.runtime/setup-validation/second-install.log`. Bu deneme mevcut bağımlılıklar üzerinde ROS derleme ve tekrar çalıştırma kanıtıdır; temiz OS kurulumu kanıtı değildir.

## Henüz doğrulanmayanlar

- Temiz Ubuntu 24.04 / WSL2'de uçtan uca `--install --allow-system`.
- Yeni apt kaynak ekleme/kurulum işlemlerinin gerçek çalıştırılması ve depolardan indirilebilirlik.
- Yeni `.venv-px4` kurulumu, constraint sürümlerinin PyPI'da bulunabilirliği ve PX4'ün sıfırdan derlenmesi.
- Proje içine ayrı bağımlılıklarla Agent derlenmesi/kurulması ve yeni binary'nin canlı UDP/DDS iletişimi.
- arm64 kurulum.
- Phase 4 `sim.sh start → check → stop` canlı yaşam döngüsü ve bu launcher ile üçlü uçuş.

Apt patch sürümleri Noble/Jazzy/Harmonic depolarını takip eder. Python constraints çalışan makineden çıkarılmıştır; distro metadata'sı bazı geçişli bağımlılıkları listelemeyebilir. Tam çözümlenmiş Python sürümleri kurulumda `.runtime/setup/px4-python-resolved.txt` dosyasına yazılır. Kaynak commit'leri sabittir; bu çalışma byte-identical apt/PyPI snapshot veya tamamen offline kurulum sağlamaz.

## Kullanıcının WSL terminalinde kabul adımları

Mevcut makinede önce salt-okunur kontroller:

```bash
cd ~/projects/thermal-swarm-sitl-poc
./scripts/setup.sh --help
./scripts/setup.sh --plan
source scripts/env.sh
./scripts/setup.sh --check
ros2 pkg prefix px4_msgs
ros2 pkg prefix thermoswarm_control
cat "$(ros2 pkg prefix thermoswarm_control)/share/thermoswarm_control/config/fleet.yaml"
./scripts/sim.sh start --dry-run
```

Yeni Ubuntu makinesinde README clone adımlarından sonra planı okuyup `./scripts/setup.sh --install --allow-system` çalıştırın; sudo/apt sorularını inceleyin. Mevcut bağımlılıkları hazır makinede `--install` yeterlidir. Aynı komutu yeniden çalıştırıp kaynakların/binary'lerin korunduğunu ve ROS build için `KEEP ROS workspace` çıktısını doğrulayın.

Canlı yaşam döngüsü kabulü kullanıcı tarafından ayrıca yapılır:

```bash
source scripts/env.sh
./scripts/sim.sh start
./scripts/sim.sh status
./scripts/sim.sh logs --tail 40
./scripts/sim.sh check
```

`GZ CHECK PASS`, `AUDIT PASS (3/3)` ve genel `CHECK PASS` görülmeden uçuşa geçmeyin. Bu kontroller geçerse kullanıcı ayrıca şu komutu çalıştırabilir:

```bash
python3 scripts/thermoswarm_three_x500_v3.py --execute
# 15/15 waypoint; 3/3 iniş ve disarm sonrasında:
./scripts/sim.sh stop
./scripts/sim.sh status
```

Yeni terminalde `source scripts/env.sh` tekrarlanır. Manuel başlatılmış eski oturumlar launcher tarafından devralınmaz veya kapatılmaz. Canlı doğrulama tamamlandıktan sonra tarih, log yolları ve PASS/FAIL kanıtı bu belgeye eklenmelidir.
