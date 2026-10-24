# Phase 6 — Kullanıcı uçuşunda gerçek sıcak bölge kanıtı

2026-10-24, ROS 2 Jazzy / Gazebo Harmonic / PX4 v1.17. Kullanıcı uçuşu kendi terminalinden başlattı. İzleyici yalnızca ROS görüntüsü ve PX4 çıkış telemetrisi dinledi; hiçbir ARM, Offboard, konumlandırma veya durdurma komutu göndermedi.

## Sonuç

**HOTSPOT DETECTED PASS:** Gerçek ROS görüntüsünde sıcak bölge uçuş sırasında görüldü. Önceki yerdeki tek renk görüntünün nedeni sıcak bölgenin dar görüş alanı dışında kalmasıdır. Kamera, SDF, sıcaklıklar veya render ayarlarında düzeltme gerekmedi.

| Ölçüm | Kaydedilen değer |
| --- | --- |
| Kayıt zamanı | 2026-10-24 09:43:06 UTC / 12:43:06 Türkiye |
| Görüntü | 320×240, mono8 |
| Topic | `/thermoswarm/x500_1/thermal/image` |
| Frame | `x500_1/thermal_camera_optical` |
| Simülasyon timestamp | 498.500 s |
| Min/max gri değer | 100 / 200 |
| Kontrast | 100 gri seviye |
| Sıcak piksel | 6.370 (%8.29) |
| `hot_visible` | **true** |
| Ardışık kabul edilen kare | 3 |
| Yakın dönem geliş hızı | 7.900 Hz (simülasyon: 8.003 Hz) |
| X500-1 telemetrisi | SYSID 2, ARMED, `landed=false` |
| Yerel NED'den hesaplanan irtifa | 1.060 m |

Telemetri alım yaşı görüntü kabulü sırasında 4–27 ms aralığındaydı. Bu, aynı süreçte alınan taze telemetri bağlamıdır; donanımsal/senkronize görüntü-poz eşlemesi iddiası değildir. Yerel NED irtifası mutlak Gazebo dünya z koordinatıyla birebir aynı kabul edilmez.

Değiştirilmeden kopyalanan ham görüntü:

![Uçuşta gerçek ROS termal görüntüsü; üst solda kısmen görünür sıcak yüzey](evidence/phase6-flight/hotspot.png)

[PNG](evidence/phase6-flight/hotspot.png), [kare ve kabul metrikleri](evidence/phase6-flight/result.json), [periyodik canlı kayıt](evidence/phase6-flight/metrics.jsonl). PNG ölçeklenmedi, renklendirilmedi veya yapay pikselle tamamlanmadı. PNG SHA256: `8133507df06fbec9ec95a37e7e2fdaf4254f80de04617af69da570ef2fc5ade8`; raporda piksel SHA256 değeri de bulunur.

Kare tırmanış sırasında alındı; sıcak yüzeyin yalnızca bir kısmı görüntüye giriyor. Bu kabul testi sıcak bölgenin ayırt edilebildiğini kanıtlar, yüzeyin tamamının aynı karede görünmesini şart koşmaz.

## Kamera geometrisi ve kök neden

Kaynaklar: `models/x500_thermal/thermal_sensor.sdf`, PX4 `x500_base/model.sdf`, türetilmiş `.runtime/wildfire/models/x500_thermal/model.sdf` ve `worlds/thermoswarm_wildfire.sdf`.

- PX4 taban çerçevesi model orijininden +0.24 m; kamera `base_link` altında −0.12 m. Dolayısıyla düz duruşta kamera dünya yüksekliği `z_model + 0.12`.
- Sensör +Y ekseninde +90° döndürülmüş; kameranın ileri +X ekseni dünyada −Z yönüne, yere bakar.
- Yatay FOV 90°, görüntü oranı 320/240. Dikey FOV `2 atan(0.75 tan(45°)) = 73.74°`.
- Fire patch merkezi (1,1), boyutu 1.2×1.2×0.01 m, z merkezi 0.006 m. Üst yüzey **z=0.011 m**; zemin üstü z=0. Yüzey zemin tarafından örtülmüyor. Termal eklenti visual üzerinde, 600 K; arka plan/ambient 300 K. Sensors sistemi OGRE2 kullanıyor.
- `h = z_model + 0.12 − 0.011` sıcak yüzey üstüne göre kamera yüksekliğidir. Yaw=0, roll=pitch=0 için görüntü ayak izi dünya X yönünde ±0.75h, Y yönünde ±h'dir. Görüntü genişlik×yüksekliği **2h × 1.5h**.
- Yerde model z≈−0.01 m olduğundan h≈0.099 m; ayak izi yaklaşık **0.20×0.15 m**. Patch x,y=[0.4,1.6] olduğundan tamamen dışarıdadır.

Kalkış XY=(0,0) ve düz duruş varsayımı:

| Görünürlük | Gerekli h | Yaklaşık yerden yükseliş |
| --- | --- | --- |
| İlk kenarın geometrik girişi | >0.533 m | >0.434 m |
| Merkezin girişi | ≥1.333 m | ≥1.234 m |
| Yüzeyin tamamının girişi | ≥2.133 m | ≥2.034 m |

İlk kenarın girmesi 20 sıcak piksel kabulünü tek başına garanti etmez. Pitch/roll/yaw, yatay hareket ve iniş takımı örtmesi görüntüdeki yeri etkiler. Kare 1.06 m yerel irtifada kısmi yüzeyi gösterdi; hesapla tutarlıdır. 5 m yükselişte ayak izi yaklaşık **10.20×7.65 m**; mevcut kare rota köşelerinde (0/2,0/2) merkezden en büyük kenar uzaklığı 1.6 m olduğundan düz duruşta tüm patch için yeterlidir. Bu geometrik hesap her uçuş anının görüntü kaydı yerine geçmez.

## Salt-okunur izleyici

Yeni `scripts/thermal_watch.py`, sürekli ROS Image aboneliği açar. Her 2 saniyede min/max, sıcak piksel, `hot_visible`, hız, timestamp ve X500-1 uçuş bağlamını JSON olarak raporlar. Bellekte yalnızca son görüntü ve en fazla 64 zaman örneği tutulur. İlk doğrulanmış sıcak dizide gerçek kareyi `hotspot.png`, metrikleri `hotspot.json`/`result.json` olarak kaydeder ve exit 0 ile çıkar.

- `--require-airborne`: taze SYSID 2, ARMED, `landed=false`, geçerli yerel irtifa ≥0.5 m koşullarını da ister. Nihai uçuş kabulünde bu seçenek kullanılmalı.
- Üç ardışık uygun sıcak kare ve geliş hızı ≥1 Hz gerekir. Kare timestamp'leri pozitif ve artan olmalı; frame/encoding/boyut/stride doğrulanır.
- Bir saniyeden uzun kare aralığı sıcak-kare serisini sıfırlar. Varsayılan 10 saniyelik görüntü kesintisi FAIL; toplam 300 saniyede sıcak bölge görülmezse FAIL. Ctrl-C exit 130 verir.
- Her çalıştırmada **yeni çıktı dizini** gerekir; önceki kanıtların üzerine yazılmaz. Başarısızlıkta yalnızca gerçekten alınmış son kare `last-frame-not-accepted.png` adıyla saklanır; sıcak bölge kanıtı olarak sunulmaz. Hiç mesaj yoksa PNG üretilmez.
- Süreç sadece kendi aboneliklerini kapatır. Simülasyona veya uçuşa müdahale etmez.

## Kesin WSL komutları

Önce normal, salt-okunur ön kontrol:

```bash
cd ~/projects/thermal-swarm-sitl-poc
source scripts/env.sh
./scripts/sim.sh status
./scripts/sim.sh check
```

Oturum yoksa `./scripts/sim.sh start --world wildfire` ile başlatıp kontrolü tekrarlayın. Mevcut çalışan oturumu yeniden başlatmayın.

**Terminal A — uçuş boyunca ayrı terminalde çalışan izleyici:**

```bash
cd ~/projects/thermal-swarm-sitl-poc
source scripts/env.sh
python3 -u scripts/thermal_watch.py --require-airborne --timeout 300 --interval 2 \
  --output "artifacts/phase6/flight-$(date -u +%Y%m%dT%H%M%SZ)"
```

`WATCH READY` sonrasında **Terminal B — yalnızca kullanıcı tarafından başlatılan uçuş:**

```bash
cd ~/projects/thermal-swarm-sitl-poc
source scripts/env.sh
ros2 run thermoswarm_control thermoswarm_three_x500_node --execute
```

İzleyici `HOTSPOT DETECTED PASS` verdiğinde yolu yazdırır; `hotspot.png` ve `result.json` dosyalarını açın. İzleyicinin çıkması uçuşu sonlandırmaz. Görev bitip üç araç iniş/disarm yaptıktan sonra normal `./scripts/sim.sh stop` kullanılabilir; stop guard değiştirilmedi.

İsteğe bağlı kısa ölçüm (uçuşta, ayrı sourced terminalde):

```bash
python3 scripts/thermal_check.py --transport ros --frames 32 --require-hot --min-hz 1 \
  --output "artifacts/phase6/hotspot-$(date -u +%Y%m%dT%H%M%SZ).png"
```

Bu kısa ölçüm en iyi gerçek kareyi değerlendirir; uçuş telemetrisini ve ardışık sıcak kare koşulunu izleyen nihai kabul için `thermal_watch.py --require-airborne` kullanın.

## Kabul kriterlerinin ayrılması

**THERMAL CHECK PASS:** Gerçek mesaj, doğru 320×240 mono8 metadata/frame/timestamp ve istenen hız. Sıcak alan zorunlu değildir; startup/yerde PASS olması normaldir.

**HOTSPOT DETECTED PASS:** `hot_visible=true`; en az 20 piksel gri≥170, sıcak oranı <%70, gri 85–115 aralığında en az 100 çevre pikseli, min/max farkı≥60. Bu sensörün 3 K/gri seviyesi için yaklaşık sıcaklıklar 300/600 K'dir. İzleyici ayrıca üç ardışık kare, hız ve `--require-airborne` koşullarını uygular. Nihai uçuş kabulünde PNG ve başarılı JSON raporu da mevcut olmalıdır. Bu bir kontrast kontrolüdür, AI yangın sınıflandırması değildir.

## Gerçek ve offline doğrulamanın kapsamı

- Uçuş öncesi: 16 gerçek ROS karesi, 100/100, sıfır sıcak piksel. THERMAL CHECK PASS, HOTSPOT DETECTED FAIL, exit 1 (`--require-hot` ile).
- Kullanıcı uçuşu sırasında: 421 görüntü dinlendi; üç ardışık kabul karesinden sonuncusu yukarıdaki gerçek PNG'ye yazıldı. HOTSPOT DETECTED PASS, exit 0.
- X500-1 inişinden sonra ikinci 45 saniyelik izleme: 353 gerçek kare, 100/100, sıfır sıcak piksel, ARMED=false/LANDED=true; beklenen timeout ve HOTSPOT DETECTED FAIL. Ayrıca 16 karelik kısa kontrol de sıcak alan için FAIL verdi. Bunlar ikinci uçuş kanıtı değildir.
- 75 offline regresyon testi (dünya güvenliği, wildfire, launcher, setup), Python AST ve `git diff --check` geçti. Canlı soğuk/sıcak raporlar üzerinden ayrı kabul sonuçları doğrulandı. Bu offline testler yeni uçuş/render kanıtı olarak sunulmadı.
- İlk izleyici süreci başladıktan sonra, taze olmayan/hatalı görüntüyü stream PASS saymamasına ilişkin küçük ek koruma eklendi; ikinci yerdeki izleme son kodla çalıştı. Pozitif uçuş kanıtını üreten sıcaklık, ardışık kare ve airborne kriterleri değiştirilmedi.
- 15/15 waypoint tamamlanmasını ve üçlü iniş/disarm toplam sonucunu bu araç ölçmez; bunlar uçuş terminalinden ayrıca teyit edilmelidir. Bu rapor sıcak bölgenin gerçek uçuşta görünmesine ilişkindir.

Bu adımda `thermal_watch.py` eklendi, `thermal_check.py` kabul çıktıları ayrıldı; README/Phase 6 belgeleri ve gerçek kanıt dosyaları güncellendi. Dünya/model SDF, filo YAML, launcher, dünya kontrolü, stop guard, mission/safety/v3 ve PX4 kaynakları değiştirilmedi. Commit/push yapılmadı.
