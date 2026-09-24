# AVCI — Orin + laptop yazılımı (tek repo)

AVCI FOLLOW KODU LAPTOPTA ÇALIŞAN --- HUNTER_ORİN DRONE KONTROL ALGOTİRMTASI, GÖRÜNTÜ AKTARIMI İÇEREN TARAF -- ORIN TILED  TRACKER COK ONEMLI DEGIL TRACKER DENEMESI AMA EKLEDİM

HUNTER'ın Orin tarafı, tiled tracker ve laptop tarafı (`avci_follow`) tek repoda. Yeni bir Orin'e **tek komutla** kurulur, çalışan Orin'de her şey eskisi gibi çalışır.

| Klasör | Ne | Ayrıntı |
|---|---|---|
| `hunter-orin/` | HUNTER — otonom FPV önleme. KCF / ORTrack tracker (C++, TensorRT, RTSP giriş/çıkış, OSD), MAVLink, GUIDED_NOGPS kontrolcü (`controller_guided.py`, `guided_sender.py`) ve teacher'lar (`hc_air*.py`) | `hunter-orin/README.md` |
| `orin-tiled-tracker/` | Modelden bağımsız döşemeli (tiled) tespit + takip, TensorRT, hedef 25 FPS (ekmek sayma, araç tespiti). Model: `UAV-YOLOv11m.pt` | `orin-tiled-tracker/README_KURULUM.md` |
| `setup/` | Kurulum ve kontrol scriptleri + çalışan ortamın kaydı (`referans_ortam/`) | aşağıda |
| `avci_follow/` | **Laptop (Windows)** tarafı — GPS'li takip / relay: telemetri radyolarıyla takipçi drone'a GUIDED hedef (`follow_gps.py`, `fpv_gate_il/trajectory/`) | `avci_follow/README.md` |

---

## Yeni Orin kurulumu (~1 saat, engine dahil)

**0. Hazırlık:** JetPack **6.2.1** (L4T R36.4.7) kur, kullanıcı adı **`avci2`**, internete bağla (Wi-Fi ya da kablo).

**1. GitHub SSH anahtarı** (her Orin'in kendi anahtarı olur):
```bash
ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519 -C "$(hostname)"
cat ~/.ssh/id_ed25519.pub     # GitHub > Settings > SSH and GPG keys > New SSH key
ssh -T git@github.com         # "Hi hasanucas!" görmelisin
```

**2. Klonla ve kur:**
```bash
git clone git@github.com:hasanucas/avci.git ~/Desktop/avci
cd ~/Desktop/avci
./setup/kurulum_orin.sh                  # engine'i sonraya bırakmak için: --skip-engine
```
Başta sudo şifresi sorar, sonra kendi başına gider (sadece boş RAM azsa engine öncesi onay ister). Bir adım hata verirse durmaz, en sonda listeler. Tekrar çalıştırmak güvenli (hazır olan adımı atlar). Log: `setup/kurulum_<tarih>.log`.

**3. Bitince:**
```bash
sudo reboot
sudo jetson_clocks          # her açılışta
./setup/kontrol.sh          # ✗ kalmamalı
```

**4. Saha ağı** — internet işin bittikten sonra (eno1'den internet alıyorsan bağlantıyı keser):
```bash
sudo nmcli con add type ethernet ifname eno1 con-name hunter-net \
     ipv4.method manual ipv4.addresses 192.168.144.50/24 ipv6.method ignore
nmcli con show              # eno1'de TEK profil kalsın; diğerini: sudo nmcli con delete "<ad>"
```

### Kurulum ne yapıyor?
| Adım | İş |
|---|---|
| 0 | Ön kontrol: Jetson mı, L4T referansla aynı mı, Python 3.10, internet, disk, kullanıcı |
| 1 | apt: derleme araçları, GStreamer (+rtsp-server dev), ffmpeg, ethtool… |
| 2 | JetPack bileşenleri eksikse (TensorRT / CUDA / OpenCV) `nvidia-jetpack` |
| 3 | Güç modu **MAXN_SUPER** (ID'ye değil isme bakar), 8G `/swapfile` + fstab |
| 4 | `hunter-orin/scripts/setup_orin.sh` (C++ derleme, eno1 EEE-off, dialout…) + MAVProxy |
| 5 | `setup/tiled_ortam.sh`: cuSPARSELt, torch/torchvision (Jetson wheel), ultralytics, onnx, onnxruntime-gpu |
| 6 | pip opencv silinir (GStreamer'lı sistem cv2 kalsın), numpy < 2 |
| 7 | TensorRT engine'ler: ORTrack (`hunter-orin/scripts/build_ortrack_engine.sh`, ~2 dk), hunter YOLO (`hunter-orin/scripts/build_yolo_engine.sh`), tiled YOLO (`orin-tiled-tracker/build_engine.sh`, 20–50 dk; Docker durdurulur, sonra açılır) |
| 8 | `~/Desktop/hunter-orin` ve `~/Desktop/orin-tiled-tracker` kısayolları (eski yollar çalışsın) |
| 9 | `setup/kontrol.sh` |

Python paket sürümleri `setup/referans_ortam/pip_user.txt`'ten (çalışan Orin'in kaydı) sabitlenir.

---

## Laptop (Windows) — avci_follow

Orin kurulumu gerekmez; sadece Python + telemetri radyosu. PowerShell'de:

**1. Git + SSH anahtarı** (laptop'un kendi anahtarı):
```powershell
ssh-keygen -t ed25519 -C "laptop"               # 3 kez Enter
Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub  # GitHub > Settings > SSH and GPG keys > New SSH key
ssh -T git@github.com                           # "Hi hasanucas!"
cd $env:USERPROFILE\Desktop
git clone git@github.com:hasanucas/avci.git
```

**2. Python:** python.org'dan kur ("Add python.exe to PATH" işaretli). Microsoft Store sürümünü kullanma. Temel paketler: `pip install pymavlink pyserial`; gerisi ve çalıştırma: `avci_follow/README.md`.

**3. Telemetri radyosu (SiK / CP2102):** Aygıt Yöneticisi'nde COM portu yoksa Silicon Labs **CP210x VCP** sürücüsü gerekir. Kontrol: `python -m serial.tools.list_ports -v`. Mission Planner açıksa kapat — aynı COM portunu iki program açamaz.

**Laptopta günlük git** (Windows PowerShell 5.1'de `&&` yok, komutlar ayrı satırda):
```powershell
cd $env:USERPROFILE\Desktop\avci
git pull                      # işe başlamadan önce
git add -A
git commit -m "follow: ne değişti"
git push
```
Kod artık `Desktop\avci\avci_follow` içinde; eski `Desktop\avci_follow1` eski bir kopya, repoda yok.

---

## setup/ scriptleri

| Script | Ne zaman | Bir şey değiştirir mi? |
|---|---|---|
| `kontrol.sh` | Her zaman, uçuş öncesi dahil | **Hayır**, sadece okur |
| `kurulum_orin.sh` | Yeni Orin, bir kez | Evet (apt / pip / derleme / engine) |
| `tiled_ortam.sh` | Sadece tiled tracker ortamı (başka bir Orin'de de tek başına) | Evet (apt / pip) |
| `ortam_kaydet.sh` | Çalışan ortamı bilerek değiştirdiğinde | Sadece `setup/referans_ortam/` |
| `repo_hazirla.sh` | Tek seferlik: repoyu oluşturmak için (yapıldı) | Taşıma + commit |

> ⚠️ **Mevcut (uçan) Orin'de `kurulum_orin.sh` çalıştırma.** Orada sadece `./setup/kontrol.sh`.

---

## Git'te olmayanlar

| Ne | Neden | Yeni Orin'de |
|---|---|---|
| `build/` (kcfTracker) | Her makinede derlenir | Kurulum derler |
| `*.engine` ve yedekleri (`*.bak`) | Cihaza ve TensorRT sürümüne özel, **başka Orin'e taşınmaz** | Kurulum üretir |
| `orin-tiled-tracker/*.onnx` | `.pt`'den yeniden üretilir | Engine build sırasında üretilir |
| `ORTrack_ep0300.pth.tar` (102 MB) | GitHub 100 MB sınırı; Orin'de gerekmez | **Yedeği repo dışında (Drive) tut** |
| Uçuş logları: `*.csv`, `*.BIN`, `*.tlog`, `logs/` | Veri, büyük | Repo dışında arşivle (Drive / disk) |
| Videolar (`*.mp4`, `*.h264`, `*.265`…) | Büyük | Repo dışında |
| `*.whl`, `*.deb`, `*.zip` | İndirilebilir | Kurulum indirir |

**İstisna:** `hunter-orin/models/*.onnx` git'te — hunter'ın C++ engine'leri (ORTrack, YOLO) bunlardan üretilir; ORTrack `.pth.tar` git'e giremediği için yeni Orin'de tek kaynak bunlar. GitHub push'ta 50 MB üstü dosyalar için çıkan uyarı normal (sınır 100 MB).

Test için gereken **tek bir CSV**'yi repoya almak istersen `.gitignore`'a istisna ekle:
```
!hunter-orin/tests/ornek_log.csv
```

---

## Günlük git

```bash
cd ~/Desktop/avci
git status                          # ne değişti
git add -A
git commit -m "hc_air_1: pitch seed düzeltmesi"
git push
```
Diğer Orin'de / laptopta: `git pull`

- **`git clean -x` kullanma:** git'te olmayan her şeyi (engine, build, loglar) siler.
- Ortamı bilerek değiştirdiysen (yeni paket, sürüm): `./setup/ortam_kaydet.sh && git add setup/referans_ortam && git commit -m "ortam kaydı"`

---

## Orin tuzakları

- **nvpmodel:** bu ünitede **0 = 15W**, **2 = MAXN_SUPER**. Mod reboot'ta kalır. ID'ye değil isme bak: `sudo nvpmodel -q`.
- **Her açılış:** `sudo jetson_clocks`. Eski Orin'de ayrıca `sudo swapon /swapfile` (yeni kurulumda swap fstab'da, otomatik).
- **numpy 2'ye zıplarsa** Jetson wheel'leri bozulur. Her pip kurulumundan sonra bak; düzeltme: `pip3 install --user --force-reinstall --no-deps "numpy<2"`
- **pip ile opencv kurma:** sistem cv2'yi gölgeler, GStreamer gider. Düzeltme: `pip3 uninstall -y opencv-python opencv-python-headless`
- **Engine build:** 20–50 dk, boş RAM ister (Firefox / VS Code / Docker kapalı, swap açık). "Skipping tactic / out of memory" satırları normal.
- **Orin Nano'da NVENC yok:** encode `x264enc` (CPU) ile.
- **MAVProxy `--streamrate=-1` zorunlu** (`hunter-orin/scripts/run_router_mavproxy.sh`).

---

## HUNTER ağı ve donanım

| Cihaz | Adres |
|---|---|
| SIYI VTX (hava) | 192.168.144.11 |
| GCS radyo | 192.168.144.12 |
| MK15 (Android / QGC) | 192.168.144.20 |
| Kamera | 192.168.144.25 — `rtsp://192.168.144.25:8554/main.264` (HEVC) |
| Orin (eno1) | 192.168.144.50/24 — NetworkManager profili `hunter-net` (tek profil) |
| FC (Matek F405-TE) | `/dev/ttyACM0` @ 115200 — **özel ArduPilot** (GUIDED_NOGPS açık; stok firmware'de kapalı) |

---

## Bu repo nasıl oluştu / geri alma

`setup/repo_hazirla.sh --uygula` (mevcut Orin'de, Eylül 2026):
- `~/Desktop/hunter-orin` ve `~/Desktop/orin-tiled-tracker` repoya **taşındı**, eski yerlerine **kısayol** bırakıldı. Eski yollar, scriptler ve build klasörü aynen çalışır; tek kopya var.
- İç içe `.git` klasörleri (eski geçmiş) → `~/Desktop/_repo_yedek/<tarih>/`
- Scriptlerde `nvpmodel -m 0` (bu ünitede 15W) geçen satırlar → MAXN_SUPER yapıldı (yorum satırlarına dokunulmadı).
- Çalışan ortamın kaydı → `setup/referans_ortam/`

Geri almak gerekirse:
```bash
cd ~/Desktop
rm hunter-orin orin-tiled-tracker        # SADECE kısayollar — sonuna / koyma!
mv avci/hunter-orin avci/orin-tiled-tracker .
mv _repo_yedek/<tarih>/hunter-orin/.git hunter-orin/      # eski git geçmişi geri
```

---

## Yapılacaklar

- [ ] FC özel firmware (`.apj`) + doğrulanmış param dosyası → `hunter-orin/fc/`
- [x] `avci_follow/` (laptop) eklendi
- [ ] `hunter-orin/README.md` güncel mimariye göre (GUIDED_NOGPS, teacher'lar, EKF source switch, GPS fazı)

## Geçmiş

- `github.com/hasanucas/hunter-orin` — eski deploy reposu (Haziran 2026). Artık arşiv; GitHub'da *Settings → Archive* ile kilitle ki yanlışlıkla push edilmesin.
- `github.com/hasanucas/avci_ucus` — Ağustos 2026 pre-patch snapshot, arşiv.
- **Eylül 2026'dan itibaren tek kaynak bu repo.**
