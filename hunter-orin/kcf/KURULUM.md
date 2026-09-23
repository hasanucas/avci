# hc_air_2 kurulum + saha notu (24 Ağu 2026)

Base: `hasanucas/avci_ucus` @ `df7a5ee` ("20 Agu 2026 ucus hali — patch oncesi").
**Üç dosya da arkadaşın orijinali — tek karakter değişmedi (md5 doğrulandı).**
Yapılan tek şey: yerleştirme.

---

## 1. Dosya yerleşimi

```
kcf/
  hc_air_2.py                          <- YENİ
  fpv_gate_2.yaml                      <- YENİ
  fpv_gate_il/
    teacher/
      camera_geometry.py               <- YENİ (bu import shim'de yoktu)
```

`hc_air_2.py` şunu istiyor, shim'de yoktu, o yüzden 3. dosya gerekli:

```python
from fpv_gate_il.teacher.camera_geometry import normalized_bbox_size_at_range
```

`mpc_teacher1.py`, `hc_air.py`, `hc_air_1.py`, `controller_guided.py`,
`guided_sender.py`, `controller_orin.py` — **hiçbirine dokunulmadı**.

---

## 2. Saha komutu

```bash
python3 controller_guided.py \
    --teacher hc_air_2 \
    --action-units attitude_deg \
    --teacher-config fpv_gate_2.yaml \
    --exit-mode loiter \
    --hover 0.10 \
    --enable-override
```

`--teacher-config fpv_gate_2.yaml` **ZORUNLU.** Verilmezse controller otomatik
arama yapıp `kcf/fpv_gate.yaml`'ı (eski hc_air_1 config'i) bulur. Hata vermez,
sessizce yüklenir ama `terminal_size = 0` ve `ground_floor = kapalı` olur.

Kapalı alan / pervanesiz (komut GÖNDERİLMEZ):

```bash
python3 controller_guided.py --teacher hc_air_2 --action-units attitude_deg \
    --teacher-config fpv_gate_2.yaml --exit-mode stabilize --hover 0.10
```

Uçuş öncesi Orin'de: `python3 test_governor_offline.py` → hepsi OK olmalı.

---

## 3. ⚠ attitude modunda ÇALIŞMAYAN bayraklar

Masada ölçüldü. `send_attitude()` yolu `axis_scale` / `rate_scale` / `max_rate_dps`'ye
bakmıyor:

| Bayrak | rate modu | attitude modu |
|---|---|---|
| `--scale-roll` / `--scale-pitch` / `--scale-yaw` | çalışır | **ETKİSİZ** |
| `--max-rate` | çalışır | **ETKİSİZ** |
| `--rate-scale` | çalışır | **ETKİSİZ** |
| `--att-roll-max` / `--att-pitch-max` / `--att-yaw-off-max` | — | **çalışır** |
| `--scale-thrust` / `--thrust-dev` / `--thrust-cap` | çalışır | **çalışır** |

Ölçüm (roll 30° / pitch −20° / yaw ofset +30° komutu):

```
bayraksız                   -> roll=+30.00 pitch=-20.00 yawoff=+30.0
--scale-roll/pitch/yaw 0.5  -> roll=+30.00 pitch=-20.00 yawoff=+30.0   DEĞİŞMEDİ
--max-rate 20               -> roll=+30.00 pitch=-20.00 yawoff=+30.0   DEĞİŞMEDİ
--att-*-max 10/8/15         -> roll=+10.00 pitch= -8.00 yawoff=+15.0   kırpıldı
```

Kademeli gitmek istersen attitude modunda karşılığı:
`--att-roll-max 12 --att-pitch-max 10 --att-yaw-off-max 20`

Teacher kendi içinde zaten 40° bank / [−24°, +8°] pitch'e kırpıyor.

---

## 4. Beklenen davranış — 3 m balon kümesi, 30 m irtifa

Kinematik koşum (60 m'den 12 m/s kapanma, `ang_x` sabit +2°):

```
   R(m)  z-zbalon   roll   pitch   thr_action  bbox_sz  TERM
   59.6    -0.00    +1.8   -7.2      +0.050     0.040     1
   45.2    -1.52    +7.6   -6.0      +0.075     0.052     1
   30.8    -2.51    +7.6   -6.0      +0.103     0.076     1   <- en derin sarkma -2.56 m
   16.4    -1.78    +3.8   -6.0      +0.128     0.142     1
    1.2    +0.02    +3.8   -6.0      +0.103     0.900     1
```

- Balonun ~2.5 m altına sarkıp sonra tırmanarak temasta hizalanıyor.
  Sebebi `reticle_elev_bias_deg: -6.0`; gate-top reticle düzeltmesi bbox
  büyüdükçe bias'ı iptal ediyor (kendini düzelten tuning).
- Motor bandı 0.03–0.16 (hover 0.10 etrafında dar).
- **TERM baştan sona 1.** Terminal eşiği 0.019863 = 0.5 m kapı @ 20 m.
  3 m hedefte bu eşik **120 m'de** doluyor → terminal bandı sürekli açık:
  roll kazancı ×2, yaw ×1.35, `thrust_cap 0.38`, climb cap 1.5 m/s.
  Roll agresif gelirse sebebi bu — teacher hatası değil, referans uyumsuzluğu.
  Değiştirmek istersen: `terminal.ref_width_m/ref_height_m` = gerçek hedef boyutu,
  `ref_range_m` = terminalin başlamasını istediğin menzil.
  (3 m hedef, terminal 12 m'de → `ref_width_m: 3.0, ref_range_m: 12.0`)
- `overfly` hiç ateşlenmedi (elev −2°'nin üstünde kaldı).

---

## 5. ⚠ Kat koruması

Bu commit'te **FloorMonitor YOK** (`grep FloorMonitor` boş; commit "patch öncesi").
20 Ağu'da yazdığımız wave-off katmanı bu repo halinde değil.

Tek kat koruması `hc_air_2`'nin kendi `ground_floor` bloğu:
`soft_start 12 m` · `floor 4 m` · `crash_floor 1.2 m`

Ölçüm:

```
alt=14.0 m -> pitch=-6.00 thr=+0.118 vz_des=-2.00  GF=0
alt=11.0 m -> pitch=-1.60 thr=+0.238 vz_des=+0.38  GF=1   <- devreye girdi
alt= 8.0 m -> pitch=-0.01 thr=+0.358 vz_des=+1.50  GF=1
alt= 3.0 m -> pitch=-0.00 thr=+0.518 vz_des=+3.00  GF=1   <- TIRMANMA komutu
```

12 m altında tırmanma komutlayıp burnu düzlüyor. **Balonları 25 m+ irtifada tut**
ki marj kalsın. CH6 elde.

---

## 6. Masa testi sonuçları (24 Ağu)

| Kontrol | Sonuç |
|---|---|
| `obs[12]` shim (irtifa ≠ kamera pitch) | ✅ alt 0/3/25/60 m → hepsinde −15.0° |
| `ang_y` → thrust/vz_des işaret zinciri | ✅ ang_y + → sink, − → climb |
| `ang_x` → roll/yaw işaret zinciri | ✅ ang_x + → roll +, yaw ofset + |
| `frame_id` dedup | ✅ aynı kare 2× → step_count artmıyor |
| privileged YOK (GPS-free fallback) | ✅ patlamıyor, pitch −6.63 fallback |
| NaN / bant taşma (2000 rastgele kare) | ✅ 0 ihlal (roll ≤40°, pitch [−24,+8], thr [−0.30,+0.75]) |
| `test_governor_offline.py` | ✅ HEPSI OK (bozulmadı) |

Pitch kanalı cruise −6…−7.5°'de oturup dikey nişanı **thrust**'a bırakıyor —
`hc_air` ailesinin beklenen davranışı. `mpc_teacher1`'in `cmd_pitch↔ang_y ≈ −0.75`
korelasyonunu burada arama.

---

## 7. FC parametreleri (`mav.parm`'dan okundu)

| Param | Repo'daki | Durum |
|---|---|---|
| `ARMING_RUDDER` | 2 | ✅ düzelttin |
| `PILOT_ACCEL_Z` | 250 | ✅ 400 yaptın (= 4.0 m/s²; 8 m/s sink → 2.0 s / 8.0 m frenleme) |
| `MOT_THST_HOVER` | 0.125 | AP tabanı = öğrenilmemiş → `--hover 0.10` ŞART |
| `GUID_OPTIONS` | 8 | ✅ doğru |

`PILOT_ACCEL_Z` sadece LOITER/ALT_HOLD çıkışında iş görür — GUIDED_NOGPS'te
`GUID_OPTIONS=8` thrust'ı mutlak yaptığı için Z kontrolcüsü baypas. STABILIZE'da
da devrede değil. Yani `--exit-mode loiter` ile test et.

---

## 8. Sahada bakılacaklar

1. Orta menzilde sarkma gerçekten ~2.5 m mi (bias −6 etkisi)
2. Roll salınımı — slew 45°/s, `hc_air_1`'in 90-140'ından çok yumuşak, chatter beklemiyorum
3. `ground_floor` ateşlendi mi (12 m altına inersen)
4. CH6 kesince LOITER'a geçiş sert mi — `PILOT_ACCEL_Z 400` yeterli mi, 500 gerekir mi

Uçuş sonrası CSV → log'a bakarak tek değişken değiştiririz.
