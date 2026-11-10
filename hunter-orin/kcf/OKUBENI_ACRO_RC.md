# HUNTER — arkadaşın ACRO paketi + `--acro` bayrağı (9 Kas 2026)

Arkadaşın `acro.tar.gz` paketi `kcf/` içine girdi. **Hiçbir mevcut dosya
ezilmedi.** Varsayılan davranış değişmedi: bayrak vermezsen yine
GUIDED_NOGPS + SET_ATTITUDE_TARGET.

```
kcf/
  hc_air_5.py                        YENİ  arkadaşın teacher'ı (BİREBİR)
  fpv_gate_5.yaml                    YENİ  arkadaşın config'i  (BİREBİR)
  acro_rc_sender.py                  YENİ  PWM geçirmeli ACRO taşıma katmanı
  test_acro_rc_offline.py            YENİ  119/119 geçiyor
  OKUBENI_ACRO_RC.md                 YENİ  bu dosya
  controller_guided.py               YAMALI  --acro bayrağı
  fpv_gate_il/teacher/
    los_geometry.py                  YENİ  arkadaşın dosyası (BİREBİR)
    hc_air_config.py                 YAMALI  `hc_air:` bloğunu açıyor
    hc_air_acro.py                   DOKUNULMADI (HUNTER emniyeti korundu)
    ardupilot_acro_adapter.py        DOKUNULMADI (upstream ile BİREBİR aynıydı)
  fpv_gate_il/features.py            DOKUNULMADI (obs[12] shim'i korundu)
```

Sahaya çıkmadan **beşini de** koş, hepsi yeşil olmadan uçma:

```bash
cd kcf
python3 test_acro_rc_offline.py       # YENİ yol        119/119
python3 test_acro_offline.py          # eski ACRO yolu   46/46
python3 test_acro_exit_offline.py     # çıkış sıralaması 37/37
python3 test_follow_handover_offline.py                 # 220/220
python3 check_config_split.py                           # çatışma yok
```

---

## Neden arkadaşın dosyalarının üstüne yazmadım

Paket, repoda **zaten bulunan** üç dosyanın upstream sürümünü de getiriyordu.
Üçü de HUNTER'da bilerek saptırılmış ve üzerine yazmak uçuşu bozardı:

| Dosya | Repoda | Upstream | Üzerine yazılsa |
|---|---|---|---|
| `hc_air_acro.py` | 316 satır | 218 satır | `_map_with_fallback` giderdi → `map_command` reddi **kontrol döngüsünü öldürürdü**. Ayrıca `rates_to_rc_pwm` (acro_sender giriş noktası) ve `map_*` sayaçları giderdi, param okuma 8×0.6 s → 3×3.0 s'ye düşerdi |
| `features.py` | 104 satır (shim) | 248 satır | `camera_pitch_from_obs` obs[12]'yi okumaya başlardı. HUNTER'da obs[12] = `alt_rel`. 50 m'de **65° kamera açısı hatası**, yerde doğru çalışıp yükseldikçe bozulur |
| `hc_air_config.py` | 78 satır (shim) | 42 satır | Düz `kcf/` yapısında YAML arama yolu tutmaz |

`ardupilot_acro_adapter.py` upstream ile **birebir aynıydı** (0 satır fark),
dokunmaya gerek yoktu.

`test_acro_rc_offline.py` K bölümü bu üçünü **kilitliyor** — biri ileride
ezilirse testte düşer, uçuşta değil.

---

## `hc_air_config.py`'ye eklenen tek şey: `hc_air:` açma

Repodaki iki yaml şekli var ve ikisi de kullanılıyor:

- **düz** — `teacher_hc_air.yaml`, `teacher_hc_air_3.yaml`: en üstte doğrudan
  `platform:`, `camera:`, `yaw:` …
- **sarmalı** — `fpv_gate.yaml`, `fpv_gate_2/4/5.yaml`: en üstte `image:`,
  `observation:`, `bridge:`, `hc_air:` — teacher parametreleri `hc_air:` altında

Shim şimdiye kadar dökümanın tamamını döndürüyordu. Sorun çıkmamasının sebebi:
`hc_air_1.py` ve `hc_air_4.py` **kendi** `load_hc_air_config`'ini tanımlıyor ve
orada açma var. Bu shim'i import eden tek teacher `hc_air_3` idi ve o **düz**
dosya okuyor.

`hc_air_5` bu shim'i import ediyor **ama sarmalı** `fpv_gate_5.yaml` okuyor.
Açma olmadan `cfg["platform"]` KeyError verir, teacher ilk satırda çöker.
Upstream'in kendi `hc_air_config`'inde bu açma **zaten var** — eksik olan
upstream davranışıydı, yeni bir icat değil.

**Geriye uyumlu:** düz dosyalarda `hc_air` anahtarı yok, aynen dönmeye devam
ediyorlar. `acro_sender.py`'nin `'acro' not in cfg` kontrolü de etkilenmiyor.
46/46 ve 37/37 bunu doğruluyor.

---

## Nasıl uçulur

**Varsayılan (değişmedi):**

```bash
python3 controller_guided.py --teacher hc_air --action-units rad_s \
  --hover 0.101 --exit-mode althold --enable-override
```

**YENİ ACRO yolu — pervanesiz bench (A0), ilk adım:**

```bash
python3 controller_guided.py --acro --teacher hc_air_5 \
  --teacher-config kcf/fpv_gate_5.yaml \
  --hover 0.101 --acro-max-rate 20 --scale-thrust 0 --thrust-cap 0.20 \
  --exit-mode althold --enable-override
```

Dikey yetki kapalı, gaz tavanlı, 20 °/s zarf.

**Dry-run (hiçbir komut gitmez, teacher + çevrim yine koşar):**

```bash
python3 controller_guided.py --acro --teacher hc_air_5 --exit-mode stabilize
```

**A/B için: aynı ACRO yolu, SENİN saha konfiginle.** `hc_air_3` de
`output_mode` kabul ediyor ve `teacher_hc_air_3.yaml` senin gerçek
kalibrasyonunu taşıyor (aşağıdaki tabloya bak):

```bash
python3 controller_guided.py --acro --teacher hc_air_3 \
  --teacher-config kcf/teacher_hc_air_3.yaml \
  --hover 0.101 --acro-max-rate 20 --scale-thrust 0 --thrust-cap 0.20 \
  --exit-mode althold --enable-override
```

Tek değişken disiplini: **önce bu satır**, sonra `hc_air_5`'e geç. Böylece
"ACRO taşıma çalışıyor mu" sorusu ile "arkadaşın yeni teacher'ı/config'i
ne yapıyor" sorusu karışmaz.

`hc_air`, `hc_air_1`, `hc_air_2`, `hc_air_4` `output_mode` bilmiyor;
`--acro` ile verirsen controller **çöker ve sebebini söyler**. Onları
ACRO'da uçurmak için eski yol: `--action-units acro_pwm`.

---

## UÇMADAN ÖNCE OKU — `fpv_gate_5.yaml` sim değerleri taşıyor

Arkadaşın `acro:` bloğu senin saha kalibrasyonun **değil**. Beş fark var:

| değer | arkadaş | senin saha | FC okuması düzeltir mi |
|---|---|---|---|
| `rc.*` min/trim/max/dz | 1000/1500/2000, dz 20 | 1045/1495/1945, dz 0 | ✅ `RC{n}_MIN/TRIM/MAX/DZ` |
| `thr_mid_curve` | 0.5 | 0.125 | ✅ `ACRO_THR_MID`, yoksa `clamp(MOT_THST_HOVER, .125, .6875)` |
| `rp_expo` | 0.3 | 0.0 | ✅ `ACRO_RP_EXPO` |
| **`hover_thrust_firmware`** | **0.33** | **0.101** | ❌ **HAYIR — sadece yaml** |

İlk üçü sahada kendiliğinden doğruya çekiliyor: `load_adapter_from_vehicle`
onları FC'den okuyup yaml'ı eziyor.

**Dördüncüsü ezilmiyor.** `load_adapter_from_vehicle` `hover_thrust_firmware`
ve `thrust_scale` parametrelerini **hiç okumuyor** — FC'de karşılıkları yok.
Ve dikey kanal doğrudan onlardan geliyor:

```
firmware = hover_thrust_firmware + (thrust_cmd − platform.hover_thrust) × thrust_scale
```

yani teacher hover'ında çıktı **tam olarak** `hover_thrust_firmware`.
Senin RC kalibrasyonun ve `thr_mid = 0.125` ile:

| `hover_thrust_firmware` | gaz PWM | çubuk |
|---|---|---|
| 0.33 (arkadaşın yaml'ı) | **1667** | %69 |
| 0.101 (ölçülmüş hover) | **1465** | %47 |

**202 µs fark = teacher hover isterken ~3.3× hover thrust.** İlk ACRO
angajmanında sert tırmanış. Ve bu **sessiz** olurdu: açılışta
`config source: vehicle` yazar, rc/expo/thr_mid doğru okunmuş görünür,
tek bozuk sayı yaml'dan gelmeye devam ederdi.

**Kapatılan şekli:** `acro_rc_sender._resolve_hover_fw()`. Sıra:

1. `--hover` verilmişse **o kazanır** (mini-G1 ölçümü) ve converter'a yazılır
2. verilmemişse FC'den `MOT_THST_HOVER` okunur
3. o da okunamazsa yaml'a düşer **ve gürültülü basar**

Her üç durumda `guided_sender`/`acro_sender` ile **aynı** makul bant kapıları
(0.05–0.50, 0.35 fabrika reddi, 0.125 taban notu) uygulanır — saha ritmi
değişmiyor. Yaml ile çözülen değer 0.02'den fazla ayrılırsa **PWM farkı
sayıyla** basılır.

> **Pratik kural: `--acro` ile `--hover`'ı HER ZAMAN ver.**

### Ama `--hover` TEK BAŞINA YETMEZ — 9 Kas Orin logunun çıkardığı tuzak

Orin koşusunda aynı fiziksel istek iki yaml'da bambaşka PWM üretti:

| yaml | istenen firmware thrust | üretilen gaz PWM |
|---|---|---|
| `fpv_gate_5.yaml` (rc 1000/1500/2000, `thr_mid` 0.5) | 0.101 | **1101** |
| `teacher_hc_air_3.yaml` (rc 1045/1495/1945, `thr_mid` 0.125) | 0.101 | **1465** |

364 µs fark. Asıl mesele PWM'in kendisi değil, **FC'nin onu nasıl
yorumladığı**: FC kendi gerçek kalibrasyonunu kullanır. 1101'i
1045/1495/1945 + `thr_mid` 0.125 ile yorumlayınca çubuk **%6.2** çıkıyor,
yani gerçek thrust **0.0002** — hover isterken gaz neredeyse kapalı.
ACRO angajmanında drone düşer.

Ekranda ise `firmware 0.101 -> gaz PWM 1101` yazıyor. Sayı makul görünüyor.

`--hover` sadece `hover_thrust_firmware`'i düzeltir; `rc` kalibrasyonu ve
`thr_mid_curve` FC param okumasından gelir. **Okuma başarısız olursa**
sim değerleri kalır ve yukarıdaki durum oluşur.

**Kapatılan şekli:** `_gate_unverified_mapping()`. Canlı komut gönderiyorsan
(`--enable-override`) ortada bir FC var; kalibrasyonunu okuyamadıysak mapping
doğrulanmamış demektir ve **başlatılmaz**. Üç yol gösterilir:

1. **Param linkini düzelt** — doğru çözüm bu. Açılışta
   `config source: vehicle (MAVLink params)` görmelisin.
2. **`--teacher hc_air_3 --teacher-config kcf/teacher_hc_air_3.yaml`** ile uç.
   O dosyanın `acro:` bloğu `mav.parm`'dan elle dolduruldu, fallback güvenli.
3. **`--acro-accept-yaml-cal`** — yaml'ın FC ile birebir olduğunu
   doğruladıysan.

Dry-run'da kesmez, sadece uyarır (gönderilen komut yok).

Eski yolda (`acro_sender`) neden uyarı yetiyordu: onun yaml'ı
`teacher_hc_air_3.yaml` ve FC ile birebir. Orada fallback **güvenli**.
`fpv_gate_5.yaml` sim taşıyor — aynı fallback burada güvenli değil.
Risk farklı, kapı farklı.

Kalıcı çözüm istersen `fpv_gate_5.yaml`'daki `hover_thrust_firmware`'i 0.101
yap — ama o zaman dosya arkadaşın gönderdiğinden sapar ve sonraki paket
birleştirmesi elle çatışma çözmek olur. Şu an dosya **birebir** duruyor,
düzeltme `--hover` ile runtime'da yapılıyor. Hangisini istediğine karar ver.

---

## Governor bu yolda ne yapar — dürüst tablo

Teacher PWM'i kendisi üretiyor, yani rate uzayındaki kelepçeler artık
"komutu kırpma" değil. Sessizce etkisiz kalmasınlar diye her biri anlamlı
bir işe bağlandı:

| Bayrak | Bu yolda |
|---|---|
| `--acro-max-rate` | **TEACHER'in** converter zarfını daraltır (rad/s). **TAM** |
| `--max-rate` | **Aynı zarfa bağlandı** (ikisinin küçüğü geçer). GUIDED'da teacher komutunu kırpıyordu; burada kırpacak komut yok |
| `--scale-roll/pitch/yaw` | PWM uzayında trim etrafında ölçek. **YAKLAŞIK** — aşağıdaki expo notu |
| `--scale-thrust` | Gaz PWM'i hover PWM'ine doğru ölçeklenir. `0` = gaz **hover PWM'ine kilitli** |
| `--thrust-cap` | Gaz PWM üst sınırı (bench) |
| `--thrust-dev` | Gaz, hover ± dev bandında (saha) |
| `--hover` | Dikey kanalın **tek** doğru kaynağı (yukarı bak) |
| `--att-*-max`, `--gimbal` | controller **reddeder** |
| `--acro-no-tilt-comp` | controller **reddeder** — bu yolda tilt telafisi teacher'ın içinde (`platform.tilt_cos_min`) |

**EXPO NOTU.** `ACRO_RP_EXPO = 0.3` olduğu için `rate = f(çubuk)` doğrusal
değil. `--scale-roll 0.5` PWM uzayında çubuğu yarıya indirir ama rate'i
**tam yarıya indirmez** — expo küçük çubukta rate'i daha da kıstığı için
gerçek azalma yarıdan **fazla** olur. Yetki azaltmak için doğru yönde, ama
"yarım rate" diye okuma. Kesin rate tavanı istiyorsan `--acro-max-rate`.

Senin saha konfiginde (`teacher_hc_air_3.yaml`) `rp_expo = 0`, yani
`hc_air_3` ile A/B yaparsan PWM ölçeği **doğrusal** ve tam olur.

---

## Çıkış sıralaması — değişmedi, ama neden kritik olduğunu tekrar yaz

`RC_CHANNELS_OVERRIDE` **moddan bağımsızdır.** ACRO'dan ALTHOLD'a geçsen
bile override hâlâ CH1–4'ü sürüyor ve yeni mod onu **pilot çubuğu** sanar.
Hover PWM'i orta çubuğun altındaysa ALTHOLD "pilot alçalmak istiyor" diye
okur ve iner — ekranda `mod = ALTHOLD ✓` yazarken.

Bu yüzden sıra kesin: **önce release, sonra mod**, ve watchdog armed olduğu
sürece her döngüde tekrar release. Release `0` gönderir, `65535` **değil**
(65535 override'ı *tutar*; `controller_orin.py`'de tam bu hata yaşandı).

`--acro` bu sıralamanın **zaten kurulu** olduğu `acro_path` kancalarını
açıyor — `auto_mode = ACRO`, watchdog `acro_path=True`, 4. kapı release'i,
taban sigortası release'i, `finally` release_burst. Yeni bir çıkış yolu
yazılmadı; `test_acro_rc_offline.py` I bölümü sıralamayı tekrar kilitliyor.

---

## Mimari — iki ACRO yolu yan yana duruyor

```
                        ┌─ guided_sender.py ──→ GUIDED_NOGPS
                        │   SET_ATTITUDE_TARGET      (varsayılan)
  teacher ── rad/s ─────┤
  (body_rate)           └─ acro_sender.py ────→ ACRO + RC_OVERRIDE
                            SENDER çevirir          (--action-units acro_pwm)
                            dikey: guided_sender yolu

  teacher ── PWM ────────── acro_rc_sender.py ─→ ACRO + RC_OVERRIDE
  (acro_rc)                 sadece TAŞIR            (--acro)
  çevrim TEACHER'in içinde  dikey: teacher'ın içinde
  (HCAirAcroConverter)
```

İkisi de yaşıyor çünkü **dikey kanalları başka eğriden geçiyor.** `acro_sender`
sahada bench'e kadar gelmiş, incelenmiş bir yol. Birini diğerinin üstüne
yazmak tek değişken disiplinini bozardı.

`acro_rc_sender` ayrı bir adapter **kurmaz** — teacher'ın converter'ını
referansla kullanır (tek doğruluk kaynağı). Bu yüzden `--acro-max-rate`
teacher'ın zarfını daraltıyor ve FC param okuması teacher'ın converter'ını
tazeliyor.

**Arkadaşın dosyasının özel alanına bir yerde dokunuyorum:**
`AcroRcSender._teacher_converter()` → `teacher._acro_converter`. `HCAir`'in
public yüzeyinde converter'ı veren bir metot yok. Alternatif `hc_air_5.py`'ye
iki satır accessor eklemekti; o zaman dosya arkadaşın gönderdiğinden sapardı.
`test_acro_rc_offline.py` D bölümü bu bağı kilitliyor — upstream alanı yeniden
adlandırırsa testte düşer.

---

## 9 Kas offline testinin yakaladığı kod hatası

İlk yazımda PWM min/max kelepçesi **koşulluydu** — sadece bir ölçek bayrağı
verilmişse çalışıyordu. Governor yoksa teacher'ın PWM'i **ham** geçiyordu.
Teacher zaten bant içinde üretiyor, yani pratikte tetiklenmezdi; ama buradan
çıkan sayı doğrudan `RC_CHANNELS_OVERRIDE`'a gidiyor ve **negatif** bir değer
uint16 paketlemesinde bambaşka bir PWM'e döner.

Kelepçe **koşulsuz** yapıldı. Dişi doğrulandı: kelepçeyi geçici kaldırdım,
test 109'dan 108'e düştü (o an 109 maddeydi) (`geldi (9999, -9999, 2000, 0)`). Yani bu test
yeşil olduğunda gerçekten bir şey söylüyor.

---

## Açık işler

- [ ] **Sahada tek satır bile uçmadı.** 119/119 + mevcut 46/37/220 yeşil, o kadar.
- [ ] `--hover` kararı: runtime'da mı verilecek (şu anki şekil, dosya birebir
      kalır) yoksa `fpv_gate_5.yaml`'a 0.101 yazılacak mı (dosya sapar)?
- [ ] Masada CH1–4 override + release testi: `--acro --enable-override` ile
      çalıştır, CH6'yı aç-kapa, QGC'de CH1–4'ün gidip **geri döndüğünü** gör.
      Açılışta `[ACRO] Rate→PWM adapter config source:` satırının
      `vehicle (MAVLink params)` dediğini gör.
- [ ] `hc_air_3` vs `hc_air_5` A/B — ikisi de ACRO'da koşabiliyor.
- [ ] `ACRO_RP_RATE` düşürme kararı (31 Ağu bulgusu 1 hâlâ geçerli): 360 →
      0.400 °/s çözünürlük, 120 → 0.133 °/s. `fpv_gate_5.yaml` 360 varsayıyor.
- [ ] `features.py` `image:` bloğunu `kcf/fpv_gate.yaml`'dan okuyor (1280×720),
      `fpv_gate_5.yaml` 1920×1080 diyor. **Şu an zararsız**: en-boy oranı
      ikisinde de 16:9, `hfov_deg` 80, `pitch_deg` −15 → türetilen VFOV aynı.
      Ama `fpv_gate.yaml`'ın `image:` bloğuna dokunursan `hc_air_5` de sessizce
      etkilenir. `check_config_split.py`'ı o zaman koş.
- [ ] `los_geometry.py` sahada hiç koşmadı — `hc_air_5`'in LOS hız takibi
      buradan geliyor ve bu paketteki gerçekten yeni matematik o.
