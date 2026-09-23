# HUNTER — yeni teacher'lar + ACRO yolu (31 Ağu 2026)

`kcf/` içine kopyala. Eski dosyalar **yerinde kalır**, hiçbiri ezilmez.

```
kcf/
  hc_air_3.py                 YENİ  rate teacher
  hc_air_4.py                 YENİ  açı teacher
  teacher_hc_air_3.yaml       YENİ  hc_air_3 config
  fpv_gate_4.yaml             YENİ  hc_air_4 config
  acro_sender.py              YENİ  ACRO + RC_OVERRIDE taşıma katmanı
  test_acro_offline.py        YENİ  46/46 geçiyor
  test_acro_exit_offline.py   YENİ  37/37 geçiyor
  check_config_split.py       YENİ  iki config çatışıyor mu
  controller_guided.py        YAMALI
  guided_sender.py            değişmedi (paket bütünlüğü için dahil)
  controller_orin.py          değişmedi (aynı sebep)
  fpv_gate_il/
    features.py               aday listesine fpv_gate_4.yaml eklendi
    teacher/
      camera_geometry.py      YENİ (hedef boyutu varsayımı kilitli)
      ardupilot_acro_adapter.py  YENİ (upstream, dokunulmadı)
      hc_air_acro.py          YENİ (upstream + uçuş emniyeti)
      hc_air_config.py        yalnızca yorum eklendi
```

Kopyaladıktan sonra **önce ikisini de koş**, ikisi de yeşil olmadan sahaya çıkma:

```bash
cd kcf
python3 test_acro_offline.py        # hesap: rate→PWM, thrust, kanal maskesi
python3 test_acro_exit_offline.py   # zamanlama: çıkışta release mi mod mu önce
python3 check_config_split.py       # iki config çatışıyor mu
```

`check_config_split.py` neden var: `hc_air_4` çalışırken **iki ayrı dosyadan**
ilgili sayılar okuyor. `image:` bloğu (width/height/hfov) `fpv_gate.yaml`'dan
geliyor — features shim'in aday sırası bilerek değiştirilmedi ki `hc_air_1`
davranışı bozulmasın. `hc_air.camera:` bloğu ise `fpv_gate_4.yaml`'dan.
Bedeli şu: `fpv_gate.yaml`'ın `image:` bloğuna dokunursan `hc_air_4` sessizce
etkilenir, oysa `--teacher-config fpv_gate_4.yaml` verdiğin için öyle
olmadığını sanırsın. `camera_vfov_deg` oradan türetilip `bbox_h` ile çarpılıyor
ve irtifa nişan açısını belirliyor. Config'lere dokunduktan sonra bu betiği koş.

---

## Teacher / config / bayrak eşlemesi

Devir notundaki tabloya eklenen satırlar:

| Teacher | Komut | `--action-units` | Config | Taşıma |
|---|---|---|---|---|
| `hc_air` | hız (rad/s) | `rad_s` | `teacher_hc_air.yaml` | GUIDED |
| `hc_air_1` | açı | `attitude_deg` | `fpv_gate.yaml` | GUIDED |
| `hc_air_2` | açı | `attitude_deg` | `fpv_gate_2.yaml` | GUIDED |
| **`hc_air_3`** | **hız** | **`rad_s`** | **`teacher_hc_air_3.yaml`** | **GUIDED** |
| **`hc_air_3`** | **hız** | **`acro_pwm`** | **`teacher_hc_air_3.yaml`** | **ACRO** |
| **`hc_air_4`** | **açı** | **`attitude_deg`** | **`fpv_gate_4.yaml`** | **GUIDED** |

Bayrak şeritleri:

| Bayrak | rate (GUIDED) | attitude | **acro_pwm** |
|---|---|---|---|
| `--scale-roll/pitch/yaw` | çalışır | etkisiz | **çalışır** |
| `--max-rate`, `--rate-scale` | çalışır | etkisiz | **çalışır** |
| `--att-roll-max/pitch-max/yaw-off-max` | — | çalışır | **reddedilir (hata verip çıkar)** |
| `--scale-thrust`, `--thrust-dev`, `--thrust-cap` | çalışır | çalışır | **çalışır** |
| `--acro-max-rate` | — | — | **yeni** |

`--acro-max-rate` ile `--max-rate` farkı: `--max-rate` teacher **komutunu** kırpar, `--acro-max-rate` adapter'ın **fiziksel zarfını** daraltır. İkisi birlikte çalışır, ikisini de kullanabilirsin.

**Config tuzağı kapatıldı.** `controller_guided.py` artık teacher adına göre config seçiyor. `--teacher-config` vermezsen ve doğru dosya yoksa **çöker** — başka bir yaml'a düşmez. Açılışta `config yüklendi:` satırını yine de oku.

---

## Nasıl uçulur

**hc_air_3, GUIDED, mevcut yolla A/B** (önerilen ilk adım):

```bash
python3 controller_guided.py --teacher hc_air_3 --action-units rad_s \
  --teacher-config kcf/teacher_hc_air_3.yaml \
  --scale-roll 0.5 --scale-pitch 0.8 --scale-yaw 0.8 --scale-thrust 0.6 \
  --max-rate 35 --hover 0.101 --exit-mode althold --enable-override
```

Bu, 27 Ağu vuruş konfigürasyonunun teacher'ı değiştirilmiş hali. Dört değişken birden farklı (aşağıya bak).

**hc_air_4, açı yolu:**

```bash
python3 controller_guided.py --teacher hc_air_4 --action-units attitude_deg \
  --teacher-config kcf/fpv_gate_4.yaml \
  --hover 0.101 --exit-mode althold --enable-override
```

`--att-pitch-max` **kullanma** — pitch tohumunu sender'da ezer, sıçrama geri gelir.

**ACRO yolu (henüz hiç uçmadı):**

```bash
python3 controller_guided.py --teacher hc_air_3 --action-units acro_pwm \
  --teacher-config kcf/teacher_hc_air_3.yaml \
  --acro-max-rate 20 --scale-thrust 0 --thrust-cap 0.20 \
  --hover 0.101 --exit-mode althold --enable-override
```

Bu satır **pervanesiz bench** içindir: dikey yetki kapalı, thrust tavanlı, 20 °/s zarf.

---

## ACRO yolunda dikkat edilecek tek şey

`RC_CHANNELS_OVERRIDE` **moddan bağımsızdır**. ACRO'dan ALTHOLD'a geçtiğinde override hâlâ CH1–4'ü sürüyor ve ALTHOLD onu **pilot çubuğu** sanıyor. Hover 0.101'in PWM karşılığı orta çubuğun altında → ALTHOLD "pilot alçalmak istiyor" diye okur. Ekranda `mod = ALTHOLD ✓` yazarken drone iner.

Bu yüzden kodda sıra kesin: **önce release, sonra mod**, ve watchdog armed olduğu sürece her döngüde tekrar release. `test_acro_exit_offline.py` tam bunu doğruluyor — hiçbir hesap testi bu hatayı yakalayamaz.

Release `0` gönderir, `65535` değil. 65535 override'ı **tutar**; `controller_orin.py`'de tam bu hata yaşanmıştı.

Uçmadan önce masada: `--enable-override` ile çalıştır, CH6'yı aç-kapa, QGC'de CH1–4'ün gidip geri döndüğünü gör.

---

## Sahaya taşınan üç bulgu

**1. ACRO çözünürlüğü kaba.** `ACRO_RP_RATE=360` + 900 PWM adımı = adım başına **0.8 °/s**. Tek karede en iyi hata 0.4 °/s. Kapanma fazındaki 1–2 °/s'lik komutlar gürültü seviyesinde. `TemporalRateQuantizer` hatayı zamana yayıyor ama asıl çözüm FC parametresi:

| `ACRO_RP_RATE` | çözünürlük |
|---|---|
| 360 (şimdiki) | 0.400 °/s |
| 200 | 0.222 °/s |
| 120 | 0.133 °/s |

**2. Adapter'ın emniyet ağı şu an ölü kod.** Dairesel stick reddi `p_max=q_max=1.20 rad/s` + `ACRO_RP_RATE=360`'ta asla tetiklenmiyor (yarıçap 0.27, limit 0.85). `ACRO_RP_RATE`'i 110'un altına indirirsen tetiklenmeye başlar. Upstream'de bu durumda istisna kontrol döngüsünü öldürüyordu — emniyeti ekledim (son geçerli PWM'i tut, sayacı artır, CSV'ye yaz) ve testte dar konfigle doğruladım.

**3. `hc_air_3` vuruş yapan kod değil.** Dört değişken birden farklı:
- `_update_pitch_des_governor` içindeki `pitch_assist` boost'u **kaldırılmış**
- `cruise_des_deg` −7.5 → −9.5
- `des_accel_max_deg_s` 9 → 12
- `warmup_s` 2.0 → 1.0

Tek değişken disiplini gereği: ACRO'yu üstüne bindirmeden önce `hc_air` vs `hc_air_3`'ü GUIDED'da A/B yap.

---

## Açık işler

- [ ] **Sahada tek satır bile uçmadı.** İkisi de offline yeşil, o kadar.
- [ ] `MOT_THST_HOVER` 0.125 → 0.102. ACRO tarafında ayrıca: `thr_mid_curve` `clamp(MOT_THST_HOVER, 0.125, 0.6875)` olduğu için 0.102 yazsan bile eğri ortası 0.125'te kalır. Değişmez ama bilmen gerekiyor.
- [ ] Baro kalibrasyonu (`BARO1_GND_PRESS` ~1000 m kaymış) — `hc_air_4`'ün `ground_floor`'ı bunu bekliyor.
- [ ] `ground_floor` açma kararı. Şu an `false`; `FloorMonitor` (20 m) tek otorite. Açarsan bu **tek değişken** olsun.
- [ ] `ACRO_RP_RATE` düşürme kararı (bulgu 1).
- [ ] Masada CH1–4 override + release testi. `--params-set-manually` KULLANMA
      demeye gerek yok artık (aşağıdaki düzeltme), ama açılışta
      `[ACRO] Rate→PWM adapter config source:` satırının
      `vehicle (MAVLink params)` dediğini gör.
- [ ] `hc_air` vs `hc_air_3` A/B (bulgu 3).
- [ ] Closure-stall bekçisi — devir notundan devam eden iş, bu pakete girmedi.

---

## Değişmeyen ilkeler — nerede korundu

**Hedef boyutu varsayımı.** Upstream açı sürümü `terminal.ref_width_m: 0.5 / ref_height_m: 0.5 / ref_range_m: 20.0` ile mutlak bbox eşiği hesaplıyordu. `fpv_gate_4.yaml`'da bu kapatıldı, `size_ratio: 2.0` (kilit anındaki bbox'a oran, ölçüm/ölçüm) geri kondu. `ref_*`'ı >0 yaparsan `hc_air_4` açılışta görünür uyarı basar. `test_acro_offline.py` B bölümü bunu kilitliyor.

**obs[12].** Yeni paketlerin ikisi de sim `features.py`'ını getiriyordu; o `camera_pitch_from_obs`'ta obs[12]'yi okuyor. HUNTER'da obs[12] = `fc.alt_rel`. 50 m'de 65° kamera açısı hatası, yerde doğru çalışıp yükseldikçe bozulur. Mevcut shim korundu.

**Pitch tohum yaması.** `hc_air_4` kusuru yapısal olarak taşıyordu. 25 Ağu yaması yeniden uygulandı (12 satır, sıfır değer değişikliği). Testte kontrol grubu var: yama atlanınca −20°'den +18.6° sıçrama geri geliyor, yamayla 0.00°.

---

## 31 Ağu — Orin'de koşturma sonrası düzeltme

İlk koşturmanın çıktısı iki şey gösterdi.

**Düzeltilen hata: `--params-set-manually` ACRO param okumasını kapatıyordu.**
`guided_sender`'da bu bayrağın anlamı "GUID_OPTIONS/GUID_TIMEOUT'u elle yazdım,
sen **yazmaya** çalışma". `acro_sender`'da yanlışlıkla **okumayı** da
kapatıyordum. Yazma ile okuma ayrı şeyler: ACRO/RCMAP/RC*_MIN..MAX okuması
FC'yi hiç değiştirmiyor. Eski hali sessiz tuzaktı — `--params-set-manually`
ile uçarsan ACRO yaml fallback'ine düşerdin ve bunu ancak log satırından
anlardın. Okuma zaten başarısızlığa karşı korumalı (istisnayı yakalar, yaml'da
kalır, gürültülü basar), denemenin maliyeti yok. Test buna kilitlendi (26/26).

**Beklenen davranış: `rate zarfi daraltildi` satırı iki kez basıyor.**
Bu doğru ve ikincisi *yük taşıyan* olan. `try_load_from_vehicle` başarılı
olursa adapter'ı **yeniden kuruyor** ve `limits`'i yaml'daki `rate_limits`
bloğundan tazeliyor — yani `__init__`'te yaptığım daraltmayı siliyor. Bu
yüzden `setup()` içinde, FC okumasından **sonra** tekrar daraltıyorum. İki
satır görüyorsan doğru çalışıyor; tek satır görürsen bir şey ters.

---

## 31 Ağu — ikinci koşturma sonrası: kapsam boşluğu kapatıldı

Sahte FC param döndürmüyordu, yani **hep yaml fallback yolu** test ediliyordu.
Sahada olacak yol ise FC okumasının **başarılı** olduğu yol, ve orada bir
tuzak var:

`try_load_from_vehicle` başarılı olunca `self.adapter` **yeni bir nesneyle**
değiştiriliyor ve `limits` yaml'ın `rate_limits` bloğundan tazeleniyor
(1.2/1.2/1.0). Yani `__init__`'te yapılan `--acro-max-rate` daraltması
**siliniyor**. `setup()` içinde FC okumasından sonraki ikinci daraltma bu
yüzden yük taşıyor.

Silinseydi kimse fark etmezdi: ekranda yine `rate zarfi daraltildi
p/q/r <= 20.0 deg/s` yazıyor olurdu (birinci çağrıdan), ama uçan zarf
**68 °/s** olurdu. `--acro-max-rate 20` ile ilk ACRO uçuşuna çıkıp üç kat
yetkiyle uçmak demek.

Teste `A2` bölümü eklendi: gerçek `mav.parm` değerlerini döndüren sahte bir
FC ile başarılı yol koşuluyor ve şunlar kilitleniyor — config kaynağı
`vehicle` oldu mu, daraltma hayatta kaldı mı, quantizer daralmış adapter'a
bağlı mı (limits'i kopyalamıyor, referans tutuyor), 40 °/s isteği gerçekten
20'ye kırpıldı mı, FC'den okunan RC kalibrasyonu yaml fallback'iyle aynı mı.

**Testin dişi doğrulandı:** `setup()` içindeki daraltmayı geçici olarak
kaldırdım, test 2 maddeden düştü (`p=1.2000` ve `40.00 deg/s`). Yani bu test
yeşil olduğunda gerçekten bir şey söylüyor.

33/33.

---

## 31 Ağu — üçüncü koşturma: param okuma kadansı

Üç betik de Orin'de yeşil çıktı. Bench'e gitmeden önce FC param okuma yolunu
inceledim, çünkü bench oturumunu yakabilecek tek yer orası.

**Bulunan:** `try_load_from_vehicle` upstream'in sabit `3 deneme × 3.0 s`
ayarını kullanıyordu. HUNTER'ın udpout linki param isteklerini düşürüyor —
bu `guided_sender.read_param`'ın kendi yorumunda yazılı saha dersi:
*"MOT_THST_HOVER bir kere okunup sonraki çalıştırmada timeout verdi"*, istek
0.6 saniyede bir tekrarlanmak zorunda kalınmıştı. Aynı link, aynı sorun.

Bedeli ağır: gruplardan biri eksik kalırsa istisna atılıyor, **tüm** konfig
yaml'a düşüyor (okunan gruplar dahil), ve `_vehicle_load_failed` yapışkan
olduğu için oturum boyunca bir daha denenmiyor. Tek düşen paket = bütün
oturum yaml fallback'inde.

**Mekanik:** `fetch_parameters_strict` her denemede tüm pending'i yeniden
soruyor. Yani dayanıklılık istek aralığından değil **deneme sayısından**
geliyor. 4 parametrelik acro grubu için:

| | denemeler | toplam istek | en kötü süre |
|---|---|---|---|
| upstream | 3 × 3.0 s | 12 | 9.0 s |
| HUNTER | 8 × 0.6 s | 32 | 4.8 s |

Daha çok şans, üstelik daha kısa sürede — FC gerçekten yoksa açılışı 9 saniye
değil 5 saniye bekletiyor.

`attempts`/`timeout_per_attempt` `try_load_from_vehicle`'dan dışarı açıldı,
`AcroSender` 8 × 0.6 geçiyor (`acro_param_attempts` / `acro_param_timeout`
ile değiştirilebilir).

**Test `A3`:** ilk 12 isteği kasten düşüren sahte bir link. Kontrol grubuyla
birlikte — upstream ayarı (3 × 3.0) aynı linkte yaml'a düşüyor, HUNTER ayarı
4. turda kurtarıyor. İlk yazdığım kontrol grubu 8 paket düşürüyordu ve
**düşmedi**, yani test bir şey söylemiyordu; mekaniği yanlış modellemiştim.
Düzeltildi.

37/37.
