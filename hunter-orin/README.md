# HUNTER — runtracker: 1080p işleme · 720p yayın · KCF | ORTrack | Hibrit

14 Eylül 2026. Taban: `runtracker__2_.cpp` (9 Eylül video/link sürümü).
Üç değişiklik tek dosyada birleşti:

| Etiket | Ne |
|---|---|
| `[1080]` | İşleme kaynak çözünürlüğünde, yayın `--width/--height`'te |
| `[ORT]` | KCF yerine ORTrack (TensorRT fp16), manuel ROI ile |
| `[HYB]` | YOLO tespit + ORTrack takip, otomatik yakalama |

**Hiçbir bayrak vermezsen KCF çalışır.** Tek fark 1080p işleme; onu da
`--proc-out` ile kapatabilirsin, o zaman davranış eski sürümle aynıdır.

---

## 1. Dosyalar nereye

```
hunter-orin/
├── models/                              ← models.zip buraya
│   ├── ORTrack_ep0300.onnx
│   ├── ORTrack_ep0300-fp16.engine
│   ├── ORTrack_ep0300-fp16.engine.json
│   ├── UAV-YOLOv11m-768.onnx
│   └── UAV-YOLOv11m-768.engine
│
├── kcf/
│   ├── runtracker.cpp                   ← DEĞİŞTİ (eskisini .bak al!)
│   ├── ortrack_trt.cpp                  ← YENİ
│   ├── yolo_trt.cpp                     ← YENİ
│   ├── CMakeLists.txt                   ← DEĞİŞTİ
│   ├── include/                         (mevcutlara EKLENİR, üzerine yazmaz)
│   │   ├── ortrack_core.hpp   ortrack_trt.hpp   target_manager.hpp
│   │   └── tracker_backend.hpp  yolo_core.hpp   yolo_trt.hpp
│   └── tests/                           ← YENİ klasör, 9 dosya
│       ├── test_ortrack_offline.cpp  golden.txt          gen_golden.py
│       ├── test_target_manager.cpp   manager_golden.txt  gen_manager_golden.py
│       └── test_yolo_offline.cpp     yolo_golden.txt     gen_yolo_golden.py
│
└── scripts/
    ├── build_ortrack_engine.sh          ← YENİ
    └── build_yolo_engine.sh             ← YENİ
```

`ORTrack_ep0300.pth.tar` (107 MB) ve 512'lik YOLO dosyaları Orin'de gerekmiyor.

```bash
cp kcf/runtracker.cpp kcf/runtracker.cpp.bak
cp kcf/CMakeLists.txt kcf/CMakeLists.txt.bak
```

---

## 2. Derleme

`CMakeLists.txt`'te iki eksik pkg-config modülü vardı; 9 Eylül sürümü onlarsız
link olmuyor (`undefined reference to gst_rtsp_message_get_header`).
`gstreamer-rtsp-1.0` ve `gstreamer-video-1.0` eklendi.

```bash
cd ~/Desktop/hunter-orin/kcf
mkdir -p build && cd build
cmake .. && make -j4
```

Çıktıda görmen gereken: `ORTrack   : ACIK  (nvinfer=...)`.
`KAPALI` ise TensorRT/CUDA bulunamamış; ikili yine derlenir ama sadece KCF yapar.

---

## 3. Testler — uçuştan önce

TensorRT, CUDA, kamera, FC gerektirmez:

```bash
cd kcf/build && make tests && ctest --output-on-failure
```

Beklenen:

```
ortrack_core   ... Passed      1444/1444
target_manager ... Passed     26087/26087
yolo_core      ... Passed       290/290
```

İlk ikisi Python referansına karşı doğrulanmış. **`yolo_core` bir istisna:**
ultralytics'in NMS semantiğini numpy'de yeniden kurup C++'ı ona karşı test
ettim — C++ ile kendi referansım uyuşuyor, ultralytics'in kendisiyle uyuştuğu
kanıtlanmadı. Gerçek modele karşı doğrulanan tek şey çıktı tensörünün düzeni.

---

## 4. Motorları **kendi Orin'inde** üret

TensorRT planı GPU mimarisine + TRT sürümüne + sürücüye bağlıdır.

```bash
cd ~/Desktop/hunter-orin
sudo nvpmodel -m 0 && sudo jetson_clocks
./scripts/build_ortrack_engine.sh     # ~90-150 sn, beklenen ~7 ms
./scripts/build_yolo_engine.sh        # ~12 dk sürer, ölçülen 26 ms
```

İki motorun formatı farklı, kod ikisini de tanıyor:

| Dosya | İlk baytlar | Ne |
|---|---|---|
| `ORTrack_ep0300-fp16.engine` | `ftrt...` | ham TensorRT planı |
| `UAV-YOLOv11m-768.engine` | `a\x02\x00\x00{"desc...` | ultralytics: 4B uzunluk + JSON + plan |

Ultralytics başlığı otomatik atlanıyor, yani cihaz uyuyorsa mevcut `.engine`
doğrudan çalışır. Uymuyorsa net hata verir ve **alt moda düşer**:
`--hybrid` → manuel ORTrack → KCF. Uçuş motor dosyası yüzünden iptal olmaz.

---

## 5. Çalıştırma

`run_tracker.sh` zaten `"$@"` geçiriyor.

```bash
./scripts/run_tracker.sh --width 1280 --height 720              # KCF, 1080p işleme
./scripts/run_tracker.sh --width 1280 --height 720 --proc-out   # KCF, tamamen eski
./scripts/run_tracker.sh --width 1280 --height 720 --ortrack    # manuel ROI + ORTrack
./scripts/run_tracker.sh --width 1280 --height 720 --hybrid     # YOLO + ORTrack, OTOMATİK
./scripts/run_tracker.sh --width 1280 --height 720 --hybrid-kcf # YOLO + KCF,     OTOMATİK
```

**Beş mod:**

| bayrak | tracker | hedef seçimi |
|---|---|---|
| (yok) | KCF | manuel ROI (CH8/CH13/CH5) |
| `--ortrack` | ORTrack | manuel ROI |
| `--hybrid` | ORTrack | YOLO, otomatik |
| `--hybrid-kcf` | KCF | YOLO 20/30 kilitler, sonra sadece KCF |
| `--detect-only` | **yok** | sadece YOLO kutusu (tanılama) |

`--hybrid-kcf` ORTrack motorunu **hiç yüklemez**; sadece YOLO gerekir.

| Bayrak | Varsayılan | Ne |
|---|---|---|
| `--width/--height` | 1280×720 | **yayın** boyutu |
| `--proc-out` | kapalı | işlemeyi de yayın boyutunda yap (eski) |
| `--ortrack` / `--hybrid` / `--hybrid-kcf` | kapalı | tracker seçimi |
| `--ortrack-min-box N` | 4.0 | kutu tabanı (eski 10.0) |
| `--ortrack-min-score S` | 0 (kapalı) | Hann tepe eşiği |
| `--ortrack-lost N` | 5 | **manuel** modda N geçersiz kare → kilit düşer |
| `--yolo-conf S` / `--yolo-iou S` | 0.35 / 0.5 | dedektör eşikleri |
| `--hyb-legacy` | kapalı | eski manager parametreleri |
| `--hyb-search-interval N` | 2 | arama sırasında YOLO aralığı |
| `--hyb-verify-interval N` | 0 (kapalı) | kilitliyken periyodik YOLO doğrulaması |
| `--hyb-verify-miss-patience N` | 3 | kaç ardışık desteksiz doğrulamada bırakılır |
| `--hyb-low-score S` / `--hyb-recover-score S` | 0.4 / 0.5 | zayıf/toparlandı histerezisi |
| `--hyb-accept-conf S` | 0.4 | YOLO kutusu kabul eşiği |
| `--hyb-max-growth S` | 2.5 | kutu ölçek sürüklenmesi sınırı (0 = kapalı) |
| `--hyb-confirm-hits N` | 2 | kilit için ardışık eşleşen tespit |
| `--hyb-low-patience N` / `--hyb-miss-patience N` | 3 / 2 | zayıf yol sabırları |
| `--hyb-draw-dets` | kapalı | tüm YOLO kutularını çiz (tanılama) |
| `--box-smooth N` | 0 (kapalı) | kutu w/h için N kareli medyan |
| `--track-csv YOL` | — | kare kare CSV (ham **ve** yumuşatılmış) |

---

## 6. 1080p işleme — ne kazandırıyor, ne kazandırmıyor

**ORTrack'te gerçek kazanç var.** Arama kırpması `sqrt(w·h)·4` native pikselden
alınıp 256'ya ölçekleniyor:

| | hedef | kırpma | 256'ya |
|---|---|---|---|
| 720p | 30 px | 120 px | **2.13× büyütme** (saf interpolasyon) |
| 1080p | 45 px | 180 px | 1.42× büyütme |
| 1080p | 90 px | 360 px | **0.71× küçültme** (gerçek doku) |

Yan fayda: belgelediğin `min_box_size` arızası (uzaklaşan drone'da kutu tabana
çakılıp tracker'ın ölçeği kaybetmesi) 1080p'de 1.5× daha geç tetikleniyor.

**YOLO tam kare tespitte kazanç SIFIR.** Karar notu "sınırlı kalır" diyor, ama
tam olarak sıfır:

| Kaynak | letterbox 768 | etkin |
|---|---|---|
| 1280×720 | r = 0.600 | **768×432** |
| 1920×1080 | r = 0.400 | **768×432** |

Native crop tespiti (karar notu bölüm 6) bunu çözer ama doğrulanmış Python
yolundan sapmak demek — bilinçli olarak yapılmadı.

**KCF pahalılaşmıyor:** `kcftracker.cpp:143` → `template_size = 96`,
`cell_size = 4`, `padding = 4.8`. Patch her hâlükârda 96'ya ölçeklenir.
Karar notundaki açık soru buydu, kapandı.

---

## 7. Yerden ne göreceksin

Sol üst rozet (OSD'nin `drawTopLeft`'i kapalı, çakışma yok):

```
KCF                                         ← manuel, takip yok
ORTRACK  s=0.83  9.4ms                      ← manuel, takipte
HYBRID tracking s=0.62  trk=9ms det=43ms    ← hibrit
```

Kutu renkleri (hibrit):
**mavi kalın** kilit sağlam · **turuncu kalın** kilit var ama skor düşük ·
**sarı ince** aday · **yeşil ince** ham YOLO kutuları (`--hyb-draw-dets`)

Konsolda 1 Hz:

```
[TRK] HYBRID tracking s=0.62 trk=9ms det=31ms fps=29.4 dt=34.0/58.2ms slow=2 valid=1 bboxA=0.00412 st=tracking hits=2 low=0 miss=0 unver=0 gen=3 rej=0
[HYB] verifying -> tracking (confirmed_acquisition)
```

`det=` **son ölçülen** YOLO süresidir, canlı değil — kilitliyken YOLO durduğu
için (`verify_interval=0`) o sayı donar. Canlı gösterge `yolo=N/M`: saniyedeki
çağrı sayısı / ham kutu sayısı. Kilitliyken `yolo=0/0` beklenir.

`fps` son 1 saniyede **ölçülen** kare hızı, `dt` ortalama/en kötü kare
periyodu, `slow` 40 ms'yi aşan kare sayısı. **`drops=0` fps'in yerinde
olduğunu kanıtlamaz** — capture yavaşlarsa appsrc'ye daha az kare gider,
encoder rahat yetişir, sayaç 0 kalır. Asıl ölçü `fps`.

`bboxA` normalize bbox alanı — **büyümüyorsa hedefe yaklaşılmıyordur.**
27 Ağustos'taki ikinci uçuşta bbox 36 saniye 0.019'da donmuştu; artık loga düşüyor.

---

## 8. Hibrit mod — davranış

Hedefi **manager seçiyor**, CH12 gerekmiyor: 2 ardışık eşleşen YOLO tespiti
(`confirmation_hits`) → otomatik kilit. Güvenli, çünkü asıl kapı controller
tarafında: **CH6 yüksek olmadan hiçbir şey FC'ye komut göndermiyor.**

Sınıf filtresi: model 4 sınıflı (`airplane, bird, drone, helicopter`), sadece
`drone` tutuluyor. Ultralytics semantiği birebir: en yüksek sınıfı `bird` olan
bir kutu, `drone` skoru eşiğin üstünde olsa bile atılır.

**Pilot ana anahtarı: CH5 düşük = kilidi bırak.** Hedefi manager seçiyor, ama
pilotun yanlış kilidi reddetme yolu olmak zorunda. CH5 düşükken manager hiç
sürülmez (yakalama duraklar, rozet `HYBRID beklemede (CH5)`), CH5 yükseğe
alınca sıfırdan arar. RC geçersizken (tezgâh, FC yok) hibrit çalışır.

**Manuel moddan iki bilinçli fark:**

1. **Güvenli alan kilidi düşürmüyor.** Manuel modda %85 alandan çıkmak takibi
   durdurur. Hibritte durdurmaz — manager'ın kendi kayıp mantığı var ve
   düşürürsek YOLO hemen yeniden yakalar, kenarda çırpınma olur. Bunun yerine o
   karelerde beyne `valid=0 / state=lost` gider; hedef geri girerse takip sürer.
2. **Geçersiz tracker karesi anında kayıp sayılır** (`--ortrack-lost` hibritte
   yok sayılır). Doğru, çünkü YOLO yeniden yakalayabilir. Manuel modda böyle bir
   kurtarma yok, orada 5 kare sabır var.

**FRAME protokolü değişmedi.** State stringleri `idle`/`tracking`/`lost`.
Manager'ın `verifying` durumu FRAME'e **yansıtılmıyor** — yansıtsaydık
`controller_orin.py:332` yüzünden beyin sürekli ayrılıp bağlanırdı. Kilit varken
hep `tracking` gider; gerçek durum rozette ve `[TRK]` logunda.

`fcx/fcy/fbw/fbh` zaten normalize (0..1), çözünürlük değişimi PID ayarını
bozmaz. **Böyle tutulmalı.**

---

## 9. Parametreler

Varsayılanlar `configs/orin-camera-quality-768.yaml`'dan:

| | quality-768 (varsayılan) | `--hyb-legacy` |
|---|---|---|
| `min_box_size` | 4.0 | 10.0 |
| `tracker_low_score` | 0.4 | 0.2 |
| `tracker_recover_score` | 0.5 | 0.3 |
| `detection_accept_confidence` | 0.4 | 0.5 |
| `max_box_growth` | 2.5 | kapalı |

Açılışta `manager : accept=0.4 low=0.4 recover=0.5 growth=2.5` satırını
**doğrula**. `low=0.2` görüyorsan eski bir ikili çalışıyordur.

Senin ölçtüğün arızayı hedefliyor (kutunun kareyi yutması, 1489 takip karesinin
%12.4'ü ≥1000 px). Ama kendi config notun geçerli: *"Unvalidated: no annotated
evaluation exists yet."* Karşılaştırma için `--hyb-legacy` duruyor.

---

## 10. İlk uçuş — tek değişken disiplini

Her adımda **bir** şey değişsin:

1. **Yerde:** `ctest` → üç test de geçsin.
2. **Yerde:** iki motor scripti → ORTrack ~7 ms, YOLO'nun gerçek süresini not al.
3. **Yerde, kamera bağlı, motorsuz, KCF, 1080p işleme:** `[LINK] drops` artıyor
   mu, `[RTSP] encoder geride` var mı? **Sadece 1080p'yi test ediyorsun.**
4. **Havada, KCF, 1080p işleme:** uzak hedefte kilit tutuyor mu? Referans uçuş.
5. **`--ortrack`:** aynı hedefe kilitlen, `[TRK]` loglarını 4. adımla
   karşılaştır. `s` dağılımını not al.
6. **Yerde, `--hybrid --hyb-draw-dets`:** YOLO neyi buluyor, kaç fps kalıyor?
   Arama ~20 fps, kilitten sonra 30 (`verify_interval=0`).
7. **Havada `--hybrid`,** teacher YOK, sadece takip.
8. **Ancak bundan sonra** `--ortrack-min-score` / `--hyb-verify-interval` ekle —
   değerleri 5-7. adımlardaki gerçek veriden seç.

---

## 10b. Sahte kilidi kesmek — ölçülmüş davranış

**`verify_interval=0` iken sistem yanlış kilitten ASLA vazgeçmez.** Tasarım
böyle: kilit varken ve tracker zayıf değilken YOLO hiç çağrılmıyor. ORTrack
0.45'te takılı kalmış bir sürüklenmede "sağlıklı" sayılır, doğrulama yolu
açılmaz, ekrandaki apaçık drone tespit bile edilmez. Manager ayrıca yanlış
kutudan daha iyi bir tespite **atlamaz** — önce kaybetmesi gerekir.

`--hyb-verify-interval 10` bunu açar, ama tezgâh ölçümünde
`verify_miss_patience=3` fazla sert çıktı: YOLO recall'ü düşük olduğu için
(`yolo=3/1`, `yolo=2/1`) doğru kilitler de düştü. Başlangıç noktası:

```
--hyb-verify-interval 15 --hyb-verify-miss-patience 5
```

Yanlış kilidi ~2.5 s'de keser, doğru kilidi bırakma olasılığı belirgin düşer.

**`trk` süresi kutu boyutunun bedava göstergesi.** ORTrack arama kırpması
`sqrt(w·h)×4`; kutu kareyi yutunca 2700 px'lik bölge kırpılıp 256'ya iniyor.
Normal 9-12 ms; **20 ms üstü = kutu şişmiş**. Tezgâhta `bboxA=0.58 s=0.97
trk=25ms` görüldü — tam da quality-768 config'inde belgelediğin arıza.

## 10c. Kutu titreşimi

ORTrack kutu boyutunu **her karede sıfırdan** tahmin eder (`size_map`),
hiçbir zamansal sönümleme yoktur. KCF çok ölçekli modda sönümler, ORTrack
sönümlemez. `bboxA` yakınlık vekili olarak kullanıldığı için bu titreşim
beyin tarafını bozabilir.

Önce **ölç**:

```bash
./scripts/run_tracker.sh --width 1280 --height 720 --hybrid \
  --track-csv ~/track_$(date +%m%d_%H%M).csv
```

CSV sütunları: `frame,t_ms,state,valid,cx,cy,w_raw,h_raw,w_out,h_out,score,trk_ms,det_ms`.
Ham ve yumuşatılmış ayrı sütunlarda — hiçbir şey gizlenmiyor.

Bakılacaklar:
- `w_raw`/`h_raw` kare kare ne kadar oynuyor (yüzde olarak)
- `cx`/`cy` da oynuyor mu, yoksa sadece boyut mu? Merkez skor tepesinden
  gelir ve **farklı bir kafadır**; boyut kadar oynamaması beklenir. Oynuyorsa
  sorun daha derin.

Sonra filtrele:

```bash
--box-smooth 5
```

5 kareli **medyan** (ortalama değil): tek karelik sıçramaları tamamen siler,
sürekli değişimde — yani yaklaşırken — gecikme yaratmaz. Sadece `w/h`
yumuşatılır, merkez ham kalır.

**Tracker'ın kendi kutusuna dokunulmaz.** Yumuşatılmış kutuyu geri beslemek
arama kırpmasını değiştirir ve doğrulanmış yoldan saptırırdı. Bu sadece çıkış
tarafı: FRAME'e giden ve ekrana çizilen kutu.

## 10d. `--hybrid-kcf` — neden ve nasıl

Üç modun kare kare CSV karşılaştırmasından (15 Eylül, tezgâh):

| | merkez gürültüsü p95 | kutu adımı p95 |
|---|---|---|
| **KCF** | **0.08-0.46°** | **5.1%** |
| ORTrack | 1.29-2.04° | 30.0% |

KCF merkez gürültüsünde 4-10 kat, kutu kararlılığında 6 kat daha iyi. Sebebi
`scale_step`: KCF boyutu basamaklı değiştirir, ORTrack her karede `size_map`'ten
sıfırdan tahmin eder. ORTrack'in sorunu titreşim değil **kaçış**: `h_raw`
0.17'den 0.58'e çıkıp 5 saniye orada kaldı — medyan filtresi bunu düzeltmez.

KCF'in eksiği kaybettiğini fark edememesiydi; manager tam olarak onu çözüyor.

**KCF `peak_value` artık dışa açık** (`kcftracker.hpp`, iki satırlık ekleme).
Rozette, `[TRK]` satırında ve CSV'de görünür — manuel KCF modunda da.

**Skor kapısı `--hybrid-kcf`'te KAPALI.** KCF tepe yanıtı ORTrack'in Hann'lı
tepesiyle aynı ölçekte değil ve kalibre edilmedi. Uydurma eşik koymak yerine
`tracker_low_score = tracker_recover_score = 0` yapıldı: kayıp kararı tamamen
YOLO doğrulamasından gelir. Bu yüzden `verify_interval` **zorunlu** — verilmezse
otomatik 15 yapılır ve bu konsola yazılır.

Dağılımı ölçtükten sonra açabilirsin:

```bash
--hybrid-kcf --track-csv ~/kcf_peak.csv     # score sütunu = KCF tepe yanıtı
--hybrid-kcf --hyb-low-score 0.2 --hyb-recover-score 0.3   # ölçtükten SONRA
```

Herhangi birini açıkça verirsen otomatik varsayılan devreye girmez.

## 10e. KCF uyarlamalı padding — büyük kutuda çökmeyi önler

15 Eylül tezgâh ölçümünde `--hybrid-kcf` kilidi ortalama 2.5 saniyede
düşüyordu. Sebep manager değil, KCF'in arama penceresiydi.

KCF penceresi = `kutu × padding` (4.8), ve bu pencere sabit bir şablona
(96 px) indirilip **o karedeki içerikle bir filtre eğitilir**. Pencere kareyi
taşarsa taşan kısım **siyah** dolar (`recttools.hpp:102`, BORDER_CONSTANT).

O siyah bant **kareye sabittir, hedefe değil.** Hedef hareket eder, bant
yerinde kalır. Filtre için en tutarlı desen odur, dolayısıyla en iyi eşleşme
hep bandın hizalandığı yer olur ve **kutu donar**. Ölçüm: 355×370 px kutu →
pencere 1704×1776, kare 1920×1080 → %39 siyah → kutu 90 kare hiç kımıldamadı,
hedef kareyi boydan boya geçti (kilit düştüğü an ile yeniden yakalandığı an
arasındaki IoU medyanı **0.038**).

Çözüm — pencereyi kareye sığdır:

```
padding = min(4.8, 0.9 × min(kareW/kutuW, kareH/kutuH)),  taban 1.5
```

| kutu (1080p) | alan | padding | pencere | |
|---|---|---|---|---|
| 40×26 (uzak) | 0.05% | 4.80 | 192×124 | no-op |
| 100×70 | 0.34% | 4.80 | 480×336 | no-op |
| 200×140 | 1.35% | 4.80 | 960×672 | no-op |
| 225×225 | 2.44% | 4.32 | 972×972 | devreye girdi |
| 355×370 (arıza) | 6.33% | 2.63 | 932×972 | devreye girdi |

**Uçuş boyutlarında tam no-op** — 1080p'de eşik ~225 px kutu yüksekliği
(karenin %21'i). Manuel KCF modunda da aynı güvenlik var, davranış değişmez.
Devreye girdiğinde konsola `[KCF] kutu ... buyuk -> padding ...` düşer.

### Büyük kutuda: padding düşürmek yerine kutuyu küçült

Uyarlamalı padding pencereyi sığdırıyordu ama KCF'in geometrisini bozuyordu.
Gauss eğitim hedefi ve kosinüs penceresi hedefin pencerenin 1/4.8'ini
kaplamasına göre kurulu. 15 Eylül ölçümü:

| | kutu | padding | merkez gürültüsü p95 |
|---|---|---|---|
| manuel KCF | 154 px | 4.80 | **0.46°** |
| hibrit (eski) | 346 px | 3.02 | 1.21° |

Artık KCF'e kutunun `shrink` katı veriliyor, **raporlanırken geri büyütülüyor**.
KCF ölçeği tek katsayıyla değiştirdiği için oran korunur — bildirilen kutu ve
bbox yakınlık vekili **değişmez**.

| kutu | shrink | KCF kutusu | padding |
|---|---|---|---|
| 154×154 | 1.00 | 154×154 | 4.80 (no-op) |
| 225×225 | 1.00 | 225×225 | 4.80 (no-op) |
| 346×322 | 0.70 | 242×225 | **4.80** |
| 546×511 | 0.45 | 246×230 | 4.23 (taban) |

Küçültme tabana (0.45) dayanırsa padding devralır — iki aşamalı.

Kalan sınır: KCF **en-boy oranını dondurur** (w ve h aynı katsayıyla
ölçeklenir). Yaklaşırken hedefin duruşu değişirse oranı takip edemez. Alan
uyum sağlar, ki yakınlık vekili olarak kullanılan zaten o.

## 10f. `--hybrid-kcf` — bir kez kilitle, karışma

`TargetManager` **kullanılmıyor** (`simple_lock.hpp`).

| durum | ne oluyor |
|---|---|
| **ARAMA** | YOLO ayrı thread'de sürekli. Son **30** sonucun **20**'sinde aynı yerde drone → KCF'i bir kez kur |
| **KİLİT** | **Sadece KCF, sonsuza kadar.** Tespit durur, thread boşta, kare periyodu sabit |

Önceki sürümde kilitliyken periyodik kontrol ve "daha iyi hedefe geçiş" vardı.
Kaldırıldı: geçişler 11-22°'lik ışınlanmalar üretiyordu (1337 karede 5 kez) ve
kutunun büyümesini zaten düzeltmiyordu.

### Kilidi düşüren üç yol — üçü de tespit onayından bağımsız

| | ne zaman | bayrak |
|---|---|---|
| 1 | KCF 30 kare üst üste geçersiz kutu verir | `--kcf-invalid-limit` |
| 2 | Kutu 60 kare güvenli alan (%85) dışında kalır | `--kcf-outside-limit` |
| 3 | **Varlık kontrolü** — N karede 1 tespit, karede **hiç** drone yoksa sayar; 3 kez üst üste boşsa düşer | `--kcf-presence-check` (**varsayılan 0 = KAPALI**) |

3. yol kritik bir ayrımla çalışır: *"benim kutumu onaylamadı"* değil, *"karede
drone **yok**"*. YOLO arada kaçırsa bile bir yerde drone görüyorsa kilit durur.
Drone'u 180° çevirip hedefi kaybetme testinde 2. yol genelde yeter; KCF
karenin ortasındaki bir buluta/direğe yapışırsa 2. yol ateşlemez, 3. yol gerekir.

**CH5 bu modda kullanılmıyor** (uçuş modu anahtarı).

### Büyüme sınırı (referans güncellemeli)

Kutu **50 px**'in üstündeyken referansın **2.5** katını aşarsa, aynı merkezde
referansın **1.5** katına çekilir ve **KCF yeniden kurulur** (biriken arka plan
modeli silinir). Sonra **referans yeni boyuta güncellenir.**

```
 50×50 kilit → 125×125'te tetikler → 75×75'e döner   (yeni ref 75)
100×100 kilit → 250×250'de tetikler → 150×150'e döner (yeni ref 150)
```

Referansın güncellenmesi sayesinde gerçek yaklaşmada kutu basamak basamak
büyüyebilir; sürüklenme her basamakta kesilir. 50 px altında kural hiç
uygulanmaz — uzak hedefte kutu serbest büyür.

Bayraklar: `--kcf-growth-trigger 2.5 --kcf-growth-reset 1.5 --kcf-growth-min-px 50`

### Rozet ve log

```
KCF-LOCK search 14/30
KCF-LOCK locked s=0.42  trk=14ms  23s  clamp2
[LOCK] locked : lock 20/30  (612,388 340x310)
[LOCK] locked : growth_clamp x2.58  878x800 -> 510x465  (ref 340x310 -> 510x465)
[LOCK] search : outside_safe_zone
```

## 10g. `--detect-only` — sadece tespit

Tracker yok. YOLO ayrı thread'de sürekli koşar, bulduğu **tüm** drone kutuları
çizilir: en güvenli olan kalın mavi, diğerleri ince yeşil. Ham dedektör
performansını sahada görmek için — hangi menzilde yakalıyor, ne sıklıkta
kaçırıyor, neye yanlış drone diyor.

Tespitler ~2 karede bir geldiği için aradaki karelerde son liste çizilmeye
devam eder; `--det-hold N` kare (varsayılan 15 ≈ 0.5 s) yeni sonuç gelmezse
kutu kaybolur. Bu sadece **çizim** içindir, yapay bir takip değildir.

FRAME'e en güvenli tespit gider (`valid=1`, `state=tracking`). **Dikkat:** bu
veri kare kare atlar, tracker gibi sürekli değildir. Beyin bağlıyken CH6'yı
açarsan hedefi sıçraya sıçraya kovalar. Tanılama modu olarak düşün.

Rozet: `DETECT n=2  c=0.81  det=52ms` — kaç kutu, en iyisinin güveni, son
tespit süresi. Log: `[TRK] ... st=detect n=2 age=1 yolo=18/3`.

## 11. Bilinen sınırlar

- **Üç parça ayrı ayrı doğrulandı, hiçbir yerde birlikte çalıştırılmadı.**
  Gerçek boru hattı ilk kez senin Orin'inde dönecek.
- **YOLO 768 ölçüldü: 26.05 ms** (median 25.6, p95 28.0, p99 31.2 — Orin,
  `trtexec --noDataTransfers`). Önceki ~42 ms tahminim yanlıştı; referans
  aldığım "TRT 512 = 25.5 ms" ultralytics üzerinden ölçülmüştü, yani Python
  overhead'i inference diye etiketlenmişti. Gerçek 512 muhtemelen ~12 ms.
- **Kare başına gerçek maliyet 26 ms değil.** `--noDataTransfers` H2D/D2H'yi
  saymıyor; runtracker'da 7.1 MB giriş transferi + letterbox + normalize var.
  Beklenen ~31-37 ms. `[TRK] fps=` satırından doğrula.
- **Arama sırasında ölçülen: 21-25 fps** (1080p kaynak, `search_interval=2`).
  `det=` 35-66 ms, ortalama ~47 — trtexec'in 26 ms'si sadece GPU'ydu
  (`--noDataTransfers`); üstüne letterbox + normalize + 7.1 MB transfer geliyor.
  Kilitlendikten sonra `verify_interval=0` olduğu için YOLO durur ve 30 fps'e
  dönülür. Aramayı hızlandırmak yerine akıcılık istiyorsan
  `--hyb-search-interval 3`.
- **`slow` sayacı boşta bile 12 civarı.** Bu bizim işlememiz değil, RTSP paket
  varış jitter'ı: `--proc-out` ile birebir aynı çıkıyor.
- **YOLO recall'ü düşük:** kendi ölçümünde 934 çağrının 21'inde kutu üretmiş.
  Yeniden yakalama uzun sürebilir.
- **Şablon yenilenmiyor** (`refresh_template: false`).
- **Motor taşınabilir değil.** Flash / JetPack / TensorRT değişimi → yeniden üret.
- **`--width 1920 --height 1080`** verilirse resize no-op olur ve `clone()`
  devreye girer (capture buffer'a çizmemek için) — kare başına bir kopya maliyeti.

---

## 12. Karar notundan kalan açık işler

Bu değişikliklerden bağımsız: SIYI FPV UDP mi TCP mi · RTCP RR geliyor mu ·
1080p30'da superfast/veryfast yetişiyor mu · Main profile kabul ediliyor mu ·
bitrate kalibrasyon uçuşu (`g_tiers`) · `videoconvert` BGRx optimizasyonu ·
native-res crop = bedava zoom.
