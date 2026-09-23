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
./scripts/build_yolo_engine.sh        # beklenen ~42 ms (TAHMİN — ölç!)
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
```

| Bayrak | Varsayılan | Ne |
|---|---|---|
| `--width/--height` | 1280×720 | **yayın** boyutu |
| `--proc-out` | kapalı | işlemeyi de yayın boyutunda yap (eski) |
| `--ortrack` / `--hybrid` | kapalı | tracker seçimi |
| `--ortrack-min-box N` | 4.0 | kutu tabanı (eski 10.0) |
| `--ortrack-min-score S` | 0 (kapalı) | Hann tepe eşiği |
| `--ortrack-lost N` | 5 | **manuel** modda N geçersiz kare → kilit düşer |
| `--yolo-conf S` / `--yolo-iou S` | 0.35 / 0.5 | dedektör eşikleri |
| `--hyb-legacy` | kapalı | eski manager parametreleri |
| `--hyb-search-interval N` | 2 | arama sırasında YOLO aralığı |
| `--hyb-verify-interval N` | 0 (kapalı) | kilitliyken periyodik YOLO doğrulaması |
| `--hyb-draw-dets` | kapalı | tüm YOLO kutularını çiz (tanılama) |

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
[TRK] HYBRID tracking s=0.62 trk=9ms det=43ms valid=1 bboxA=0.00412 st=tracking hits=2 low=0 miss=0 unver=0 gen=3 rej=0
[HYB] verifying -> tracking (confirmed_acquisition)
```

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

## 11. Bilinen sınırlar

- **Üç parça ayrı ayrı doğrulandı, hiçbir yerde birlikte çalıştırılmadı.**
  Gerçek boru hattı ilk kez senin Orin'inde dönecek.
- **YOLO ~42 ms tahmindir.** PyTorch oranından türetildi (512: 41.2 → 25.5).
  `trtexec` çıktısı gerçeği söyler.
- **Arama sırasında video ~20 fps.** YOLO senkron koşuyor (Python
  pipeline'ındaki gibi — manager'ın katı "aynı kare" politikası gerektiriyor).
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
