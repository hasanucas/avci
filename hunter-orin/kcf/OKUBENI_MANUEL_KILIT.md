# HUNTER — manuel kilit butonu (`--ch-lock`) — 9 Kasım 2026

İstenen akış: drone öne giderken detection çalışır, **pilot tuşa basar**, detection
komple durur, o pencere KCF'ye devredilir, pilot iptal edene kadar KCF çalışır.

```
kcf/
  include/simple_lock.hpp        YAMALI  forceLock() + manualMinHits + manualOnly
  runtracker.cpp                 YAMALI  --ch-lock N, CH latching okuma, döngü bağı
  test_manual_lock_offline.py    YENİ    driver — A 36/36, B 13/13
  test_manual_lock_logic.cpp     YENİ    SimpleLock mantığı (gerçek başlığı derler)
  test_manual_lock_rc.cpp        YENİ    readRcControl (gövdeyi runtracker'dan çeker)
  OKUBENI_MANUEL_KILIT.md        YENİ    bu dosya
```

Sahaya çıkmadan:

```bash
cd kcf
python3 test_manual_lock_offline.py    # A 36/36 + B 13/13
```

---

## Neden bu kadar az kod yazıldı

İstenen davranış zaten `--hybrid-kcf`'in tasarımı. `simple_lock.hpp` başlığındaki
15 Eylül mutabakatı aynen bunu diyor:

> **ARAMA:** YOLO sürekli koşar. Son 30 sonucun 20'sinde aynı yerde drone varsa →
> KCF'i bir kez kur → KİLİT.
> **KİLİT:** SADECE KCF. Tespit DURUR (thread boşta). Hiçbir şey karışmaz.
> Kilit "YOLO onaylamadı" diye ASLA düşmez.

Eksik olan tek şey **tetiğin pilotta olması**. `forceLock()` aynı kapıyı
(`initBox_` + `wantInit_`) daha erken açıyor — **ayrı bir kod yolu değil**.
Kilit kurulumu, büyüme referansı, düşme koşulları otomatik yolla birebir aynı.

`--detect-only` ile **çalışmaz**: o modda hiç tracker nesnesi kurulmuyor
(`tracker` null kalıyor). `--hybrid` ise YOLO+ORTrack, KCF değil.
Bu özellik **sadece `--hybrid-kcf`** modunda anlamlı.

---

## Nasıl uçulur

```bash
cd ~/Desktop/avci/hunter-orin
T=$(date +%m%d_%H%M)
./scripts/run_tracker.sh --width 1280 --height 720 --hybrid-kcf \
  --yolo-engine ../../models/yolo11s-plane-640x768.engine --yolo-conf 0.25 \
  --ch-lock 9 \
  --track-csv ~/track_lock_$T.csv --lock-snap ~/kilit_$T --rec-raw ~/talon_lock_$T.ts
```

Açılışta şu satırı **oku**:

```
  kilit   : MANUEL — CH9 latching butonu (otomatik 20/30 kilit KAPALI)
            buton ACIK  -> aday >=3 karede tutturmussa KCF'e devredilir, YOLO DURUR
            buton KAPALI-> kilit birakilir, YOLO yeniden baslar
```

Uçuşta:

| Buton | Davranış |
|---|---|
| KAPALI | YOLO arıyor, aday kutu ince çizgiyle görünür, rozette `btn kapali` |
| BASILI, aday zayıf | rozette `[BTN bekliyor: aday yok]` ya da `kanit az` — **henüz kilitlenmedi** |
| BASILI, aday yeterli | `[LOCK] locked : manual_lock #1 5/30 (...)`, YOLO durur, rozet `[BTN]` |
| BIRAKILDI | `[LOCK] search : manual_release (buton birakildi)`, YOLO yeniden başlar |

Rozette `m2` gibi bir sayı görürsen butonla kaç kez kilitlendiğini gösterir.

---

## Üç tasarım kararı — ve nedenleri

### 1. Otomatik 20/30 kilidi KAPATILDI (`manualOnly`)

İkisi birden açık kalırsa makine pilottan **önce** kilitler ve tespit durur —
yani buton işlevsiz kalır, üstelik "detection çalışırken ben basınca" akışı
bozulur. `--ch-lock N` verilince `manualOnly = true` olur ve açılışta basılır.

Arama mantığı (aday takibi, pencere sayaçları) **yine koşar** — buton bastığında
`forceLock()` onlara bakıyor.

### 2. Butonun devrettiği kutu: ADAY, en yüksek skorlu tek kare DEĞİL

En yüksek skor false positive'e karşı **en zayıf** seçenek: bastığın karede bir
bulut/doku tesadüfen yüksek skor verirse ona kilitlenir, ve YOLO o andan sonra
kapalı olduğu için düzeltecek hiçbir şey kalmaz.

27 Ağustos ikinci uçuşu tam bu sınıftaydı: KCF balonu kaybedip uzaktaki sabit
dokuya yapıştı, bbox 36 saniye 0.019'da dondu, irtifa 68→198 m.

`cand_` zaten zamansal kanıt taşıyor: IoU ile kareler boyunca yapışkan tutuluyor,
kaç karede tutturduğu sayılıyor. Tek karelik yanlış tespit aday olamıyor.

Açığı şu: ilk tespit karesinde `cand_` doğrudan en yüksek skorludan kuruluyor,
yani o anda ikisi aynı. Bu yüzden **asgari kanıt kapısı** kondu:
`manualMinHits` (varsayılan **3 kare ≈ 0.1 s**). Otomatik kilit 20/30 bekliyor,
buton 3'te geçiyor — çok daha hızlı, ama parlamaya kilitlenmiyor.
`--lock-manual-min-hits N` ile değiştirilir.

Offline test F bölümü bunu kilitliyor: tek karelik 0.99 skorlu sahte tespitte
buton kilitlemiyor.

### 3. Buton bir DURUM, kenar değil (latching'e uygun)

- **AÇIK + ARAMA** → her karede denenir, aday yeterli kanıta ulaşınca kilitlenir.
  Yani "erken bastım" reddedilmek yerine **"emin olunca kilitle"** anlamına gelir.
- **AÇIK + KİLİT** → dokunulmaz.
- **KAPANDI** → kenar tetiği, kilit bırakılır.

`release()` sürekli çağrılmıyor: ARAMA'da adayı da sildiği için her karede
çağırsak aday hiç birikmezdi.

**Kilit kendi kendine düşerse** (KCF geçersiz / kadraj dışı) buton açık olduğu
için yeniden kilitler — ama `manual_lock #2` diye basar, sessiz olmaz.

---

## GEÇERSİZ RC — en kritik detay

Butonun durumu `lastLock` ile **kalıcı** tutuluyor, CH8'in `lastRoi` mantığıyla
aynı. Sebebi: RC düştüğünde kanal 0 okunur; korumasız kalsa "pilot bıraktı"
sanılıp **uçuş ortasında kilit düşer** ve YOLO yeniden başlar.

Üç yol da korunuyor: kanal 0 okunursa, tüm RC düşerse (erken dönüş yolu), ve
ara bölgede (1350–1600) kalırsa son durum korunur.

Offline test B/D bölümü bunu kilitliyor.

---

## UÇMADAN ÖNCE BİLİNMESİ GEREKENLER

**1. `--hybrid-kcf` sahada hiç uçmadı.** README'de var (10d/10f, 15 Eylül tezgâh
ölçümünde kilit ortalama 2.5 s), ama 6 Kasım saha akışında yok — orada A (manuel
KCF), B1 (detect-only), B2 (hybrid+ORTrack) var. B2 uçtu, bu uçmadı.
Yani bu uçuşta **iki yeni şey birden** var: mod ve buton. Tek değişken
disiplinini istiyorsan önce `--hybrid-kcf`'i butonsuz (otomatik kilitle) bir
pencerede gör.

**2. YOLO kapalıyken sahte kilidi yakalayacak bağımsız kontrol YOK.**
`SimpleLock`'ın üç düşme koşulu da tespitten bağımsız — ama üçü de 27 Ağustos
senaryosunu **kaçırır**: kutu geçerli, kadrajın içinde, büyümüyor.

`SimpleLock`'ta bunun için opsiyonel bir **varlık kontrolü** duruyor
(`--kcf-presence-check N`, varsayılan KAPALI): N karede bir YOLO koşar ve sadece
"kadrajda herhangi bir yerde drone var mı" sorar — "benim kutumu onayla" demez,
yani geçiş/ışınlanma üretmez. 30 karede bir ≈ %3 YOLO yükü.

Öneri: **ilk uçuşta kapalı tut** (istenen saf hal), sonra ayrı bir uçuşta
`--kcf-presence-check 30` ile aç. Tek değişken.

**3. Güç tarafında kazanç var.** Orin'in YOLO+ORTrack yükünde 16 V/3 A
regülatörde 1–1.5 dakikada kapandığı not edilmiş. Kilitten sonra YOLO ölünce o
yük angajmanın geri kalanında tamamen gidiyor. `--hybrid-kcf` ayrıca ORTrack
motorunu **hiç yüklemiyor**. Yer testi 3'ü (güç, ≥5 dk) bu modla tekrar koş —
büyük ihtimalle B2'den rahat geçer.

**4. GUIDED imleci bu modda da kapalı.** YOLO modlarında PTR kapısı OFF
(hedefi makine buluyor). Buton hedefi *seçmiyor*, adayı *onaylıyor*. Kadrajda
birkaç hedef varsa YOLO'nun sıralaması kazanır. Tek hedef beklentisi varsa
sorun değil; değilse imleci bu moda açmak ayrı bir iş.

---

## Açık işler

- [ ] **Sahada hiç uçmadı.** Offline A 36/36 + B 13/13 yeşil, o kadar.
- [ ] Masada CH9 ataması: kumandada latching butona CH9 ata, `--ch-lock 9` ile
      çalıştır, rozette `btn kapali` → `[BTN bekliyor: aday yok]` → bas-bırak
      geçişlerini gör. FC'ye hiçbir şey gitmiyor, pervanesiz bile gerekmez.
- [ ] `--hybrid-kcf` butonsuz bir pencere (tek değişken disiplini).
- [ ] `manualMinHits` 3 mü kalacak? Masada aday sayacının ne hızla dolduğunu gör
      (`[BTN bekliyor: kanit az]` ne kadar sürüyor).
- [ ] `--kcf-presence-check 30` kararı (madde 2) — ayrı uçuş.
- [ ] Saha akışı dokümanına yeni mod olarak eklenmesi (A / B1 / B2 / **B3**).
