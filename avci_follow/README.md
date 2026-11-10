# follow_gps — GPS ile drone-to-drone takip/kesme (filmcilik)

Takipçi drone, senin elle uçurduğun hedef drone'u **GPS'le** otonom kovalar: karşıdan
gelirsen **önünü keser** (intercept), çaprazda **arkadan takip eder** (follow).

**İki çalışma şekli, tek beyin** (`chase_core.TrackController` — ikisi birebir aynı mantığı uçurur):

| | Beyin nerede | Komut yolu | Script(ler) |
|---|---|---|---|
| **Laptop** | laptop | laptop → takipçi radyosu | `follow_gps.py` |
| **Orin + radyo (SAHA, 24 Ekim)** | takipçinin Orin'i | Orin → USB; 3DR Orin USB'sinde, sadece hedef verisi | Victus: `relay_radio.py`, Orin: `chase_orin.py --target-link ...` |
| Orin, eski (radyo FC'de) | takipçinin Orin'i | Orin → USB | laptop: `relay_target.py`, Orin: `chase_orin.py` |

Orin şekilleri aşağıda kendi bölümlerinde ("Orin + radyo düzeni", "Orin mimarisi"). Bu giriş laptop şeklini anlatır:
bütün komutlar laptopta iki telemetri radyosundan hesaplanıp takipçiye MAVLink GUIDED ile
gönderilir.

```
HEDEF (sen uçuruyorsun)  --telem-->  laptop  --telem-->  TAKIPCI (GUIDED)
     COM_hedef                         |                    COM_takipci
                                       | UDP tetik (ops.)
                                       v
                              ORIN (record_agent.py)
```

"Beyin", attığın `fpv_gate_il.trajectory` paketidir (radar/GPS → Kalman →
INTERCEPT/FOLLOW → kayan carrot). Paket **birebir** taşındı; sadece Gazebo'ya bağlı
tek `import` satırı nötrlendi. Planlayıcı gerçek takipçiye `chaser_env.ChaserEnv`
adaptörüyle bağlanır — yani planlayıcının kendi göndericisi, gerçek drone'a karşı
**değişmeden** çalışır. Transport katmanı (iki Vehicle, RX thread'leri, snapshot,
sysid kontrolü, kayıt tetiği, çıkışta LOITER, güvenlik geçitleri) `follow_relay.py`'den
alındı.

## Davranış seçimi kısıtsız
İstediğin buydu: hiçbir `--follow-only` yok. Seçici dosyadaki gibi çalışır —
sen karşıdan üstüne gelirken INTERCEPT'e geçip takipçiyi **önüne kesme noktasına**
(hedefin önündeki bir noktaya) gönderir; çapraz/uzaklaşan geometride kendiliğinden
FOLLOW'a düşer.

## Dosyalar
```
chase_core.py       ORTAK BEYIN: TRACK_PLAN tiki (geçitler, yerinde-tut, KF, carrot) + ortak bayraklar
follow_gps.py       laptop şekli (iki radyo + ortak beyin)
relay_radio.py      Orin+radyo, VICTUS tarafı: hedef kaynağı (UDP/COM) -> 3DR; pasif; radyo RTT; takipçi durumu
relay_target.py     Orin eski şekil, LAPTOP tarafı: hedef paketlerini takipçi radyosuna aktarır (uçuş komutu yok; RTT ping'i)
chase_orin.py       Orin şekli, ORIN tarafı: beyin + faz makinesi (TRACK_PLAN; VISUAL/REALIGN kapalı); --target-link = radyo modu
mav_hub.py          Orin: radyo modunda hedef radyodan / takipçi FC'den (ayrı linkler); eski modda tek linkte sysid ile ayırma
chaser_env.py       pymavlink adaptörü (planlayıcı → gerçek drone)
link_log.py         telemetri/gecikme loglama
fpv_gate_il/        vendorlanan yörünge paketi (birebir; sadece mav_bridge'de Gazebo import'u nötr)
test_offline.py     planlayıcı mantığı regresyonu
test_wiring.py      montaj/adaptör entegrasyon testi
test_loop.py        follow_gps ANA DÖNGÜSÜ uçtan uca (sahte araçlar, sanal saat)
test_orin.py        Orin şekli: hub, sabitleme, relay, uçtan uca aktarım, canlı UDP, orkestratör
test_radio.py       Orin+radyo: link ayrımı, TIMESYNC, aşağı link, olaylar, yeniden bağlanma, relay_radio,
                    sinyaller, canlı uçtan uca (sahte Talon -> relay -> sanal 3DR çifti -> hub <-> sahte FC)
```
Hepsi donanımsız çalışır; Python 3.10 (Orin JetPack 6) ve numpy 1.21/2.x ile doğrulandı.

## Kurulum
```
pip install pymavlink pyserial numpy
```

## ZORUNLU iş akışı (sırasıyla)
1. **Önce offline testler** — her saha çıkışından önce (beşi de `FAIL=0` vermeli;
   `test_radio.py` sanal seri port kullandığı için Linux'ta/Orin'de koşar):
   ```
   python test_offline.py && python test_wiring.py && python test_loop.py && python test_orin.py && python test_radio.py
   ```
2. **Aynı kottan kalkış için yerde kontrol.** İki araç da aynı zemindeyken:
   ```
   python follow_gps.py --target-port COM12 --chaser-port COM5 --alt-ref same-home --dry-run --no-rec
   ```
   Her iki aracın home referansı oluştuktan sonra `olculen dikey ayrim (same-home)` sıfıra yakın olmalı.
   Beklenmeyen fark varsa iki aracın `rel` değerlerini ve home noktalarını kontrol et.
3. **Saha çalışması.** Varsayılanlar zaten filmcilik için ayarlı (temiz irtifa modu açık,
   3 m altta, ~200 m'ye kadar tam 20 m/s, 100 m içinde 10 m/s):
   ```
   python follow_gps.py --target-port COM12 --chaser-port COM5 --alt-ref same-home --set-mode
   ```
   Farklı bir dikey mesafe istersen sadece offset'i değiştir (ör. 5 m altta: --alt-offset -5).

`Ctrl-C` → takipçi LOITER. Kumandada mod anahtarı her an ezer; takipçi GUIDED'dan
çıkarsa script kapanır ve kontrolü sana bırakır.

## Hız davranışı — ÖNEMLİ
Eski "hep 10 m/s, üstüne çıkmıyor" sorunu burada çözüldü: başlangıçta `WP_SPD` tavanı
`--max-speed`'e çıkarılır **ve** her tikte `DO_CHANGE_SPEED` gönderilir (aktif GUIDED
hızını değiştiren tek şey budur). Başlangıçta `WP_SPD` geri okunup doğrulanır; tavan
kalkmadıysa uyarı basılır (firmware'de `WP_SPD` yerine `WPNAV_SPEED` olabilir).

Ama dikkat: **tepe hız sadece uzaktayken.** Şema, `--deadline-range` (varsayılan 100 m)
içinde bilinçli olarak `--close-approach`'a (varsayılan 10 m/s) iner — senin
"yaklaşınca yavaşla" isteğin. Yani:

| Mesafe | Komut hızı |
|---|---|
| `> --decel-start-range` (200 m) | tepe (`--max-speed`, 20) |
| 200 m → 100 m arası | doğrusal iniş |
| `< --deadline-range` (100 m) | taban (`--close-approach`, 10) |

Kısa menzilde test edersen 10 m/s görürsün — bu **doğru**. Kırılma mesafelerini
`--deadline-range` / `--decel-start-range` ile istediğin gibi ayarla.

## İrtifa
Orin ve laptop aynı `--alt-ref` seçeneğini kullanır. Çıkış her zaman takipçinin home kotundan yukarı metre cinsindendir.

| Referans | Hedefin takipçi home'una göre yüksekliği | Kullanım |
|---|---|---|
| `--alt-ref same-home` | Talon `GLOBAL_POSITION_INT.relative_alt` (m'ye çevrilir) | İki aracın home noktaları aynı fiziksel kottaysa. `--sep-bias` sıfır olmalı. |
| `--alt-ref amsl` (varsayılan) | Hedef AMSL − `sep_bias` − takipçi home AMSL | Önceki sürümün hesabı. Mutlak referansların uyumu doğrulanmalı. Takipçi home AMSL = takipçi AMSL − takipçi relative_alt. |

Dikey ayrım, tablodaki yükseklikten takipçinin `relative_alt` değeri çıkarılarak bulunur.
Komut için tablodaki yüksekliğe `--alt-offset` eklenir. `--min-sep` ölçülen ayrıma, `--min-alt` komuta uygulanır.
Aynı kotta kalkışta Talon 100 m ve ofset +10 m ise `--alt-match` ile komut 110 m olur.
Farklı kotları `same-home` telafi etmez. Seçilen irtifa verisi eksikse takip durur; takipçinin geçerli konumu ve relative_alt verisi varsa mevcut pozisyonu tutulur.

Orin `chase_state_*.csv` kayıtlarında `t_alt_rel`, `alt_ref`, `alt_offset_m` ve `sep_bias_m` her satırın sonundadır.
Yeni link loglarının başlıklarında da referans ve bias bulunur. Eski CSV'lerde bu alanlar yoktur.

- `--alt-offset D`: hedefin irtifasına göre ofset. **Pozitif = üstünde, negatif = altında.**
  Varsayılan **−3** (unutulsa bile küçük bir dikey marj). "5 m alttan takip" = `--alt-offset -5`.
- `--alt-match`: **varsayılan AÇIK.** Temiz mod — komut irtifası = hedef irtifası + offset,
  birebir, her tik yeniden. Filmcilik için istediğin davranış budur; normalde hep açık kalır,
  sen sadece `--alt-offset`'i değiştirirsin.
- `--no-alt-match`: temiz modu kapatıp dosyanın orijinal **balon** mantığına döner (hedef
  < 50 m ise +10 m/yakında +5 m üstünden geç, 50 m'de tavan). Drone çekiminde gerekmez.
- `--min-alt`: komut irtifası bu tabanın altına inmez (çok alçak hedefte yere komut vermeyi önler).

## GÜVENLİK — bir kez oku
İki **gerçek** drone, biri diğerinin önünü **20 m/s'ye kadar** hızla kesiyor, üstelik
GPS/EKF irtifa/konum kestiriminde birkaç metre hata payı var. Bu **gerçek bir çarpışma
riskidir** — özellikle INTERCEPT + küçük `|--alt-offset|`.

- **Dikey marj ver:** varsayılan `--alt-offset -3` seni *tam aynı hizadan* (en kötü durum)
  uzak tutar, ama 3 m yine de yakındır. Agresif karşıdan kesmelerde marjı büyüt (ör.
  `--alt-offset -8`). Aynı irtifa (`--alt-offset 0`) + intercept, iki EKF birkaç metre
  saparsa temas demektir.
- **RC'de moda hazır dur.** Nihai güvenlik sensin; takipçi GUIDED'dan çıkınca script çekilir
  ve **modu bir daha asla değiştirmez** (senin seçtiğin mod kalır). Script sadece Ctrl-C'de
  ya da beklenmeyen bir hatada, takipçi hâlâ GUIDED'daysa LOITER'a alır.
- **Bekleme = yerinde tut.** Hedef bayatlarsa, fix'i kaybederse, `--max-dist` aşılırsa ya da
  `--min-sep` tetiklenirse takipçi **olduğu yerde tutulur** (sim'in `--stale-hold` davranışı).
  Son carrot'a (250 m'ye kadar ileride) uçmaya devam etmez. Takipçinin kendi GPS'i yoksa
  tutamaz; o durumda komut gönderilmez, FC failsafe'leri devrededir.
- `--min-sep` opsiyonel bir **ölçülen dikey ayrım sigortasıdır** (varsayılan 0 = KAPALI):
  açıksa `|ayrim| < min-sep` olunca takipçi yerinde tutulur. Intercept doğal olarak yaklaştığı için
  varsayılan kapalıdır; **aynı irtifada geçiş** istiyorsan `--min-sep 0` bırak (yoksa
  amaçladığın yakın geçişte tetiklenir). Küçük bir dikey ofsetle uçuyorsan (ör. -10 m),
  `--min-sep 4` gibi bir taban güvenli bir yedek olur.
- `--max-dist` (kelepçe), GPS-fix geçitleri ve `--max-age` (bayat konum) hep açık.

## Loglama (varsayılan AÇIK, `--no-log` ile kapat)
> **Not:** Önceki sürümde hedef radyosunun mesajları yanlışlıkla `chaser_link` dosyasına
> yazılıyordu (`hedef_link` boş kalıyordu). Düzeldi; o sürümle alınmış balkon logları
> radyo-radyo ayrılamaz, yeniden alınmalı.

`--log-dir` (varsayılan `logs/`) altında dört CSV:
- `hedef_link_<zaman>.csv`, `chaser_link_<zaman>.csv`: her radyo için **mesaj başına**
  tip, önceki aynı-tip mesajdan bu yana geçen süre (gap), anlık Hz. `RADIO_STATUS`/`RADIO`
  mesajları ayrıca SiK link alanlarını yazar (rssi, remrssi, noise, remnoise, rxerrors,
  fixed, txbuf) — **B-cube v5 vs X-Rock klonu** hangi radyonun link telemetrisi gönderdiğini
  ve linkin ne kadar dolu olduğunu böyle görürsün.
- `relay_lat_<zaman>.csv`: her kontrol tikinde tek yönlü **veri yaşı** bütçesi — hedef fix
  yaşı, takipçi fix yaşı, planlayıcı hesap süresi, gönderim süresi, toplam + o anki
  davranış/hız/mesafe. Baskın terim genelde `target_age`: son hedef paketinin laptopa
  gelişinden beri geçen süre (paketin havada geçirdiği süre buna dahil değil).
- `cmd_lat_<zaman>.csv`: **komut → tepki gecikmesi.** İki tür olay:
  - `event=rtt`: takipçiye gönderdiğimiz bir komutun (`DO_CHANGE_SPEED`, ya da hız sabitken
    veri gelsin diye ~1 sn'de bir gönderilen zararsız `REQUEST_MESSAGE` ping'i) `COMMAND_ACK`
    ile geri dönüş süresi. Bu **gerçek gidiş-dönüş gecikmesidir** — takipçi komut linki iki
    yönde + FC işleme; saat senkronizasyonu gerektirmez (iki damga da laptopun saatinde).
    `event=rtt_lost` çok çıkıyorsa komut linkin paket düşürüyor demektir (link kalitesi ölçütü).
  - `event=spd_start`: bir hız komutundan sonra takipçinin bildirdiği yer hızının yeni
    hedefe doğru ilk kez kımıldadığı ana kadar geçen süre. **Bu, FC + fiziksel ivmelenmeyi
    (WP_ACC) içerir**, saf transport değildir — "ne kadar sürede gözle görülür tepki verdi"
    diye oku. Hızın hedefe *ulaşma* süresi zaten WP_ACC'ye bağlıdır (ör. 10→20, 4 m/s²'de
    2.5 sn), o yüzden transport sayısı için `rtt`'ye bak, tepki hissi için `spd_start`'a.

  Not: `cmd_lat` verisi **sadece gerçek uçuşta** dolar (dry-run'da FC'ye komut gönderilmez).
  Ping aralığını `--rtt-ping-s` ile ayarla (0 = kapalı).

Özetle: hangi radyo zayıf ve ne zaman kötüleşiyor (`*_link`), komutu üretirken veri ne kadar
bayat (`relay_lat`), ve komutun karşıya gidip cevap dönmesi ne kadar sürüyor (`cmd_lat` rtt) —
üçünü ayrı ayrı görürsün. Göremediğin tek şey: ortak saat olmadığı için bir paketin havada
geçirdiği **tek yönlü** süre; ama RTT'nin ~yarısı iyi bir tahmindir.

## Orin + radyo düzeni (SAHA, 24 Ekim 2026) — 3DR Orin USB'sinde

Takipçinin 3DR'si FC'ye lehimli değil, USB-C ile Orin'e takılı. Radyo **sadece hedef verisini**
taşır; komutlar ve takipçinin kendi durumu Orin↔FC USB'sinde kalır.

```
hedef kaynağı ──▶ Victus: relay_radio.py ──3DR──▶ Orin USB: chase_orin.py --target-link ... ──MAVProxy 14551/USB──▶ takipçi FC
(Talon/Kırmızı UDP                        ◀──3DR── takipçi durumu ~2 Hz + chase_orin olayları
 ya da hedef radyosu)
```

- Hedef radyodan, takipçi FC'den okunur (ayrı bağlantılar) → **aynı sysid olabilir** (Talon genelde 1).
  Hedef sysid'i radyoda görülen ilk araç heartbeat'ine kilitlenir (`--target-sysid N` ile zorla).
- Radyodan gelen hiçbir paket FC'ye iletilmez; FC'ye komut veren tek süreç chase_orin.
- Victus **pasif**: kaynağa (Talon/MP) tek bayt gitmez. Kendi hedefin serideyse `--request-hz 10`.
- Orin radyoya sadece şunları basar: Victus'un TIMESYNC'ine cevap (radyo RTT), takipçinin
  HEARTBEAT (1 Hz) / konum (`--downlink-hz`, 2) / fix (0.5 Hz) / FC STATUSTEXT'leri ve chase_orin
  olayları (başladı, intercept/follow, BEKLE, çıkış). `--downlink-hz 0` = sadece TIMESYNC cevabı.

### Kurulum
**Orin:**
1. Radyoyu tak; `dmesg | tail` (çip: FTDI/CP210x/CH340) ve `ls -l /dev/serial/by-id/` ile yolu al.
   Hep **by-id yolunu** ver: USB bir an çıkıp girerse numara (ttyUSB0/1) değişebilir, RX thread
   aynı yolu 1 s'de bir yeniden açar. CH340 çıkar da port kayboluyorsa sebep brltty'dir:
   `sudo systemctl mask --now brltty-udev.service` (Orin'de dpkg yarım; `apt remove` deneme).
   `groups` çıktısında `dialout` olmalı.
2. MAVProxy her zamanki gibi: `scripts/run_router_mavproxy.sh` (14551 = brain; değişiklik yok).
   14551'e başka süreç bağlanmasın; izlemek için 14552.
3. chase_orin **tmux içinde**: `tmux new -s takip` (SSH koparsa yeniden bağlan: `tmux attach -t takip`).

**Victus:** iki radyo çifti (hedef + Orin) farklı NetID'de olmalı. Aynı bantta iki 500 mW radyo
yan yana durursa biri yayın yaparken diğeri sağırlaşır; kendi hedef drone'unla test ederken hedef
radyosunu Kırmızı'ya alıp Ethernet'ten (Talon'daki gibi) UDP ile ver.

### Çalıştırma
```
Victus> python relay_radio.py --target udpin:0.0.0.0:14550 --radio COM5            # Talon / Kırmızı UDP
Victus> python relay_radio.py --target COM12 --radio COM5 --request-hz 10          # hedef radyosu bu laptopta
Orin>   tmux new -s takip
Orin>   python3 chase_orin.py --target-link /dev/serial/by-id/usb-XXXX --alt-ref same-home --dry-run
Orin>   python3 chase_orin.py --target-link /dev/serial/by-id/usb-XXXX --alt-ref same-home --set-mode
```
Victus satırı: `HEDEF` (kaynak) | `TAKIPCI` (Orin'den aşağı) | `d` / `ayrim` | `aktarim /s` | radyo `rtt`.
Victus üzerindeki relay'in `ayrim` alanı ham AMSL farkını gösterir. Seçtiğin irtifa referansındaki ayrım Orin ekranında ve Orin logundadır.
`>>> [ORIN] ...` satırları chase_orin olaylarıdır, `>>> [TAKIPCI FC] ...` FC'nin kendi mesajları.

### FC parametreleri — GCS failsafe chase_orin'e bağlı (önce SITL'de doğrula)
| Param | Değer | Not |
|---|---|---|
| `MAV_GCS_SYSID` (4.7; eskisi `SYSID_MYGCS`) | 251 | chase_orin'in sysid'i (`--src-sysid`) |
| `FS_GCS_ENABLE` | 7 | Brake (firmware'de yoksa Land) |
| `FS_GCS_TIMEOUT` | 3 | s |
| `FS_OPTIONS` | + "Continue if in pilot controlled modes on GCS failsafe" | LOITER/ALTHOLD'da etkisiz |

Etki: chase_orin'in heartbeat'i 3 s kesilir **ve** takipçi GUIDED'daysa fren yapar; elle uçarken bir
şey olmaz. Bilinmesi gerekenler: sysid 255'ten RC_OVERRIDE artık reddedilir; RTL pozisyonuyla abort
edilirse 3 s sonra BRAKE'e çevirir (LOITER ile abort sorunsuz); failsafe'ten çıkmak için mod anahtarını
oynat. Bu parametreler olmadan chase_orin'in/Orin'in ölümünü FC fark etmez (MAVProxy'nin heartbeat'i
sysid 250 ile sürer): takipçi son carrot'a (≤250 m önde) gider ve GUIDED'da asılı kalır.

### Arıza davranışı
| Olay | Sonuç |
|---|---|
| radyo kopar / Victus relay durur | 3 s sonra hedef bayat → takipçi yerinde tutulur; veri gelince devam |
| Orin'de radyo USB'si çıkar | aynısı; port kendiliğinden yeniden açılır |
| SSH kopar, chase_orin tmux içinde | hiçbir şey; takip sürer |
| SSH kopar tmux dışında (SIGHUP) / SIGTERM | temiz çıkış: GUIDED'daysa LOITER |
| GUIDED'a geçiş sırasında Ctrl-C/sinyal | FC GUIDED'a geçmişse LOITER'a geri alınır |
| chase_orin/Orin ölür (kill -9, güç) | failsafe yoksa son carrot + GUIDED'da asılı; varsa 3 s sonra BRAKE |
| pilot mod anahtarını çevirir | chase_orin çıkar, moda dokunmaz |

### Loglar
- **Orin:** `hedef_link` = radyo bağlantısının tamamı (hedef + 250 Victus TIMESYNC + 51 SiK
  RADIO_STATUS varsa rssi'leriyle), `chaser_link` (USB), `relay_lat`, `cmd_lat` (USB RTT), `chase_state`.
  Yukarı yönün kaybı = `hedef_link`'te hedefin `GLOBAL_POSITION_INT` `gap_ms` dağılımı.
- **Victus:** `hedef_link` (kaynak), `chaser_link` (radyodan aşağı gelenler), `cmd_lat`
  (`command=TIMESYNC`: radyonun gidiş-dönüşü; `rtt_lost` = cevapsız ping).

## Orin mimarisi (Aşama 1, ESKİ DÜZEN: radyo FC'de) — beyin takipçinin Orin'inde

```
HEDEF FC --SiK--> laptop: relay_target.py --SiK--> TAKIPCI FC --USB--> MAVProxy --UDP--> chase_orin.py
                  (hedef paketlerini AYNEN aktarır,                         (beyin: aynı chase_core)
                   uçuş komutu göndermez)            komutlar <--USB-- MAVProxy <--UDP--
```

Laptop sadece "radar"dır: hedefin kendi paketlerini (`GLOBAL_POSITION_INT`, `GPS_RAW_INT`,
`HEARTBEAT`) bayt-birebir takipçinin radyosuna basar. Takipçi FC'nin MAVLink yönlendirmesi
bunları USB'den Orin'e iletir. Orin iki aracı **sysid** ile ayırır. Komutlar USB'den gider:
**radyo gecikmesi/kaybı komut yolunda yok.** Beyin laptop şekliyle birebir aynıdır.
Laptop'un takipçiye kendi gönderdiği tek şey, takipçi radyosunun gidiş-dönüşünü ölçen zararsız
bir `REQUEST_MESSAGE` ping'idir (laptop modundakiyle aynı; `--rtt-ping-s`, 0 = kapalı).

### Ön koşullar (sırayla)
1. **Hedefte `SYSID_THISMAV = 2`** (Mission Planner → Full Parameter List → yaz → FC'yi yeniden
   başlat). Takipçi 1 kalır. Aynı sysid ile ne FC ne Orin iki drone'u ayırabilir;
   `relay_target.py` bu durumda başlamayı reddeder.
2. **Takipçi radyo linki sağlam olmalı.** Hedef verisi artık bu linkten geçiyor. Balkon
   testindeki "takipçi 0–0.8 Hz" sorunu önce çözülmeli (Mission Planner'ı COM14'e tek başına
   bağla, NetID/çapraz konuşma kontrolü).
3. **Orin'de MAVProxy çalışıyor olmalı.** ArduPilot yayın paketlerini sadece üzerinde daha önce
   bir sistem görülen kanala iletir; MAVProxy'nin ve `chase_orin`'in heartbeat'leri USB kanalını
   bu listeye sokar. `chase_orin` MAVProxy'nin **başka kimsenin kullanmadığı** bir `udpin`
   çıkışına bağlanır (varsayılan `--fc udpout:127.0.0.1:14551` = MAVProxy `--out=udpin:...:14551`).
   Aynı porta ikinci bir süreç bağlanırsa MAVProxy sadece sonuncusuna gönderir. Takipçiye komut
   veren başka bir süreç (ör. hunter controller) aynı anda çalışmamalı: tek beyin.
4. Takipçi FC'de ilgili seri portlarda `SERIALx_OPTIONS` → "Don't forward mavlink to/from"
   **işaretsiz** olmalı (varsayılan işaretsiz).
5. Orin'de: `cd ~/Desktop/avci && git pull`, sonra `python3 -c "import numpy, pymavlink"`.

### Balkon yönlendirme testi (pervanesiz, uçuş yok)
```
laptop> python relay_target.py --target-port COM5 --chaser-port COM14
Orin>   python3 chase_orin.py --dry-run
```
- Laptop satırında `aktarim ~10/s` görmelisin (hedef 10 Hz geliyorsa).
- Orin'de `HEDEF` ve `TAKIPCI` satırları ayrı ayrı çıkmalı. `baska sistemler` listesinde
  255 (MAVProxy) ve 250 (laptop) olağandır.
- Balkonda takipçinin fix'i yoksa `chase_orin` "3D fix olmadan..." deyip çıkar. Yönlendirme
  yine de doğrulanmış olur, çünkü HEDEF satırı Orin'e ulaşmıştır.
- `[X] Hedef konumu gelmedi` → mesajdaki dört maddeyi sırayla kontrol et.

### Saha
```
laptop> python relay_target.py --target-port COM5 --chaser-port COM14
Orin>   python3 chase_orin.py --alt-ref same-home --dry-run
Orin>   python3 chase_orin.py --alt-ref same-home --set-mode
```
Bayraklar ve varsayılanlar laptop şekliyle aynıdır (`--max-speed`, `--alt-offset`,
`--min-sep` …). Orin'e özgü olanlar: `--fc`, `--chaser-sysid 1`, `--target-sysid 2`,
`--pos-hz 10`, `--att-hz 10`, `--rate 10`. Döngü ayrıca her yeni hedef fix'inde hemen uyanır.
`--print-hz 2` ile SSH'e seyrek yazar; loglar her tiki alır.

**İptal:** RC mod anahtarı. Script takipçinin GUIDED'dan çıktığını görür, durur ve moda bir daha
dokunmaz. SSH'ten Ctrl-C → takipçi LOITER. **SSH/SIYI linki koparsa tek iptal RC'dir; elin
anahtarda olsun.** Laptop'taki relay durursa hedef bayatlar ve takipçi `--max-age` (3 s) sonra
yerinde tutulur.

### Loglar
- **Orin** (`logs/`): `hedef_link` (hedef, iki atlama sonrası Orin'in gördüğü hâli),
  `chaser_link` (USB), `relay_lat` (tik başına veri yaşları), `cmd_lat` (USB üzerinden RTT,
  birkaç ms olmalı), **`chase_state`** (tik başına iki aracın tam durumu + takipçi attitude +
  planlayıcı kararı). `chase_state` Orin saatiyle yazılır, yani Orin'deki video kaydıyla aynı
  saattir. Her GPS uçuşu, Aşama 2 için "hedef kamerada nerede olmalıydı" gerçek verisi olur.
- **Laptop** (`logs/`): iki radyonun `hedef_link`/`chaser_link`'i (link sağlığı) ve
  `cmd_lat` (takipçi radyosunun RTT'si, ping ile — laptop modundakiyle aynı yöntem, aynı saat).

### Gecikme: iki modu karşılaştırma
| Ölçüm | Laptop modu (`follow_gps`) | Orin modu (`relay_target` + `chase_orin`) |
|---|---|---|
| Radyo sağlığı (mesaj başı gap/Hz, RSSI, txbuf) | laptop | laptop |
| Takipçi radyosu RTT (`cmd_lat`) | laptop (komutlar bu radyodan) | laptop (ping) |
| USB RTT (`cmd_lat`) | – | Orin |
| Tik başı veri yaşı (`relay_lat`) | laptop | Orin |
| Tam durum (`chase_state`) | – | Orin |

- **Ek atlamanın gecikmesi** (hedef verisinin laptoptan Orin'e gidişi) ≈
  (laptop `cmd_lat` RTT + Orin `cmd_lat` RTT) / 2. Radyo ve USB yollarının iki yönde simetrik
  olduğunu varsayar; saat senkronu gerektirmez.
- **Ek atlamanın kaybı/titreşimi:** laptoptaki ve Orin'deki `hedef_link` dosyalarında hedefin
  `GLOBAL_POSITION_INT` `gap_ms` dağılımlarını karşılaştır (10 Hz'de ~100 ms beklenir).
- `relay_lat.target_age` iki makine arasında **kıyaslanamaz**: son paketin gelişinden beri
  geçen süredir, paketin yolda geçirdiği süreyi görmez.
- Hedef radyosunun havadaki süresi (hedef FC → laptop) iki modda da **ölçülmüyor**; ama iki modda
  ortak olduğu için karşılaştırmayı etkilemez.
- **Beklenti (sahada sınanacak):** takipçi radyosu iki modda da bir kez aşılıyor (laptop modunda
  komut, Orin modunda hedef verisi), yani toplam algı→komut gecikmesi benzer olmalı; fark ~USB
  RTT kadar. Orin modunun kazancı başka yerde: takipçinin kendi durumu radyodan değil USB'den
  taze gelir, takipçi radyosu koparsa da yerinde tutma komutu USB'den ulaşır.

### Kapalı olanlar (bilerek)
`chase_orin.py` sim oturumunun faz makinesini izler:
`TRACK_PLAN → (kilit) → VISUAL → (kayıp) → REALIGN → TRACK_PLAN`. Aşama 1'de sadece
TRACK_PLAN bağlı. `VISUAL_ENABLED`/`REALIGN_ENABLED = False`. Gölge gözlemci (Aşama 2) için
her tikte çağrılan kanca hazır (`NullObserver`); komut veremez, sadece gözler. VISUAL (Aşama 3)
ve REALIGN (Aşama 4) kendi offline testleriyle birlikte açılacak.

## Ayar knob'ları (özet)
| Knob | Varsayılan | Ne yapar |
|---|---|---|
| `--max-speed` | 20 | uzaktaki tepe hız (WP_SPD tavanı + DO_CHANGE_SPEED) |
| `--deadline-range` | 100 | bu mesafede close-approach'a in |
| `--close-approach` | 10 | deadline içi yaklaşma hızı |
| `--decel-start-range` | 200 | tepe hızdan yavaşlamaya başla |
| `--accel` | 4 | WP_ACC / planlayıcı ivme sınırı |
| `--carrot` | 250 | carrot ön-bakış mesafesi |
| `--alt-offset` | -3 | hedefe göre dikey ofset (+üst / −alt) |
| `--alt-ref` | amsl | aynı kotta kalkış için `same-home` seçilir |
| `--alt-match` | AÇIK | temiz irtifa modu; `--no-alt-match` ile balon mantığına dön |
| `--sep-bias` | 0 | AMSL modunda aynı fiziksel yükseklikte ölçülen ham ayrım |
| `--min-alt` | 10 | komut irtifası tabanı |
| `--min-sep` | 0 (kapalı) | ölçülen dikey ayrım sigortası |
| `--rate` | 10 | kontrol/carrot gönderme hızı (Hz) |
| `--stream-hz` | 10 | telemetriden konum isteme hızı |
| `--att-hz` | 2 | takipçi ATTITUDE hızı; yaw = hover'da burun yönü (sim ile aynı). 0 = isteme |
| `--posvel` | kapalı | komuta hedef hızını da kat (carrot'la nadiren gerekli) |
| `--rtt-ping-s` | 1.0 | komut→tepki RTT ping aralığı (s); 0 = kapalı |

## Sorun giderme
- **Takipçi 10 m/s'yi geçmiyor:** menzil `--deadline-range` içinde mi? (tasarım gereği yavaşlar).
  Değilse başlangıçtaki `WP_SPD ceiling ... m/s` doğrulama satırına bak; uyarı varsa
  firmware'de param adı `WPNAV_SPEED` olabilir.
- **`gelen` Hz istenen `--stream-hz`'in çok altında:** telemetri linki doymuş; `--stream-hz`
  düşür (ör. 5) veya `--rate`'i ayır.
- **Sürekli `[BEKLE] hedef konumu ... bayat`:** hedef telemetrisi kopuk/yavaş; `--max-age`
  artır ama asıl link/anteni kontrol et.
- **`--min-sep` sürekli tetikleniyor:** aynı irtifada uçuyorsun; `--min-sep 0` ver veya dikey
  ofset büyüt.
- **`takipci ATTITUDE gelmiyor` uyarısı:** planlayıcı hover'da burun yönü yerine hız yönünü
  kullanır (hareket hâlinde fark etmez). `--att-hz`'i ve takipçi linkini kontrol et.

## Notlar
- `Vehicle`/`Recorder`/geometri yardımcıları `follow_relay.py`'den alındı; tek eklenen
  `on_msg` kancasıdır (link_log için, snapshot kontratını değiştirmez).
- NED çerçevesi orijini takipçinin **başlangıç** lat/lon'una sabitlenir; planlayıcı bu
  sabit çerçevede çalışır (yatay dönüşüm çeviriye göre değişmez, geometri korunur).
- `guided_goto.py` olduğu gibi kalır; bu script ayrıdır (`follow_gps.py`).

## Bu sürümde düzeltilenler (önceki teslimata göre)
Ana döngüyü uçtan uca koşturan `test_loop.py` yazılırken bulundu. Önceki sürüm sahada ilk
geçerli tikte çökerdi:
1. **Çökme:** döngü `ts["last_raw"]` okuyordu (anahtar `raw`) → ilk geçerli tikte `KeyError`.
2. **Log:** hedef mesajları `chaser_link`'e yazılıyor, `hedef_link` boş kalıyordu (ad dolgusu).
3. **WP_SPD doğrulaması:** parametre okuması RX thread'iyle aynı linki okuyordu → sahte
   "read-back timed out" uyarısı. Artık tek okuyucu RX thread'i.
4. **`--posvel`:** ileri besleme takipçinin kendi hızıydı; hedefin hızı olmalı (follow_relay gibi).
5. **Hover burun yönü:** sim ATTITUDE yaw'ını kullanır; biz hız vektöründen (hover'da gürültü)
   türetiyorduk. Artık yaw (`--att-hz`), yoksa hız yönü.
6. **Yeni fix tespiti:** zaman farkı yerine tam varış damgası (tekrar besleme riski yok).
7. **Bekleme:** takipçi son carrot'a uçmaya devam ediyordu; artık yerinde tutulur (sim `--stale-hold`).
8. **Pilot devri:** script çıkarken takipçiyi zorla LOITER'a alıyordu (follow_relay'den miras);
   artık pilotun seçtiği moda dokunmaz. Beklenmeyen hatada da güvenli çıkar.
9. **Orin modunda radyo gecikmesi:** relay artık takipçi radyosunun RTT'sini laptop modundakiyle
   aynı ping'le ölçüyor (`cmd_lat`). README'deki "`target_age` farkı = ek atlama" kıyası yanlıştı,
   kaldırıldı (bkz. "Gecikme: iki modu karşılaştırma").
