# follow_gps — GPS ile drone-to-drone takip/kesme (filmcilik)

Takipçi drone, senin elle uçurduğun hedef drone'u **GPS'le** otonom kovalar: karşıdan
gelirsen **önünü keser** (intercept), çaprazda **arkadan takip eder** (follow). Kamera yok,
takipçide Orin yok — bütün komutlar laptopta iki telemetri radyosundan hesaplanıp
takipçiye MAVLink GUIDED ile gönderilir.

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
follow_gps.py       ana script (transport + planlayıcı sürücü)
chaser_env.py       pymavlink adaptörü (planlayıcı → gerçek drone)
link_log.py         telemetri/gecikme loglama
fpv_gate_il/        vendorlanan yörünge paketi (13/14 dosya birebir; mav_bridge tek satır nötr)
test_offline.py     planlayıcı mantığı regresyonu (donanımsız)
test_wiring.py      montajlı yığın entegrasyon testi (donanımsız)
```

## Kurulum
```
pip install pymavlink pyserial numpy
```

## ZORUNLU iş akışı (sırasıyla)
1. **Önce offline testler** — her saha çıkışından önce:
   ```
   python test_offline.py && python test_wiring.py
   ```
2. **Dry-run + dikey ayrım okuması** (FC'ye komut YOK):
   ```
   python follow_gps.py --target-port COM12 --chaser-port COM5 --dry-run --no-rec
   ```
   İki drone'u **gerçekten aynı irtifaya** getir, ekrandaki `olculen dikey ayrim`
   değerini oku. Sıfır okumuyorsa o değeri bir sonraki adımda `--sep-bias` ile ver.
3. **Saha çalışması.** Varsayılanlar zaten filmcilik için ayarlı (temiz irtifa modu açık,
   3 m altta, ~200 m'ye kadar tam 20 m/s, 100 m içinde 10 m/s):
   ```
   python follow_gps.py --target-port COM12 --chaser-port COM5 --sep-bias <olculen> --set-mode
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
Gönderilen irtifa her tikte hedefin canlı irtifasından **yeniden** hesaplanır (latch yok):
```
gate_alt = (hedef_amsl - sep_bias - takipci_home_amsl) + alt_offset
```
`takipci_home_amsl = alt_amsl - alt_rel` — baro sürüklense bile uçuş boyunca sabittir
(iki alan birlikte sürüklenir), bu yüzden çapraz-araç irtifa matematiğinin dayanağı odur.

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
- **RC'de moda hazır dur.** Nihai güvenlik sensin; takipçi GUIDED'dan çıkınca script çekilir.
- `--min-sep` opsiyonel bir **ölçülen dikey ayrım sigortasıdır** (varsayılan 0 = KAPALI):
  açıksa `|ayrim| < min-sep` olunca komut kesilir. Intercept doğal olarak yaklaştığı için
  varsayılan kapalıdır; **aynı irtifada geçiş** istiyorsan `--min-sep 0` bırak (yoksa
  amaçladığın yakın geçişte tetiklenir). Küçük bir dikey ofsetle uçuyorsan (ör. -10 m),
  `--min-sep 4` gibi bir taban güvenli bir yedek olur.
- `--max-dist` (kelepçe), GPS-fix geçitleri ve `--max-age` (bayat konum) hep açık.

## Loglama (varsayılan AÇIK, `--no-log` ile kapat)
`--log-dir` (varsayılan `logs/`) altında dört CSV:
- `hedef_link_<zaman>.csv`, `chaser_link_<zaman>.csv`: her radyo için **mesaj başına**
  tip, önceki aynı-tip mesajdan bu yana geçen süre (gap), anlık Hz. `RADIO_STATUS`/`RADIO`
  mesajları ayrıca SiK link alanlarını yazar (rssi, remrssi, noise, remnoise, rxerrors,
  fixed, txbuf) — **B-cube v5 vs X-Rock klonu** hangi radyonun link telemetrisi gönderdiğini
  ve linkin ne kadar dolu olduğunu böyle görürsün.
- `relay_lat_<zaman>.csv`: her kontrol tikinde tek yönlü **veri yaşı** bütçesi — hedef fix
  yaşı, takipçi fix yaşı, planlayıcı hesap süresi, gönderim süresi, toplam + o anki
  davranış/hız/mesafe. Baskın gecikme genelde `target_age` (telemetri yaşı).
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
| `--alt-match` | AÇIK | temiz irtifa modu; `--no-alt-match` ile balon mantığına dön |
| `--sep-bias` | 0 | dry-run'da ölçülen sistematik ayrım düzeltmesi |
| `--min-alt` | 10 | komut irtifası tabanı |
| `--min-sep` | 0 (kapalı) | ölçülen dikey ayrım sigortası |
| `--rate` | 10 | kontrol/carrot gönderme hızı (Hz) |
| `--stream-hz` | 10 | telemetriden konum isteme hızı |
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
- **`--min-sep` sürekli kesiyor:** aynı irtifada uçuyorsun; `--min-sep 0` ver veya dikey
  ofset büyüt.

## Notlar
- `Vehicle`/`Recorder`/geometri yardımcıları `follow_relay.py`'den alındı; tek eklenen
  `on_msg` kancasıdır (link_log için, snapshot kontratını değiştirmez).
- NED çerçevesi orijini takipçinin **başlangıç** lat/lon'una sabitlenir; planlayıcı bu
  sabit çerçevede çalışır (yatay dönüşüm çeviriye göre değişmez, geometri korunur).
- `guided_goto.py` olduğu gibi kalır; bu script ayrıdır (`follow_gps.py`).
