# HUNTER — Kontrolcü Arayüz Sözleşmesi (Controller Interface Contract)

Bu döküman, drone'un otonom kontrolcüsüne giren **14 girdiyi** ve çıkan **4 komutu**
tanımlar. Kendi kontrol kodunu yazacak biri SADECE bu sözleşmeye uymak zorundadır;
tracker, MAVLink, kamera vb. ile uğraşmaz.

> **Tek cümlede:** Her frame'de 14 sayı girer, 4 sayı çıkar.
> Girdi = hedefin görüntüdeki durumu + drone'un IMU durumu.
> Çıktı = roll / pitch / throttle / yaw komutları (-1..+1).

---

## 1) GİRDİ — Observation (14-dim float vektör)

Sıra **sabittir**. `obs[i]` indexleri:

| idx | İsim         | Birim       | Tipik aralık      | Anlam / işaret yönü                                            | PN teacher kullanıyor mu? |
|----:|--------------|-------------|-------------------|----------------------------------------------------------------|:------------------------:|
|  0  | `cx`         | normalize   | 0.0 – 1.0         | Hedef kutu merkezi X. 0=sol kenar, 0.5=orta, 1=sağ kenar       | hayır                    |
|  1  | `cy`         | normalize   | 0.0 – 1.0         | Hedef kutu merkezi Y. 0=üst, 0.5=orta, 1=alt                   | hayır                    |
|  2  | `bbox_w`     | normalize   | 0.0 – 1.0         | Kutu genişliği (görüntü genişliğinin oranı)                    | **EVET**                 |
|  3  | `bbox_h`     | normalize   | 0.0 – 1.0         | Kutu yüksekliği (görüntü yüksekliğinin oranı)                  | **EVET**                 |
|  4  | `ang_x`      | derece (°)  | ~ -40 .. +40      | Hedefe yatay açı. **+ = hedef sağda, − = solda.** `(cx−0.5)·HFOV` | **EVET**              |
|  5  | `ang_y`      | derece (°)  | ~ -25 .. +25      | Hedefe dikey açı. **+ = hedef merkezin ALTINDA, − = üstünde.** `(cy−0.5)·VFOV` | **EVET**     |
|  6  | `roll_deg`   | derece (°)  | -180 .. +180      | Drone IMU roll açısı (yatış)                                    | **EVET**                 |
|  7  | `pitch_deg`  | derece (°)  | -90 .. +90        | Drone IMU pitch açısı (burun yukarı/aşağı)                     | **EVET**                 |
|  8  | `yaw_deg`    | derece (°)  | -180 .. +180      | Drone IMU yaw açısı (yön/heading)                              | hayır                    |
|  9  | `roll_rate`  | derece/s    | —                 | Gövde açısal hızı (gyro). **derece/s'ye çevrilmiş** (MAVLink rad/s değil) | **EVET**       |
| 10  | `pitch_rate` | derece/s    | —                 | Gövde pitch açısal hızı                                        | **EVET**                 |
| 11  | `yaw_rate`   | derece/s    | —                 | Gövde yaw açısal hızı                                          | **EVET**                 |
| 12  | `aux`        | —           | —                 | **Rezerve / şu an kullanılmıyor.** (controller.py göreli irtifa besliyordu, teacher okumuyor) | hayır |
| 13  | `bbox_valid` | bayrak      | 0.0 veya 1.0      | **1 = hedef takip ediliyor**, 0 = hedef yok/kayıp             | **EVET**                 |

### Önemli notlar (girdi)

- **Açılar lineer ölçek:** `ang_x = (cx − 0.5)·HFOV`, `ang_y = (cy − 0.5)·VFOV`.
  Pinhole/atan **değil**, doğrudan lineer. Kontrolcü farklı varsayım yaparsa uyumsuzluk olur.
- **FOV kameraya bağlı (KRİTİK):** Referans değerler `HFOV = 80°`, `VFOV = 50.534°`
  (1280×720, 16:9'dan türetilmiş). **Gerçek kameranın FOV'u farklıysa `ang_x/ang_y`
  sabit bir oranla kayar** → açıya dayalı gain'ler farklı agresiflikte tepki verir.
  Kendi kontrolcünü yazıyorsan FOV'u parametre al, sabit gömme.
- **Görüntü koordinatı:** `cy` aşağı doğru artar (OpenCV konvansiyonu). Bu yüzden
  hedef görüntünün alt yarısındaysa `ang_y > 0`.
- **`bbox_valid = 0` ise** `ang_x/ang_y/cx/cy/w/h` anlamsızdır; kontrolcü güvenli
  davranış üretmeli (örn. komutları sıfırla / hover, son yaw'u yumuşat).
- **Açısal hızlar derece/s.** Kaynak MAVLink ATTITUDE rad/s verir; arayüze girmeden
  önce dereceye çevrilir. Kontrolcü dereceyle çalışır.

---

## 2) ÇIKTI — Action (4-dim float vektör)

Sıra sabittir. Her eleman **[-1.0, +1.0]** aralığında:

| idx | İsim       | Aralık     | Anlam                                                        |
|----:|------------|------------|--------------------------------------------------------------|
|  0  | `roll`     | -1 .. +1   | Roll komutu (− sola yatış, + sağa) *                         |
|  1  | `pitch`    | -1 .. +1   | Pitch komutu (− burun aşağı/dalış, + yukarı) *               |
|  2  | `throttle` | -1 .. +1   | İtki komutu. Hover ~ +0.07 civarı; teacher ~[-0.2, +0.5] üretir |
|  3  | `yaw`      | -1 .. +1   | Yaw rate komutu (− sola dönüş, + sağa) *                     |

\* **Eksen yönü gerçek donanımda doğrulanmalı.** RC_OVERRIDE, yer kontrol
istasyonunun kalibrasyonunu **atlar** → ham eksen yönü geçerli olur. Bir veya
birden çok eksen ters çıkabilir (sim'de `pitch` tersti: `AUTO_PITCH_DIR = -1`).
Eksen yön düzeltmeleri kontrolcünün DIŞINDA, çıktı→PWM katmanında uygulanır.

### Action → PWM dönüşümü (kontrolcünün dışında yapılır)

```
pwm = clamp( action * 500 + 1500 , 1000 , 2000 )
# -1 → 1000,  0 → 1500,  +1 → 2000
```

---

## 3) Sözleşme imzası (kontrolcünün uygulaması gereken)

```
// Girdi:  14 float (yukarıdaki sıra)
// dt:     bir önceki frame'den geçen süre (saniye)  — EMA/integral/türev için
// Çıktı:  4 float [-1..+1]  (roll, pitch, throttle, yaw)

Action compute(const float obs[14], float dt);
void    reset();   // yeni angajman / tracker yeniden başladığında durumu sıfırla
```

- `compute` deterministik olmalı: aynı girdi dizisi → aynı çıktı.
- Kontrolcü kendi iç durumunu (filtre, integral, önceki frame) tutabilir;
  `reset()` çağrıldığında bunları temizler.
- `dt` gerçek frame süresidir (sabit 1/30 varsayma — kamera FPS'i değişebilir).

---

## 4) Hızlı özet (handoff için)

> Her görüntü karesinde sana 14 sayı veriyorum: hedefin ekrandaki yeri/boyutu
> (`cx,cy,w,h`), hedefe açı (`ang_x` yatay, `ang_y` dikey, derece), drone'un
> açıları (`roll,pitch,yaw` derece) ve açısal hızları (derece/s), bir de
> "hedef geçerli mi" bayrağı (`bbox_valid`).
> Sen bana 4 sayı döndür: roll, pitch, throttle, yaw — her biri −1 ile +1 arası.
> `ang_x>0` hedef sağda, `ang_y>0` hedef merkezin altında. Hedefi
> `ang_x=0, ang_y=0`'a getirip üstüne sürmek istiyoruz.
