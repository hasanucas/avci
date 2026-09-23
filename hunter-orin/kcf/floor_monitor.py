"""
HUNTER — FloorMonitor.  (yeniden insa: 31 Agu 2026)

=============================================================================
NEDEN AYRI DOSYA — VE NEDEN YENIDEN YAZILDI
=============================================================================
20 Agu 2026'da controller_guided.py icinde bir FloorMonitor vardi ve
test_governor_offline.py ile 40/40 dogrulanmisti. 31 Agu'da ACRO yolunu
eklerken controller_guided.py'yi avci_ucus repo ANLIK GORUNTUSU uzerine
kurdum — o snapshot PRE-PATCH, yani FloorMonitor'lu surumden ONCE. Sonuc:
dosya Orin'de EZILDI ve taban sigortasi sessizce KAYBOLDU. Bu benim
hatam; ne git'te ne deploy reposunda kopyasi vardi (dosya hic commit
edilmemisti).

Kurtaran sey: test_governor_offline.py bolum 11 duruyordu ve o testin
kendisi SARTNAMEYDI. Bu dosya o 11 iddiadan yeniden insa edildi.

AYRI DOSYA cunku:
  * saf sinif — FC yok, print yok, MAVLink yok -> offline test edilebilir
    (orijinalinin de tasarim ilkesi buydu)
  * bir daha controller_guided.py ezilse bile bu dosya ayakta kalir

DIKKAT — DAVRANIS AYNI, KOD YENI. 40/40 gecen kodun BIREBIR AYNISI degil.
Testi geciyor ama sahada ucmus surum bu degil.

=============================================================================
NASIL CALISIR
=============================================================================
  KURULUM (arm) : irtifa taban+5 m'yi ASINCA kurulur. Yerden angaje
                  olurken (bench, dusuk irtifa) KURULMAZ, dolayisiyla
                  tetiklemez.
  TETIK         : kurulduktan SONRA irtifa taban altinda KESINTISIZ
                  0.25 s kalirsa TAM BIR KEZ True doner, sonra kilitlenir.
  BAYAT VERI    : telemetri yasi > 0.5 s ise KARAR VERILMEZ (ne sayac
                  ilerler ne tetik). GPS/baro kesilmesinde yanlis alarm yok.
  RESET         : CH6 kapat-ac ile cagrilir; kilit ve kurulum sifirlanir.

=============================================================================
IRTIFA KAYNAGI — 31 Agu 2026 UYARISI
=============================================================================
Bu sinif kendisine VERILEN irtifaya guvenir; dogrulugunu kontrol etmez.
31 Agu masa kosusunda (guided_20260831_174627.csv) drone masada dururken
`fc.alt_rel` 16.5 m okuyordu. MAVProxy da acilista "height 21" basmisti.
Yani sifir noktasi ~16-21 m KAYIK.

Boyle bir kayikla taban sigortasi ISE YARAMAZ: 20 m taban, gercekte
~3 m'ye karsilik gelir. Ucustan ONCE yerde alt_rel'in ~0 oldugu
DOGRULANMALI (baro kalibrasyonu). Kontrol icin: floor_preflight_check().
"""


class FloorMonitor:
    """Taban sigortasi. Saf sinif: FC yok, print yok, offline test edilir."""

    ARM_MARGIN_M = 5.0        # taban + bu kadar asilinca kurulur
    DWELL_S = 0.25            # taban altinda kesintisiz kalma suresi
    STALE_S = 0.5             # bundan eski telemetriyle KARAR VERILMEZ

    def __init__(self, floor_alt_m):
        self.floor = float(floor_alt_m)
        self.enabled = self.floor > 0.0
        self.armed = False
        self.latched = False
        self._below_since = None

    def reset(self):
        """CH6 kapat-ac. Kilit acilir, kurulum sifirlanir."""
        self.armed = False
        self.latched = False
        self._below_since = None

    def update(self, alt_m, age_s, t_s):
        """
        alt_m : FC'nin bildirdigi bagil irtifa (m)
        age_s : o olcumun yasi (s) — tazelik kapisi
        t_s   : simdiki zaman (s), monotonik

        Donus: SADECE tetik aninda True (tek atim). Sonrasinda kilitli.
        """
        if not self.enabled or self.latched:
            return False

        # BAYAT VERI KAPISI — sayac da ilerlemez. Telemetri kesilirken
        # "irtifa dusuyor" sanip tetiklemek en kotu yanlis alarm olurdu.
        if age_s > self.STALE_S:
            return False

        alt = float(alt_m)

        # KURULUM: taban+marj asilmadan sigorta CALISMAZ. Yerden/alcaktan
        # angaje olurken (bench) tetiklememesinin sebebi bu.
        if not self.armed:
            if alt > self.floor + self.ARM_MARGIN_M:
                self.armed = True
            return False

        if alt >= self.floor:
            self._below_since = None
            return False

        # Taban altindayiz.
        if self._below_since is None:
            self._below_since = t_s
            return False
        if t_s - self._below_since >= self.DWELL_S:
            self.latched = True
            return True
        return False


def floor_preflight_check(alt_m, tolerans_m=3.0):
    """
    Yerde alt_rel ~0 mi? Taban sigortasinin ON KOSULU.

    31 Agu masa kosusunda 16.5 m okunuyordu; oyle bir kayikla 20 m taban
    gercekte ~3 m eder ve sigorta ISE YARAMAZ. Ucustan once, drone YERDE
    DURURKEN cagir.

    Donus: (uygun_mu, mesaj)
    """
    a = float(alt_m)
    if abs(a) <= tolerans_m:
        return True, f"alt_rel = {a:+.1f} m — sifira yakin, taban sigortasi anlamli."
    return False, (
        f"alt_rel = {a:+.1f} m ama drone YERDE. Sifir noktasi {a:+.1f} m KAYIK.\n"
        f"  Bu kayikla --floor-alt 20 gercekte {20.0 - a:.1f} m'ye karsilik gelir.\n"
        f"  Once baro kalibrasyonu yap (Mission Planner > Setup > Accel/Baro),\n"
        f"  ya da --floor-alt degerini {20.0 + a:.0f} yaz (KOTU cozum: irtifa\n"
        f"  gostergesi de ayni kadar yanlis olur, sahada kafa karistirir)."
    )
