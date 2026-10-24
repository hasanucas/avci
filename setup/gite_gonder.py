#!/usr/bin/env python3
"""
gite_gonder.py — CALISAN Orin'in son halini GitHub'a gonderir (main + etiket)
=============================================================================
  cd ~/Desktop/avci
  python3 setup/gite_gonder.py --kontrol   # sadece rapor: repoda hicbir sey degismez (git fetch haric)
  python3 setup/gite_gonder.py             # sorar -> hazirlar -> son onay -> commit + etiket + push

Adimlar
  0) On kontrol : main dali, origin = hasanucas/avci, git fetch.
                  GitHub'da bu Orin'de olmayan commit varsa (laptop push ettiyse) once SADECE onlari
                  almayi sorar (--ff-only --no-overwrite-ignore) ve durur: sonra tekrar calistirirsin.
                  Push edilmemis eski commit'lerde >99 MB / yeni ic ice repo / sifre izi varsa DURUR.
  1) Tarama     : yeni / degisen / silinen dosyalar, ic ice repolar (git clone ile gelmis paketler),
                  .gitignore yuzunden girmeyen model dosyalari, okunamayan dosya ve klasorler,
                  sembolik linke donusmus klasorler.
  2) Kararlar   : ic ice repo -> icerigi girsin mi          >99 MB -> otomatik disarida
                  veri dosyalari -> sorar (varsayilan hayir)  sifre/token izi -> sorar (varsayilan dur)
  3) Hazirlik   : setup/ortam_kaydet.sh + sistem envanteri GECICI klasore.
                  Repoya sadece kisisel olmayan teknik ozet + paket listeleri girer. Sistem dosyalarinin
                  KOPYALARI ve tam envanter repoya girmez: ~/Desktop/avci_sistem_<tarih>.tar.gz
                  (sifre degerleri ***; bunu Claude'a sohbetten gonder).
  4) Son onay   : varsayilan hayir. Buraya kadar repoda hicbir sey degismez.
  5) Gonderim   : SADECE taramada gosterilen dosyalar sahnelenir (sonradan olusan dosya girmez)
                  -> dogrulama: yeni ic ice repo baglantisi yok, >99 MB yok, sahnelenen HER dosyada
                     yeniden sifre taramasi (onay, dosyanin o anki icerigine bagli)
                  -> commit -> etiket orin-YYYY-MM-DD -> push --atomic

Guvenlik
  - Senin dosyalarin silinmez. Yerinden alinanlar (ic ice .git, eski envanter, uzerine yazilan
    kayit dosyalari) yedege TASINIR: ~/Desktop/_repo_yedek/<tarih>/
    (ayni diskte degilse .git/gite_gonder_yedek/<tarih>/)
  - Repoya girmeyecekler sadece bu Orin'in .git/info/exclude dosyasina yazilir (scriptin blogu,
    .gitignore'daki '!' istisnalarindan da ustundur; satiri silersen tekrar sorulur).
  - Commit'ten once hata / Ctrl-C / SSH kopmasi / SIGTERM -> her sey geri alinir (sahne, .git'ler,
    exclude, kayitlar); geri alma sirasinda gelen sinyaller onu yarida birakamaz.
  - push --force YOK. Push olmazsa commit bu Orin'de kalir; tekrar calistir -> sadece push yapar.
  - Repo PUBLIC: gonderilen her sey herkese acik.
"""
from __future__ import annotations

import argparse
import bisect
import errno
import getpass
import hashlib
import os
import re
import shutil
import signal
import socket
import stat
import subprocess
import sys
import tarfile
import tempfile
import traceback
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

sys.dont_write_bytecode = True

MB = 1024 * 1024
SINIR_ENGEL = 99 * MB          # GitHub 100 MiB ustunu reddeder
SINIR_UYARI = 45 * MB          # GitHub 50 MB ustunu uyarir (sorun degil)
ORIGIN_RX = re.compile(r"(^|[/:])hasanucas/avci(\.git)?/?$", re.I)
ZAMAN = datetime.now()
DAMGA = ZAMAN.strftime("%Y%m%d_%H%M%S")
REF_REL = "setup/referans_ortam"
SISTEM_REL = REF_REL + "/sistem"
URETILEN = ("ortam.txt", "pip_freeze.txt", "pip_user.txt", "dpkg_secili.txt", "ag.txt")
RAPOR_ADI = "gonderim_raporu.txt"
ONAY_ADI = "gite_gonder_onay"   # .git/ altinda: "gercek degil" denen bulgular (blob sha + tur), GitHub'a gitmez
ENV_ISARET = "# AVCI sistem envanteri"
KULLANICI = getpass.getuser()
MAKINE = socket.gethostname()
# Git LFS kuruluysa paket dosyalari isaretci olarak girmesin: gercek icerik sahnelensin
LFS_KAPALI = ("-c", "filter.lfs.required=false", "-c", "filter.lfs.clean=",
              "-c", "filter.lfs.smudge=", "-c", "filter.lfs.process=")
LFS_ISARET = b"version https://git-lfs.github.com/spec/"
# arkadastan gelen paketin kendi .git/config'i bir komut calistirmasin (core.fsmonitor=<script> gibi)
ALT_GUVENLI = ("-c", "core.fsmonitor=false", "-c", "core.untrackedCache=false", "-c", "core.pager=cat")
EXC_BAS = "# >>> gite_gonder.py: bu Orin'de kalanlar (GitHub'a gitmez). Satiri silersen tekrar gorunur."
EXC_SON = "# <<< gite_gonder.py"

MODEL_UZ = {".pt", ".pth", ".onnx", ".tflite", ".safetensors", ".weights", ".pb", ".caffemodel", ".ckpt"}
URETILMIS_UZ = {".engine", ".plan", ".trt"}
VIDEO_UZ = {".mp4", ".mkv", ".avi", ".mov", ".h264", ".h265", ".264", ".265", ".hevc", ".ts"}
VERI_UZ = {".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tif", ".tiff", ".webp", ".ppm", ".pgm",
           ".jsonl", ".ndjson", ".npy", ".npz", ".pkl", ".pickle", ".joblib", ".h5", ".hdf5",
           ".parquet", ".feather", ".arrow", ".db", ".sqlite", ".sqlite3", ".ulg", ".ulog",
           ".raw", ".yuv", ".wav", ".mp3", ".m4a", ".flac", ".ogg", ".aac", ".pcap", ".pcapng",
           ".mcap", ".bag", ".csv", ".log", ".bin", ".tlog"} | VIDEO_UZ
KOD_UZ = {".py", ".cpp", ".hpp", ".h", ".c", ".cc", ".cu", ".cuh", ".sh", ".bash", ".md", ".txt",
          ".yaml", ".yml", ".json", ".cmake", ".xml", ".cfg", ".ini", ".toml", ".param", ".parm",
          ".launch", ".service", ".rules", ".conf", ".ipynb", ".js", ".html", ".css"}
KOD_DIL = {".py", ".pyx", ".pyi", ".cpp", ".cc", ".cxx", ".c", ".h", ".hpp", ".cu", ".cuh",
           ".js", ".java", ".rs", ".go", ".ipynb"}      # kaynak kod: ayar-dosyasi desenleri uygulanmaz
UYARI_UZ = {".py", ".cpp", ".hpp", ".h", ".c", ".cc", ".cu", ".sh", ".yaml", ".yml", ".cmake",
            ".ipynb", ".launch"} | MODEL_UZ
TEKNIK_UZ = {".py", ".cpp", ".hpp", ".h", ".c", ".cc", ".cu", ".sh", ".pt", ".pth", ".onnx",
             ".engine", ".plan", ".trt", ".yaml", ".yml", ".ipynb", ".whl", ".deb", ".param",
             ".parm", ".cmake"}
IKILI_UZ = MODEL_UZ | URETILMIS_UZ | VIDEO_UZ | {
    ".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tif", ".tiff", ".webp", ".ppm", ".pgm", ".ico", ".npy", ".npz",
    ".pkl", ".pickle", ".joblib", ".h5", ".hdf5", ".parquet", ".feather", ".arrow", ".db", ".sqlite", ".sqlite3",
    ".ulg", ".ulog", ".raw", ".yuv", ".wav", ".mp3", ".m4a", ".flac", ".ogg", ".aac", ".pcap", ".pcapng", ".mcap",
    ".bag", ".bin", ".tlog", ".so", ".a", ".o", ".pyc", ".whl", ".deb", ".zip", ".gz", ".tgz", ".xz", ".7z", ".tar",
    ".pdf", ".ttf", ".otf", ".woff", ".woff2"}
KUCUK_IKILI = 2 * MB            # NUL iceren ama bilinen ikili tur olmayan kucuk dosyalar da taranir
ARSIV_RX = re.compile(r"\.(tar|tar\.gz|tgz|tar\.xz|zip|7z|whl|deb|run|part\d+)$", re.I)
SIKISTIRMA_UZ = {".xz", ".bz2", ".zst", ".lz4", ".lzma", ".rar", ".gz", ".tbz2", ".txz"}
AG_FS = {"nfs", "nfs4", "cifs", "smb3", "smbfs", "fuse.sshfs", "sshfs", "fuse.rclone", "9p", "afs"}

# ---------------------------------------------------------------------------- sifre / token desenleri
_TR = rb"|sifre|\xc5\x9fifre|\xc5\x9eifre|parola"          # sifre / şifre / Şifre / parola
_KELIME = (rb"(?:password|passwd|passphrase|secret|token|api[_-]?key|apikey|access[_-]?key"
           rb"|auth[_-]?key|authtoken|private[_-]?key|client[_-]?secret" + _TR + rb")")
_ANAHTAR = rb"[A-Za-z0-9_.-]{0,40}" + _KELIME + rb"[A-Za-z0-9_.-]{0,40}"     # satir basi desenleri icin
_GUCLU = rb"[A-Za-z0-9_.-]{0,40}(?:password|passwd|passphrase|secret" + _TR + rb")[A-Za-z0-9_.-]{0,40}"


def _d(tur, desen, bayrak=0, kodda=True):
    return tur, re.compile(desen, bayrak), kodda


# (tur, desen, kodda_da). kodda_da=False: sadece ayar / kabuk / metin dosyalarinda (kodda yanlis alarm)
# (?P<v>...) grubu varsa ekranda sadece o kisim *** olur.
GIZLI_DESEN = [
    _d("GitHub token", rb"\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,})"),
    _d("ozel anahtar", rb"-----BEGIN [A-Z ]*PRIVATE KEY(?: BLOCK)?-----"),
    _d("ozel anahtar", rb"^PuTTY-User-Key-File-\d+:", re.M),
    _d("AWS anahtari", rb"\bAKIA[0-9A-Z]{16}\b"),
    _d("Slack token", rb"\bxox[abposr]-[A-Za-z0-9-]{10,}"),
    _d("API anahtari", rb"\b(?:sk-(?:ant-|proj-)?[A-Za-z0-9_-]{20,}|hf_[A-Za-z0-9]{30,}|AIza[0-9A-Za-z_-]{35}"
                       rb"|glpat-[A-Za-z0-9_-]{20,}|tskey-[A-Za-z0-9-]{16,})"),
    _d("Telegram bot token", rb"\b\d{8,10}:AA[A-Za-z0-9_-]{30,}"),
    _d("Bearer token", rb"\bbearer[ \t]+(?P<v>[A-Za-z0-9._~+/-]{20,}=*)", re.I),
    _d("URL'de kullanici:sifre", rb"\b[a-z][a-z0-9+.-]{1,15}://(?P<v>[^\s/:@'\"%{}$]+:[^\s/@'\"%{}$]+)@", re.I),
    # password = "x" / "api_key": "x" / api_key=\"x\" (ipynb) / token: 'x'
    # (on ek yok: her konumda denenen [..]{0,40} buyuk dosyada cok yavas; eslesme ayni)
    _d("sifre/token", _KELIME + rb"[A-Za-z0-9_.-]{0,40}(?:\\?[\"'])?[ \t]*[:=][ \t]*\\?[\"'](?P<v>[^\"'\s\\]{6,})\\?[\"']",
       re.I),
    # ayni, bosluklu deger (password: "iki kelime sifre") — sadece ayar dosyalarinda (kodda mesaj metni olur)
    _d("sifre/token", _KELIME + rb"[A-Za-z0-9_.-]{0,40}(?:\\?[\"'])?[ \t]*[:=][ \t]*\\?[\"'](?P<v>[^\"'\n\\]{6,})\\?[\"']",
       re.I, kodda=False),
    # satir basi tirnaksiz: export ROBOFLOW_API_KEY=ab12.. / api_key: ab12.. / Environment="TOKEN=ab12.."
    # (deger '=' ile baslamaz: pip kaydindaki SecretStorage==3.3.1 sifre degil)
    _d("sifre/token", rb"^[ \t]*(?:export[ \t]+|Environment=[\"']?)?(?P<k>" + _ANAHTAR +
       rb")[ \t]*[:=][ \t]*(?![\"'$({<\s=])(?P<v>(?=[^\s#\"']*[A-Za-z])(?=[^\s#\"']*[0-9])[^\s#\"']{8,})",
       re.I | re.M, kodda=False),
    # satir basi tirnaksiz, rakamsiz da olsa: WIFI_PASSWORD=ornek-wifi / db_secret: abcdef
    _d("sifre/token", rb"^[ \t]*(?:export[ \t]+|Environment=[\"']?)?(?P<k>" + _GUCLU +
       rb")[ \t]*[:=][ \t]*(?![\"'$({<\s=])(?=[^#\"'\n]{6,})(?P<v>[^#\"'\n]*[^\s#\"'])", re.I | re.M, kodda=False),
    # BUYUK_HARF_KEY=... / X_PASS=...
    _d("sifre/token", rb"^[ \t]*(?:export[ \t]+|Environment=[\"']?)?(?P<k>[A-Z][A-Z0-9_]*_(?:KEY|PASS|PW|PWD))"
       rb"[ \t]*=[ \t]*[\"']?(?![$({<\s~/.=])(?P<v>[^\s#\"']{8,})", re.M, kodda=False),
    # kodda da: ROBOFLOW_KEY = "rf12..." (harf+rakam, >=12) / WIFI_PASS = "..." (>=6)
    _d("sifre/token", rb"\b[A-Z][A-Z0-9_]*_(?:KEY|TOKEN)\b[ \t]*[:=][ \t]*[\"'](?P<v>(?=[^\"'\s]*[A-Za-z])"
                      rb"(?=[^\"'\s]*[0-9])[^\"'\s]{12,})[\"']"),
    _d("sifre/token", rb"\b[A-Z][A-Z0-9_]*_(?:PASS|PW|PWD)\b[ \t]*[:=][ \t]*[\"'](?P<v>[^\"'\s]{6,})[\"']"),
    _d("Authorization", rb"\bAuthorization[\"']?[ \t]*[:=][ \t]*[\"']?(?:Token|Basic|Bot|Bearer)[ \t]+"
                        rb"(?P<v>[A-Za-z0-9._~+/=-]{8,})", re.I),
    _d("sudo sifresi", rb"\b(?:echo|printf)[ \t]+(?P<v>[\"']?[^\s|;&$\"']{2,}[\"']?)[ \t]*\|[ \t]*sudo\b[^\n]{0,300}?"
                       rb"[ \t]-[A-Za-z]*S\b"),
    _d("sudo sifresi", rb"\bsudo\b(?=[^\n]{0,300}?[ \t]-[A-Za-z]*S\b)[^\n<]{0,300}<<<[ \t]*(?P<v>\S+)"),
    _d("nmcli sifresi", rb"\bnmcli\b[^\n]{0,300}?\b(?:password|wifi-sec\.psk|802-1x\.password|psk)[ \t]+"
                        rb"(?P<v>\"[^\"\n]{4,}\"|'[^'\n]{4,}'|[^\s\"']{4,})", re.I),
    _d("Wi-Fi sifresi", rb"^[ \t]*psk[ \t]*=[ \t]*(?P<v>\"[^\"\n]{8,}\"|\S{8,})", re.I | re.M),
    _d("Wi-Fi sifresi", rb"\bwpa_passphrase[ \t]+\S+[ \t]+(?P<v>\S+)"),
    _d("mount sifresi", rb"(?:^|[,\s;])(?:password|passwd|pass)=(?P<v>[^,\s\"'$]{3,})", re.I | re.M, kodda=False),
    _d("komut satiri sifresi", rb"\bcurl\b[^\n]{0,300}?[ \t](?:-u|--user)[ \t=]*[\"']?(?P<v>[^\s\"':$]+:[^\s\"'$]+)"),
    _d("komut satiri sifresi", rb"--(?:password|passwd|pass)(?:=|[ \t]+)(?P<v>[\"']?[^\s\"'$-][^\s\"']*[\"']?)", re.I),
    _d("komut satiri sifresi", rb"\bmysql(?:dump|admin)?\b[^\n]{0,300}?[ \t]-p(?P<v>[^\s\"'$-][^\s\"']*)"),
    _d("komut satiri sifresi", rb"\bsmbclient\b[^\n]{0,300}?[ \t]-U[ \t]*[\"']?[^\s%\"']+%(?P<v>[^\s\"']+)"),
    _d("sshpass", rb"\bsshpass[ \t]+-p[ \t]*(?P<v>\S+)"),
    _d(".netrc", rb"\bmachine[ \t]+\S+[ \t]+login[ \t]+\S+[ \t]+password[ \t]+(?P<v>\S+)", re.I),
]
OZEL_ANAHTAR_RX = re.compile(rb"-----BEGIN [A-Z ]*PRIVATE KEY(?: BLOCK)?-----|^PuTTY-User-Key-File-\d+:", re.M)
GIZLI_AD = re.compile(r"(^|/)(\.env(\.[^/]*)?|[^/]*\.(pem|key|p12|pfx|jks|keystore|ovpn|ppk|asc|gpg)|id_(rsa|dsa|ecdsa|ed25519)"
                      r"|\.git-credentials|\.netrc|\.pypirc|\.npmrc|wpa_supplicant[^/]*\.conf"
                      r"|credentials(\.json)?|token\.json|client_secret[^/]*\.json)$", re.I)

REPO = Path(".")
REF = Path(".")
YEDEK = Path(".")
RENK = sys.stdout.isatty()
_UZAK_ETIKET = None
BEKLEYEN_ONAY = []             # bu calismada onaylananlar (blob sha, tur): commit / push sonrasi yazilir
YEDEK_ISKELET = []             # yedek yerinde bu calismada olusturulan ara klasorler (temizlikte sadece bunlar)
_AG_BAGLAMA = None


class Dur(Exception):
    """Aciklamasi yapilmis, kontrollu cikis (hata)."""


class Bitti(Exception):
    """Aciklamasi yapilmis, kontrollu cikis (basari, tekrar calistir)."""


# ---------------------------------------------------------------------------- sinyaller
class _Sinyal:
    kritik = 0
    bekleyen = None
    cikista = False      # durduruluyor (ilk sinyal / hata): sonraki sinyaller sadece kaydedilir, geri alma bolunmez


def _sinyal_isleyici(signum, frame):
    """Ctrl-C / SSH kopmasi (SIGHUP) / SIGTERM: kritik bolumde ertelenir, degilse KeyboardInterrupt.
    Ikinci Ctrl-C (ya da SIGHUP ardindan SIGTERM) geri almayi yarida birakamaz."""
    if _Sinyal.kritik or _Sinyal.cikista:
        _Sinyal.bekleyen = signum
        return
    _Sinyal.cikista = True
    raise KeyboardInterrupt


@contextmanager
def kritik(bekleyeni_at: bool = False):
    """Bu blok yarim kalmasin: sinyal gelirse blok bitince islenir (bekleyeni_at: hic islenmez)."""
    _Sinyal.kritik += 1
    try:
        yield
    finally:
        _Sinyal.kritik -= 1
        if _Sinyal.kritik == 0 and _Sinyal.bekleyen is not None:
            _Sinyal.bekleyen = None
            if not bekleyeni_at and not _Sinyal.cikista:
                _Sinyal.cikista = True
                raise KeyboardInterrupt


# ---------------------------------------------------------------------------- ekran
def yaz(s: str = "") -> None:
    """Ekrana yaz; terminal gittiyse (SSH koptu) sessizce gec."""
    try:
        print(s, flush=True)
    except (OSError, ValueError):
        pass


def _r(kod: str, s: str) -> str:
    return f"\033[{kod}m{s}\033[0m" if RENK else s


def bas(s: str) -> None:
    yaz("\n" + _r("1;36", f"== {s} =="))


def ok(s: str) -> None:
    yaz(f"  {_r('32', '✓')} {s}")


def bi(s: str) -> None:
    yaz(f"  • {s}")


def uy(s: str) -> None:
    yaz(f"  {_r('33', '!')} {s}")


def ha(s: str) -> None:
    yaz(f"  {_r('31', '✗')} {s}")


def sor(soru: str, varsayilan: bool) -> bool:
    ek = "[E/h]" if varsayilan else "[e/H]"
    try:
        c = input(f"  {soru} {ek} ").strip().lower()
    except (EOFError, OSError):
        yaz()
        return varsayilan
    if not c:
        return varsayilan
    return c[0] in ("e", "y")


# ---------------------------------------------------------------------------- yardimcilar
def mb(n: float) -> str:
    return f"{n / MB:.1f} MB"


def goster(s: str) -> str:
    """Dosya adini ekranda guvenle gostermek icin (bozuk utf-8 -> ?)."""
    try:
        return s.encode("utf-8", "surrogateescape").decode("utf-8", "replace")
    except UnicodeError:
        return s.encode("utf-8", "replace").decode("utf-8")


def uzanti(rel: str) -> str:
    return os.path.splitext(rel)[1].lower()


def kod_mu(rel: str) -> bool:
    return uzanti(rel) in KOD_DIL


def altinda(yol: str, dizin: str) -> bool:
    dizin = dizin.rstrip("/")
    return yol == dizin or yol.startswith(dizin + "/")


def web_adres(url: str) -> str:
    m = re.search(r"github\.com[:/]+([^/\s]+)/([^/\s]+?)(?:\.git)?/?$", url.strip())
    return f"https://github.com/{m.group(1)}/{m.group(2)}" if m else ""


def temiz_url(u: str) -> str:
    return re.sub(r"(https?://)[^/@\s]+@", r"\1", u.strip())


def sha256_dosya(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for blok in iter(lambda: f.read(4 * MB), b""):
            h.update(blok)
    return h.hexdigest()


def pathspec(yollar) -> bytes:
    """git --pathspec-from-file=- --pathspec-file-nul icin; her yol AYNEN (glob yok)."""
    return b"".join(b":(literal)" + os.fsencode(y) + b"\0" for y in yollar)


def atomik_yaz(f: Path, veri: bytes) -> None:
    tmp = f.with_name(f"{f.name}.gite_{os.getpid()}")
    tmp.write_bytes(veri)
    os.replace(tmp, f)


def ag_baglama() -> set:
    """Ag dosya sistemi baglama noktalari (dolasilmaz: takili kalabilir)."""
    global _AG_BAGLAMA
    if _AG_BAGLAMA is None:
        _AG_BAGLAMA = set()
        try:
            for satir in Path("/proc/mounts").read_text(errors="replace").splitlines():
                p = satir.split()
                if len(p) >= 3 and p[2] in AG_FS:
                    _AG_BAGLAMA.add(p[1].replace("\\040", " "))
        except OSError:
            pass
    return _AG_BAGLAMA


def dolas(kok: Path, onerror=None):
    """os.walk; ag baglamalarina girmez."""
    ag = ag_baglama()
    for dp, dns, fns in os.walk(kok, onerror=onerror):
        if ag:
            dns[:] = [d for d in dns if os.path.join(dp, d) not in ag]
        yield dp, dns, fns


def teknik_ornek(d: Path) -> list:
    """Klasorde kod/model dosyasi var mi? (en fazla 3 ornek, derinlik 3)"""
    ornek, n = [], 0
    for dp, dns, fns in dolas(d, onerror=lambda e: None):
        try:
            derinlik = len(Path(dp).relative_to(d).parts)
        except ValueError:
            derinlik = 0
        if derinlik >= 3:
            dns[:] = []
        dns[:] = [x for x in dns if not x.startswith(".") and x not in ("node_modules", "__pycache__")]
        for f in fns:
            n += 1
            if uzanti(f) in TEKNIK_UZ:
                ornek.append(os.path.relpath(os.path.join(dp, f), d))
                if len(ornek) >= 3:
                    return ornek
            if n > 3000:
                return ornek
    return ornek


def du(p: Path) -> str:
    rc, out, _ = calistir(["du", "-skx", str(p)], timeout=20)
    try:
        return mb(int(out.split()[0]) * 1024) if rc == 0 else "?"
    except (IndexError, ValueError):
        return "?"


def bos_klasorleri_sil(kok: Path) -> None:
    if not kok.is_dir():
        return
    for dp, dns, fns in os.walk(kok, topdown=False):
        try:
            os.rmdir(dp)
        except OSError:
            pass


def yok_mu(p: Path):
    """True: yok (silinmis), False: var, None: bakilamiyor (izin) — izin sorunu 'silinmis' sayilmasin."""
    try:
        os.lstat(p)
        return False
    except OSError as e:
        return True if e.errno in (errno.ENOENT, errno.ENOTDIR) else None


def ust_link(rel: str):
    """rel'in ust klasorlerinden biri sembolik link mi? (or. models/ -> ~/ssd/models) — link olan klasor."""
    yol = REPO
    parcalar = rel.split("/")[:-1]
    for i, p in enumerate(parcalar):
        yol = yol / p
        if os.path.islink(yol):
            return "/".join(parcalar[:i + 1])
    return None


# ---------------------------------------------------------------------------- komut calistirma
def _durdur(p, grup: bool) -> None:
    for sinyal, bekle in ((signal.SIGTERM, 10), (signal.SIGKILL, 5)):
        try:
            if grup:
                os.killpg(p.pid, sinyal)
            else:
                p.send_signal(sinyal)
            p.wait(bekle)
            return
        except subprocess.TimeoutExpired:
            continue
        except OSError:
            return


def calistir(cmd, cwd=None, timeout=120, girdi=None, env=None, yakala=True, grup=False):
    """(rc, stdout, stderr). Zaman asiminda once SIGTERM (git kilit dosyalarini temizlesin), sonra SIGKILL.
    grup=True: komut kendi oturumunda (terminal Ctrl-C / SSH kopmasi ona ulasmaz); zaman asimi ya da
    Ctrl-C'de tum alt surecleriyle durdurulur."""
    e = dict(os.environ if env is None else env)
    e["GIT_OPTIONAL_LOCKS"] = "0"          # git status vb. .git/index'i yeniden yazmasin
    if str(cmd[0]) == "git":               # git mesajlari Ingilizce: uyarilar ayristirilabilsin (tr_TR'de de)
        e["LC_ALL"] = "C"
        e.pop("LANGUAGE", None)
    e["PYTHONDONTWRITEBYTECODE"] = "1"
    if yakala:
        e.setdefault("GIT_TERMINAL_PROMPT", "0")
    if girdi is not None:
        stdin = subprocess.PIPE
    else:
        stdin = subprocess.DEVNULL if (yakala or grup) else None
    try:
        p = subprocess.Popen([str(c) for c in cmd], cwd=cwd, env=e, stdin=stdin,
                             stdout=subprocess.PIPE if yakala else None,
                             stderr=subprocess.PIPE if yakala else None,
                             start_new_session=grup)
    except OSError as x:                     # bulunamadi / calistirma izni yok
        return 127, b"", f"{cmd[0]}: {x.strerror or x}".encode()
    try:
        out, err = p.communicate(girdi, timeout=timeout)
    except subprocess.TimeoutExpired:
        _durdur(p, grup)
        try:
            out, err = p.communicate(timeout=15)
        except (subprocess.TimeoutExpired, ValueError, OSError):
            out, err = b"", b""
        return 124, out or b"", (err or b"") + b"\n(zaman asimi)"
    except BaseException:
        _durdur(p, grup)
        raise
    return p.returncode, out or b"", err or b""


def git(*args, timeout=None, girdi=None, izin=(0,), grup=False) -> bytes:
    rc, out, err = calistir(["git", "-C", str(REPO), *args], timeout=timeout, girdi=girdi, grup=grup)
    if rc not in izin:
        m = (err or out).decode("utf-8", "replace").strip()
        kisa, atla = [], False
        for a in args:
            if atla:
                atla = False
                continue
            if a == "-c":
                atla = True
                continue
            kisa.append(str(a))
        raise Dur(f"git {' '.join(kisa)[:80]}: {m[-600:]}")
    return out


def git_s(*args, **kw) -> str:
    return git(*args, **kw).decode("utf-8", "replace").strip()


def git_z(*args, **kw) -> list:
    return [os.fsdecode(x) for x in git(*args, **kw).split(b"\0") if x]


def git_yolu(ad: str) -> Path:
    p = Path(os.fsdecode(git("rev-parse", "--git-path", ad).rstrip(b"\n")))
    return p if p.is_absolute() else REPO / p


def nesne_boyutlari(shas) -> dict:
    """sha -> (tur, boyut)"""
    shas = sorted(set(shas))
    if not shas:
        return {}
    out = git("cat-file", "--batch-check=%(objectname) %(objecttype) %(objectsize)",
              girdi=("\n".join(shas) + "\n").encode())
    sonuc = {}
    for satir in out.decode("utf-8", "replace").splitlines():
        p = satir.split()
        if len(p) == 3 and p[2].isdigit():
            sonuc[p[0]] = (p[1], int(p[2]))
    return sonuc


def _blob_oku(shas) -> dict:
    shas = sorted(set(shas))
    if not shas:
        return {}
    out = git("cat-file", "--batch", girdi=("\n".join(shas) + "\n").encode())
    sonuc, i = {}, 0
    while i < len(out):
        nl = out.find(b"\n", i)
        if nl < 0:
            break
        p = out[i:nl].split()
        if len(p) == 3 and p[2].isdigit():
            n = int(p[2])
            sonuc[p[0].decode()] = out[nl + 1:nl + 1 + n]
            i = nl + 1 + n + 1
        else:
            i = nl + 1                         # "<sha> missing"
    return sonuc


def blob_icerikleri(liste):
    """liste: [(sha, yol, boyut)] -> (sha, yol, icerik) — bellek icin 64 MB'lik gruplar halinde."""
    grup, toplam = [], 0
    for kayit in sorted(liste, key=lambda x: x[2]):
        grup.append(kayit)
        toplam += kayit[2]
        if toplam >= 64 * MB:
            icerik = _blob_oku([s for s, _, _ in grup])
            for s, y, _ in grup:
                if s in icerik:
                    yield s, y, icerik[s]
            grup, toplam = [], 0
    if grup:
        icerik = _blob_oku([s for s, _, _ in grup])
        for s, y, _ in grup:
            if s in icerik:
                yield s, y, icerik[s]


def blob_sha_veri(rel: str, veri: bytes):
    """Taranan baytlarin git add'de olusacak blob sha'si (arada dosya degisirse onay eski icerige baglanmasin)."""
    rc, out, _ = calistir(["git", "-C", str(REPO), *LFS_KAPALI, "hash-object", "--stdin", f"--path={rel}"],
                          girdi=veri, timeout=300)
    s = out.decode("ascii", "replace").strip()
    return s if rc == 0 and re.fullmatch(r"[0-9a-f]{40,64}", s) else None


def blob_sha(rel: str, link: bool = False):
    """git add'in bu dosya icin uretecegi blob sha'si (satir sonu donusumu dahil, LFS kapali)."""
    if link:
        try:
            hedef = os.fsencode(os.readlink(REPO / rel))
        except OSError:
            return None
        rc, out, _ = calistir(["git", "-C", str(REPO), "hash-object", "--stdin"], girdi=hedef, timeout=60)
    else:
        rc, out, _ = calistir(["git", "-C", str(REPO), *LFS_KAPALI, "hash-object", f"--path={rel}", "--", rel],
                              timeout=300)
    s = out.decode("ascii", "replace").strip()
    return s if rc == 0 and re.fullmatch(r"[0-9a-f]{40,64}", s) else None


# ---------------------------------------------------------------------------- sifre taramasi
def ikili_mi(bas_: bytes, boyut: int, rel: str) -> bool:
    """Taranmayacak ikili dosya mi? NUL'lu ama kucuk ve bilinmeyen turde dosyalar (or. elektrik kesintisinde
    NUL dolmus yaml) yine taranir; UTF-16 metin taranir."""
    if bas_[:2] in (b"\xff\xfe", b"\xfe\xff") or b"\0" not in bas_[:8192]:
        return False
    return boyut > KUCUK_IKILI or uzanti(rel) in IKILI_UZ or bas_[:4] == b"\x7fELF"


def metin_icerik(veri: bytes, rel: str = ""):
    """Taranacak icerik (bytes) ya da None (ikili)."""
    if veri[:2] in (b"\xff\xfe", b"\xfe\xff"):
        return veri.decode("utf-16", "replace").encode("utf-8", "replace")
    return None if ikili_mi(veri, len(veri), rel) else veri


def _yeni_satirlar(veri: bytes):
    i = veri.find(b"\n")
    while i >= 0:
        yield i
        i = veri.find(b"\n", i + 1)


def gizli_icerik(veri: bytes, kod: bool = False, sinir: int = 0) -> list:
    """[(satir_no, tur, maskeli_satir)] — TUM bulgular (ayni satir+tur bir kez); sinir>0 ise o kadar."""
    bulunan, gorulen = [], set()
    satir_basi = None
    for tur, rx, kodda in GIZLI_DESEN:
        if kod and not kodda:
            continue
        for m in rx.finditer(veri):
            v = m.span("v") if "v" in rx.groupindex and m.group("v") is not None else None
            poz = v[0] if v else m.start()
            if satir_basi is None:
                satir_basi = [0] + [i + 1 for i in _yeni_satirlar(veri)]
            no = bisect.bisect_right(satir_basi, poz)
            if (no, tur) in gorulen:
                continue
            gorulen.add((no, tur))
            bs = satir_basi[no - 1]
            sn = veri.find(b"\n", poz)
            sn = len(veri) if sn < 0 else sn
            if v:
                a, b = max(v[0], bs), min(v[1], sn)
            else:
                a, b = max(m.start(), bs), min(m.end(), sn)
            pb, ps = max(bs, a - 50), min(sn, b + 40)      # uzun satirda bulgunun cevresi
            gizlenen = b"***" if v else veri[a:min(a + 4, b)] + b"***"
            metin = (veri[pb:a] + gizlenen + veri[b:ps]).decode("utf-8", "replace").strip()
            metin = ("..." if pb > bs else "") + metin + ("..." if ps < sn else "")
            bulunan.append((no, tur, metin[:140]))
            if sinir and len(bulunan) >= sinir:
                return bulunan
    return sorted(bulunan)


def maskele(satir: bytes) -> bytes:
    """Satirdaki TUM bulgulari *** yapar, gerisi (komut, ad) kalir. Emin olunamazsa b"" (satir cikarilir)."""
    araliklar = []
    for _, rx, _ in GIZLI_DESEN:
        for m in rx.finditer(satir):
            if "v" in rx.groupindex and m.group("v") is not None:
                a, b = m.span("v")
            else:
                a, b = m.start() + min(4, m.end() - m.start()), m.end()
            # tirnakli deger ("iki kelime" gibi) kapanan tirnaga kadar gizlensin
            q = satir[a:a + 1] if satir[a:a + 1] in (b'"', b"'") else (
                satir[a - 1:a] if a > 0 and satir[a - 1:a] in (b'"', b"'") else b"")
            if q:
                kapan = satir.find(q, a + 1 if satir[a:a + 1] == q else a)
                b = len(satir) if kapan < 0 else max(b, kapan + (1 if satir[a:a + 1] == q else 0))
            araliklar.append((a, b))
    sonuc, son = b"", 0
    for a, b in sorted(araliklar):
        a = max(a, son)
        if b <= a:
            continue
        sonuc += satir[son:a] + b"***"
        son = b
    sonuc += satir[son:]
    if not araliklar:
        return b""
    for _, rx, _ in GIZLI_DESEN:                 # gizlenmis halde hala iz var mi?
        for m in rx.finditer(sonuc):
            if "v" not in rx.groupindex or m.group("v") is None:
                return b""
            if len(m.group("v").replace(b"***", b"").strip(b"\"' ")) >= 3:
                return b""
    return sonuc


def temizle(veri: bytes, maske: bool = False):
    """(temiz_veri, temizlenen_satir_sayisi, birak). Ozel anahtar varsa dosya hic kullanilmaz (birak).
    maske=False: iz olan satir tamamen cikarilir (GitHub'a gidenler). maske=True: sadece degerler *** olur
    (yerel arsiv; ne yapildigi anlasilsin), emin olunamayan satir yine cikarilir."""
    if OZEL_ANAHTAR_RX.search(veri):
        return b"", 0, True
    satirlar = veri.split(b"\n")
    n = 0
    for i, s in enumerate(satirlar):
        if s.strip() and gizli_icerik(s, sinir=1):
            m = maskele(s) if maske else b""
            satirlar[i] = m or b"# [gite_gonder: sifre/token izi - bu satir cikarildi]"
            n += 1
    return b"\n".join(satirlar), n, False


def onaylar_oku() -> set:
    """Daha once 'gercek degil' denenler: {(blob_sha, tur)} — icerige bagli; dosya degisirse tekrar sorulur."""
    try:
        veri = git_yolu(ONAY_ADI).read_text(encoding="ascii", errors="replace")
    except OSError:
        return set()
    return {tuple(x.split(" ", 1)) for x in veri.splitlines() if " " in x}


def onaylar_yaz() -> None:
    if not BEKLEYEN_ONAY:
        return
    try:
        eski = onaylar_oku()
        yeni = sorted({k for k in BEKLEYEN_ONAY if k not in eski and k[0] != "?"})
        if yeni:
            with open(git_yolu(ONAY_ADI), "a", encoding="ascii", errors="replace") as f:
                f.write("".join(f"{s_} {t_}\n" for s_, t_ in yeni))
    except Exception:
        pass
    BEKLEYEN_ONAY.clear()


def dosya_oku(rel: str):
    """(ham, taranacak): ham = diskteki baytlar (onay bunun blob sha'sina baglanir), taranacak = metin ya da None.
    Ikili buyuk dosyanin ilk 8 KB'si disini okumaz."""
    p = REPO / rel
    try:
        with open(p, "rb") as f:
            bas_ = f.read(8192)
            if ikili_mi(bas_, os.fstat(f.fileno()).st_size, rel):
                return None, None
            ham = bas_ + f.read()
    except OSError:
        return None, None
    return ham, metin_icerik(ham, rel)


def gizli_tara(adaylar: list) -> list:
    """[(rel, satir, tur, maskeli, blob_sha)] — TUM metin dosyalari (boyut siniri yok, <=99 MB).
    Daha once 'gercek degil' denen AYNI icerik + tur tekrar sorulmaz."""
    onay = onaylar_oku() | set(BEKLEYEN_ONAY)
    sonuc = []
    for rel, b, l in adaylar:
        bulgular = []
        if GIZLI_AD.search(rel):
            bulgular.append((0, "hassas dosya adi", ""))
        ham = None
        if not l and 0 < b <= SINIR_ENGEL:
            ham, v = dosya_oku(rel)
            if v is not None:
                bulgular += gizli_icerik(v, kod=kod_mu(rel))
        if not bulgular:
            continue
        sha = (blob_sha_veri(rel, ham) if ham is not None else blob_sha(rel, l)) or "?"
        for no, tur, ms in bulgular:
            if (sha, tur) not in onay:
                sonuc.append((rel, no, tur, ms, sha))
    return sonuc


def gizli_goster(gizli: list) -> None:
    """Dosya basina en fazla 6 satir — her bulgu TURU en az bir kez gosterilir; kalan sayisi yazilir."""
    dosyalar = {}
    for kayit in gizli:
        dosyalar.setdefault(kayit[0], []).append(kayit)
    for i, (rel, lst) in enumerate(dosyalar.items()):
        if i >= 15:
            yaz(f"      ... +{len(dosyalar) - 15} dosya daha (toplam {len(gizli)} bulgu)")
            break
        secilen, turler = [], set()
        for kayit in lst:                        # once her turden bir tane
            if kayit[2] not in turler:
                turler.add(kayit[2])
                secilen.append(kayit)
        for kayit in lst:
            if len(secilen) >= 6:
                break
            if kayit not in secilen:
                secilen.append(kayit)
        for _, no, tur, ms, _ in sorted(secilen, key=lambda x: (x[1], x[2])):
            yer = f"{goster(rel)}:{no}" if no else goster(rel)
            yaz(f"      {yer}  [{tur}]  {ms}")
        if len(lst) > len(secilen):
            yaz(f"      ... {goster(rel)}: +{len(lst) - len(secilen)} bulgu daha (dosyayi ac, hepsine bak)")


# ---------------------------------------------------------------------------- repo + yedek yeri
def repo_bul() -> Path:
    adaylar = [Path.cwd(), Path(__file__).resolve().parent, Path.home() / "Desktop" / "avci"]
    for a in adaylar:
        if not a.is_dir():
            continue
        rc, out, _ = calistir(["git", "-C", str(a), "rev-parse", "--show-toplevel"], timeout=20)
        if rc == 0:
            return Path(os.fsdecode(out.strip()))
    raise Dur("git reposu bulunamadi. Once: cd ~/Desktop/avci")


def yedek_kok() -> Path:
    """Repo ile AYNI diskte yedek yeri (tasima = os.rename: atomik, yarim kalmaz)."""
    aday = Path(os.environ.get("DESKTOP") or (Path.home() / "Desktop")) / "_repo_yedek" / DAMGA
    try:
        dev = os.stat(REPO).st_dev
        p = aday
        while not p.exists():
            p = p.parent
        if os.stat(p).st_dev == dev and os.access(p, os.W_OK | os.X_OK):
            return aday
    except OSError:
        pass
    return Path(git_s("rev-parse", "--absolute-git-dir")) / "gite_gonder_yedek" / DAMGA


def iskelet_kur(d: Path) -> None:
    """Klasoru (ve eksik ustlerini) olusturur; olusturulanlari temizlik icin kaydeder."""
    eksik = []
    while not os.path.lexists(d):
        eksik.append(d)
        d = d.parent
    for x in reversed(eksik):
        os.mkdir(x)
        YEDEK_ISKELET.append(x)


def yedek_temizle() -> None:
    """Gecici index kopyasini ve bos kalan ara klasorleri siler. Tasinan .git'lerin ICINE girmez
    (bos refs/ gibi klasorler silinirse o .git bozulur)."""
    try:
        (YEDEK / "_git_index").unlink()
    except OSError:
        pass
    for d in sorted(set(YEDEK_ISKELET), key=lambda x: len(x.parts), reverse=True):
        try:
            os.rmdir(d)
        except OSError:
            pass


def yedege_tasi(src: Path, rel: str) -> Path:
    dst = YEDEK / rel
    if os.path.lexists(dst):
        raise Dur(f"yedekte zaten var: {dst}")
    iskelet_kur(dst.parent)
    try:
        os.rename(src, dst)
    except OSError as e:
        neden = "farkli disk" if e.errno == errno.EXDEV else (e.strerror or str(e))
        ek = ""
        if e.errno in (errno.EACCES, errno.EPERM):
            ek = f" Izin icin: sudo chown -R {KULLANICI}:{KULLANICI} {goster(str(src.parent))}"
        raise Dur(f"tasinamadi: {goster(rel)} -> {dst} ({neden}).{ek}")
    return dst


# ---------------------------------------------------------------------------- 0) on kontrol
@dataclass
class Onbilgi:
    taban: str = ""
    ileride: int = 0
    kimlik: tuple = ()
    web: str = ""


def on_kontrol(kontrol: bool) -> Onbilgi:
    bas("0) On kontrol")
    yaz(f"  Repo: {REPO}")
    b = Onbilgi()
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        raise Dur("sudo ile degil, normal kullanici olarak calistir: python3 setup/gite_gonder.py")
    m = re.search(r"(\d+)\.(\d+)", git_s("version"))
    if not m or (int(m.group(1)), int(m.group(2))) < (2, 25):
        raise Dur(f"git 2.25 veya ustu gerekli ({git_s('version')})")
    gd = Path(git_s("rev-parse", "--absolute-git-dir"))
    for f in ("MERGE_HEAD", "rebase-merge", "rebase-apply", "CHERRY_PICK_HEAD", "REVERT_HEAD"):
        if (gd / f).exists():
            raise Dur(f"yarim kalmis git islemi var ({f}). Once onu bitir ya da iptal et.")
    if (gd / "index.lock").exists():
        raise Dur(f"{gd / 'index.lock'} var: baska bir git islemi calisiyor ya da yarim kaldi.")
    dal = git_s("symbolic-ref", "--short", "-q", "HEAD", izin=(0, 1))
    if dal != "main":
        raise Dur(f"su anki dal '{dal or 'detached HEAD'}'. Bu script main icin: git switch main")
    ok("dal: main")

    url = git_s("remote", "get-url", "origin", izin=(0, 2, 128))
    purl = git_s("remote", "get-url", "--push", "origin", izin=(0, 2, 128))
    if not url:
        raise Dur("origin tanimli degil: git remote add origin git@github.com:hasanucas/avci.git")
    if not (ORIGIN_RX.search(url) and ORIGIN_RX.search(purl or url)):
        uy(f"origin beklenenden farkli: {temiz_url(url)}" + (f"  (push: {temiz_url(purl)})" if purl != url else ""))
        if kontrol or not sor("Yine de bu origin'e gonderilsin mi?", False):
            raise Dur("iptal — hicbir sey degismedi")
    else:
        ok(f"origin: {temiz_url(url)}")
    b.web = web_adres(purl or url)
    if url.lower().startswith("https://") and "github.com" in url.lower():
        uy("origin HTTPS. Push icin SSH anahtari onerilir:")
        yaz("      git remote set-url origin git@github.com:hasanucas/avci.git")

    if not (git_s("config", "user.name", izin=(0, 1)) and git_s("config", "user.email", izin=(0, 1))):
        b.kimlik = ("-c", f"user.name={KULLANICI} ({MAKINE})", "-c", f"user.email={KULLANICI}@{MAKINE}.local")
        bi("git kullanici adi/e-posta yok — commit'te bu Orin'in adi kullanilacak (ayar dosyasina yazilmaz)")

    bi("GitHub'daki son durum aliniyor (git fetch)...")
    rc, _, err = calistir(["git", "-C", str(REPO), "fetch", "origin", "--quiet"], timeout=None)
    if rc != 0:
        raise Dur("git fetch basarisiz — internet var mi, SSH anahtari GitHub'da mi? "
                  "(ssh -T git@github.com)\n    " + err.decode("utf-8", "replace").strip()[-400:])
    if calistir(["git", "-C", str(REPO), "rev-parse", "--verify", "-q", "origin/main"])[0] != 0:
        raise Dur("GitHub'da main dali yok (origin/main).")
    if calistir(["git", "-C", str(REPO), "merge-base", "--is-ancestor", "origin/main", "HEAD"])[0] != 0:
        geride_birak(kontrol)
    b.taban = git_s("log", "-1", "--format=%h %cs %s", "origin/main")
    b.ileride = int(git_s("rev-list", "--count", "origin/main..HEAD") or "0")
    if b.ileride:
        uy(f"bu Orin'de GitHub'a gitmemis {b.ileride} commit var — onlar da gidecek (kontrol ediliyor)")
        yerel_commit_kontrol(kontrol)
    ok(f"GitHub main: {b.taban}")
    return b


def geride_birak(kontrol: bool) -> None:
    """GitHub'daki main bu Orin'de olmayan commit iceriyor (or. laptop avci_follow'u push etti).
    Sadece onlari alir ve DURUR — gonderim icin tekrar calistirilir (her adim ayri, net)."""
    n = git_s("rev-list", "--count", "HEAD..origin/main")
    yerel = int(git_s("rev-list", "--count", "origin/main..HEAD") or "0")
    if yerel:
        raise Dur(f"GitHub'da bu Orin'de olmayan {n} commit var, bu Orin'de de GitHub'da olmayan {yerel} commit var.\n"
                  "    Hicbir sey degismedi. Su ciktiyi bana gonder, birlikte birlestirelim:\n"
                  "      git log --oneline --graph --stat HEAD...origin/main")
    dosyalar = git_z("diff", "--name-only", "-z", "--no-renames", "HEAD", "origin/main")
    uy(f"GitHub'daki main'de bu Orin'de olmayan {n} commit var:")
    for s in git_s("log", "--format=%h %cs %an: %s", "HEAD..origin/main").splitlines()[:10]:
        yaz(f"      {s}")
    yaz(f"    Degistirdikleri dosyalar ({len(dosyalar)}):")
    for f in dosyalar[:20]:
        yaz(f"      {goster(f)}")
    if len(dosyalar) > 20:
        yaz(f"      ... +{len(dosyalar) - 20}")
    # bu Orin'de sembolik linke donusmus klasorlerin icini degistiren commit: git linki silip gercek klasor
    # acar (oradaki diger dosyalar git'te 'silinmis' gorunur) — otomatik ALINMAZ
    linkli = sorted({ust_link(f) for f in dosyalar if ust_link(f)})
    if linkli:
        for d in linkli[:10]:
            ha(f"bu Orin'de sembolik link: {goster(d)}  (GitHub'daki commit'ler bunun icini degistiriyor)")
        raise Dur("bunlari git otomatik alirsa link yerine gercek klasor acar. Hicbir sey degismedi.\n"
                  "    Bana yaz; ya da linki gecici kaldirip klasoru geri koy, sonra tekrar calistir.")
    orin_kodu = any(f.startswith(("hunter-orin/", "orin-tiled-tracker/", "setup/")) for f in dosyalar)
    if orin_kodu:
        uy("Bunlarin bir kismi bu Orin'de CALISAN kodu degistirir (hunter-orin / tiled / setup).")
    if kontrol:
        bi("gonderimde once sadece bunlarin alinmasi sorulur (git merge --ff-only); sonra tekrar calistirilir")
        raise Bitti("--kontrol: once GitHub'dakiler alinmali; diger kontroller ondan sonra")
    if not sor("Bunlar simdi bu Orin'e alinsin mi (git merge --ff-only)? Sonra scripti tekrar calistirirsin.",
               not orin_kodu):
        raise Dur("iptal — hicbir sey degismedi")
    # --no-overwrite-ignore: git'te olmayan (yok sayilan) yerel dosyalarin uzerine YAZMASIN.
    # Kendi oturumunda + kritik: Ctrl-C / SSH kopmasi agaci yarim birakmasin.
    with kritik():
        rc, out, err = calistir(["git", "-C", str(REPO), "merge", "--ff-only", "--no-overwrite-ignore", "-q",
                                 "origin/main"], timeout=None, grup=True)
    if rc != 0:
        raise Dur("alinamadi (bu Orin'deki dosyalarla cakisiyor). Hicbir sey degismedi.\n    "
                  + (err or out).decode("utf-8", "replace").strip()[-500:]
                  + "\n    Bu ciktiyi bana gonder, birlikte birlestirelim.")
    raise Bitti(f"GitHub'daki {n} commit bu Orin'e alindi (yerel degisikliklere ve dosyalara dokunulmadi).\n"
                "    Simdi gondermek icin tekrar calistir: python3 setup/gite_gonder.py")


def yerel_commit_kontrol(kontrol: bool) -> None:
    """Push edilmemis commit'lerde GitHub'in reddedecegi ya da herkese acilmamasi gereken sey var mi?"""
    commitler = git_s("rev-list", "origin/main..HEAD").split()
    uzak_link = set()
    for kayit in git("ls-tree", "-r", "-z", "origin/main").split(b"\0"):
        if kayit.startswith(b"160000 "):
            uzak_link.add(kayit)
    gitlink = []
    for c in commitler:
        for kayit in git("ls-tree", "-r", "-z", c).split(b"\0"):
            if kayit.startswith(b"160000 ") and kayit not in uzak_link:
                gitlink.append((c[:8], os.fsdecode(kayit.split(b"\t", 1)[1])))
    yol = {}
    for satir in git("rev-list", "--objects", "origin/main..HEAD").splitlines():
        p = satir.split(b" ", 1)
        if p and p[0]:
            yol.setdefault(p[0].decode(), os.fsdecode(p[1]) if len(p) > 1 else "")
    boy = nesne_boyutlari(list(yol))
    buyuk = [(yol[s], n) for s, (tur, n) in boy.items() if tur == "blob" and n > SINIR_ENGEL]
    if buyuk or gitlink:
        for y, n in buyuk:
            ha(f"push edilmemis commit'te {mb(n)}: {goster(y)}")
        for c, y in sorted(set(gitlink)):
            ha(f"push edilmemis commit {c}'de ic ice repo baglantisi (icerigi gitmez): {goster(y)}")
        raise Dur("bu commit'ler GitHub'a bu haliyle gidemez. Hicbir sey degismedi; "
                  "ciktiyi bana gonder (gecmisi birlikte duzeltelim).")
    tara = [(s, yol[s], n) for s, (tur, n) in boy.items() if tur == "blob"]
    onay = onaylar_oku()
    gizli = []
    for s, y, v in blob_icerikleri(tara):
        m = metin_icerik(v, y)
        bulgular = gizli_icerik(m, kod=kod_mu(y)) if m is not None else []
        if GIZLI_AD.search(y):
            bulgular.append((0, "hassas dosya adi", ""))
        for no, tur, ms in bulgular:
            if (s, tur) not in onay:
                gizli.append((y, no, tur, ms, s))
    if gizli:
        uy("Push edilmemis commit'lerde sifre/token izi olabilir (repo PUBLIC!):")
        gizli_goster(gizli)
        yaz("      Gercekse commit gecmisinden cikarilmasi gerekir — gondermeden once bana yaz.")
        if kontrol:
            bi("gonderimde sorulacak (varsayilan: dur)")
            return
        if not sor("Bunlar gercek sifre/token DEGIL — devam edilsin mi?", False):
            raise Dur("durduruldu — hicbir sey degismedi")
        BEKLEYEN_ONAY.extend((sha, tur) for _, _, tur, _, sha in gizli)


# ---------------------------------------------------------------------------- 1) tarama
@dataclass
class IcRepo:
    rel: str
    gitler: list = field(default_factory=list)          # bulunan tum .git'ler
    tasinacak: list = field(default_factory=list)       # icerik girerse yedege tasinacak .git'ler
    dosyalar: list = field(default_factory=list)        # (rel, boyut, link) ana repoya girebilecekler
    okunamayan: list = field(default_factory=list)
    ozel_yoksay: list = field(default_factory=list)     # paketin kendi git'inin yok saydiklari
    izlenen_yoksay: list = field(default_factory=list)  # (rel, boyut) paketin izledigi ama ana .gitignore'a takilan
    lfs_attr: list = field(default_factory=list)        # filter=lfs iceren .gitattributes (girmez)
    venvler: list = field(default_factory=list)
    notlar: list = field(default_factory=list)
    izin_sorunu: str = ""
    sahnede: bool = False                               # elle 'git add' ile baglanti olarak sahnelenmis
    lfs: bool = False
    remote: str = "-"
    head: str = "-"
    engel: list = field(default_factory=list)
    veri: list = field(default_factory=list)
    varsayilan: bool = True

    def toplam(self) -> int:
        return sum(b for _, b, _ in self.dosyalar)


@dataclass
class Tarama:
    yeni: list = field(default_factory=list)            # (rel, boyut, link)
    degisen: list = field(default_factory=list)
    silinen: list = field(default_factory=list)         # rel (git rm --cached ile sahnelenir)
    sahne_silme: list = field(default_factory=list)     # elle 'git rm --cached': git'ten cikar, diskte kalir
    sahne_gitlink: list = field(default_factory=list)   # elle sahnelenmis ic ice repo baglantilari
    dosya_yerine_repo: list = field(default_factory=list)  # izlenen dosya silinip yerine repo klonlanmis
    head_gitlink: set = field(default_factory=set)      # HEAD'de zaten olan baglantilar (dokunulmaz)
    link_degisim: dict = field(default_factory=dict)    # klasor -> link olmus: {klasor: [izlenen yollar]}
    ic: list = field(default_factory=list)              # IcRepo
    okunamayan: list = field(default_factory=list)      # yeni ama okunamayan dosya
    okunamayan_dizin: list = field(default_factory=list)
    okunamayan_izlenen: list = field(default_factory=list)
    head_dizinler: set = field(default_factory=set)
    yoksayilan_model: list = field(default_factory=list)  # (rel, boyut)
    uretilmis: list = field(default_factory=list)         # (tur, rel, boyut, zaman)
    venvler: list = field(default_factory=list)
    gizli_kod: list = field(default_factory=list)         # disarida tutulan veri klasorlerinde kod
    onceden_disarida: list = field(default_factory=list)  # onceki kararlarla (exclude blogu) disarida kalanlar


def bilgi_ekle(t: Tarama, yol: str, yeni: bool) -> None:
    try:
        st = os.lstat(REPO / yol)
    except OSError as e:
        if e.errno in (errno.EACCES, errno.EPERM):
            (t.okunamayan if yeni else t.okunamayan_izlenen).append(yol)
        elif not yeni:
            t.silinen.append(yol)
        return
    if stat.S_ISLNK(st.st_mode):
        kayit = (yol, 0, True)
    else:
        if not os.access(REPO / yol, os.R_OK):
            (t.okunamayan if yeni else t.okunamayan_izlenen).append(yol)
            return
        kayit = (yol, st.st_size, False)
    (t.yeni if yeni else t.degisen).append(kayit)


def durum_oku(t: Tarama) -> list:
    rc, out, err = calistir(["git", "-C", str(REPO), "-c", "core.quotePath=false", "status", "--porcelain=v1",
                             "-z", "--untracked-files=all", "--no-renames", "--ignore-submodules=none"],
                            timeout=None)
    if rc != 0:
        raise Dur("git status: " + err.decode("utf-8", "replace").strip()[-600:])
    for satir in err.decode("utf-8", "surrogateescape").splitlines():
        m = re.search(r"could not open directory '(.+?)'", satir)
        if m:
            t.okunamayan_dizin.append(m.group(1).rstrip("/"))
            continue
        m = re.match(r"^(?:error: |warning: )?(?:lstat '|unable to stat ')?(.+?)'?: "
                     r"(?:Permission denied|Operation not permitted)$", satir)
        if m:                            # izlenen dosyaya bakilamadi: status onu HIC listelemez
            t.okunamayan_izlenen.append(m.group(1))
    parca = out.split(b"\0")
    ic_dizin = []
    i = 0
    while i < len(parca):
        e = parca[i]
        i += 1
        if len(e) < 4:
            continue
        xy = e[:2].decode("ascii", "replace")
        yol = os.fsdecode(e[3:])
        if xy[0] in "RC" or xy[1] in "RC":
            i += 1                       # -z: kaynak yol ayri girdi (--no-renames ile gelmemeli)
        if "U" in xy or xy in ("AA", "DD"):
            raise Dur(f"cozulmemis cakisma var: {goster(yol)} — once onu coz (git status)")
        if xy == "!!":
            continue
        if yol.rstrip("/") in t.head_gitlink:
            continue                     # HEAD'de zaten baglanti (submodule): dokunulmaz, tarada uyarilir
        if xy == "??":
            if yol.endswith("/"):
                ic_dizin.append(yol.rstrip("/"))
            else:
                bilgi_ekle(t, yol, yeni=True)
            continue
        p = REPO / yol
        if os.path.isdir(p) and not os.path.islink(p) and os.path.lexists(p / ".git"):
            if "A" in xy:
                t.sahne_gitlink.append(yol)  # elle 'git add pkg': ic ice repo baglantisi sahnede
            else:
                t.dosya_yerine_repo.append(yol)   # izlenen dosyanin yerinde artik klonlanmis bir repo var
                t.silinen.append(yol)             # eski dosya git'ten cikar; klasor ic ice repo olarak sorulur
            ic_dizin.append(yol)
            continue
        ul = ust_link(yol)
        if ul:                           # izlenen klasor sembolik linke donusmus
            t.link_degisim.setdefault(ul, []).append(yol)
            continue
        yok = yok_mu(p)
        if yok is None:                  # klasor izni yuzunden bakilamiyor: silinmis SAYILMAZ
            t.okunamayan_izlenen.append(yol)
            continue
        if xy[0] == "D":
            if yok:
                t.silinen.append(yol)
            else:
                t.sahne_silme.append(yol)   # 'git rm --cached': git'ten cikar, diskte kalir
            continue
        if yok:
            t.silinen.append(yol)
        elif "A" in xy:                  # HEAD'de yok (sahnelenmis ya da git add -N)
            bilgi_ekle(t, yol, yeni=True)
        else:
            bilgi_ekle(t, yol, yeni=False)
    if t.sahne_silme:                    # '??' ikizleri: kullanici bunlari git'ten cikariyor
        ss = set(t.sahne_silme)
        t.yeni = [x for x in t.yeni if x[0] not in ss]
    return list(dict.fromkeys(ic_dizin))     # ayni klasor iki girdiyle gelebilir (' D dosya' + '?? dosya/')


def check_ignore(yollar: list) -> set:
    if not yollar:
        return set()
    girdi = b"\0".join(os.fsencode(x) for x in yollar) + b"\0"
    out = git("check-ignore", "--stdin", "-z", "--no-index", girdi=girdi, izin=(0, 1))
    return {os.fsdecode(x) for x in out.split(b"\0") if x}


def filtre_attr(yollar: list) -> dict:
    """{yol: filtre} — .gitattributes 'filter=' tanimli dosyalar (lfs, git-crypt ...)."""
    if not yollar:
        return {}
    girdi = b"\0".join(os.fsencode(x) for x in yollar) + b"\0"
    out = git("check-attr", "-z", "--stdin", "filter", girdi=girdi)
    p = out.split(b"\0")
    sonuc = {}
    for i in range(0, len(p) - 2, 3):
        deger = p[i + 2].decode("utf-8", "replace")
        if deger not in ("unspecified", "unset", ""):
            sonuc[os.fsdecode(p[i])] = deger
    return sonuc


def ic_repo_tara(rel: str) -> IcRepo:
    kok = REPO / rel
    r = IcRepo(rel=rel)
    tum, hatalar = [], []
    for dp, dns, fns in dolas(kok, onerror=lambda e: hatalar.append(getattr(e, "filename", "?"))):
        p = Path(dp)
        if ".git" in dns:
            r.gitler.append(os.path.relpath(p / ".git", REPO))
            dns.remove(".git")
        for d in list(dns):
            if os.path.islink(p / d):
                tum.append(os.path.relpath(p / d, REPO))
                dns.remove(d)
        for f in fns:
            q = os.path.relpath(p / f, REPO)
            if f == ".git":                           # gitfile (worktree / submodule)
                r.gitler.append(q)
            else:
                tum.append(q)
    yoksay = check_ignore(tum)
    r.venvler = sorted({os.path.dirname(x) for x in tum if os.path.basename(x) == "pyvenv.cfg"})
    for x in tum:
        if x in yoksay:
            continue
        try:
            st = os.lstat(REPO / x)
        except OSError as e:
            if e.errno in (errno.EACCES, errno.EPERM):
                r.okunamayan.append(x)
            continue
        if stat.S_ISLNK(st.st_mode):
            r.dosyalar.append((x, 0, True))
        elif os.access(REPO / x, os.R_OK):
            r.dosyalar.append((x, st.st_size, False))
        else:
            r.okunamayan.append(x)
    # .git sadece altinda repoya girecek dosya varsa tasinir (or. build/_deps/*/.git, venv/src/*/.git kalir)
    ataler = set()
    for x, _, _ in r.dosyalar:
        d = os.path.dirname(x)
        while d and d not in ataler:
            ataler.add(d)
            d = os.path.dirname(d)
    r.tasinacak = [g for g in r.gitler if os.path.dirname(g) in ataler]
    kokler = [os.path.dirname(g) for g in r.tasinacak]
    girecek = {x for x, _, _ in r.dosyalar}

    for kk in kokler:                     # paketin kendi git'i ne diyor?
        derin = [d for d in kokler if d != kk and altinda(d, kk)]

        def sahibi(x, kk=kk, derin=derin):
            return altinda(x, kk) and not any(altinda(x, d) for d in derin)

        rc1, o1, _ = calistir(["git", "-C", str(REPO / kk), *ALT_GUVENLI, "ls-files", "-z", "-co",
                               "--exclude-standard"], timeout=300)
        rc2, o2, _ = calistir(["git", "-C", str(REPO / kk), *ALT_GUVENLI, "ls-files", "-z", "-c"], timeout=300)
        if rc1 or rc2:
            r.notlar.append(f"{goster(kk)}: paketin git'i okunamadi — karsilastirma yapilmadi")
            continue
        on = kk + "/"
        gorunen = {on + os.fsdecode(x) for x in o1.split(b"\0") if x and not x.endswith(b"/")}
        izlenen = {on + os.fsdecode(x) for x in o2.split(b"\0") if x}
        r.ozel_yoksay += [x for x in sorted(girecek) if sahibi(x) and x not in gorunen]
        for x in sorted(izlenen):
            if x in girecek or not sahibi(x) or x in r.okunamayan:
                continue
            try:
                st = os.lstat(REPO / x)
            except OSError as e:
                if e.errno in (errno.EACCES, errno.EPERM):
                    hatalar.append(str(REPO / x))     # paketin izledigi dosyaya erisilemiyor
                continue                              # (ENOENT) paket izliyor ama diskte yok
            if stat.S_ISLNK(st.st_mode):
                r.izlenen_yoksay.append((x, 0))
            elif stat.S_ISREG(st.st_mode) and os.access(REPO / x, os.R_OK):
                r.izlenen_yoksay.append((x, st.st_size))
    for x in tum:
        if os.path.basename(x) == ".gitattributes" and x not in yoksay:
            try:
                if b"filter=lfs" in (REPO / x).read_bytes():
                    r.lfs = True
                    r.lfs_attr.append(x)
            except OSError:
                pass
    filtreler = filtre_attr([x for x, _, _ in r.dosyalar])
    baska = sorted({v for v in filtreler.values() if v != "lfs"})
    if baska:
        r.izin_sorunu = (f"paket .gitattributes 'filter={','.join(baska)}' kullaniyor (git-crypt gibi sifreleme olabilir) "
                         "— dosyalar sifresiz gidebilir, bu paket ATLANDI. Bana yaz.")
    if hatalar:
        ornek = ", ".join(goster(os.path.relpath(str(h), REPO)) if os.path.isabs(str(h)) else goster(str(h))
                          for h in hatalar[:3])
        r.izin_sorunu = (f"pakette okunamayan klasor/dosya var ({ornek}) — eksik gider ve gecmisi yedege "
                         f"tasinir; ATLANDI. Duzeltmek icin: sudo chown -R {KULLANICI}:{KULLANICI} {goster(rel)}")
    for g in r.tasinacak:
        P = REPO / g
        yazilabilir = os.access(P.parent, os.W_OK | os.X_OK)
        if os.path.isdir(P) and not os.path.islink(P):
            yazilabilir = yazilabilir and os.access(P, os.W_OK)      # tasimak icin '..' guncellenir
        if not yazilabilir:
            r.izin_sorunu = (f"{goster(g)} tasinamaz (yazma izni yok). Duzeltmek icin: "
                             f"sudo chown -R {KULLANICI}:{KULLANICI} {goster(r.rel)}  — sonra tekrar calistir")
            break

    def alt(*a):
        rc, o, _ = calistir(["git", "-C", str(kok), *ALT_GUVENLI, *a], timeout=120)
        return o.decode("utf-8", "replace").strip() if rc == 0 else ""
    r.remote = temiz_url(alt("remote", "get-url", "origin")) or "(remote yok)"
    r.head = alt("--no-pager", "log", "-1", "--no-show-signature", "--format=%h %cs %s")[:70] or "(commit yok)"
    return r


def model_mi(ad: str) -> bool:
    a = ad.lower()
    if uzanti(a) in MODEL_UZ or a.endswith((".pth.tar", ".ckpt.tar")):
        return True
    return uzanti(a) == ".bin" and re.search(r"model|weight|ckpt|ortrack|yolo|track", a) is not None


def exclude_oku(veri: bytes):
    """(diger_satirlar, yonetilen: [(desen, neden)]) — v1 bloklarini da tanir."""
    diger, yonetilen = [], []
    icinde, neden = False, ""
    for s in os.fsdecode(veri).split("\n"):
        if s == EXC_BAS or re.match(r"^# --- gite_gonder\.py .*---$", s):
            icinde, neden = True, ""
            continue
        if icinde and (s == EXC_SON or s == "# --- son ---"):
            icinde = False
            continue
        if icinde:
            if s.startswith("# [") and s.endswith("]"):
                neden = s[3:-1]
            elif s.strip() and not s.startswith("#"):
                yonetilen.append((s, neden))
            continue
        diger.append(s)
    while diger and diger[-1] == "":
        diger.pop()
    return diger, yonetilen


def desen_coz(d: str):
    """haric_desen'in tersi: '/a/b\\ c/' -> ('a/b c', True)"""
    dizin = d.endswith("/") and not d.endswith("\\/")
    if dizin:
        d = d[:-1]
    if d.startswith("/"):
        d = d[1:]
    return re.sub(r"\\(.)", r"\1", d), dizin


def haric_desen(rel: str, dizin: bool) -> str:
    s = rel.replace("\\", "\\\\")
    for ch in "*?[":
        s = s.replace(ch, "\\" + ch)
    if s.endswith(" "):
        s = s[:-1] + "\\ "
    return "/" + s + ("/" if dizin else "")


def yonetilen_disarida() -> list:
    """Onceki calismalarin disarida biraktigi yollar: [(rel, dizin_mi, neden)]"""
    try:
        veri = git_yolu("info/exclude").read_bytes()
    except OSError:
        return []
    return [(*desen_coz(d), n) for d, n in exclude_oku(veri)[1]]


def yoksayilan_tara(t: Tarama, onceki: list) -> None:
    """.gitignore'a takilan dosyalar: model (sorulur), engine / arsiv / venv (bilgi)."""
    yollar = git_z("-c", "core.quotePath=false", "ls-files", "-z", "-o", "-i", "--exclude-standard")
    t.venvler += sorted({os.path.dirname(y) for y in yollar if os.path.basename(y) == "pyvenv.cfg"})

    def venvde(y):
        return "/site-packages/" in y or "/dist-packages/" in y or any(altinda(y, v) for v in t.venvler)

    def onceden(y):
        return any((altinda(y, r) if d else y == r) for r, d, _ in onceki)
    for y in yollar:
        if venvde(y) or onceden(y):
            continue
        ad = os.path.basename(y)
        try:
            st = os.lstat(REPO / y)
        except OSError:
            continue
        if not stat.S_ISREG(st.st_mode):
            continue
        z = datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M")
        uz = uzanti(y)
        if uz in URETILMIS_UZ:
            t.uretilmis.append(("engine", y, st.st_size, z))
        elif model_mi(ad):
            kok = y[:-len(uz)] if uz else y
            if uz == ".onnx" and any(os.path.exists(REPO / (kok + e)) for e in (".pt", ".pth")):
                t.uretilmis.append(("onnx", y, st.st_size, z))     # .pt'den yeniden uretilir
            else:
                t.yoksayilan_model.append((y, st.st_size))
        elif ARSIV_RX.search(ad):
            t.uretilmis.append(("arsiv", y, st.st_size, z))


def veri_klasoru_kodu(onceki: list) -> list:
    """Disarida tutulan VERI klasorlerine sonradan konmus kod/model dosyalari (GitHub'a gitmiyorlar)."""
    sonuc = []
    for rel, dizin, n in onceki:
        if not dizin or n != "veri klasoru":
            continue
        say = 0
        for dp, dns, fns in dolas(REPO / rel, onerror=lambda e: None):
            for fn in fns:
                say += 1
                if uzanti(fn) in UYARI_UZ:
                    sonuc.append(os.path.relpath(os.path.join(dp, fn), REPO))
            if say > 20000 or len(sonuc) > 50:
                break
    return sonuc


def veri_sec(adaylar: list) -> list:
    sayac = {}
    for rel, b, l in adaylar:
        if not l and uzanti(rel) in VERI_UZ:
            d = os.path.dirname(rel)
            sayac[d] = sayac.get(d, 0) + 1
    secilen = []
    for rel, b, l in adaylar:
        if l or b > SINIR_ENGEL:
            continue
        uz = uzanti(rel)
        if uz in VIDEO_UZ or ARSIV_RX.search(rel) or uz in SIKISTIRMA_UZ:
            secilen.append((rel, b))            # arsivin ici taranamaz; repoya girmesi de istenmez (kopya kod)
        elif uz in VERI_UZ:
            if b > 512 * 1024 or sayac.get(os.path.dirname(rel), 0) >= 3:
                secilen.append((rel, b))
        elif uz == ".txt" and b > 2 * MB:
            secilen.append((rel, b))
        elif uz == ".json" and b > 5 * MB:
            secilen.append((rel, b))
        elif b > 20 * MB and uz not in MODEL_UZ and uz not in KOD_UZ:
            secilen.append((rel, b))
    return secilen


def tara() -> Tarama:
    bas("1) Tarama")
    t = Tarama()
    for kayit in git("ls-tree", "-r", "-z", "HEAD").split(b"\0"):
        if kayit.startswith(b"160000 "):
            t.head_gitlink.add(os.fsdecode(kayit.split(b"\t", 1)[1]))
    onceki = yonetilen_disarida()
    ic_dizin = durum_oku(t)

    # Onceki kararlar (exclude'daki yonetilen blok) USTUNDUR: .gitignore'daki '!' istisnasi (or.
    # !hunter-orin/models/*.onnx) exclude'u gecersiz kilsa bile o yollar yine disarida kalir.
    def onceden(y):
        return any((altinda(y, r) if d else y == r) for r, d, _ in onceki)
    t.onceden_disarida = [x[0] for x in t.yeni if onceden(x[0])] + [d for d in ic_dizin if onceden(d)]
    t.yeni = [x for x in t.yeni if not onceden(x[0])]
    t.okunamayan = [x for x in t.okunamayan if not onceden(x)]
    t.okunamayan_dizin = [x for x in t.okunamayan_dizin if not onceden(x)]
    ic_dizin = [d for d in ic_dizin if not onceden(d)]
    t.head_dizinler = set(git_z("ls-tree", "-z", "-d", "-r", "--name-only", "HEAD"))
    for rel in ic_dizin:
        r = ic_repo_tara(rel)
        r.sahnede = rel in t.sahne_gitlink
        r.engel = [(x, b) for x, b, l in r.dosyalar if not l and b > SINIR_ENGEL]
        r.veri = veri_sec(r.dosyalar)
        kalan = r.toplam() - sum(b for _, b in r.engel) - sum(b for _, b in r.veri)
        r.varsayilan = bool(r.dosyalar) and kalan <= 100 * MB and len(r.dosyalar) <= 3000
        t.venvler += r.venvler
        t.ic.append(r)
    yoksayilan_tara(t, onceki)
    t.gizli_kod = veri_klasoru_kodu(onceki)
    if t.okunamayan_izlenen:
        for x in t.okunamayan_izlenen[:20]:
            ha(f"git'te izlenen ama okunamayan dosya: {goster(x)}")
        raise Dur(f"izlenen dosyalar okunamiyor (izin). Duzelt: sudo chown -R {KULLANICI}:{KULLANICI} <yol> "
                  "— sonra tekrar calistir. Hicbir sey degismedi.")
    buyuk = [(x, b) for x, b, l in t.degisen if not l and b > SINIR_ENGEL]
    if buyuk:
        for x, b in buyuk:
            ha(f"git'te izlenen dosya {mb(b)} oldu: {goster(x)}")
        raise Dur("izlenen bir dosya 99 MB'i gecti — GitHub kabul etmez. Hicbir sey degismedi; bana yaz.")
    toplam = sum(b for _, b, _ in t.yeni) + sum(b for _, b, _ in t.degisen)
    bi(f"yeni {len(t.yeni)}  degisen {len(t.degisen)}  silinen {len(t.silinen)}  "
       f"ic ice repo {len(t.ic)}  ({mb(toplam)}, ic ice repolar haric)")
    if t.onceden_disarida:
        bi(f"onceki calismalarda disarida birakilan {len(t.onceden_disarida)} yol yine disarida (.git/info/exclude)")
    for g in sorted(t.head_gitlink):
        uy(f"repoda zaten ic ice repo baglantisi (submodule) var: {goster(g)} — dokunulmadi")
    if t.gizli_kod:
        uy("Disarida tutulan veri klasorlerinde kod/model dosyasi var (GitHub'a GITMIYOR):")
        for x in t.gizli_kod[:15]:
            yaz(f"      {goster(x)}")
        yaz("      Almak istersen .git/info/exclude'da o klasorun satirini sil, sonra tekrar calistir.")
    return t


# ---------------------------------------------------------------------------- 2) kararlar
@dataclass
class Kararlar:
    haric: list = field(default_factory=list)       # (rel, dizin_mi, neden) -> .git/info/exclude
    atla: list = field(default_factory=list)        # (rel, neden) -> bu sefer sahnelenmez, exclude'a yazilmaz
    notlar: list = field(default_factory=list)      # (rel, aciklama) -> rapor
    tasinacak_git: list = field(default_factory=list)
    dahil_ic: list = field(default_factory=list)
    zorla: list = field(default_factory=list)       # (rel, boyut) -> git add -f
    link_kabul: list = field(default_factory=list)  # klasor -> link degisimi kabul edilenler
    link_silinen: list = field(default_factory=list)
    gizli_onay: list = field(default_factory=list)
    modeller: list = field(default_factory=list)
    yeni_son: list = field(default_factory=list)    # (rel, boyut, link)
    uretilen_eski: list = field(default_factory=list)
    elle_sistem: list = field(default_factory=list)
    lfs_isaretci: list = field(default_factory=list)
    etiket: str = ""
    _hd: set = field(default_factory=set)
    _hk: tuple = ()

    def filtre(self) -> None:
        self._hd = {r for r, d, _ in self.haric if not d} | {r for r, _ in self.atla}
        self._hk = tuple(r.rstrip("/") + "/" for r, d, _ in self.haric if d) + \
            tuple(r.rstrip("/") + "/" for r, _ in self.atla)

    def disarida(self, rel: str) -> bool:
        return rel in self._hd or (bool(self._hk) and (rel + "/").startswith(self._hk))


def uretilen_mi(rel: str) -> bool:
    return rel in {f"{REF_REL}/{a}" for a in URETILEN + (RAPOR_ADI,)} or altinda(rel, SISTEM_REL)


def grupla(dosyalar: list) -> list:
    """(rel, boyut) -> ekran satirlari; ayni klasorde >=3 ise tek satir."""
    g = {}
    for rel, b in dosyalar:
        g.setdefault(os.path.dirname(rel), []).append((rel, b))
    satirlar = []
    for d in sorted(g):
        lst = g[d]
        if len(lst) >= 3:
            satirlar.append(f"{goster(d or '.') + '/':<52} {len(lst):5d} dosya {mb(sum(b for _, b in lst)):>10}")
        else:
            satirlar += [f"{goster(r):<58} {mb(b):>10}" for r, b in lst]
    return satirlar


def lfs_isaretci_bul(liste: list) -> list:
    sonuc = []
    for rel, b, l in liste:
        if l or not 0 < b <= 1024:
            continue
        try:
            with open(REPO / rel, "rb") as f:
                if f.read(len(LFS_ISARET)) == LFS_ISARET:
                    sonuc.append(rel)
        except OSError:
            pass
    return sonuc


def model_listesi(liste: list) -> list:
    sonuc = []
    for rel, b, l in liste:
        if not l and model_mi(os.path.basename(rel)) and b <= SINIR_ENGEL:
            try:
                sonuc.append((rel, b, sha256_dosya(REPO / rel)))
            except OSError:
                pass
    return sonuc


def etiket_gecerli(istenen: str) -> None:
    if istenen and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,60}", istenen):
        raise Dur(f"gecersiz etiket adi: {istenen} (harf/rakamla baslasin; sadece A-Z a-z 0-9 . _ -)")
    if istenen and calistir(["git", "check-ref-format", f"refs/tags/{istenen}"])[0] != 0:
        raise Dur(f"gecersiz etiket adi: {istenen}")


def etiket_sec(istenen: str) -> str:
    etiket_gecerli(istenen)
    taban = istenen or "orin-" + ZAMAN.strftime("%Y-%m-%d")
    yerel = set(git_s("tag", "-l").split())
    uzak = uzak_etiketler()
    if uzak is None:
        uy("GitHub'daki etiketler okunamadi — sadece yerel etiketlere bakildi")
        uzak = set()
    ad, i = taban, 2
    while ad in yerel or ad in uzak:
        ad = f"{taban}-{i}"
        i += 1
    return ad


def kararlar(t: Tarama, kontrol: bool, istenen_etiket: str) -> Kararlar:
    k = Kararlar()
    bas("2) Kararlar" + ("  (--kontrol: varsayilanlar gosteriliyor, soru yok)" if kontrol else ""))

    # setup/referans_ortam kayitlari bu calismada yeniden uretilir (eskileri yedege)
    yeni_set = {x for x, _, _ in t.yeni}
    for liste in (t.yeni, t.degisen):
        k.uretilen_eski += [x for x, _, _ in liste if uretilen_mi(x)]
        liste[:] = [x for x in liste if not uretilen_mi(x[0])]
    k.uretilen_eski += [x for x in t.silinen if uretilen_mi(x)]
    t.silinen[:] = [x for x in t.silinen if not uretilen_mi(x)]
    k.elle_sistem = [x for x in k.uretilen_eski if altinda(x, SISTEM_REL) and x in yeni_set]
    if k.elle_sistem:
        uy(f"{SISTEM_REL}/ altinda elle eklenmis ya da yarim kalmis bir calismadan kalan {len(k.elle_sistem)} "
           "dosya var: klasor yeniden uretilecek, bunlar yedege tasinacak:")
        for x in k.elle_sistem[:10]:
            yaz(f"      {goster(x)}")

    # --- elle 'git rm --cached' yapilmislar
    if t.sahne_silme:
        yaz()
        bi(f"elle git'ten cikarilmis (git rm --cached) {len(t.sahne_silme)} dosya: GitHub'dan da cikar, diskte kalir")
        for x in t.sahne_silme[:10]:
            yaz(f"      {goster(x)}")
        yoksay_ss = check_ignore(t.sahne_silme)
        k.haric += [(x, False, "elle git'ten cikarildi") for x in t.sahne_silme if x not in yoksay_ss]

    # --- okunamayan klasorler
    if t.okunamayan_dizin:
        yaz()
        uy("Okunamayan klasorler (izin) — icleri GITMEZ, yeni Orin'de olmaz:")
        for x in t.okunamayan_dizin[:15]:
            yaz(f"      {goster(x)}/")
        yaz(f"      Duzeltmek icin: sudo chown -R {KULLANICI}:{KULLANICI} <klasor>  (sonra tekrar calistir)")
        if kontrol:
            yaz("      Varsayilan: DUR")
        elif not sor("Bunlar olmadan devam edilsin mi?", False):
            raise Dur("durduruldu — hicbir sey degismedi. Izinleri duzeltip tekrar calistir.")

    # --- klasor -> sembolik link degisimi
    for d, izli in sorted(t.link_degisim.items()):
        try:
            hedef = os.readlink(REPO / d)
        except OSError:
            hedef = "?"
        yaz()
        uy(f"Izlenen klasor sembolik linke donusmus: {goster(d)} -> {goster(hedef)}")
        yaz(f"      Git'te bu klasordeki {len(izli)} dosya SILINMIS gorunur, yerine sadece link girer.")
        yaz("      Yeni Orin'de link hedefi yoksa bu dosyalar olmaz.")
        if kontrol:
            yaz("      Varsayilan: GITMESIN (git'teki dosyalar oldugu gibi kalir)")
        if not kontrol and sor("Bu degisiklik (klasor yerine link) gitsin mi?", False):
            k.link_kabul.append(d)
            k.link_silinen += izli
        else:
            k.atla.append((d, "klasor->link degisimi istenmedi"))

    # --- ic ice repolar
    for x in t.dosya_yerine_repo:
        yaz()
        uy(f"git'te DOSYA olan {goster(x)} silinmis, yerine klonlanmis bir repo (klasor) gelmis:")
        yaz("      eski dosya git'ten cikar; klasorun icerigi asagida ic ice repo olarak soruluyor.")
    for r in t.ic:
        yaz(f"\n  Ic ice repo: {goster(r.rel)}/  ({len(r.dosyalar)} dosya, {mb(r.toplam())})")
        yaz(f"      kaynak: {r.remote}")
        yaz(f"      son commit: {r.head}")
        if r.sahnede:
            yaz("      (elle 'git add' ile BAGLANTI olarak sahnelenmis — o baglanti sahneden cikarilacak)")
        for n in r.notlar:
            yaz(f"      not: {n}")
        if not r.dosyalar:
            yaz("      Icinde repoya girecek dosya yok (hepsi .gitignore'da) — dokunulmadi.")
            k.atla.append((r.rel, "ic ice repo: girecek dosya yok"))
            continue
        if r.izin_sorunu:
            uy(r.izin_sorunu)
            k.atla.append((r.rel, "ic ice repo: atlandi"))
            continue
        if r.lfs:
            uy("paket Git LFS kullaniyor — dosyalar GERCEK icerikle girer; paketin LFS .gitattributes'u girmez")
        if len(r.tasinacak) > 1:
            yaz(f"      icinde {len(r.tasinacak)} .git var (hepsi tasinacak)")
        if len(r.gitler) > len(r.tasinacak):
            yaz(f"      {len(r.gitler) - len(r.tasinacak)} .git yok sayilan klasorlerde (build/venv) — dokunulmaz")
        if r.ozel_yoksay:
            yaz(f"      paketin kendi git'inin yok saydigi {len(r.ozel_yoksay)} dosya disarida kalir")
        yaz("      Boyle kalirsa GitHub'a sadece bos bir baglanti gider, icerigi GITMEZ.")
        yaz(f"      Icerik girsin derseniz .git klasoru yedege tasinir: {YEDEK}/")
        if kontrol:
            yaz(f"      Varsayilan: {'icerik GIRSIN' if r.varsayilan else 'GIRMESIN (buyuk)'}")
        evet = r.varsayilan if kontrol else sor(f"'{goster(r.rel)}' icerigi repoya girsin mi?", r.varsayilan)
        if not evet:
            k.haric.append((r.rel, True, "ic ice repo"))
            continue
        k.tasinacak_git += r.tasinacak
        k.dahil_ic.append(r)
        k.haric += [(x, False, "paket yok sayiyor") for x in r.ozel_yoksay]
        k.haric += [(x, False, "LFS .gitattributes") for x in r.lfs_attr]
        k.atla += [(x, "okunamiyor") for x in r.okunamayan]
        if r.izlenen_yoksay:
            veri_iy = veri_sec([(x, b, False) for x, b in r.izlenen_yoksay])
            toplam_iy = sum(b for _, b in r.izlenen_yoksay)
            vars_iy = not veri_iy and toplam_iy <= 50 * MB
            yaz(f"      Paketin kendi git'inde olup ana .gitignore'a takilan {len(r.izlenen_yoksay)} dosya "
                f"({mb(toplam_iy)}):")
            for s in grupla(r.izlenen_yoksay)[:15]:
                yaz("        " + s)
            yaz("      (paketin yazari bunlari paketin parcasi saymis; girmezse yeni Orin'de olmaz)")
            if veri_iy:
                yaz(f"      Icinde veri/video gibi gorunen {len(veri_iy)} dosya var.")
            if kontrol:
                yaz(f"      Varsayilan: {'GIRSIN' if vars_iy else 'GIRMESIN (veri/buyuk)'}")
            if (vars_iy if kontrol else sor("Bunlar da girsin mi?", vars_iy)):
                for x, b in r.izlenen_yoksay:
                    if b > SINIR_ENGEL:
                        k.notlar.append((x, f">99 MB ({mb(b)}) — GitHub'a giremez, Drive'a yedekle"))
                    else:
                        k.zorla.append((x, b))

    aday = list(t.yeni)
    for r in k.dahil_ic:
        oz = set(r.ozel_yoksay) | set(r.lfs_attr)
        aday += [d for d in r.dosyalar if d[0] not in oz]

    # --- >99 MB
    engel = [(x, b) for x, b, l in aday if not l and b > SINIR_ENGEL]
    if engel:
        yaz()
        uy("GitHub 100 MB ustunu kabul etmez — bunlar repoya GIRMEYECEK (bu Orin'de kalir, Drive'a yedekle):")
        for x, b in engel:
            yaz(f"      {goster(x):<58} {mb(b):>10}")
        k.haric += [(x, False, ">99 MB") for x, _ in engel]

    # --- okunamayan dosyalar
    if t.okunamayan:
        yaz()
        uy("Okunamayan dosyalar (izin) — bu sefer girmez:")
        for x in t.okunamayan[:15]:
            yaz(f"      {goster(x)}")
        yaz(f"      Duzeltmek icin: sudo chown {KULLANICI}:{KULLANICI} <dosya>  (sonra tekrar calistir)")
        k.atla += [(x, "okunamiyor") for x in t.okunamayan]

    # --- veri
    engel_set = {x for x, _ in engel}
    veri = veri_sec([a for a in aday if a[0] not in engel_set])
    if veri:
        yaz()
        uy("Veri gibi gorunen dosyalar (kayit, kare, log ciktisi, arsiv...):")
        for s in grupla(veri):
            yaz("      " + s)
        if kontrol:
            yaz("      Varsayilan: repoya GIRMESIN (sadece bu Orin'de kalir)")
        if kontrol or not sor("Bunlar repoya girsin mi?", False):
            veri_set = {r for r, _ in veri}
            aday_yollar = [a[0] for a in aday]
            izli = [x for x, _, _ in t.degisen] + list(t.silinen)
            klasorler = {}
            for r_, _ in veri:
                klasorler.setdefault(os.path.dirname(r_), []).append(r_)
            for d, lst in klasorler.items():
                # klasor toptan sadece: git'te hic yoksa, altinda izlenen dosya yoksa, icindeki her sey veriyse
                toptan = (d and len(lst) >= 20 and d not in t.head_dizinler
                          and not any(altinda(y, d) for y in izli)
                          and all(y in veri_set for y in aday_yollar if altinda(y, d)))
                if toptan:
                    k.haric.append((d, True, "veri klasoru"))
                else:
                    k.haric += [(y, False, "veri") for y in lst]

    # --- .gitignore'a takilan modeller
    if t.yoksayilan_model:
        yaz()
        uy(".gitignore yuzunden repoya GIRMEYEN model dosyalari (yeni Orin'de olmazlar):")
        for s in grupla(t.yoksayilan_model)[:20]:
            yaz("      " + s)
        yaz("      (.pt'den uretilebilen .onnx'ler bu listede yok. Paketle gelen TEK kopyaysa 'e' de.)")
        if kontrol:
            yaz("      Varsayilan: GIRMESIN")
        if not kontrol and sor("Bunlar da girsin mi?", False):
            for x, b in t.yoksayilan_model:
                if b > SINIR_ENGEL:
                    k.notlar.append((x, f">99 MB ({mb(b)}) — GitHub'a giremez, Drive'a yedekle"))
                else:
                    k.zorla.append((x, b))
        elif not kontrol:
            k.haric += [(x, False, "model (istenmedi)") for x, _ in t.yoksayilan_model]

    k.filtre()
    k.zorla = [z for z in k.zorla if not k.disarida(z[0])]
    zorla_set = {x for x, _ in k.zorla}
    k.yeni_son = [a for a in aday if not k.disarida(a[0]) and a[0] not in zorla_set] + \
        [(x, b, False) for x, b in k.zorla]

    # --- filtre (git-crypt vb.) altindaki dosyalar: sifresiz gidebilir -> atla
    ozel = {y: f for y, f in filtre_attr([x for x, _, _ in k.yeni_son + t.degisen]).items() if f != "lfs"}
    if ozel:
        yaz()
        uy(".gitattributes 'filter=' altindaki dosyalar (git-crypt gibi sifreleme olabilir) — bu sefer GITMEZ:")
        for y, f in sorted(ozel.items())[:15]:
            yaz(f"      {goster(y)}  [filter={f}]")
        yaz("      Bana yaz; nasil gitmesi gerektigine birlikte bakalim.")
        k.atla += [(y, f"filter={f}") for y, f in ozel.items()]
        k.filtre()
        k.yeni_son = [a for a in k.yeni_son if not k.disarida(a[0])]
        k.zorla = [z for z in k.zorla if not k.disarida(z[0])]
        t.degisen[:] = [a for a in t.degisen if not k.disarida(a[0])]

    # --- LFS isaretcileri
    k.lfs_isaretci = lfs_isaretci_bul(k.yeni_son)
    if k.lfs_isaretci:
        yaz()
        uy("Bunlar gercek dosya degil, Git LFS isaretcisi (paket LFS'siz indirilmis) — girer ama ise yaramaz:")
        for x in k.lfs_isaretci[:10]:
            yaz(f"      {goster(x)}")
        yaz("      Gercek dosya icin pakette: git lfs pull")

    # --- sifre / token
    gizli = gizli_tara(k.yeni_son + t.degisen)
    if gizli:
        yaz()
        uy("Token / sifre izi olabilecek satirlar (repo PUBLIC!):")
        gizli_goster(gizli)
        yaz("      Gercek sifre/token ise: dosyadan cikar, sonra scripti tekrar calistir.")
        yaz("      ('gercek degil' dersen bu icerik hatirlanir; dosya degisirse tekrar sorulur.)")
        if kontrol:
            yaz("      Varsayilan: DUR")
        elif not sor("Bunlar gercek sifre/token DEGIL — devam edilsin mi?", False):
            raise Dur("durduruldu — hicbir sey degismedi. Dosyalari duzeltip tekrar calistir.")
        k.gizli_onay = gizli
        if not kontrol:
            BEKLEYEN_ONAY.extend((sha, tur) for _, _, tur, _, sha in gizli)

    k.modeller = model_listesi(k.yeni_son + t.degisen)
    k.etiket = etiket_sec(istenen_etiket)
    return k


# ---------------------------------------------------------------------------- 3) sistem envanteri
class _Iki(list):
    """Envanter satirlari: genel (repoya giden envanter.txt) + tam (sadece arsivdeki envanter_tam.txt)."""

    def __init__(self, tam: list):
        super().__init__()
        self.tam = tam

    def append(self, x) -> None:
        super().append(x)
        self.tam.append(x)

    def __iadd__(self, xs):
        xs = list(xs)
        super().__iadd__(xs)
        self.tam.extend(xs)
        return self


class Envanter:
    """Repo disinda yasayan ayarlarin kaydi. Sistemi sadece OKUR.
    liste/  -> repoya girer: envanter.txt (kisisel olmayan teknik ozet) + paket listeleri. Kopya YOK, kopyalarin
               sha256'si YOK (kisa sifreler sozlukle bulunabilir), ag adlari/IP/ev dizini ayrintisi YOK.
    kopya/  -> repoya GIRMEZ: dosya kopyalari + envanter_tam.txt (~/Desktop/avci_sistem_<tarih>.tar.gz olur)
    Her bolum ayri korunur: biri hata verirse digerleri devam eder."""

    def __init__(self, liste: Path, kopya: Path):
        self.h = liste
        self.k = kopya
        liste.mkdir(parents=True)
        kopya.mkdir(parents=True)
        self.T = [f"{ENV_ISARET} (TAM — sadece yerel arsivde) — {ZAMAN:%Y-%m-%d %H:%M} — {MAKINE} — {KULLANICI}"]
        self.E = _Iki(self.T)                 # E'ye yazilan T'ye de yazilir; sadece T'ye: self.T.append
        list.extend(self.E, [f"{ENV_ISARET} — {ZAMAN:%Y-%m-%d %H:%M} — {MAKINE} — {KULLANICI}",
                             "# gite_gonder.py uretir. Repo DISINDA yasayan ayarlarin teknik ozeti (adlar, surumler).",
                             "# Dosya kopyalari ve ayrintilar repoda degil: avci_sistem_<tarih>.tar.gz"])
        self.ozet, self.ozet_genel, self.kopya, self.atlanan, self.kirpilan, self.ham = [], [], [], [], [], []

    def oz(self, satir: str, genel: bool = True) -> None:
        """Ekrandaki ozet; genel=True ise repodaki rapora da girer."""
        self.ozet.append(satir)
        if genel:
            self.ozet_genel.append(satir)

    def bolum(self, ad: str) -> None:
        self.E += ["", f"## {ad}"]

    def yaz(self, rel: str, veri: bytes, calistirilabilir: bool = False, kaynak: str = "") -> bool:
        temiz, n, birak = temizle(veri, maske=True)
        if birak:
            self.atlanan.append((kaynak or rel, "ozel anahtar — kopyalanmadi"))
            return False
        if n:
            self.kirpilan.append((kaynak or rel, n))
        d = self.k / rel
        d.parent.mkdir(parents=True, exist_ok=True)
        d.write_bytes(temiz)
        os.chmod(d, 0o700 if calistirilabilir else 0o600)
        self.kopya.append((rel, hashlib.sha256(veri).hexdigest()))
        return True

    def liste_yaz(self, ad: str, veri: bytes) -> None:
        """Uretilen liste dosyasi (repoya girer) — o da sifre taramasindan gecer."""
        temiz, n, birak = temizle(veri)
        if birak:
            self.atlanan.append((ad, "ozel anahtar izi — yazilmadi"))
            return
        if n:
            self.kirpilan.append((ad, n))
        (self.h / ad).write_bytes(temiz)

    def kopyala(self, src: Path, rel: str) -> bool:
        try:
            st = src.lstat()
        except OSError:
            return False
        if stat.S_ISLNK(st.st_mode):
            try:
                self.E.append(f"  {goster(str(src))} -> {goster(os.readlink(src))}  (link)")
            except OSError:
                pass
            return False
        if not stat.S_ISREG(st.st_mode):
            return False
        if st.st_size > MB:
            self.atlanan.append((str(src), f"buyuk ({mb(st.st_size)})"))
            return False
        try:
            veri = src.read_bytes()
        except OSError as e:
            self.atlanan.append((str(src), f"okunamadi ({e.strerror})"))
            return False
        if b"\0" in veri[:8192]:
            self.atlanan.append((str(src), "ikili dosya"))
            return False
        self.ham.append(veri)
        return self.yaz(rel, veri, bool(st.st_mode & 0o111), str(src))

    def klasor(self, kaynak: Path, on: str, desen=None, derin: bool = False) -> int:
        n = 0
        try:
            if not kaynak.is_dir():
                return 0
        except OSError:
            return 0
        for dp, dns, fns in dolas(kaynak, onerror=lambda e: None):
            if not derin:
                dns[:] = []
            for f in sorted(fns):
                if desen and not re.search(desen, f):
                    continue
                p = Path(dp) / f
                if self.kopyala(p, f"{on}/{os.path.relpath(p, kaynak)}"):
                    n += 1
        return n

    def cikti(self, cmd, timeout: int = 60) -> str:
        rc, out, _ = calistir(cmd, timeout=timeout)
        return out.decode("utf-8", "replace") if rc == 0 else ""

    def uret(self) -> None:
        for fn in (self.kimlik, self.ag, self.usr_local_bin, self.servisler, self.kabuk, self.ev_scriptleri,
                   self.repo_disi, self.indirilenler, self.venvler, self.apt, self.docker_ssh, self.elle_kurulum,
                   self.jetson_clocks):
            try:
                fn()
            except Exception as e:                           # bir bolum bozuksa digerleri devam
                self.E.append(f"  (bu bolumde hata: {type(e).__name__}: {str(e)[:150]})")
        self.bitir()

    # --- bolumler
    def kimlik(self) -> None:
        self.bolum("Kimlik")
        self.E.append(f"  hostname: {MAKINE}")
        self.E.append(f"  kullanici: {KULLANICI}  gruplar: {self.cikti(['id', '-nG']).strip()}")
        for f in ("/etc/hostname", "/etc/hosts", "/etc/timezone", "/etc/fstab", "/etc/environment", "/etc/rc.local"):
            self.kopyala(Path(f), "etc/" + Path(f).name)

    def ag(self) -> None:
        self.bolum("Ag arayuzleri (MAC yazilmaz)")
        self.T.extend("  " + s for s in self.cikti(["ip", "-br", "addr"]).splitlines())
        prof = self.cikti(["nmcli", "-t", "-f", "NAME,TYPE,DEVICE", "con", "show"])
        if prof:
            self.T.append("  NetworkManager profilleri (ad:tur:cihaz):")
            self.T.extend("    " + s for s in prof.splitlines())
            self.E.append(f"  NetworkManager profili: {len(prof.splitlines())} (adlar ve IP'ler arsivde)")
        ethtool = shutil.which("ethtool") or next((x for x in ("/usr/sbin/ethtool", "/sbin/ethtool")
                                                    if os.path.exists(x)), None)
        eee = []
        try:
            arayuzler = sorted(os.listdir("/sys/class/net"))
        except OSError:
            arayuzler = []
        for n in arayuzler:
            s = Path("/sys/class/net") / n
            try:
                if not (s / "device").exists() or (s / "wireless").exists():
                    continue
                if (s / "type").read_text().strip() != "1":
                    continue
            except OSError:
                continue
            try:
                surucu = os.path.basename(os.readlink(s / "device" / "driver"))
            except OSError:
                surucu = "?"
            try:
                hiz = (s / "speed").read_text().strip()
            except OSError:
                hiz = "?"
            durum = "?"
            if ethtool:
                for cmd in ([ethtool, "--show-eee", n], ["sudo", "-n", ethtool, "--show-eee", n]):
                    _, out, _ = calistir(cmd, timeout=10)
                    m = re.search(rb"EEE status:\s*([^\n]+)", out)
                    if m:
                        durum = m.group(1).decode("utf-8", "replace").strip()
                        break
            self.E.append(f"  {n}: surucu {surucu}, hiz {hiz}, EEE {durum}")
            eee.append(f"{n}({surucu}) EEE={durum}")
        n = self.klasor(Path("/etc/NetworkManager/dispatcher.d"), "etc/NetworkManager/dispatcher.d", derin=True)
        self.E.append(f"  NetworkManager dispatcher: {n} dosya (kopyalari arsivde)")
        if eee:
            acik = any("enabled" in x for x in eee)
            self.oz("ag: " + ", ".join(eee) + ("   <-- EEE ACIK (SIYI link flap riski)" if acik else ""))

    def usr_local_bin(self) -> None:
        self.bolum("/usr/local/bin")
        adlar = []
        try:
            ogeler = sorted(Path("/usr/local/bin").iterdir())
        except OSError:
            ogeler = []
        for p in ogeler:
            try:
                st = p.lstat()
            except OSError:
                continue
            if stat.S_ISLNK(st.st_mode):
                try:
                    self.E.append(f"  {goster(p.name)} -> {goster(os.readlink(p))}")
                except OSError:
                    pass
                continue
            if not stat.S_ISREG(st.st_mode):
                continue
            try:
                with open(p, "rb") as f:
                    bas_ = f.read(1024)
            except OSError:
                self.E.append(f"  {goster(p.name):<24} (okunamadi)")
                adlar.append(goster(p.name) + " (okunamadi)")
                continue
            tur = "ELF" if bas_[:4] == b"\x7fELF" else ("script" if bas_[:2] == b"#!" else "dosya")
            if tur == "script" and b"sys.exit(" in bas_ and (b"-script.pyw" in bas_ or b"-script\\.pyw" in bas_):
                tur = "pip"                       # pip'in urettigi giris scripti (paketle gelir)
            try:
                h = sha256_dosya(p)[:16] if st.st_size <= 200 * MB else "-"
            except OSError:
                h = "-"
            ek = ""
            if p.name in ("ffmpeg", "ffprobe") and tur == "ELF":
                v = self.cikti([str(p), "-version"], timeout=20).splitlines()
                ek = "  " + v[0][:70] if v else ""
            genel_h = h if tur in ("ELF", "pip") else "(arsivde)"
            list.append(self.E, f"  {goster(p.name):<24} {tur:<6} {mb(st.st_size):>9}  sha256 {genel_h}{ek}")
            self.T.append(f"  {goster(p.name):<24} {tur:<6} {mb(st.st_size):>9}  sha256 {h}{ek}")
            if tur == "script":
                self.kopyala(p, f"usr/local/bin/{p.name}")
            if tur != "pip":
                adlar.append(goster(p.name) + (f" ({ek.strip()[:40]})" if ek else ""))
        if adlar:
            self.oz("/usr/local/bin: " + ", ".join(adlar)[:150])

    def servisler(self) -> None:
        self.bolum("udev / systemd / cron / autostart / boot (kopyalari arsivde)")
        home = Path.home()
        n_udev = self.klasor(Path("/etc/udev/rules.d"), "etc/udev/rules.d")
        sd = Path("/etc/systemd/system")
        n_sd = self.klasor(sd, "etc/systemd/system", desen=r"\.(service|timer|path|socket|mount|target)$")
        try:
            dlar = sorted(d for d in sd.iterdir() if d.name.endswith(".d") and d.is_dir())
        except OSError:
            dlar = []
        for d in dlar:
            n_sd += self.klasor(d, f"etc/systemd/system/{d.name}", desen=r"\.conf$")
        n_sdu = self.klasor(home / ".config/systemd/user", "home/.config/systemd/user", derin=True)
        acik = self.cikti(["systemctl", "list-unit-files", "--state=enabled", "--no-legend", "--no-pager"])
        if acik:
            self.liste_yaz("systemd_enabled.txt", acik.encode("utf-8", "replace"))
        rc, out, _ = calistir(["crontab", "-l"], timeout=10)
        cron = rc == 0 and bool(out.strip())
        if cron:
            self.ham.append(out)
            self.yaz(f"kabuk/crontab_{KULLANICI}.txt", out, kaynak="crontab -l")
        n_crond = self.klasor(Path("/etc/cron.d"), "etc/cron.d")
        n_auto = self.klasor(home / ".config/autostart", "home/.config/autostart")
        self.kopyala(Path("/boot/extlinux/extlinux.conf"), "boot/extlinux/extlinux.conf")
        try:
            dtbo = sorted(x.name for x in Path("/boot").glob("*.dtbo"))
        except OSError:
            dtbo = []
        for d in ("/etc/ld.so.conf.d", "/etc/modules-load.d", "/etc/modprobe.d"):
            self.klasor(Path(d), "etc/" + Path(d).name, desen=r"\.conf$")
        for f in ("/etc/pip.conf", str(home / ".config/pip/pip.conf"), str(home / ".pip/pip.conf")):
            self.kopyala(Path(f), "pip/" + f.strip("/").replace("/", "_"))
        self.E.append(f"  udev kurali: {n_udev}  systemd (ozel): {n_sd}  systemd kullanici: {n_sdu}  "
                      f"cron.d: {n_crond}  autostart: {n_auto}  crontab: {'var' if cron else 'yok'}")
        self.E.append(f"  /boot dtbo overlay: {', '.join(dtbo) if dtbo else 'yok'}")
        try:
            self.E.append(f"  /etc/nvpmodel.conf -> {os.readlink('/etc/nvpmodel.conf')}")
        except OSError:
            pass

    def kabuk(self) -> None:
        self.bolum("Kabuk (skel'e gore eklenen satirlar; kopyalari arsivde)")
        home = Path.home()
        for ad, skel in ((".bashrc", "/etc/skel/.bashrc"), (".profile", "/etc/skel/.profile")):
            try:
                benim = (home / ad).read_bytes().split(b"\n")
            except OSError:
                continue
            try:
                temel = set(Path(skel).read_bytes().split(b"\n"))
            except OSError:
                temel = set()
            ek = [s for s in benim if s not in temel and s.strip()]
            if ek:
                veri = b"\n".join(ek) + b"\n"
                self.ham.append(veri)
                self.yaz(f"kabuk/{ad.lstrip('.')}_ekler.txt", veri, kaynak=f"~/{ad}")
                self.E.append(f"  ~/{ad}: {len(ek)} eklenmis satir")
        self.kopyala(home / ".bash_aliases", "home/.bash_aliases")
        self.kopyala(home / ".mavinit.scr", "home/.mavinit.scr")     # MAVProxy acilis komutlari

    def ev_scriptleri(self) -> None:
        self.bolum("Ev dizini scriptleri (~/*.sh ~/*.py ~/bin ~/scripts; kopyalari arsivde)")
        home = Path.home()
        n = 0
        try:
            ogeler = sorted(list(home.glob("*.sh")) + list(home.glob("*.py")))
        except OSError:
            ogeler = []
        for p in ogeler:
            n += self.kopyala(p, f"home/{p.name}")
        for d in ("bin", "scripts"):
            n += self.klasor(home / d, f"home/{d}")
        self.E.append(f"  kopyalanan: {n}")
        if n:
            self.oz(f"ev dizini scripti: {n} (arsivde)")

    def repo_disi(self) -> None:
        self.bolum("Repo DISINDA kod/model iceren klasorler (kopyalanmaz — repoya alinacak mi karar verilecek)")
        home = Path.home()
        try:
            repo_r = str(REPO.resolve())
        except (OSError, RuntimeError):
            repo_r = str(REPO)
        ag = ag_baglama()
        disari = []
        for kok in (home, home / "Desktop"):
            try:
                ogeler = sorted(kok.iterdir())
            except OSError:
                continue
            for p in ogeler:
                try:
                    if p.name.startswith(".") or p.name in ("_repo_yedek", "snap", "Downloads") or str(p) in ag:
                        continue
                    if os.path.islink(p):
                        try:
                            hedef = str(p.resolve())
                        except (OSError, RuntimeError):
                            self.T.append(f"  {goster(str(p))} -> (kirik / dongulu link)")
                            continue
                        if altinda(hedef, repo_r):
                            self.T.append(f"  {goster(str(p))} -> {goster(hedef)}  (kisayol, repoya bakiyor)")
                        continue
                    if not p.is_dir():
                        continue
                    pr = str(p.resolve())
                    if altinda(repo_r, pr):          # repo bunun icinde (or. ~/Desktop) ya da kendisi
                        continue
                    ornek = teknik_ornek(p)
                    if ornek:
                        boy = du(p)
                        self.T.append(f"  {goster(str(p))}  ({boy})  ornek: {goster(', '.join(ornek))}")
                        disari.append(f"{goster(str(p.relative_to(home)))} ({boy})")
                except (OSError, RuntimeError, ValueError):
                    continue
        self.E.append(f"  {len(disari)} klasor (adlar arsivde)")
        if disari:
            self.oz("repo DISINDA kod/model klasoru: " + ", ".join(disari)[:200], genel=False)
            self.ozet_genel.append(f"repo DISINDA kod/model klasoru: {len(disari)} (adlar arsivde)")

    def indirilenler(self) -> None:
        self.bolum("~/Downloads teknik dosyalar (isim + boyut)")
        d = Path.home() / "Downloads"
        satirlar = []
        try:
            ogeler = list(d.iterdir())
        except OSError:
            ogeler = []

        def zaman(p):
            try:
                return p.lstat().st_mtime
            except OSError:
                return 0
        for p in sorted(ogeler, key=zaman, reverse=True)[:300]:
            try:
                st = p.lstat()
                if stat.S_ISREG(st.st_mode) and (uzanti(p.name) in TEKNIK_UZ or ARSIV_RX.search(p.name)):
                    satirlar.append(f"  {goster(p.name):<56} {mb(st.st_size):>10}  "
                                    f"{datetime.fromtimestamp(st.st_mtime):%Y-%m-%d}")
                elif stat.S_ISDIR(st.st_mode) and teknik_ornek(p):
                    satirlar.append(f"  {goster(p.name)}/  (klasor)")
            except (OSError, RuntimeError):
                continue
        self.T.extend(satirlar[:60] or ["  yok"])
        self.E.append(f"  {len(satirlar)} teknik dosya/klasor (adlar arsivde)")
        if satirlar:
            self.oz(f"Downloads: {len(satirlar)} teknik dosya/klasor")

    def venvler(self) -> None:
        self.bolum("Python sanal ortamlari (venv) — paket listeleri venv_*.txt")
        home = Path.home()
        bulunan, ziyaret = [], 0
        atla = {"snap", "node_modules", "__pycache__", "_repo_yedek", "recordings", "logs", "site-packages",
                "dist-packages", "build", "Downloads", "datasets", "runs"}
        for dp, dns, fns in dolas(home, onerror=lambda e: None):
            ziyaret += 1
            if ziyaret > 30000:
                self.E.append("  (arama siniri: 30000 klasor)")
                break
            p = Path(dp)
            if "pyvenv.cfg" in fns:
                bulunan.append(p)
                dns[:] = []
                continue
            if len(p.relative_to(home).parts) >= 5:
                dns[:] = []
                continue
            dns[:] = [x for x in dns if (not x.startswith(".") or x in (".venv", ".venvs", ".virtualenvs"))
                      and x not in atla]
        for v in bulunan:
            try:
                cfg = (v / "pyvenv.cfg").read_text(errors="replace")
            except OSError:
                cfg = ""
            m = re.search(r"^version(?:_info)?\s*=\s*(\S+)", cfg, re.M)
            surum = m.group(1) if m else "?"
            rc, out, _ = calistir([str(v / "bin" / "python"), "-m", "pip", "freeze", "--all"], timeout=300)
            if rc != 0:                           # pip'siz venv (uv vb.): dist-info'lardan
                di = sorted({re.sub(r"\.dist-info$", "", x.name).replace("-", "==", 1)
                             for x in v.glob("lib/python*/site-packages/*.dist-info")}, key=str.lower)
                if di:
                    rc = 0
                    out = ("# (pip yok; site-packages/*.dist-info listesi)\n" + "\n".join(di) + "\n").encode(
                        "utf-8", "replace")
            slug = re.sub(r"[^A-Za-z0-9._-]+", "_", goster(str(v.relative_to(home)))).strip("_") or "venv"
            if rc == 0:
                liste = re.sub(rb"(https?://)[^/@\s]+@", rb"\1", out)
                self.liste_yaz(f"venv_{slug}.txt", f"# venv: {goster(str(v))}\n# python: {surum}\n".encode(
                    "utf-8", "replace") + liste)
                self.E.append(f"  {goster(str(v))}  python {surum}  {len(liste.splitlines())} paket "
                              f"-> venv_{slug}.txt")
            else:
                self.E.append(f"  {goster(str(v))}  (paket listesi alinamadi)")
        if bulunan:
            self.oz("venv: " + ", ".join(goster(str(v.relative_to(home))) for v in bulunan)[:200])

    def apt(self) -> None:
        self.bolum("apt / dpkg")
        man = self.cikti(["apt-mark", "showmanual"], timeout=120)
        if man:
            self.liste_yaz("apt_manual.txt", "".join(sorted(man.splitlines(keepends=True))).encode())
        rc, out, _ = calistir(["dpkg", "--audit"], timeout=120)
        yarim = [s for s in out.decode("utf-8", "replace").splitlines() if s.startswith(" ")]
        if yarim:
            self.liste_yaz("dpkg_audit.txt", out)
        self.E.append(f"  elle kurulan paket (apt-mark showmanual): {len(man.splitlines())} -> apt_manual.txt")
        self.E.append(f"  yarim kalmis paket (dpkg --audit): {len(yarim)}" + (" -> dpkg_audit.txt" if yarim else ""))
        self.oz(f"apt elle kurulan: {len(man.splitlines())} paket" +
                         (f"; dpkg YARIM: {len(yarim)} paket" if yarim else ""))

    def docker_ssh(self) -> None:
        self.bolum("Docker / SSH")
        bicim = ["--format", "{{.Names}}\t{{.Image}}\t{{.Status}}"]
        dps = self.cikti(["docker", "ps", "-a", *bicim], timeout=30)
        dim = self.cikti(["docker", "images", "--format", "{{.Repository}}:{{.Tag}}\t{{.Size}}"], timeout=30)
        if not (dps or dim) and shutil.which("docker"):
            dps = self.cikti(["sudo", "-n", "docker", "ps", "-a", *bicim], timeout=30)
            dim = self.cikti(["sudo", "-n", "docker", "images", "--format", "{{.Repository}}:{{.Tag}}\t{{.Size}}"],
                             timeout=30)
        self.T.append("  docker konteyner:" + ("\n    " + "\n    ".join(dps.splitlines()) if dps.strip()
                                              else " yok/erisilemedi"))
        self.T.append("  docker imaj:" + ("\n    " + "\n    ".join(dim.splitlines()) if dim.strip()
                                         else " yok/erisilemedi"))
        list.append(self.E, f"  docker: {len(dps.splitlines())} konteyner, {len(dim.splitlines())} imaj (adlar arsivde)")
        if dps.strip() or dim.strip():
            self.oz(f"docker: {len(dps.splitlines())} konteyner, {len(dim.splitlines())} imaj")
        ssh = Path.home() / ".ssh"
        try:
            anahtar = sorted(p.name for p in ssh.glob("id_*") if not p.name.endswith(".pub"))
        except OSError:
            anahtar = []
        self.T.append(f"  ~/.ssh anahtar: {', '.join(anahtar) if anahtar else 'yok'} (kopyalanmaz)")

    def elle_kurulum(self) -> None:
        self.bolum("Elle kurulmus yazilim izleri (sadece isimler)")
        home = Path.home()
        for d, atla in (("/opt", ()), ("/usr/local/lib", ("python3", "python2")), ("/usr/local/include", ()),
                        ("/usr/local/lib/gstreamer-1.0", ()), (str(home / ".local/share/gstreamer-1.0/plugins"), ())):
            try:
                adlar = sorted(x for x in os.listdir(d) if not (atla and x.startswith(atla)))
            except OSError:
                continue
            if adlar:
                self.E.append(f"  {d}: {goster(', '.join(adlar))[:400]}")

    def jetson_clocks(self) -> None:
        var = b"jetson_clocks" in b"\n".join(self.ham)
        jc = "var" if var else "YOK (her acilista elle)"
        self.E += ["", f"## jetson_clocks otomatik (systemd/cron/rc.local/bashrc): {jc}"]
        self.oz(f"jetson_clocks otomatik: {jc}")

    def bitir(self) -> None:
        self.bolum("Kopyalar")
        list.append(self.E, f"  {len(self.kopya)} dosya kopyasi yerel arsivde (adlar, sha256, gizlenen satirlar orada)")
        self.T += ["  (sha256 = orijinal dosyanin; arsivdeki kopyada sifre degerleri *** olabilir)"]
        self.T += [f"  {h[:16]}  {goster(r)}" for r, h in self.kopya] or ["  yok"]
        self.T += ["", "## Sifre/token izi yuzunden degerleri gizlenen dosyalar"]
        self.T += [f"  {goster(s)}: {n} satir" for s, n in self.kirpilan] or ["  yok"]
        self.T += ["", "## Kopyalanmayanlar"]
        self.T += [f"  {goster(s)}  — {n}" for s, n in self.atlanan] or ["  yok"]
        temiz, _, _ = temizle(("\n".join(self.E) + "\n").encode("utf-8", "replace"))
        (self.h / "envanter.txt").write_bytes(temiz)
        tam, _, _ = temizle(("\n".join(self.T) + "\n").encode("utf-8", "replace"), maske=True)
        (self.k / "envanter_tam.txt").write_bytes(tam)
        os.chmod(self.k / "envanter_tam.txt", 0o600)
        ilk = (f"sistem dosyasi kopyasi (arsive): {len(self.kopya)}, degeri gizlenen: {len(self.kirpilan)}, "
               f"kopyalanmayan: {len(self.atlanan)}")
        self.ozet.insert(0, ilk)
        self.ozet_genel.insert(0, f"sistem dosyasi kopyasi (arsive): {len(self.kopya)}")
        self.ham = []


# ---------------------------------------------------------------------------- 3) hazirlik
@dataclass
class Hazirlik:
    tmp: Path
    ro: Path
    yerlestir: list = field(default_factory=list)       # yerlestirilecek URETILEN dosyalari
    env: Envanter = None
    arsiv: Path = None


def paketli(v: bytes) -> bool:
    return b"==" in v or b" @ " in v


def ortam_gecerli(ad: str, f: Path, pip_calisiyor: bool, eski: bytes = b"") -> bool:
    try:
        v = f.read_bytes()
    except OSError:
        return False
    if ad == "pip_user.txt":
        # bos olabilir (--user ile kurulu paket yok) — ama eskisi doluysa supheli: kurulum surumleri buradan
        return paketli(v) or (pip_calisiyor and not paketli(eski))
    if not v.strip():
        return False
    if ad == "ortam.txt":
        return re.search(rb"(?m)^L4T=R\d", v) is not None
    if ad == "pip_freeze.txt":
        return b"==" in v or b" @ " in v
    return True


def arsiv_yolu() -> Path:
    for d in (Path.home() / "Desktop", Path.home()):
        if d.is_dir() and os.access(d, os.W_OK):
            return d / f"avci_sistem_{DAMGA}.tar.gz"
    return Path(tempfile.gettempdir()) / f"avci_sistem_{DAMGA}.tar.gz"


def hazirla(kontrol: bool, tmp: Path) -> Hazirlik:
    h = Hazirlik(tmp=tmp, ro=tmp / "referans_ortam")
    h.ro.mkdir()
    bas("3) Hazirlik: ortam kaydi + sistem envanteri  (gecici klasor; repoya henuz yazilmaz)")
    if kontrol:
        bi("--kontrol: ortam kaydi (ortam_kaydet.sh) atlandi; envanter onizlemesi:")
    else:
        sh = REPO / "setup" / "ortam_kaydet.sh"
        if sh.is_file():
            bi("ortam kaydi aliniyor (setup/ortam_kaydet.sh, 1-3 dk)...")
            rc = calistir(["bash", str(sh), str(h.ro)], timeout=900, env=dict(os.environ, AVCI_ICERDEN="1"),
                          yakala=False, grup=True)[0]
            if rc != 0:
                uy(f"ortam_kaydet.sh cikis kodu {rc} — uretilenlere tek tek bakiliyor")
            pip_ok = ortam_gecerli("pip_freeze.txt", h.ro / "pip_freeze.txt", False)
            for ad in URETILEN:
                f = h.ro / ad
                try:
                    eski = (REF / ad).read_bytes()
                except OSError:
                    eski = b""
                if not ortam_gecerli(ad, f, pip_ok, eski):
                    if ad == "pip_user.txt" and pip_ok:
                        uy(f"pip_user.txt: bu sefer BOS cikti ('pip freeze --user' paket gormedi), repodaki eskisinde "
                           f"{len([x for x in eski.splitlines() if x.strip() and not x.startswith(b'#')])} paket var "
                           "— eskisi kalir. Kontrol: python3 -m pip freeze --user | head")
                    else:
                        uy(f"{ad}: uretilemedi / eksik — repodaki eski hali kalir")
                    continue
                temiz, n, birak = temizle(f.read_bytes())
                if birak:
                    uy(f"{ad}: ozel anahtar izi — kullanilmadi")
                    continue
                f.write_bytes(temiz)
                if n:
                    uy(f"{ad}: {n} satir sifre izi yuzunden cikarildi")
                h.yerlestir.append(ad)
        else:
            uy("setup/ortam_kaydet.sh yok — ortam kaydi atlandi")
    h.env = Envanter(h.ro / "sistem", tmp / "kopya")
    h.env.uret()
    h.arsiv = arsiv_yolu()
    for s in h.env.ozet:
        bi(s)
    return h


def envanter_goster(h: Hazirlik, kontrol: bool) -> None:
    env = h.env
    yaz()
    yaz("  Repoya sistem dosyasi KOPYASI girmez; sadece teknik ozet (envanter.txt) ve paket listeleri.")
    if env.kopya:
        gruplar = {}
        for rel, _ in env.kopya:
            gruplar.setdefault(rel.split("/")[0], []).append(rel)
        yaz(f"  Kopyalar ({len(env.kopya)}) yerel arsive gider" +
            (" (gonderimde olusur)" if kontrol else "") + f": {h.arsiv}")
        for ust in sorted(gruplar):
            lst = gruplar[ust]
            if ust in ("home", "kabuk") or len(lst) <= 6:
                for r in lst:
                    yaz(f"      {goster(r)}")
            else:
                ornek = ", ".join(goster(os.path.basename(x)) for x in lst[:4])
                yaz(f"      {ust}/  {len(lst)} dosya ({ornek}, ...)")
        yaz("  Bu arsivi kurulum hazirligi icin Claude'a sohbetten gonder (GitHub'a degil).")
    if env.kirpilan:
        uy("Sifre/token izi: su dosyalarda degerler arsivde de *** yapildi (emin olunamayan satir cikarildi):")
        for s, n in env.kirpilan[:20]:
            yaz(f"      {goster(s)}: {n} satir")


# ---------------------------------------------------------------------------- ozet + rapor
def yeni_klasor(rel: str, head_dizinler: set) -> str:
    parcalar = rel.split("/")[:-1]
    for i in range(1, len(parcalar) + 1):
        d = "/".join(parcalar[:i])
        if d not in head_dizinler:
            return d
    return ""


def ozet_satirlari(t: Tarama, k: Kararlar) -> list:
    satirlar = []
    yk = {}
    for rel, b, _ in k.yeni_son:
        d = yeni_klasor(rel, t.head_dizinler)
        if d:
            a = yk.setdefault(d, [0, 0])
            a[0] += 1
            a[1] += b
    if yk:
        satirlar.append("Yeni klasorler:")
        for d in sorted(yk):
            satirlar.append(f"  {goster(d) + '/':<52} {yk[d][0]:5d} dosya {mb(yk[d][1]):>10}")
    silinenler = list(t.silinen) + list(k.link_silinen) + list(t.sahne_silme)
    say = {}
    for liste, i in ((k.yeni_son, 0), (t.degisen, 1)):
        for rel, _, _ in liste:
            say.setdefault(rel.split("/")[0] if "/" in rel else "(kok)", [0, 0, 0])[i] += 1
    for rel in silinenler:
        say.setdefault(rel.split("/")[0] if "/" in rel else "(kok)", [0, 0, 0])[2] += 1
    if say:
        satirlar.append("Klasor bazinda (+yeni ~degisen -silinen):")
        for d in sorted(say):
            a = say[d]
            satirlar.append(f"  {goster(d):<30} +{a[0]:<5d} ~{a[1]:<5d} -{a[2]}")
    if silinenler:
        satirlar.append(f"Git'ten cikanlar ({len(silinenler)}):")
        satirlar += [f"  - {goster(x)}" for x in silinenler[:15]]
        if len(silinenler) > 15:
            satirlar.append(f"  ... +{len(silinenler) - 15}")
    return satirlar


def ozet_goster(t: Tarama, k: Kararlar, h: Hazirlik) -> None:
    bas("Gidecekler")
    for s in ozet_satirlari(t, k):
        yaz("  " + s)
    if k.modeller:
        yaz("  Modeller:")
        for rel, b, s in k.modeller:
            isaret = "  (50 MB ustu: GitHub uyarir, sorun degil)" if b > SINIR_UYARI else ""
            yaz(f"    {goster(rel):<56} {mb(b):>9}  sha256 {s[:12]}{isaret}")
    if k.zorla:
        yaz(f"  .gitignore'a ragmen eklenecek (git add -f): {len(k.zorla)} dosya")
    yaz("  Kayitlar: setup/referans_ortam/ (" + ", ".join(h.yerlestir + [RAPOR_ADI]) + " + sistem/ listeleri)")
    yaz(f"  Sistem dosyasi kopyalari: repoya GIRMEZ -> {h.arsiv}")
    if k.haric:
        yaz(f"  Repoya girmeyecek (sadece bu Orin'de, .git/info/exclude): {len(k.haric)} kalem")
    if k.atla:
        yaz(f"  Bu sefer atlanan (okunamayan vb.): {len(k.atla)} kalem")
    eng = [u for u in t.uretilmis if u[0] == "engine"]
    if eng or t.venvler:
        yaz("  Git'e girmeyen, yeni Orin'de yeniden uretilecek:")
        for _, rel, b, z in eng[:12]:
            yaz(f"    [engine] {goster(rel)}  {mb(b)}  {z}")
        for v in t.venvler[:8]:
            yaz(f"    [venv]   {goster(v)}/")
    toplam = sum(b for _, b, _ in k.yeni_son) + sum(b for _, b, _ in t.degisen)
    cikan = len(t.silinen) + len(k.link_silinen) + len(t.sahne_silme)
    yaz(f"  Toplam: +{len(k.yeni_son)} ~{len(t.degisen)} -{cikan} dosya, {mb(toplam)} + kayitlar")
    yaz(f"  Etiket: {k.etiket}")
    yaz(f"  Yedek klasoru (gerekirse): {YEDEK}")


def rapor_yaz(yol: Path, t: Tarama, k: Kararlar, h: Hazirlik, b: Onbilgi) -> None:
    R = [f"# gite_gonder raporu — {ZAMAN:%Y-%m-%d %H:%M} — {MAKINE} — {KULLANICI}",
         f"taban (GitHub main): {b.taban}",
         f"etiket: {k.etiket}",
         f"yedek klasoru: {YEDEK}",
         f"sistem dosyasi kopyalari (repoda degil): {h.arsiv}", ""]
    R += ozet_satirlari(t, k)
    if k.modeller:
        R += ["", "Modeller (sha256):"] + [f"  {s}  {mb(n):>9}  {goster(r)}" for r, n, s in k.modeller]
    if t.ic:
        R += ["", "Ic ice repolar:"]
        for r in t.ic:
            durum = "icerik repoya alindi (.git yedekte)" if r in k.dahil_ic else "repoya ALINMADI"
            R.append(f"  {goster(r.rel)}/  kaynak {r.remote}  son commit {r.head}  -> {durum}")
    if k.link_kabul or t.link_degisim:
        R += ["", "Klasor -> sembolik link degisimleri:"]
        R += [f"  {goster(d)}  -> {'GITTI' if d in k.link_kabul else 'gitmedi (git ayni kaldi)'}"
              for d in sorted(t.link_degisim)]
    if k.zorla:
        R += ["", ".gitignore'a ragmen eklenenler (git add -f):"] + [f"  {goster(x)}  {mb(n)}" for x, n in k.zorla]
    if k.haric:
        R += ["", "Repoya girmeyenler (sadece bu Orin'de, .git/info/exclude):"]
        R += [f"  {goster(r)}{'/' if d else ''}  — {n}" for r, d, n in k.haric[:300]]
        if len(k.haric) > 300:
            R.append(f"  ... +{len(k.haric) - 300}")
    if k.atla:
        R += ["", "Bu sefer atlananlar (exclude'a yazilmadi):"] + [f"  {goster(r)}  — {n}" for r, n in k.atla[:200]]
    if t.okunamayan_dizin:
        R += ["", "Okunamayan klasorler (icleri gitmedi):"] + [f"  {goster(x)}/" for x in t.okunamayan_dizin]
    if k.notlar:
        R += ["", "Notlar:"] + [f"  {goster(r)}  — {n}" for r, n in k.notlar]
    secilmedi = [m for m in t.yoksayilan_model if m[0] not in {x for x, _ in k.zorla}]
    if secilmedi:
        R += ["", ".gitignore yuzunden girmeyen model dosyalari (yeni Orin'de yoklar):"]
        R += [f"  {goster(x)}  {mb(n)}" for x, n in secilmedi]
    if k.lfs_isaretci:
        R += ["", "Git LFS isaretcisi (gercek dosya degil):"] + [f"  {goster(x)}" for x in k.lfs_isaretci]
    if k.gizli_onay:
        R += ["", "Sifre/token supesi (kullanici 'gercek degil' dedi):"]
        R += [f"  {goster(r)}:{n}  [{tur}]" for r, n, tur, _, _ in k.gizli_onay]
    if t.uretilmis or t.venvler:
        R += ["", "Git'e girmeyen, bu Orin'de duran uretilmis dosyalar:"]
        R += [f"  [{tur}] {goster(r)}  {mb(n)}  {z}" for tur, r, n, z in t.uretilmis[:150]]
        R += [f"  [venv] {goster(v)}/" for v in t.venvler]
    if t.gizli_kod:
        R += ["", "Disarida tutulan veri klasorlerindeki kod/model dosyalari:"] + [f"  {goster(x)}" for x in t.gizli_kod]
    if t.head_gitlink:
        R += ["", "Repoda zaten olan ic ice repo baglantilari (dokunulmadi):"] + [f"  {goster(x)}" for x in
                                                                                  sorted(t.head_gitlink)]
    if k.uretilen_eski:
        R += ["", "setup/referans_ortam: yeniden uretilenlerin eskileri yedekte"]
        R += [f"  {goster(x)}" for x in k.elle_sistem[:50]]
    R += ["", "Sistem envanteri (ayrinti: sistem/envanter.txt; kopyalar ve tam liste yerel arsivde):"]
    R += [f"  {x}" for x in h.env.ozet_genel]
    temiz, _, _ = temizle(("\n".join(R) + "\n").encode("utf-8", "replace"))
    yol.write_bytes(temiz)


def arsiv_yaz(h: Hazirlik) -> bool:
    """Sistem dosyasi kopyalarini yerel arsive yazar (repo disi, sadece bu kullanici okur)."""
    if not h.env.kopya:
        return False
    try:
        eski = os.umask(0o077)
        try:
            with tarfile.open(h.arsiv, "w:gz") as tf:
                tf.add(h.tmp / "kopya", arcname=f"avci_sistem_{DAMGA}")     # kopyalar + envanter_tam.txt
        finally:
            os.umask(eski)
        os.chmod(h.arsiv, 0o600)
        return True
    except BaseException as e:                  # yarim arsiv kalmasin (Ctrl-C dahil)
        try:
            os.unlink(h.arsiv)
        except OSError:
            pass
        if isinstance(e, (OSError, tarfile.TarError)):
            uy(f"sistem arsivi yazilamadi ({e}) — gonderim yine de devam ediyor")
            return False
        raise


# ---------------------------------------------------------------------------- exclude
DUSEBILIR = {"veri", "veri klasoru", ">99 MB", "model (istenmedi)"}   # diskte yoksa satiri silinebilir


def exclude_icerik(eski, haric: list) -> bytes:
    """Yonetilen blogu yeniden yazar, yeniler eklenir. Diskte artik olmayan yol icin satir sadece tekrar
    gelirse zararsizsa silinir (veri, >99 MB: yine sorulur/disarida kalir). 'elle git'ten cikarildi',
    'ic ice repo', paketin yok saydiklari vb. yol bir sure yok olsa da KALIR (geri gelince sessizce gitmesin)."""
    diger, yonetilen = exclude_oku(eski or b"")
    if not yonetilen and not haric:
        return eski or b""                             # yonetilecek bir sey yok: dosyaya dokunma
    kalan = []
    for d, n in yonetilen:
        rel, _ = desen_coz(d)
        if os.path.lexists(REPO / rel) or (n or "eski") not in DUSEBILIR:
            kalan.append((n or "eski", d))
    for rel, dizin, n in haric:
        if "\n" in rel or "\r" in rel:
            continue                                   # exclude satirina yazilamaz (sahnelenmez yine de)
        kalan.append((n, haric_desen(rel, dizin)))
    gorulen, son = set(), []
    for n, d in sorted(kalan, key=lambda x: (x[0], x[1])):
        if d not in gorulen:
            gorulen.add(d)
            son.append((n, d))
    satirlar = list(diger)
    if son:
        if satirlar:
            satirlar.append("")
        satirlar.append(EXC_BAS)
        onceki = None
        for n, d in son:
            if n != onceki:
                satirlar.append(f"# [{n}]")
                onceki = n
            satirlar.append(d)
        satirlar.append(EXC_SON)
    return os.fsencode("\n".join(satirlar) + "\n") if satirlar else b""


# ---------------------------------------------------------------------------- 5) gonderim
class Geri:
    """Commit'ten once hata / Ctrl-C / SSH kopmasi olursa yapilanlari tersten geri alir."""

    def __init__(self):
        self.adimlar = []

    def ekle(self, ad: str, fn) -> None:
        self.adimlar.append((ad, fn))

    def geri_al(self) -> None:
        with kritik(bekleyeni_at=True):                # geri alma yarim kalmasin
            yaz()
            uy("hata/iptal — yapilanlar geri aliniyor (bu sirada Ctrl-C calismaz)")
            hatali = []
            for ad, fn in reversed(self.adimlar):
                try:
                    fn()
                    bi(f"geri alindi: {goster(ad)}")
                except Exception as e:
                    hatali.append(ad)
                    ha(f"geri ALINAMADI: {goster(ad)} ({type(e).__name__}: {e})")
            if hatali:
                ha(f"{len(hatali)} adim geri alinamadi. Yedekler burada: {YEDEK} — bu ciktiyi bana gonder.")
            else:
                yedek_temizle()
                ok("repo calistirmadan onceki haline dondu")


def index_geri(idx: Path, yedek: Path) -> None:
    tmp = idx.with_name(idx.name + ".gite_gonder")
    shutil.copy2(yedek, tmp)
    os.replace(tmp, idx)
    kilit = idx.with_name(idx.name + ".lock")
    if kilit.exists():
        uy(f"{kilit} duruyor: baska bir git calismiyorsa sil (rm {kilit})")


def dosya_geri(f: Path, eski) -> None:
    if eski is None:
        if os.path.lexists(f):
            f.unlink()
    else:
        atomik_yaz(f, eski)


def yerlestir(g: Geri, kaynak: Path, hedef: Path, rel: str) -> None:
    """Uretilen kaydi yerine koyar; oradaki eskisi (varsa) yedege tasinir. Tasima + geri alma kaydi birlikte."""
    with kritik():
        yedek = yedege_tasi(hedef, rel) if os.path.lexists(hedef) else None

        def geri(h_=hedef, y=yedek):
            if os.path.lexists(h_):
                if os.path.isdir(h_) and not os.path.islink(h_):
                    shutil.rmtree(h_)                  # bu calismada uretilen kopya
                else:
                    os.unlink(h_)
            if y is not None:
                os.rename(y, h_)
        g.ekle(rel, geri)
    if kaynak.is_dir():
        shutil.copytree(kaynak, hedef, symlinks=True)
    else:
        shutil.copy2(kaynak, hedef)


def dogrula(t: Tarama, k: Kararlar, beklenen: set) -> list:
    """Sahnelenen her sey: taramada gosterilen mi, YENI ic ice repo baglantisi yok mu, >99 MB yok mu,
    sifre yok mu? Sifre onayi dosyanin SAHNELENEN icerigine (blob sha) bagli: degistiyse kabul edilmez."""
    raw = git("diff-index", "--cached", "--raw", "-z", "--no-abbrev", "--no-renames", "--ignore-submodules=none",
              "HEAD")
    p = raw.split(b"\0")
    girdiler = []
    i = 0
    while i + 1 < len(p):
        meta, yol = p[i], p[i + 1]
        i += 2
        if not meta.startswith(b":"):
            continue
        a = meta[1:].decode().split()
        girdiler.append((a[1], a[3], a[4][:1], os.fsdecode(yol)))      # yeni_mod, yeni_sha, durum, yol
    gitlink = [y for m, _, d, y in girdiler if d != "D" and m == "160000"]
    fazla = [y for _, _, _, y in girdiler if y not in beklenen and not altinda(y, SISTEM_REL)]
    sizan = [y for _, _, d, y in girdiler if d != "D" and k.disarida(y)]
    boy = nesne_boyutlari([s for m, s, d, _ in girdiler if d != "D" and m != "160000"])
    buyuk = [(y, boy[s][1]) for m, s, d, y in girdiler if d != "D" and s in boy and boy[s][1] > SINIR_ENGEL]
    onay = onaylar_oku() | set(BEKLEYEN_ONAY)
    liste = [(s, y, boy[s][1]) for m, s, d, y in girdiler
             if d != "D" and m not in ("160000", "120000") and s in boy and boy[s][0] == "blob"
             and boy[s][1] <= SINIR_ENGEL]
    gizli = []
    for s, y, v in blob_icerikleri(liste):
        mt = metin_icerik(v, y)
        bulgular = gizli_icerik(mt, kod=kod_mu(y)) if mt is not None else []
        if GIZLI_AD.search(y):
            bulgular.append((0, "hassas dosya adi", ""))
        for no, tur, ms in bulgular:
            if (s, tur) not in onay:
                gizli.append((y, no, tur, ms))
    for m, s, d, y in girdiler:                      # ad kontrolu: link dahil
        if d != "D" and m == "120000" and GIZLI_AD.search(y) and (s, "hassas dosya adi") not in onay:
            gizli.append((y, 0, "hassas dosya adi", ""))
    sorun = 0
    for y in gitlink:
        ha(f"ic ice repo baglantisi sahnede (icerigi gitmez): {goster(y)}")
        sorun += 1
    for y in fazla[:20]:
        ha(f"taramada gosterilmeyen dosya sahnede: {goster(y)}")
        sorun += 1
    for y in sizan[:20]:
        ha(f"disarida kalmasi gereken dosya sahnede: {goster(y)}")
        sorun += 1
    for y, n in buyuk:
        ha(f"{mb(n)} dosya sahnede: {goster(y)}")
        sorun += 1
    for y, no, tur, ms in gizli[:20]:
        ha(f"sifre/token izi (sahnelenen halde; dosya taramadan sonra degismis olabilir): {goster(y)}:{no} [{tur}] {ms}")
        sorun += 1
    if sorun:
        raise Dur("dogrulama basarisiz — hicbir sey gonderilmedi")
    return girdiler


def uygula(t: Tarama, k: Kararlar, h: Hazirlik, b: Onbilgi) -> int:
    g = Geri()
    commit_oldu = False
    etiketli = False
    ref = REPO / REF_REL
    bas("4) Gonderim")
    arsiv_var = arsiv_yaz(h)                    # repo DISI; geri alinmaz
    if arsiv_var:
        ok(f"sistem dosyasi kopyalari arsive yazildi (repoda degil): {h.arsiv}")
    try:
        iskelet_kur(YEDEK)
        idx = git_yolu("index")
        with kritik():
            if idx.exists():
                idx_y = YEDEK / "_git_index"
                shutil.copy2(idx, idx_y)
                g.ekle("sahne (.git/index)", lambda: index_geri(idx, idx_y))
            else:
                g.ekle("sahne (.git/index)", lambda: idx.unlink() if idx.exists() else None)

        for gy in k.tasinacak_git:
            src = REPO / gy
            with kritik():                      # tasima + geri alma kaydi birlikte
                dst = yedege_tasi(src, gy)
                g.ekle(gy, lambda s=src, d=dst: os.rename(d, s))
            ok(f".git yedege: {goster(gy)} -> {dst}")

        rapor_yaz(h.ro / RAPOR_ADI, t, k, h, b)
        if not ref.exists():
            with kritik():
                ref.mkdir(parents=True)
                g.ekle(REF_REL + "/", lambda: bos_klasorleri_sil(ref))
        for ad in h.yerlestir + [RAPOR_ADI]:
            yerlestir(g, h.ro / ad, ref / ad, f"{REF_REL}/{ad}")
        yerlestir(g, h.ro / "sistem", ref / "sistem", SISTEM_REL)
        ok("kayitlar yerlestirildi: setup/referans_ortam/ (" + ", ".join(h.yerlestir + [RAPOR_ADI]) + " + sistem/)")

        exc = git_yolu("info/exclude")
        exc_eski = exc.read_bytes() if exc.exists() else None
        yeni_exc = exclude_icerik(exc_eski, k.haric)
        if yeni_exc != (exc_eski or b""):
            with kritik():
                exc.parent.mkdir(parents=True, exist_ok=True)
                atomik_yaz(exc, yeni_exc)
                g.ekle(".git/info/exclude", lambda: dosya_geri(exc, exc_eski))
        if k.haric:
            ok(f"{len(k.haric)} kalem sadece bu Orin'de kalacak (.git/info/exclude)")
            hala = [r for r, d, _ in k.haric if os.path.lexists(REPO / r) and not d and "\n" not in r]
            hala = sorted(set(hala) - check_ignore(hala)) if hala else []
            if hala:
                uy(f".gitignore'daki '!' istisnasi yuzunden {len(hala)} yol git'te hala 'yeni' gorunur "
                   "(bu script disarida tutar; elle 'git add -A' YAPMA):")
                for x in hala[:8]:
                    yaz(f"      {goster(x)}")

        # onceden (elle) sahnelenmis ama disarida kalacaklar sahneden cikar (.gitignore'daki !istisnalar dahil)
        onceki = git_z("diff-index", "--cached", "--name-only", "-z", "--no-renames", "HEAD")
        ss = set(t.sahne_silme) | set(t.sahne_gitlink)       # bunlar asagida ayrica islenir
        geri_cek = [y for y in onceki if k.disarida(y) and y not in ss]
        if geri_cek:
            git("reset", "-q", "--pathspec-from-file=-", "--pathspec-file-nul", girdi=pathspec(geri_cek))
            bi(f"onceden sahnelenmis {len(geri_cek)} dosya sahneden cikarildi (disarida kalacaklar)")
        if t.sahne_gitlink:                     # elle sahnelenmis ic ice repo baglantilari: index'ten cikar
            git("rm", "-q", "--cached", "-f", "--ignore-unmatch", "--pathspec-from-file=-", "--pathspec-file-nul",
                girdi=pathspec(t.sahne_gitlink))
        silinecek = list(t.silinen) + list(k.link_silinen)
        if silinecek:                           # silinenler: git rm --cached (link olmus klasor altinda da calisir)
            git("rm", "-q", "--cached", "--ignore-unmatch", "--pathspec-from-file=-", "--pathspec-file-nul",
                girdi=pathspec(silinecek))

        zorla = {x for x, _ in k.zorla}
        normal, kayip = [], []
        for y, _, _ in k.yeni_son:
            if y not in zorla:
                (normal if os.path.lexists(REPO / y) else kayip).append(y)
        normal += [y for y, _, _ in t.degisen]
        normal += [f"{REF_REL}/{ad}" for ad in h.yerlestir + [RAPOR_ADI]] + [SISTEM_REL]
        if kayip:
            uy(f"taramadan sonra kaybolan {len(kayip)} yeni dosya atlandi")
        bi("sahneleniyor (sadece taramada gosterilen dosyalar)...")
        git(*LFS_KAPALI, "add", "-A", "--pathspec-from-file=-", "--pathspec-file-nul", girdi=pathspec(normal))
        zl = [x for x in sorted(zorla) if os.path.lexists(REPO / x)]
        if zl:
            git(*LFS_KAPALI, "add", "-f", "--pathspec-from-file=-", "--pathspec-file-nul", girdi=pathspec(zl))
        beklenen = set(normal) | set(zl) | set(silinecek) | set(t.sahne_silme)
        hazir = dogrula(t, k, beklenen)
        ok(f"dogrulama tamam: {len(hazir)} dosya (yeni ic ice repo baglantisi yok, 99 MB ustu yok, "
           "sifre izi yok)")

        mesaj = (f"Orin anlik durum {ZAMAN:%Y-%m-%d} ({MAKINE})\n\n"
                 f"gite_gonder.py ile. Etiket: {k.etiket}\n"
                 + "\n".join(ozet_satirlari(t, k)[:40]) +
                 "\n\nKayitlar: setup/referans_ortam/ (ortam kaydi + sistem/ listeleri)\n"
                 f"Rapor: {REF_REL}/{RAPOR_ADI}\n")
        agac = git_s("write-tree")
        eski_head = git_s("rev-parse", "HEAD")
        with kritik():                          # commit yarim kalmasin; git kancalari (hook) calismaz
            git(*b.kimlik, "-c", "commit.gpgsign=false", "-c", "core.hooksPath=/dev/null", "commit", "-q",
                "--no-verify", "-F", "-",
                girdi=mesaj.encode("utf-8", "replace"), grup=True)
            commit_oldu = True
            if git_s("rev-parse", "HEAD^{tree}") != agac or git_s("rev-parse", "HEAD^") != eski_head:
                git("reset", "-q", "--soft", eski_head)
                commit_oldu = False
                raise Dur("commit dogrulanan sahneden farkli cikti (git kancasi?) — geri alindi, bana yaz")
            onaylar_yaz()                       # commit ile birlikte: sonraki 'sadece push' tekrar sormasin
            try:                                # etiket de ayni kesintisiz bolumde (commit etiketsiz kalmasin)
                git(*b.kimlik, "-c", "tag.gpgSign=false", "tag", "-a", k.etiket, "-m",
                    f"Orin {MAKINE} anlik durumu {ZAMAN:%Y-%m-%d %H:%M}", grup=True)
                etiketli = True
            except Dur as e:
                ha(f"etiket olusturulamadi ({e}) — commit duruyor. Elle: git tag -a {k.etiket} -m orin")
    except BaseException:
        _Sinyal.cikista = True                  # bundan sonra gelen sinyaller geri almayi bolemez
        if not commit_oldu:
            g.geri_al()
        else:
            yedek_temizle()
        raise

    yedek_temizle()
    sha = git_s("rev-parse", "--short", "HEAD")
    ok(f"commit {sha}" + (f" + etiket {k.etiket}" if etiketli else "") + " (bu Orin'de hazir)")

    eksik = [e for e in eksik_etiketler() if e != k.etiket]
    if push_et(([k.etiket] if etiketli else []) + eksik) != 0:
        return 1
    yaz()
    ok("GONDERILDI" + (f": {b.web}/tree/{k.etiket if etiketli else 'main'}" if b.web else ""))
    bi(f"Rapor: {REF_REL}/{RAPOR_ADI}   Envanter: {SISTEM_REL}/")
    if arsiv_var:
        bi(f"Sistem dosyasi kopyalari (repoda DEGIL): {h.arsiv}  -> bunu Claude'a sohbetten gonder")
    if YEDEK.exists():
        bi(f"Yedekler (tasinan .git'ler / eski kayitlar): {YEDEK}")
    yaz("\n  Sonraki adim: Claude'a 'pushladim' yaz ve arsivi ekle — repoyu acip kurulum scriptini ve PDF'i hazirlar.")
    yaz("  Laptop: avci_follow'u commit'lemeden ONCE 'git pull'.")
    return 0


# ---------------------------------------------------------------------------- push
def uzak_etiketler():
    global _UZAK_ETIKET
    if _UZAK_ETIKET is None:
        rc, out, _ = calistir(["git", "-C", str(REPO), "ls-remote", "--tags", "origin"], timeout=120)
        if rc != 0:
            return None
        _UZAK_ETIKET = {s.split("refs/tags/", 1)[1].replace("^{}", "")
                        for s in out.decode("utf-8", "replace").splitlines() if "refs/tags/" in s}
    return _UZAK_ETIKET


def eksik_etiketler() -> list:
    """Bu Orin'de olup GitHub'da olmayan orin-* etiketleri (or. onceki push yarim kaldi)."""
    uzak = uzak_etiketler()
    if uzak is None:
        return []
    return [e for e in git_s("tag", "-l", "orin-*").split()
            if e not in uzak and calistir(["git", "-C", str(REPO), "merge-base", "--is-ancestor", e, "HEAD"])[0] == 0]


def push_et(etiketler: list) -> int:
    etiketler = list(dict.fromkeys(etiketler))
    refs = ["refs/heads/main:refs/heads/main"] + [f"refs/tags/{e}:refs/tags/{e}" for e in etiketler]
    bi("GitHub'a gonderiliyor (main" + (" + " + ", ".join(etiketler) if etiketler else "") +
       ") — buyuk modeller varsa birkac dakika surebilir...")
    rc = calistir(["git", "-C", str(REPO), "push", "--atomic", "origin", *refs], timeout=None, yakala=False)[0]
    if rc != 0:
        ha("push basarisiz — GitHub'da hicbir sey degismedi (atomik). Commit ve etiket bu Orin'de duruyor.")
        yaz("    Olasi nedenler: internet / SSH anahtari (ssh -T git@github.com) /")
        yaz("    biri arada GitHub'a push etti (o zaman bana yaz, birlestirelim).")
        yaz("    Duzelince scripti tekrar calistir (sadece push yapar).")
    return rc


def sadece_push(kontrol: bool, eksik: list, ileride: int, b: Onbilgi) -> int:
    ne = ([f"{ileride} commit"] if ileride else []) + ([f"etiket: {', '.join(eksik)}"] if eksik else [])
    uy("kodda yeni degisiklik yok ama GitHub'a gitmemisler var: " + "; ".join(ne))
    # onceki calisma commit'ten hemen sonra kesildiyse (Ctrl-C / SSH) o commit etiketsiz kalmis olabilir
    yeni_etiket = ""
    if ileride and git_s("log", "-1", "--format=%s", "HEAD").startswith("Orin anlik durum") \
            and not git_s("tag", "--points-at", "HEAD", "-l", "orin-*"):
        yeni_etiket = etiket_sec("")
        bi(f"son commit etiketsiz — etiket de eklenecek: {yeni_etiket}")
    if kontrol:
        bi("gonderimde: sadece push yapilacak")
        return 0
    if not sor("Sadece push yapilsin mi?", True):
        return 0
    onaylar_yaz()
    if yeni_etiket:
        try:
            with kritik():
                git(*b.kimlik, "-c", "tag.gpgSign=false", "tag", "-a", yeni_etiket, "-m",
                    f"Orin {MAKINE} anlik durumu {ZAMAN:%Y-%m-%d %H:%M}", grup=True)
            eksik = [yeni_etiket] + list(eksik)
        except Dur as e:
            ha(f"etiket olusturulamadi ({e}) — sadece commit gidiyor")
    rc = push_et(eksik)
    if rc == 0:
        ok("GONDERILDI" + (f": {b.web}/tree/{eksik[0]}" if eksik and b.web else ""))
    return 0 if rc == 0 else 1


# ---------------------------------------------------------------------------- main
def main() -> int:
    global REPO, REF, YEDEK
    for akim in (sys.stdout, sys.stderr):
        try:
            akim.reconfigure(errors="backslashreplace")
        except (AttributeError, ValueError):
            pass
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(s, _sinyal_isleyici)
        except (ValueError, OSError, AttributeError):
            pass
    ap = argparse.ArgumentParser(description="Calisan Orin'in son halini GitHub'a gonderir (main + etiket).")
    ap.add_argument("--kontrol", action="store_true", help="sadece rapor: repoda hicbir sey degismez (git fetch haric)")
    ap.add_argument("--etiket", default="", help="etiket adi (varsayilan orin-YYYY-MM-DD)")
    a = ap.parse_args()
    tmp = None
    try:
        etiket_gecerli(a.etiket)
        REPO = repo_bul()
        REF = REPO / REF_REL
        YEDEK = yedek_kok()
        if not a.kontrol:
            yaz("  Son onaydan once repoda hicbir sey degismez (git fetch haric).")
        b = on_kontrol(a.kontrol)
        t = tara()
        eksik = eksik_etiketler()
        bos = not (t.yeni or t.degisen or t.silinen or t.ic or t.sahne_silme or t.link_degisim)
        if bos and (b.ileride or eksik):
            return sadece_push(a.kontrol, eksik, b.ileride, b)
        if bos:
            etk = git_s("tag", "--points-at", "HEAD", "-l", "orin-*")
            ok("kodda degisiklik yok, GitHub ile ayni" + (f" (etiket: {etk.split()[0]})" if etk else ""))
            if a.kontrol or not sor("Yine de ortam kaydi + envanter guncellenip gonderilsin mi?", False):
                return 0
        k = kararlar(t, a.kontrol, a.etiket)
        tmp = Path(tempfile.mkdtemp(prefix="gite_gonder_"))
        h = hazirla(a.kontrol, tmp)
        envanter_goster(h, a.kontrol)
        ozet_goster(t, k, h)
        if a.kontrol:
            yaz("\n  --kontrol: repoda hicbir sey degismedi. Gondermek icin: python3 setup/gite_gonder.py")
            return 0
        yaz()
        uy("Repo PUBLIC: gonderilen her sey herkese acik. Arkadastan gelen paketler varsa izinli olduklarindan emin ol.")
        if not sor(f"Gonderilsin mi? (commit + etiket {k.etiket} + push)", False):
            yaz("  Iptal — repoda hicbir sey degismedi.")
            return 0
        return uygula(t, k, h, b)
    except Bitti as e:
        yaz()
        ok(str(e))
        return 0
    except Dur as e:
        yaz()
        ha(str(e))
        return 1
    except KeyboardInterrupt:
        yaz()
        ha("Ctrl-C / baglanti koptu — durduruldu.")
        return 130
    except Exception as e:
        yaz()
        ha(f"beklenmeyen hata: {type(e).__name__}: {e}")
        yaz("  Bu ciktiyi bana gonder:")
        try:
            traceback.print_exc()
        except (OSError, ValueError):
            pass
        return 1
    finally:
        _Sinyal.cikista = True                  # temizlik bolunmesin
        if tmp is not None:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
