#!/usr/bin/env python3
"""
HUNTER — MANUEL KILIT (--ch-lock) offline regresyon.  (9 Kas 2026)

FC YOK, kamera YOK, YOLO motoru YOK, ucus YOK. Iki C++ katmanini derleyip
MANTIGINI dogrular. Diger test_*_offline.py'ler gibi: sahaya cikmadan once
burasi YESIL olmali.

    cd kcf && python3 test_manual_lock_offline.py

NE TEST EDILIYOR
----------------
A) simple_lock.hpp — SimpleLock'un manuel kilit mantigi
     test_manual_lock_logic.cpp, GERCEK basligi derler
       - kanit kapisi (manualMinHits): tek karelik false positive kilitlenmez
       - kilitte detectionWanted() FALSE -> YOLO thread'i bosta
       - manualOnly: otomatik 20/30 kilit KAPALI
       - manualOnly=false: ESKI davranis bozulmadi
       - buton birakma -> ARAMA, YOLO yeniden baslar, tekrar kilitlenebilir

B) runtracker.cpp — readRcControl'un CH9 latching mantigi
     test_manual_lock_rc.cpp, fonksiyon govdesini runtracker.cpp'den
     AYNEN cikarip derler (KOPYA DEGIL — kod degisirse test onu gorur)
       - >1600 ACIK / <1350 KAPALI, ara bolge son durumu korur
       - GECERSIZ RC'de kilit DUSMEZ (ucus ortasinda birakilmasin)
       - CH8/CH12 mantigina karismiyor
       - --ch-lock N ile kanal secilebiliyor

NEDEN C++ TESTI PYTHON'DAN KOSUYOR: repodaki diger kapilar python3 ile
kosuyor, saha ritmi o. Bu betik derlemeyi ve kosmayi kendi yapar; senin
g++ satiri yazmana gerek yok.

OPENCV NOTU: (A) icin simple_lock.hpp -> target_manager.hpp ->
ortrack_core.hpp zinciri <opencv2/opencv.hpp> istiyor. Betik OpenCV'yi
TAHMIN ETMEZ, DENER: aday bayrak kumeleriyle (pkg-config opencv4/opencv,
/usr/include/opencv4, sistem varsayilani) tek satirlik bir dosya gercekten
derlenir ve ilk derlenen kullanilir. Hicbiri tutmazsa asgari bir cv:: stub
uretilir ve bu ekrana BASILIR — sessizce dusmez. SimpleLock cv::
KULLANMADIGI icin stub (A)'yi gecersiz kilmaz; ortam yuzunden test KALMAZ.
(B) OpenCV'ye hic ihtiyac duymaz.

9 Kas: ilk surum pkg-config'in dondurdugu bayraklara GUVENIYORDU. Orin'de
pkg-config basariyla bayrak dondurdu ama o bayraklarla baslik bulunamadi;
betik "OpenCV: GERCEK" yazip stub uretmedi ve (A) hic kosmadi. Projenin
kendisi pkg-config kullanmiyor (CMake find_package + OpenCV_INCLUDE_DIRS).
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
CXX = os.environ.get("CXX", "g++")

STUB = r'''// OTOMATIK URETILDI (test_manual_lock_offline.py) — GERCEK OpenCV YOK.
// Sadece simple_lock.hpp zincirinin derlenmesi icin. SimpleLock cv::
// kullanmiyor; gorsel islem yapan hicbir sey TEST EDILMIYOR.
#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>
typedef unsigned char uchar;
#define CV_8UC3 16
namespace cv {
enum { BORDER_CONSTANT = 0, BORDER_ISOLATED = 16 };
struct Size { int width = 0, height = 0; Size() {} Size(int w, int h) : width(w), height(h) {} };
struct Rect {
    int x = 0, y = 0, width = 0, height = 0;
    Rect() {} Rect(int x_, int y_, int w_, int h_) : x(x_), y(y_), width(w_), height(h_) {}
};
struct Scalar { double v[4]{}; Scalar() {} Scalar(double a, double b, double c) { v[0]=a; v[1]=b; v[2]=c; } };
struct Mat {
    int rows = 0, cols = 0; size_t step = 0;
    bool empty() const { return rows == 0 || cols == 0; }
    int  type() const { return CV_8UC3; }
    int  channels() const { return 3; }
    Mat  operator()(const Rect&) const { return *this; }
    template <typename T> T* ptr(int = 0) { return nullptr; }
    template <typename T> const T* ptr(int = 0) const { return nullptr; }
    uchar* ptr(int = 0) { return nullptr; }
    void copyTo(Mat& dst) const { dst = *this; }
};
inline void copyMakeBorder(const Mat&, Mat& dst, int, int, int, int, int, const Scalar&) { dst = Mat(); }
inline void resize(const Mat&, Mat& dst, Size) { dst = Mat(); }
}
'''

# runtracker.cpp'den cikarilacak blok: struct RcControl ... RTSP bolumune kadar.
RC_BEGIN = "struct RcControl {"
RC_END = "// ================= RTSP OUTPUT (GStreamer) ================="


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def _compiles(flags, tmp, snippet, extra_inc=()):
    """Verilen bayraklarla ufak bir dosya GERCEKTEN derleniyor mu?"""
    src = os.path.join(tmp, "probe.cpp")
    with open(src, "w", encoding="utf-8") as f:
        f.write(snippet)
    out = os.path.join(tmp, "probe.bin")
    r = run([CXX, "-std=c++17", *extra_inc, "-o", out, src, *flags])
    try:
        os.remove(out)
    except OSError:
        pass
    return r.returncode == 0


def probe_opencv(tmp):
    """Gercek OpenCV bayraklarini DENEYEREK bul; bulunamazsa None.

    9 KAS NOTU — neden pkg-config'e GUVENILMIYOR:
    Orin'de `pkg-config --cflags --libs opencv4` BASARIYLA bayrak dondurdu
    ama o bayraklarla opencv2/opencv.hpp BULUNAMADI (fatal error) ve bu
    betik "OpenCV: GERCEK" yazip stub uretmedigi icin A bolumu hic
    kosmadi. Projenin kendisi pkg-config kullanmiyor: CMakeLists
    find_package(OpenCV) + ${OpenCV_INCLUDE_DIRS} ile gidiyor (JetPack'te
    tipik olarak /usr/include/opencv4). Yani pkg-config .pc dosyasi eksik
    ya da bayat olabilir.

    Dogrusu: TAHMIN ETME, DENE. Her aday bayrak kumesiyle baslik iceren
    tek satirlik bir dosya derlenir; ILK DERLENEN kazanir.
    """
    snippet = "#include <opencv2/opencv.hpp>\nint main(){ cv::Mat m; return m.empty()?0:1; }\n"

    cands = []
    for name in ("opencv4", "opencv"):
        r = run(["pkg-config", "--cflags", "--libs", name])
        if r.returncode == 0 and r.stdout.strip():
            cands.append((f"pkg-config {name}", r.stdout.split()))
    # JetPack / elle kurulum yaygin yerleri — pkg-config bozuksa bunlar tutar.
    for d in ("/usr/include/opencv4", "/usr/local/include/opencv4",
              "/usr/include", "/usr/local/include"):
        if os.path.isdir(os.path.join(d, "opencv2")):
            cands.append((f"-I{d}", [f"-I{d}", "-lopencv_core"]))
    cands.append(("sistem varsayilani", ["-lopencv_core"]))

    for label, flags in cands:
        if _compiles(flags, tmp, snippet):
            return label, flags
    return None, None


def extract_rc(tmp):
    """readRcControl govdesini runtracker.cpp'den AYNEN cikar."""
    src_path = os.path.join(HERE, "runtracker.cpp")
    with open(src_path, encoding="utf-8", errors="replace") as f:
        src = f.read()
    if RC_BEGIN not in src or RC_END not in src:
        return None, (f"runtracker.cpp icinde beklenen isaretler yok "
                      f"({RC_BEGIN!r} / {RC_END!r}). Dosya yeniden "
                      f"duzenlendiyse bu betigi guncelle.")
    i = src.index(RC_BEGIN)
    j = src.index(RC_END)
    if j <= i:
        return None, "isaretlerin sirasi ters — runtracker.cpp degismis."
    body = src[i:j]
    if "manual_lock" not in body:
        return None, ("cikarilan blokta 'manual_lock' YOK — --ch-lock yamasi "
                      "runtracker.cpp'de duruyor mu?")
    out = os.path.join(tmp, "rc_extract.inc")
    with open(out, "w", encoding="utf-8") as f:
        f.write(body)
    return body.count("\n") + 1, None


def main():
    if not shutil.which(CXX):
        print(f"HATA: '{CXX}' bulunamadi. CXX ortam degiskeni ile derleyici ver.")
        return 1

    tmp = tempfile.mkdtemp(prefix="hunter_manual_lock_")
    failures = []
    try:
        # ---------------------------------------------------------------
        print("=" * 62)
        print(" A) simple_lock.hpp — manuel kilit mantigi")
        print("=" * 62)
        label, cv_flags = probe_opencv(tmp)
        inc = ["-I", os.path.join(HERE, "include")]
        if cv_flags is not None:
            print(f"  OpenCV: GERCEK ({label}) — derleme denemesiyle dogrulandi")
        else:
            # Stub'a dusmek A bolumunu GECERSIZ KILMAZ: SimpleLock cv::
            # kullanmiyor, stub yalnizca ortrack_core.hpp zincirinin
            # derlenmesi icin. Bu yuzden ortam yuzunden test KALMIYOR.
            stub_dir = os.path.join(tmp, "stub", "opencv2")
            os.makedirs(stub_dir, exist_ok=True)
            with open(os.path.join(stub_dir, "opencv.hpp"), "w",
                      encoding="utf-8") as f:
                f.write(STUB)
            inc += ["-I", os.path.join(tmp, "stub")]
            cv_flags = []
            print("  OpenCV: calisan bayrak BULUNAMADI -> asgari stub uretildi")
            print("          (SimpleLock cv:: kullanmiyor; gorsel yol TEST EDILMIYOR)")

        binA = os.path.join(tmp, "logic")
        srcA = os.path.join(HERE, "test_manual_lock_logic.cpp")
        r = run([CXX, "-std=c++17", *inc, "-o", binA, srcA, *cv_flags])
        if r.returncode != 0:
            print("  DERLEME BASARISIZ:")
            print("  " + "\n  ".join((r.stderr or r.stdout).splitlines()[:25]))
            failures.append("A: derlenemedi")
        else:
            r = run([binA])
            print(r.stdout.rstrip())
            if r.returncode != 0:
                failures.append("A: testler dustu")

        # ---------------------------------------------------------------
        print()
        print("=" * 62)
        print(" B) runtracker.cpp — readRcControl CH9 latching mantigi")
        print("=" * 62)
        nlines, err = extract_rc(tmp)
        if err:
            print(f"  CIKARMA BASARISIZ: {err}")
            failures.append("B: readRcControl cikarilamadi")
        else:
            print(f"  runtracker.cpp'den {nlines} satir AYNEN cikarildi")
            binB = os.path.join(tmp, "rc")
            srcB = os.path.join(HERE, "test_manual_lock_rc.cpp")
            r = run([CXX, "-std=c++17", "-I", tmp, "-o", binB, srcB])
            if r.returncode != 0:
                print("  DERLEME BASARISIZ:")
                print("  " + "\n  ".join((r.stderr or r.stdout).splitlines()[:25]))
                failures.append("B: derlenemedi")
            else:
                r = run([binB])
                print(r.stdout.rstrip())
                if r.returncode != 0:
                    failures.append("B: testler dustu")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print()
    print("=" * 62)
    if failures:
        print("SONUC: KALDI")
        for f in failures:
            print("   -", f)
        return 1
    print("SONUC: A + B HEPSI GECTI")
    return 0


if __name__ == "__main__":
    sys.exit(main())
