#!/usr/bin/env python3
"""
render_overlay.py — UCUS SONRASI. Ham videoyu ve konum CSV'sini birlestirip
mesafe yazili 1080p ciktisi uretir.

    python3 render_overlay.py ~/recordings/chase_20260907_143012

Uc dosyayi bekler (record_agent.py'nin urettigi):
    <base>_raw.mkv   ham video
    <base>.csv       zaman damgali konumlar
    <base>.json      video/duvar saati senkronu

Cikti:
    <base>_osd.mp4

NASIL CALISIYOR
    CSV satirlarindan bir ASS altyazi dosyasi uretiliyor, ffmpeg onu videoya
    yakiyor. ASS secildi cunku zamanlamayi ve konumu hassas veriyor; yuzlerce
    drawtext filtresi zincirlemekten cok daha temiz.

    Zaman hizalamasi .json icindeki 'wall_at_video_zero' degerinden geliyor:
        video_saniyesi = t_wall - wall_at_video_zero
    Bu deger yoksa ilk CSV satiri video basi kabul edilir (kaba, ~1-3 s hata).

KIRPMA
    --start ve --end video zamanina gore (kaydin 0.saniyesinden itibaren).
    Saniye ("180") veya MM:SS / H:MM:SS ("3:00") kabul eder.
        --start 3:00 --end 6:00
    Sadece o aralik render edilir (ffmpeg de sadece o kismi isler, tum
    videoyu render etmekten cok daha hizli). _raw.mkv'ye dokunulmaz.

SABLON
    --template varsayilani "{dist_m:.0f} m".
    Kullanilabilir sutunlar (record_agent.py'nin yazdigi CSV basligi):
        dist_m, own_alt_rel, own_alt_amsl, own_lat, own_lon,
        tgt_alt, tgt_lat, tgt_lon, mode
    DIKKAT: mesafe sutununun adi "dist" degil "dist_m".
    Ornek:
        --template "{dist_m:.0f} m   irtifa {own_alt_rel:.0f} m"
    Alt satir icin \\N kullan (ASS'in satir sonu):
        --template "{dist_m:.0f} m\\Nirtifa {own_alt_rel:.0f} m"
"""
import argparse
import csv
import json
import os
import subprocess
import sys


def parse_ts(s):
    """'180' veya '3:00' veya '1:03:00' -> saniye (float)."""
    if ':' in s:
        parts = [float(p) for p in s.split(':')]
        while len(parts) < 3:
            parts.insert(0, 0.0)
        h, m, sec = parts
        return h * 3600 + m * 60 + sec
    return float(s)


def ass_time(t):
    """saniye -> ASS zaman formati H:MM:SS.cc"""
    t = max(0.0, t)
    h = int(t // 3600)
    m = int((t % 3600) // 60)
    s = t % 60
    return f"{h}:{m:02d}:{s:05.2f}"


def build_ass(rows, t0_wall, path, template, font, size, margin, hold_last):
    """CSV satirlarindan ASS altyazi dosyasi uret."""
    head = f"""[Script Info]
ScriptType: v4.00+
PlayResX: 1920
PlayResY: 1080
WrapStyle: 2
ScaledBorderAndShadow: yes

[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: OSD,{font},{size},&H00FFFFFF,&H000000FF,&H00000000,&H73000000,-1,0,0,0,100,100,0,0,3,6,0,3,{margin},{margin},{margin},1

[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""
    lines = []
    skipped = []
    bad_template = []
    n = len(rows)
    for i, r in enumerate(rows):
        start = float(r['t_wall']) - t0_wall
        if i + 1 < n:
            end = float(rows[i + 1]['t_wall']) - t0_wall
        else:
            end = start + hold_last
        if end <= 0:
            continue
        start = max(0.0, start)
        try:
            vals = {k: (float(v) if k != 'mode' else v)
                    for k, v in r.items() if v not in ('', None)}
        except ValueError as e:
            skipped.append(f"satir {i}: {e}")
            continue
        # --always modunda hedef raporu yoksa dist_m nan olur; "nan m" basma
        dm = vals.get('dist_m')
        if dm is not None and dm != dm:
            skipped.append(f"satir {i}: dist_m nan")
            continue
        try:
            text = template.format(**vals)
        except (KeyError, IndexError, ValueError) as e:
            bad_template.append(str(e))
            continue
        lines.append(f"Dialogue: 0,{ass_time(start)},{ass_time(end)},OSD,,0,0,0,,{text}")

    if bad_template and not lines:
        cols = ", ".join(k for k in rows[0].keys() if k != 't_wall')
        raise SystemExit(
            f"[X] Sablon hicbir satira uymadi: {bad_template[0]}\n"
            f"    Kullanilabilir sutunlar: {cols}\n"
            f"    Mesafe sutununun adi 'dist_m' (sadece 'dist' degil).")
    if skipped:
        print(f"[i] {len(skipped)} satir atlandi (ilk: {skipped[0]})")

    with open(path, 'w', encoding='utf-8') as f:
        f.write(head + "\n".join(lines) + "\n")
    return len(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('base', help='dosya onu, or: ~/recordings/chase_20260907_143012')
    ap.add_argument('--template', default='{dist_m:.0f} m')
    ap.add_argument('--font', default='DejaVu Sans')
    ap.add_argument('--font-size', type=int, default=54)
    ap.add_argument('--margin', type=int, default=40)
    ap.add_argument('--hold-last', type=float, default=2.0,
                    help='son satir kac saniye ekranda kalsin')
    ap.add_argument('--crf', type=int, default=18, help='dusuk = daha iyi kalite')
    ap.add_argument('--preset', default='medium')
    ap.add_argument('--encoder', default='libx264',
                    help='Orin NVENC destekliyorsa: h264_nvenc')
    ap.add_argument('--out', default=None)
    ap.add_argument('--keep-ass', action='store_true', help='ASS dosyasini silme')
    ap.add_argument('--offset', type=float, default=0.0,
                    help='elle senkron duzeltmesi (s). Yazi ERKEN gorunuyorsa '
                         'pozitif ver (gecikir), GEC gorunuyorsa negatif.')
    ap.add_argument('--start', type=parse_ts, default=None,
                    help='kirpma baslangici, video zamanina gore (s veya MM:SS)')
    ap.add_argument('--end', type=parse_ts, default=None,
                    help='kirpma bitisi, video zamanina gore (s veya MM:SS)')
    a = ap.parse_args()

    if a.start is not None and a.start < 0:
        sys.exit("[X] --start negatif olamaz.")
    if a.end is not None and a.start is not None and a.end <= a.start:
        sys.exit("[X] --end, --start'tan buyuk olmali.")

    base = os.path.expanduser(a.base)
    if base.endswith('_raw.mkv'):
        base = base[:-8]
    raw, csvp, jsonp = base + '_raw.mkv', base + '.csv', base + '.json'
    trimming = a.start is not None or a.end is not None
    if trimming and not a.out:
        s_tag = f"{a.start:.0f}" if a.start is not None else "0"
        e_tag = f"{a.end:.0f}" if a.end is not None else "son"
        default_out = f"{base}_osd_{s_tag}-{e_tag}s.mp4"
    else:
        default_out = base + '_osd.mp4'
    out = a.out or default_out

    for p in (raw, csvp):
        if not os.path.exists(p):
            sys.exit(f"[X] Bulunamadi: {p}")

    with open(csvp) as f:
        rows = [r for r in csv.DictReader(f) if r.get('t_wall')]
    if not rows:
        sys.exit("[X] CSV bos.")

    t0 = None
    if os.path.exists(jsonp):
        try:
            with open(jsonp) as f:
                meta = json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            print(f"[!] {os.path.basename(jsonp)} bozuk/yarim kalmis ({e}) — "
                  f"kayit muhtemelen Ctrl-C disinda bir sekilde kesildi.")
            meta = {}
        t0 = meta.get('wall_at_video_zero')
        if t0:
            method = meta.get('sync_method', 'ffmpeg_progress')
            if method == 'file_growth':
                print(f"[i] Senkron: dosya buyumesi ({meta.get('backend','?')} motoru) "
                      f"— ~0.3 s kayma olabilir, --offset ile duzeltilebilir")
            else:
                print(f"[i] Senkron: {meta.get('sync_samples', '?')} ornek (hassas)")
    if not t0:
        t0 = float(rows[0]['t_wall'])
        print("[!] Senkron bilgisi yok, ilk CSV satiri video basi kabul edildi "
              "(1-3 s kayma olabilir).")

    if a.offset:
        t0 = float(t0) - a.offset      # t0 kucuk -> yazi gec basar
        print(f"[i] Elle duzeltme: {a.offset:+.2f} s")

    if trimming:
        win_start = a.start or 0.0
        win_end = a.end if a.end is not None else float('inf')
        kept, prev = [], None
        for r in rows:
            vt = float(r['t_wall']) - t0     # bu satirin VIDEO zamani
            if vt < win_start:
                prev = r                      # pencere basinda gecerli kalsin diye
                continue
            if vt > win_end:
                break
            kept.append(r)
        if prev is not None:
            kept.insert(0, prev)
        if not kept:
            sys.exit(f"[X] {win_start:.0f}-{win_end:.0f} s araliginda CSV satiri yok.")
        rows = kept
        t0 = t0 + win_start   # ass zamanlari artik KIRPILMIS videoya gore, 0'dan baslar
        print(f"[i] Kirpma: {win_start:.0f}s - "
              f"{'son' if win_end == float('inf') else f'{win_end:.0f}s'}  "
              f"({len(rows)} satir)")

    assp = base + '.ass'
    n = build_ass(rows, t0, assp, a.template, a.font, a.font_size,
                  a.margin, a.hold_last)
    span = float(rows[-1]['t_wall']) - float(rows[0]['t_wall'])
    print(f"[i] {n} altyazi satiri, {span:.0f} s kapsam -> {os.path.basename(assp)}")

    # ASS yolunu filtre icin kacir (: ve \ ffmpeg filtre sozdiziminde ozel)
    esc = assp.replace('\\', '/').replace(':', r'\:').replace("'", r"\'")
    cmd = ['ffmpeg', '-hide_banner']
    if trimming and win_start > 0:
        # -i'den ONCE -ss: hizli VE karesi dogru (modern ffmpeg ikisini de verir)
        cmd += ['-ss', f'{win_start:.3f}']
    cmd += ['-i', raw, '-vf', f"subtitles='{esc}'", '-c:v', a.encoder]
    if a.encoder == 'libx264':
        cmd += ['-crf', str(a.crf), '-preset', a.preset]
    else:
        cmd += ['-b:v', '12M']
    if trimming and win_end != float('inf'):
        duration = win_end - win_start
        cmd += ['-t', f'{duration:.3f}']
    cmd += ['-pix_fmt', 'yuv420p', '-y', out]

    print(f"[i] Render basliyor -> {os.path.basename(out)}")
    r = subprocess.run(cmd)
    if r.returncode != 0:
        sys.exit("[X] ffmpeg hata verdi.")

    if not a.keep_ass:
        os.remove(assp)
    mb = round(os.path.getsize(out) / 1e6, 1)
    print(f"[OK] {out}  ({mb} MB)")
    print(f"[i] Arsiv bozulmadi: {os.path.basename(raw)}")


if __name__ == '__main__':
    main()
