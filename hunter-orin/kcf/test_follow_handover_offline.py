#!/usr/bin/env python3
"""
FOLLOW DEVRİ (8 Ekim) offline testi — FC / kamera / runtracker GEREKMEZ.

controller_guided.py giriş kenarı kararı (entry_decision / follow_handover_blocked):
  A) follow_gps uçarken (FC GUIDED) varsayılan: canlı otonoma GİRİLMEZ
  B) --follow-handover: GUIDED'dan girilir (tetik yine CH6 üçlü kapısı)
  C) dry-run hiç kilitlenmez (gölge log: komut gönderilmez)
  D) eski koşullar (want_auto / engaged / pending / pilot_escape) BİREBİR
  E) kilitlenince CH6 kapat-aç şart: pilot LOITER'a alır almaz saldırı BAŞLAMAZ
  F) main() bu kararı gerçekten kullanıyor mu (AST kablolama kontrolü)

Çalıştır:  python3 test_follow_handover_offline.py
"""
import ast
import itertools
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import controller_guided as cg   # noqa: E402

P = F = 0


def ck(cond, what):
    global P, F
    if cond:
        P += 1
    else:
        F += 1
        print(f"  [FAIL] {what}")


GUIDED, LOITER, ALTHOLD, NOGPS = 4, 5, 2, 20

# ---------------- A/B/C) kilit doğruluk tablosu ----------------
tbl = [
    # dry_run, allow, mode,   beklenen blocked
    (False, False, GUIDED,  True),
    (False, True,  GUIDED,  False),
    (True,  False, GUIDED,  False),
    (True,  True,  GUIDED,  False),
    (False, False, LOITER,  False),
    (False, False, ALTHOLD, False),
    (False, False, NOGPS,   False),
    (False, False, None,    False),     # mod bilinmiyor -> eski davranış
]
for dry, allow, mode, exp in tbl:
    ck(cg.follow_handover_blocked(dry, allow, mode) == exp,
       f"follow_handover_blocked(dry={dry}, allow={allow}, mode={mode}) != {exp}")
ck(cg.MODE_NUM['guided'] == 4 and cg.MODE_NUM['guided_nogps'] == 20, "mod numaralari")

# ---------------- D) eski koşullar birebir (GUIDED dışı modlarda) ----------------
for mode in (LOITER, ALTHOLD, None):
    for want, eng, pend, esc, dry, allow in itertools.product([False, True], repeat=6):
        d = cg.entry_decision(want, eng, pend, esc, dry, allow, mode)
        old_enter = want and not eng and not pend and not esc
        exp = None if not old_enter else ('simulate' if dry else 'engage')
        ck(d == exp, f"eski kosul bozuldu: mode={mode} want={want} eng={eng} "
                     f"pend={pend} esc={esc} dry={dry} allow={allow} -> {d}, beklenen {exp}")

# GUIDED'da: giriş koşulu yoksa yine None
ck(cg.entry_decision(False, False, False, False, False, False, GUIDED) is None, "want yokken None")
ck(cg.entry_decision(True, True, False, False, False, False, GUIDED) is None, "engaged iken None")
ck(cg.entry_decision(True, False, True, False, False, False, GUIDED) is None, "pending iken None")
ck(cg.entry_decision(True, False, False, True, False, False, GUIDED) is None, "pilot_escape iken None")
# GUIDED'da giriş koşulu varken
ck(cg.entry_decision(True, False, False, False, False, False, GUIDED) == 'blocked', "GUIDED canli varsayilan: blocked")
ck(cg.entry_decision(True, False, False, False, False, True, GUIDED) == 'engage', "GUIDED --follow-handover: engage")
ck(cg.entry_decision(True, False, False, False, True, False, GUIDED) == 'simulate', "GUIDED dry-run: simulate (golge)")


# ---------------- E) senaryo: main()'deki kuralın aynısı ----------------
def run(seq, dry=False, allow=False):
    """seq: (fc_mode, ch6_high, tracking) adımları. main() ile aynı sıra:
    CH6 düşükse pilot_escape sıfırlanır -> karar -> blocked ise pilot_escape."""
    pilot_escape = False
    engaged = False
    out = []
    for mode, ch6, trk in seq:
        if not ch6:
            pilot_escape = False
        want = ch6 and trk
        d = cg.entry_decision(want, engaged, False, pilot_escape, dry, allow, mode)
        if d == 'blocked':
            pilot_escape = True
        if d in ('engage', 'simulate'):
            engaged = True
        if engaged and not want:
            engaged = False
        out.append(d)
    return out


# Varsayılan: GUIDED'da kilit + CH6 HIGH -> blocked; pilot LOITER'a alır, CH6 hâlâ HIGH
seq = [(GUIDED, True, True), (GUIDED, True, True), (LOITER, True, True), (LOITER, True, True)]
r = run(seq)
ck(r[0] == 'blocked', "GUIDED + CH6 HIGH + kilit -> blocked")
ck('engage' not in r, "LOITER'a alinca CH6 hala HIGH: saldiri KENDILIGINDEN BASLAMAMALI")
# CH6 kapat-aç sonrası (artık LOITER = balon avı akışı) girilebilir
r = run(seq + [(LOITER, False, True), (LOITER, True, True)])
ck(r[-1] == 'engage', "CH6 kapat-ac sonrasi LOITER'da eskisi gibi girilmeli")
# --follow-handover: GUIDED'dan doğrudan devir
r = run([(GUIDED, True, True)], allow=True)
ck(r == ['engage'], "--follow-handover: GUIDED'dan devir")
# dry-run: GUIDED'da simüle (gölge log), kilit yok
r = run([(GUIDED, True, True), (LOITER, True, True)], dry=True)
ck(r[0] == 'simulate', "dry-run GUIDED: simulate")

# ---------------- F) AST kablolama: main() kararı gerçekten kullanıyor mu ----------------
src = open(os.path.join(HERE, 'controller_guided.py'), encoding='utf-8').read()
tree = ast.parse(src)
main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
parents = {}
for node in ast.walk(main):
    for ch in ast.iter_child_nodes(node):
        parents[ch] = node


def ancestors(n):
    while n in parents:
        n = parents[n]
        yield n


def is_decision_eq(test, val):
    return (isinstance(test, ast.Compare) and isinstance(test.left, ast.Name)
            and test.left.id == 'decision' and len(test.comparators) == 1
            and isinstance(test.comparators[0], ast.Constant) and test.comparators[0].value == val)


calls = [n for n in ast.walk(main) if isinstance(n, ast.Call)]
ed = [c for c in calls if isinstance(c.func, ast.Name) and c.func.id == 'entry_decision']
ck(len(ed) == 1, "main() entry_decision'i tam bir kez cagirmali")
if ed:
    srcs = [ast.unparse(a) for a in ed[0].args]
    ck('args.follow_handover' in srcs and 'fc.current_mode' in srcs and 'dry_run' in srcs,
       f"entry_decision argumanlari: {srcs}")

nogps = [c for c in calls
         if isinstance(c.func, ast.Attribute) and c.func.attr == 'set_mode_fast'
         and c.args and isinstance(c.args[0], ast.Name) and c.args[0].id == 'MODE_GUIDED_NOGPS']
ck(len(nogps) >= 1, "GUIDED_NOGPS komutu bulunamadi")
for c in nogps:
    ck(any(isinstance(a, ast.If) and is_decision_eq(a.test, 'engage') for a in ancestors(c)),
       f"GUIDED_NOGPS komutu (satir {c.lineno}) SADECE decision == 'engage' altinda olmali")

blocked_ifs = [n for n in ast.walk(main) if isinstance(n, ast.If) and is_decision_eq(n.test, 'blocked')]
ck(len(blocked_ifs) == 1, "decision == 'blocked' dali tek olmali")
if blocked_ifs:
    sets_escape = any(isinstance(s, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'pilot_escape'
                                                        for t in s.targets)
                      and isinstance(s.value, ast.Constant) and s.value.value is True
                      for s in blocked_ifs[0].body)
    ck(sets_escape, "blocked dalinda pilot_escape = True olmali (CH6 kapat-ac sarti)")

flag = [c for c in calls if isinstance(c.func, ast.Attribute) and c.func.attr == 'add_argument'
        and c.args and isinstance(c.args[0], ast.Constant) and c.args[0].value == '--follow-handover']
ck(len(flag) == 1 and any(k.arg == 'action' and getattr(k.value, 'value', None) == 'store_true'
                          for k in flag[0].keywords),
   "--follow-handover store_true olmali (varsayilan KAPALI)")

print("=" * 62)
print(f"SONUC: {P}/{P + F} {'GECTI' if F == 0 else 'KALDI'}")
sys.exit(1 if F else 0)
