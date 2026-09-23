#!/usr/bin/env bash
# =============================================================================
# repo_hazirla.sh — Desktop'taki Orin projelerini AVCI reposuna tasir (TEK SEFERLIK)
# =============================================================================
#   ./setup/repo_hazirla.sh            KONTROL: hicbir sey degistirmez, sadece rapor
#   ./setup/repo_hazirla.sh --uygula   tasi + kisayol + ortam kaydi + git commit
#
# --uygula adimlari:
#   1) Ic ice .git klasorleri -> ~/Desktop/_repo_yedek/<tarih>/   (eski gecmis korunur)
#   2) ~/Desktop/<proje>      -> <repo>/<proje>                   (TASIMA, kopya degil)
#   3) Eski yere kisayol: ~/Desktop/<proje> -> <repo>/<proje>
#      => eski yollar, scriptler, build klasoru AYNEN calismaya devam eder
#   4) Calisan ortamin kaydi  -> setup/referans_ortam/
#   5) git init + ozet + commit  (push elle; komutlari en sonda yazar)
# Geri alma: README.md > "Geri alma"
# =============================================================================
set -u

SETUP_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO="$(dirname "$SETUP_DIR")"
DESKTOP="${DESKTOP:-$HOME/Desktop}"
PROJELER=(hunter-orin orin-tiled-tracker)      # Orin tarafi. avci_follow (laptop) sonra.
YEDEK="$DESKTOP/_repo_yedek/$(date +%Y%m%d_%H%M%S)"
HARIC=(--exclude-dir=.git --exclude-dir=build --exclude-dir=mavlink --exclude-dir=__pycache__
       --exclude-dir=logs --exclude-dir=runs --exclude-dir=venv --exclude-dir=.venv)

UYGULA=0
case "${1:-}" in
  --uygula) UYGULA=1 ;;
  ""|--kontrol) ;;
  *) echo "Kullanim: $0 [--uygula]"; exit 1 ;;
esac

HATA=0; UYARI=0
bas() { echo -e "\n\033[1;36m== $* ==\033[0m"; }
ok()  { echo -e "  \033[32m✓\033[0m $*"; }
bi()  { echo -e "  • $*"; }
uy()  { echo -e "  \033[33m!\033[0m $*"; UYARI=$((UYARI+1)); }
ha()  { echo -e "  \033[31m✗\033[0m $*"; HATA=$((HATA+1)); }

# .gitignore ile ayni mantik (kontrol modunda "git'e girer mi?" tahmini icin)
girmez() {
  case "$1" in
    */hunter-orin/models/*.onnx) return 1 ;;   # istisna: hunter engine'leri bunlardan uretilir
    */build/*|*/logs/*|*/runs/*|*/__pycache__/*|*/venv/*|*/.venv/*) return 0 ;;
    *.engine|*.plan|*.trt|*.onnx|*.cache|*.pyc|*.bak) return 0 ;;
    *.csv|*.log|*.tlog|*.tlog.raw|*.BIN|*.bin) return 0 ;;
    *.mp4|*.mkv|*.avi|*.mov|*.h264|*.h265|*.264|*.265|*.hevc|*.ts) return 0 ;;
    *.whl|*.deb|*.zip|*.tar|*.tar.gz|*.tgz|*.7z) return 0 ;;
  esac
  return 1
}

# Bu unitede MAXN_SUPER'in nvpmodel ID'si (isimle bulunur; 0 = 15W!)
maxn_id() {
  local id
  id=$(sed -nE 's/.*POWER_MODEL[[:space:]]+ID=([0-9]+)[[:space:]]+NAME=MAXN_SUPER[[:space:]]*>.*/\1/p' /etc/nvpmodel.conf 2>/dev/null | head -1)
  echo "${id:-2}"
}
MAXN=$(maxn_id)
NVP_RE='nvpmodel[[:space:]]+-m[[:space:]]*0([^0-9]|$)'   # "-m 0" (bu unitede 15W)
NVP_SH=()

bul() {  # hunter-orin veya hunter_orin
  local c
  for c in "$DESKTOP/$1" "$DESKTOP/${1//-/_}"; do
    if [[ -e "$c" || -L "$c" ]]; then echo "$c"; return 0; fi
  done
  return 1
}

echo "Repo    : $REPO"
echo "Desktop : $DESKTOP"
[[ "$REPO" == "$DESKTOP/avci" ]] || uy "Repo $DESKTOP/avci degil (README yollari buna gore yazildi)"

# -----------------------------------------------------------------------------
bas "1) Kaynak klasorler"
declare -A KAYNAK=()
for p in "${PROJELER[@]}"; do
  if ! src=$(bul "$p"); then ha "$p: $DESKTOP altinda yok"; continue; fi
  if [[ -L "$src" ]]; then
    h=$(readlink -f "$src")
    if [[ "$h" == "$REPO/"* ]]; then ok "$p zaten repoda ($src -> $h)"
    else ha "$src bir kisayol -> $h (beklenmedik, elle bak)"; fi
    continue
  fi
  KAYNAK["$p"]="$src"
  ok "$p: $src  ($(du -sh "$src" 2>/dev/null | cut -f1))"
  if [[ -e "$REPO/$(basename "$src")" ]]; then ha "$REPO/$(basename "$src") zaten var (cakisma)"; fi
done
SRCS=("${KAYNAK[@]}")

if (( ${#SRCS[@]} == 0 )); then
  bas "OZET"
  echo "  Tasinacak proje yok (hepsi zaten repoda ya da bulunamadi). hata: $HATA"
  exit $(( HATA > 0 ))
fi

# -----------------------------------------------------------------------------
bas "2) Calisan surecler"
procs=$(pgrep -af 'kcfTracker|mavproxy|controller_guided|controller_orin|guided_sender|pipeline\.py' || true)
if [[ -n "$procs" ]]; then
  echo "$procs" | sed 's/^/    /'
  ha "Yukaridaki surecler calisiyor — tasimadan once kapat"
else
  ok "calisan HUNTER/tracker sureci yok"
fi

# -----------------------------------------------------------------------------
bas "3) Ic ice git depolari (--uygula: _repo_yedek/ altina tasinir)"
var=0
for src in "${SRCS[@]}"; do
  while IFS= read -r -d '' g; do
    var=1
    d=$(dirname "$g")
    r=$(git -C "$d" remote -v 2>/dev/null | awk '{print $2}' | sort -u | tr '\n' ' ')
    son=$(git -C "$d" log -1 --format='%h %cs %s' 2>/dev/null)
    deg=$(git -C "$d" status --porcelain 2>/dev/null | wc -l)
    bi "${g#"$DESKTOP"/}"
    echo "      remote: ${r:-yok} | son commit: ${son:--} | commit'lenmemis: $deg dosya"
  done < <(find "$src" -name .git -prune -print0)
done
if (( var )); then bi "Commit'lenmemis degisiklikler kaybolmaz: yeni reponun ilk commit'ine girer."
else ok "ic ice git yok"; fi

# -----------------------------------------------------------------------------
bas "4) Buyuk dosyalar (>10 MB)"
n=0
while IFS=$'\t' read -r s f; do
  [[ -n "$f" ]] || continue
  n=$((n+1)); mb=$((s/1048576)); kisa=${f#"$DESKTOP"/}
  if girmez "$f"; then                 bi "${mb} MB  $kisa  (git'e girmez)"
  elif (( s > 100*1048576 )); then     ha "${mb} MB  $kisa  -> GitHub 100 MB limiti! .gitignore'a ekle"
  elif (( s > 50*1048576 )); then      uy "${mb} MB  $kisa  (git'e girer; 50 MB ustu GitHub uyarir)"
  else                                 bi "${mb} MB  $kisa  (git'e GIRER)"; fi
done < <(for src in "${SRCS[@]}"; do
           find "$src" -name .git -prune -o -type f -size +10M -printf '%s\t%p\n'
         done | sort -rn)
(( n > 0 )) || ok "10 MB ustu dosya yok"

# -----------------------------------------------------------------------------
bas "5) Veri / model dosyalari"
for p in "${!KAYNAK[@]}"; do
  src=${KAYNAK[$p]}
  for ext in csv log tlog BIN bin mp4 mkv avi h264 h265 ts engine bak onnx pt; do
    read -r adet mb < <(find "$src" -name .git -prune -o -type f -name "*.$ext" -printf '%s\n' \
                         | awk '{n++; s+=$1} END{printf "%d %.1f\n", n, s/1048576}')
    (( adet > 0 )) || continue
    durum="git'e girmez"; [[ $ext == pt ]] && durum="git'e GIRER"; [[ $ext == onnx ]] && durum="git'e girmez (hunter-orin/models haric)"
    printf "  %-19s %-8s %5d dosya %9s MB   %s\n" "$p" ".$ext" "$adet" "$mb" "$durum"
  done
  amb=$(find "$src" \( -name .git -o -name build \) -prune -o -type f \( -name '*.ts' -o -name '*.bin' \) -printf '%P  (%s bayt)\n' | head -12)
  if [[ -n "$amb" ]]; then
    bi "$p .ts/.bin dosyalari (git'e girmez) — video/log ise sorun yok; KOD ya da TEST VERISI ise istisna gerekir:"
    echo "$amb" | sed 's/^/      /'
  fi
  cs=$(find "$src" -name .git -prune -o -type f -name '*.csv' -printf '%h\n' | sort | uniq -c | sort -rn | head -6)
  if [[ -n "$cs" ]]; then
    bi "$p CSV klasorleri (test fixture varsa .gitignore'a !istisna ekle):"
    echo "$cs" | sed "s|$DESKTOP/||; s/^/      /"
  fi
done

# -----------------------------------------------------------------------------
bas "6) Repo disina bakan yollar (yeni Orin'de olmayabilir)"
dis=0; say=0
while read -r adet yol; do
  [[ -n "${yol:-}" ]] || continue
  say=$((say+1))
  rel=$(sed -E 's#^(~|\$HOME|\$\{HOME\}|/home/[A-Za-z0-9_]+)/##' <<<"$yol")
  kul=$(sed -nE 's#^/home/([A-Za-z0-9_]+)/.*#\1#p' <<<"$yol")
  case "$rel" in
    Desktop/hunter-orin*|Desktop/hunter_orin*|Desktop/orin-tiled-tracker*|Desktop/avci*)
                            durum="ok (kisayolla calisir)" ;;
    .local/bin*|.local/lib*) durum="ok (pip kurulumu)" ;;
    *)                      durum="REPO DISI"; dis=1 ;;
  esac
  if [[ -n "$kul" && "$kul" != "avci2" ]]; then durum="$durum + eski kullanici '$kul'"; dis=1; fi
  printf "  %4sx  %-58s %s\n" "$adet" "$yol" "$durum"
done < <(grep -rIohE "${HARIC[@]}" \
           '(~|\$HOME|\$\{HOME\}|/home/[A-Za-z0-9_]+)/[A-Za-z0-9_.+-]+(/[A-Za-z0-9_.+-]+)?' \
           "${SRCS[@]}" 2>/dev/null | sort | uniq -c | sort -rn | head -40)
(( say > 0 )) || ok "ev dizinine sabit yol yok"
(( dis == 0 )) || uy "REPO DISI / eski kullanici yollari yeni Orin'de yok: repoya al ya da yolu duzelt"
eskiler=$(grep -rIoE "${HARIC[@]}" '/home/[A-Za-z0-9_]+/' "${SRCS[@]}" 2>/dev/null \
          | grep -v ':/home/avci2/$' | cut -d: -f1 | sort -u | head -8)
if [[ -n "$eskiler" ]]; then
  bi "baska kullanici yolu (/home/<baskasi>/) gecen dosyalar:"
  echo "$eskiler" | sed "s|$DESKTOP/||; s/^/      /"
fi
ulb=$(grep -rIn "${HARIC[@]}" '/usr/local/bin/' "${SRCS[@]}" 2>/dev/null | cut -c1-150 | head -8)
if [[ -n "$ulb" ]]; then
  echo "$ulb" | sed "s|$DESKTOP/||; s/^/    /"
  uy "/usr/local/bin/... bu Orin'e elle kurulmus (or. statik ffmpeg). Yeni Orin'de apt ile /usr/bin'e gelir -> komutu yolsuz cagir"
fi
sp=$(grep -rIn --include='*.py' "${HARIC[@]}" -E 'sys\.path\.(insert|append)' "${SRCS[@]}" 2>/dev/null | cut -c1-150 | head -8)
if [[ -n "$sp" ]]; then bi "sys.path eklemeleri (repo disini gosteriyorsa kirilir):"; echo "$sp" | sed "s|$DESKTOP/||; s/^/      /"; fi

# -----------------------------------------------------------------------------
bas "7) Gizli bilgi taramasi (token / sifre)"
desen=$'ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|(password|passwd|secret|token|api_?key)[[:space:]]*[:=][[:space:]]*["\'][^"\']{4,}'
gz=$(grep -rInEi "${HARIC[@]}" "$desen" "${SRCS[@]}" 2>/dev/null | cut -c1-150 | head -15)
dz=$(find "${SRCS[@]}" -name .git -prune -o \( -name '.git-credentials' -o -name 'id_rsa*' \
       -o -name 'id_ed25519*' -o -name '*.pem' -o -name '.env' \) -print 2>/dev/null)
if [[ -n "$gz$dz" ]]; then
  [[ -n "$gz" ]] && echo "$gz" | sed "s|$DESKTOP/||; s/^/    /"
  [[ -n "$dz" ]] && echo "$dz" | sed 's/^/    /'
  uy "Kontrol et — gercek token/sifre ise dosyadan cikar (repo private olsa da)"
else
  ok "token/sifre izi yok"
fi

# -----------------------------------------------------------------------------
bas "8) Proje kurulum dosyalari"
if [[ -n "${KAYNAK["hunter-orin"]:-}" ]]; then
  h=${KAYNAK["hunter-orin"]}
  if [[ -f "$h/scripts/setup_orin.sh" ]]; then
    ok "hunter-orin/scripts/setup_orin.sh ($(wc -l < "$h/scripts/setup_orin.sh") satir, $(date -r "$h/scripts/setup_orin.sh" +%F))"
    grep -qi 'mavproxy' "$h/scripts/setup_orin.sh" \
      || bi "setup_orin.sh MAVProxy kurmuyor -> kurulum_orin.sh ayrica kurar"
    if grep -qi 'mavlink-router' "$h/scripts/setup_orin.sh"; then
      bi "setup_orin.sh hala mavlink-router derliyor (artik MAVProxy kullaniliyor; zararsiz, sadece yavas)"
    fi
  else
    uy "hunter-orin/scripts/setup_orin.sh yok"
  fi
  for f in "$h/scripts/requirements.txt" "$h/requirements.txt"; do
    if [[ -f "$f" ]]; then bi "${f#"$h"/}: $(grep -Ev '^[[:space:]]*(#|$)' "$f" | tr '\n' ' ')"; fi
  done
fi
if [[ -n "${KAYNAK["orin-tiled-tracker"]:-}" ]]; then
  t=${KAYNAK["orin-tiled-tracker"]}
  pts=$(cd "$t" && ls -- *.pt 2>/dev/null | tr '\n' ' ')
  if [[ -n "$pts" ]]; then ok "model: $pts"; else uy "orin-tiled-tracker'da .pt yok (yeni Orin'de engine uretilemez)"; fi
fi

# 'nvpmodel -m 0' (bu unitede 15W): .sh'lerde yorum OLMAYAN satirlar --uygula'da duzeltilir
mapfile -t NVP_SH < <(grep -rlE "${HARIC[@]}" --include='*.sh' "$NVP_RE" "${SRCS[@]}" 2>/dev/null \
  | while IFS= read -r f; do
      if grep -vE '^[[:space:]]*#' "$f" | grep -E "$NVP_RE" >/dev/null; then echo "${f#"$DESKTOP"/}"; fi
    done)
for f in "${NVP_SH[@]}"; do
  uy "$f: 'nvpmodel -m 0' (bu unitede 0=15W!) -> --uygula '-m $MAXN' (MAXN_SUPER) yapar"
done
nvp_dok=$(grep -rlE "${HARIC[@]}" --include='*.md' --include='*.py' "$NVP_RE" "${SRCS[@]}" 2>/dev/null)
if [[ -n "$nvp_dok" ]]; then
  bi "'nvpmodel -m 0' gecen dokuman/kod (otomatik degismez, elle bak; 0=15W):"
  echo "$nvp_dok" | sed "s|$DESKTOP/||; s/^/      /"
fi

# -----------------------------------------------------------------------------
bas "9) Git / GitHub"
ok "$(git --version)"
em=$(git -C "$REPO" config user.email 2>/dev/null)
if [[ -n "$em" ]]; then ok "git kimlik (commit'ler bu adla): $(git -C "$REPO" config user.name 2>/dev/null) <$em>"
else uy "git user.email ayarli degil (--uygula sorar)"; fi
if [[ -f "$HOME/.ssh/id_ed25519.pub" || -f "$HOME/.ssh/id_rsa.pub" ]]; then ok "SSH anahtari var"
else bi "SSH anahtari yok (push oncesi olusturulacak)"; fi

# -----------------------------------------------------------------------------
bas "OZET"
echo "  hata: $HATA   uyari: $UYARI"
if (( UYGULA == 0 )); then
  if (( HATA )); then echo "  Once hatalari coz, sonra: ./setup/repo_hazirla.sh --uygula"
  else echo "  Uygun gorunuyorsa: ./setup/repo_hazirla.sh --uygula"; fi
  exit 0
fi
if (( HATA )); then echo "  Hata varken uygulanmaz."; exit 1; fi

# =============================================================================
bas "UYGULA"
for p in "${!KAYNAK[@]}"; do
  echo "    ${KAYNAK[$p]}  ->  $REPO/$(basename "${KAYNAK[$p]}")   (+ eski yere kisayol)"
done
if [[ "${EVET:-}" != 1 ]]; then
  read -rp "  Devam? [e/H] " c
  [[ "$c" =~ ^[eEyY]$ ]] || { echo "  Iptal, hicbir sey degismedi."; exit 0; }
fi

# 1) ic ice .git -> yedek
for src in "${SRCS[@]}"; do
  mapfile -d '' gits < <(find "$src" -name .git -prune -print0)
  for g in "${gits[@]}"; do
    rel=${g#"$DESKTOP"/}
    { mkdir -p "$YEDEK/$(dirname "$rel")" && mv "$g" "$YEDEK/$rel"; } || { ha "tasinamadi: $g"; exit 1; }
    ok ".git yedege: $rel -> ${YEDEK#"$DESKTOP"/}/$rel"
  done
done

# 2) tasi + kisayol
for src in "${SRCS[@]}"; do
  ad=$(basename "$src")
  mv "$src" "$REPO/$ad"     || { ha "tasinamadi: $src"; exit 1; }
  ln -s "$REPO/$ad" "$src"  || { ha "kisayol olusmadi: $src"; exit 1; }
  ok "$src -> $REPO/$ad  (kisayol birakildi)"
done

# 2b) scriptlerdeki guc modu hatasi (0=15W) -> MAXN_SUPER (yorum satirlarina dokunmaz)
for rel in "${NVP_SH[@]}"; do
  if sed -E -i "/^[[:space:]]*#/!s/(nvpmodel[[:space:]]+-m[[:space:]]*)0([^0-9]|$)/\1$MAXN\2/g" "$REPO/$rel"; then
    ok "$rel: nvpmodel -m 0 -> -m $MAXN (MAXN_SUPER)"
  else
    uy "$rel duzeltilemedi — elle: -m 0 -> -m $MAXN"
  fi
done

# 3) calisan ortamin kaydi
AVCI_ICERDEN=1 bash "$SETUP_DIR/ortam_kaydet.sh" "$SETUP_DIR/referans_ortam" || uy "ortam kaydi eksik kaldi"

# 4) git
cd "$REPO" || exit 1
if [[ ! -d .git ]]; then git init -q -b main || { ha "git init"; exit 1; }; fi
if [[ -z "$(git config user.email 2>/dev/null)" ]]; then
  read -rp "  git e-posta: " em
  read -rp "  git isim [hasanucas]: " nm
  git config --global user.email "$em"
  git config --global user.name "${nm:-hasanucas}"
fi
chmod +x setup/*.sh
git add -A || { ha "git add"; exit 1; }

liste=$(git ls-files -z | xargs -0 -r stat -c $'%s\t%n' | sort -rn)
echo "  Repoda: $(git ls-files | wc -l) dosya, $(awk -F'\t' '{s+=$1} END{printf "%.1f", s/1048576}' <<<"$liste") MB"
echo "  En buyuk 12:"
head -12 <<<"$liste" | awk -F'\t' '{printf "    %7.1f MB  %s\n", $1/1048576, $2}'
cok=$(awk -F'\t' '$1 > 100*1048576 {print $2}' <<<"$liste")
if [[ -n "$cok" ]]; then
  ha "100 MB ustu dosya (GitHub kabul etmez):"; echo "$cok" | sed 's/^/    /'
  echo "  -> .gitignore'a ekle, 'git rm --cached <dosya>', sonra: git commit"
  exit 1
fi

if git rev-parse -q --verify HEAD >/dev/null && git diff --cached --quiet; then
  ok "degisiklik yok — commit gereksiz"
else
  git commit -q -m "AVCI Orin: hunter-orin + orin-tiled-tracker ($(date +%F))" || { ha "commit"; exit 1; }
  ok "commit: $(git log --oneline -1)"
fi

bas "BITTI — bu Orin eskisi gibi calisir (Desktop yollari kisayol). Simdi GitHub:"
cat <<EOF
  1) github.com/new -> isim: avci, PRIVATE, README/.gitignore EKLEME (bos repo)
  2) SSH anahtari (bu Orin'de yoksa):
       ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519 -C "\$(hostname)"
       cat ~/.ssh/id_ed25519.pub     # GitHub > Settings > SSH and GPG keys > New SSH key
       ssh -T git@github.com         # "Hi hasanucas!" gormelisin
  3) cd $REPO
     git remote add origin git@github.com:hasanucas/avci.git
     git push -u origin main
EOF
