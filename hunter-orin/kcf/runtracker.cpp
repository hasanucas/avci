// =====================================================================
// 9 EYLUL — VIDEO/LINK SURUMU (1080p + uzun menzil)
//  Amac: ayni link kapasitesinde daha az bitle daha iyi goruntu; uzakta kayip
//  baslayinca BOZULMA / ILERI SARMA yerine YUMUSAMA; encoder yavaslarsa
//  gecikmenin BIRIKMEMESI. Tracker/RC/MAVLink/FRAME/OSD mantigi AYNEN korundu.
//
//  DEGISIKLIKLER ([EYLUL] etiketiyle isaretli):
//   [V1] x264: ultrafast -> --preset (varsayilan superfast); intra-refresh
//        (IDR patlamasi yok, tam tazeleme 1 s; --idr ile eski GOP'a don);
//        --main ile cabac=1 + Main profile (SIYI FPV kabul ederse ~%10-15 bit).
//   [V2] appsrc kuyrugu SINIRLI: GStreamer>=1.20 -> max-buffers=1 leaky-type=
//        downstream (en taze kare kazanir); eski surum -> kaynakta atlama.
//        Drop sayaci = "encoder geride" sensoru. Encoder sonrasi queue LEAKY DEGIL.
//   [V3] BGR->I420 OpenCV'de (pipeline'daki videoconvert kalkti), proc.clone() kalkti.
//   [V4] Calisma aninda bitrate (x264enc "bitrate" PLAYING'de degisir):
//        bitrate = min(RSSI katmani [--rssi-bitrate], RTCP kayip kontrolcusu
//        [--auto-bitrate], CHn tavani [--ch-bitrate N]). Hicbiri acik degilse
//        --bitrate sabit kalir (eski davranis). Aralik: --br-min/--br-max.
//   [V5] Yeni client PLAY deyince IDR zorlanir (intra-refresh'te baska IDR yok).
//   [V6] Her SETUP'ta client'in Transport'u loglanir; --udp-only TCP'yi reddeder
//        (TCP = kesintide tampon birikimi = ileri sarma; RTCP kayip 0 gorunur).
//   [V7] 1 Hz "[LINK]" logu: rssi%, kbps, drop, RTCP kayip -> kalibrasyon icin.
//
//  DERLEME: pkg-config listesine gstreamer-video-1.0 (force-key-unit event) ve
//            gstreamer-rtsp-1.0 (SETUP header okuma) EKLE.
//  KALIBRASYON: g_tiers tablosu YER TUTUCU; ucus [LINK] loglarina gore doldur.
// =====================================================================
// =====================================================================
// 11 HAZIRAN — OTORITER BIRLESIK SURUM (onceki cataldan birlestirme)
//  Bu dosya = OSD/RSSI duzenlemeleri + 10 Haziran RTSP audit'i + RECONNECT FIX.
//  (Not: 10 Haz duzeltmeleri onceki kopyada kaybolmustu; burada geri geldi.)
//
//  RECONNECT FIX (container'da gst-rtsp-server uzerinde kanitlandi):
//   SEMPTOM: QGC kapa-ac -> goruntu gelmiyor, runtracker restart gerekiyordu.
//   KOK NEDEN: set_reusable(TRUE) + FLUSHING'de g_appsrc=null + "configure
//   yalnizca media INSA edilirken ateslenir" uclusu. Kopusta referans silinir,
//   yeniden baglanista reusable media configure ATESLEMEDEN canlanir,
//   g_appsrc sonsuza dek null kalir -> kare gitmez.
//   TEST KANITI (ayni mantigin birebir kopyasiyla):
//     mevcut: 1.baglanti=149 veri, 2.baglanti=0  (bug uretildi)
//     fix   : 1.baglanti=148 veri, 2.baglanti=150 (cozuldu, 3 kopusta da calisti)
//   COZUM: reusable KALDIRILDI (her baglanti yeni construct -> configure
//   ateslenir), configure bayat referansi TAZELER (erken-return kalkti).
//   Ayrica: factory'de "media-unprepared" diye bir sinyal YOK (hic calismiyordu);
//   dogru yer media'nin "unprepared" sinyali -> configure icinde baglanir.
//
//  10 HAZ AUDIT (geri getirildi):
//   [1] Olu token temizligi: profile/level/nal-hrd/min-keyint/weightp
//       x264enc property DEGIL (gst-inspect dogrulamali; davranis ayni).
//   [2] Monotonik PTS (RTCP clock-skew fix); do-timestamp=false korundu.
//   [3] get_element ref sizintisi kapatildi.
//  EEE kok neden notu icin onceki changelog'a / README'ye bak.
// =====================================================================
// =====================================================================
// hasaaaaan runtracker.cpp — ORCUS gercek donanim surumu
// DEGISIKLIKLER (mavlink-router mimarisi):
//   1) MAVLink kaynagi SERI -> UDP (mavlink-router endpoint, default 14550)
//      --serial hala calisir ama router KAPALIYKEN (dogrudan seri).
//   2) TCP FRAME server (port 9999) eklendi — camera_bridge ile BIREBIR
//      protokol: "FRAME <id> <valid> <cx> <cy> <w> <h> <ang_x> <ang_y> <state>"
//      Python beyin (controller.py) bbox'i buradan okur.
//   3) ang_x/ang_y = (cx-0.5)*HFOV, (cy-0.5)*VFOV — FOV ayarlanabilir (--hfov/--vfov)
//   4) ArduPilot USB'de stream otomatik gelmedigi icin runtracker kendisi
//      REQUEST_DATA_STREAM yolluyor (Python/QGC olmadan da RC/telemetri akar).
//   RC/tracker mantigi (CH5/CH12/CH13 + swap), OSD, RTSP AYNEN korundu.
// =====================================================================
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <csignal>
#include <cstring>
#include <chrono>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <cstdint>
#include <memory>
#include <vector>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// OpenCV
#include <opencv2/opencv.hpp>

// TCP FRAME server + UDP MAVLink (mavlink-router) icin
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <iomanip>

// [ORT] Tracker arayuzu (KCF adaptorunu ve kcftracker.hpp'yi icerir)
#include "tracker_backend.hpp"
#ifdef WITH_ORTRACK
#include "ortrack_trt.hpp"
#include "yolo_trt.hpp"
#include "simple_lock.hpp"
#endif

// MAVLink
#include "../mavlink/common/mavlink.h"

// GStreamer RTSP
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>     // [EYLUL] force-key-unit event (link: gstreamer-video-1.0)

// OSD
#include "classic_ardupilot_osd.h"

// ===================== GLOBALS ======================

static std::atomic<bool> keepRunning(true);

// RC deÄŸerleri (1â€“16 kanallar)
static uint16_t rc_values[16];
static std::mutex rcMutex;

// MAVLink
static int mavlink_serial_fd = -1;
static bool g_useUdp = true;                 // varsayilan: UDP (mavlink-router); --serial ile kapatilir
static struct sockaddr_in g_routerAddr;      // (kullanim: connect edilen socket)

// TCP FRAME server (Python beyin — camera_bridge protokolu)
static std::atomic<int> g_frameServerFd(-1);
static std::atomic<int> g_frameClientFd(-1);
static uint32_t g_frameId = 0;

// ===== [ORT/HYB] TRACKER SECIMI =====
// Hicbir bayrak yoksa KCF ve kod yolu eskisiyle AYNI.
static bool        g_useOrtrack = false;   // --ortrack  : manuel ROI + ORTrack
static bool        g_hybrid     = false;   // --hybrid     : YOLO + ORTrack
static bool        g_hybridKcf  = false;   // --hybrid-kcf : YOLO + KCF
static bool        g_detectOnly = false;   // --detect-only: tracker YOK, sadece YOLO
static int         g_detHoldFrames = 15;   // tespit kaybolunca kac kare tutulur
// Kullanici bu esikleri ACIKCA verdi mi? --hybrid-kcf varsayilanlari
// ezmesin diye izliyoruz (komut satiri sirasindan bagimsiz olsun).
static bool        g_verifyIvSet = false, g_lowSet = false, g_recoverSet = false;
static std::string g_ortEngine  = "../../models/ORTrack_ep0300-fp16.engine";
static std::string g_yoloEngine = "../../models/UAV-YOLOv11m-768.engine";
static float       g_ortMinScore = 0.0f;   // 0 = kapali
static double      g_ortMinBox   = 4.0;    // orin-camera-quality-768.yaml
static int         g_ortLostLimit = 5;     // sadece MANUEL mod (hibritte manager karar verir)
static bool        g_ortVerbose  = false;
static float       g_yoloConf    = 0.35f;
static float       g_yoloIoU     = 0.5f;
static bool        g_hybDrawDets = false;  // --hyb-draw-dets
static bool        g_procAtOut   = false;  // --proc-out : ESKI davranis
static double      g_hybDetMs    = 0.0;
static std::string g_badge       = "KCF";
static bool        g_badgeActive = false;
// [OLCUM] 1 Hz penceresindeki kare istatistigi
static long long   g_loopFrames  = 0;
static double      g_loopSumMs   = 0.0;
static double      g_loopMaxMs   = 0.0;
static long long   g_loopSlow    = 0;
// [HYB] 1 Hz penceresinde YOLO istatistigi: kac cagri / kac HAM kutu.
// yolo=N/0 ise sorun manager'da DEGIL, dedektorde veya esikte demektir.
static long long   g_hybDetCalls = 0;
static long long   g_hybDetRaw   = 0;
#ifdef WITH_ORTRACK
// [LOCK] --hybrid-kcf artik TargetManager KULLANMAZ. "Bir kez kilitle,
// karisma" makinesi (simple_lock.hpp) kullanir: kilidi asla kendiliginden
// birakmaz, sadece baska yerde daha iyi hedef kaniti olursa oraya gecer.
static slock::Config g_lockCfg;
#endif
// [SMOOTH] Kutu boyutu yumusatma. ORTrack w/h'yi HER KAREDE sifirdan tahmin
// eder (size_map), hicbir zamansal sonumleme yoktur. KCF cok olcekli modda
// sonumler, ORTrack sonumlemez. bbox yakinlik vekili olarak kullanildigi icin
// titresim beyin tarafini bozar. 0/1 = kapali.
static int         g_boxSmooth   = 0;      // --box-smooth N
static std::string g_trackCsv;             // --track-csv YOL

// Son N degerin MEDYANI. Ortalama yerine medyan: tek karelik sicramalari
// tamamen siler, SUREKLI degisimde (yaklasma) gecikme yaratmaz.
class MedianWin {
public:
    void setN(int n) { n_ = std::max(1, n); buf_.clear(); }
    void clear() { buf_.clear(); }
    double push(double v) {
        buf_.push_back(v);
        if ((int)buf_.size() > n_) buf_.erase(buf_.begin());
        std::vector<double> t(buf_.begin(), buf_.end());
        std::sort(t.begin(), t.end());
        return t[t.size() / 2];
    }
private:
    int n_ = 1;
    std::vector<double> buf_;
};

// steadyMs'in ns kardesi — TargetManager monotonik ns bekler.
static inline long long steadyNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

#ifdef WITH_ORTRACK
// ===== [HYB] Manager varsayilanlari = configs/orin-camera-quality-768.yaml =====
// Bu degerler SENIN olctugun bir arizayi hedefliyor (kutunun kareyi yutmasi),
// ama config basligindaki not gecerli: "Unvalidated: no annotated evaluation
// exists yet". --hyb-legacy ile eski orin-camera-ortrack-trt.yaml degerlerine donulur.
static hybrid::ManagerConfig g_mgrCfg;
static void applyLegacyManagerCfg() {
    g_mgrCfg.detection_accept_confidence = 0.5;
    g_mgrCfg.tracker_low_score           = 0.2;
    g_mgrCfg.tracker_recover_score       = 0.3;
    g_mgrCfg.max_box_growth              = 0.0;   // kapali
    g_ortMinBox                          = 10.0;
}
// [HYB-KCF] KCF tepe yaniti ORTrack'in Hann'li tepesiyle AYNI OLCEKTE
// DEGILDIR ve henuz kalibre edilmedi. Kalibre edilmemis bir esik uydurmak
// yerine SKOR KAPISINI KAPATIYORUZ: kayip karari tamamen YOLO
// dogrulamasindan gelir (confident_track_without_yolo_support), artik
// box_scale_drift ve dejenere kutu kontrolu. Skor CSV'ye ve rozete yazilir;
// dagilimi olctukten sonra --hyb-low-score / --hyb-recover-score ile acilir.
static void applyKcfManagerCfg() {
    g_mgrCfg.tracker_low_score     = 0.0;   // zayif yol KAPALI
    g_mgrCfg.tracker_recover_score = 0.0;
}
static void applyQualityManagerCfg() {
    g_mgrCfg.detection_accept_confidence = 0.4;
    g_mgrCfg.tracker_low_score           = 0.4;
    g_mgrCfg.tracker_recover_score       = 0.5;
    g_mgrCfg.max_box_growth              = 2.5;
    g_ortMinBox                          = 4.0;
}
#endif

// Kamera FOV (ang_x/ang_y derece donusumu) — SIYI gercek degerine gore --hfov/--vfov ayarla
static float CAMERA_HFOV = 80.0f;
static float CAMERA_VFOV = 50.534f;

// RTSP OUT globals
static GstElement *g_appsrc = nullptr;
static GMainLoop *g_main_loop = nullptr;
static std::thread g_gstThread;
static std::mutex g_gstMutex;
static std::atomic<bool> g_gstRunning{false};
static gint64       g_pts_base_us = -1;   // monotonik PTS tabani (us)
static guint g_fps_n = 30, g_fps_d = 1;

// ===== [EYLUL] VIDEO / LINK KONTROL =====
static GstElement *g_x264        = nullptr;   // calisma aninda bitrate + IDR zorlama
static GObject    *g_rtpSession  = nullptr;   // RTCP Receiver Report istatistikleri
static int         g_launchKbps  = 1700;      // launch string'e yazilan baslangic (--bitrate)
static int         g_curKbps     = 0;         // encoder'da su an olan (g_gstMutex altinda)
static int         g_brMin       = 800;       // --br-min  (kbps)
static int         g_brMax       = 5000;      // --br-max  (kbps)
static int         g_chBitrate   = 0;         // --ch-bitrate N : pilot tavani (0 = kapali)
static std::string g_preset      = "superfast";   // --preset ultrafast|superfast|veryfast|faster
static bool        g_mainProfile = false;     // --main    : cabac=1 + profile=main
static bool        g_intraRefresh= true;      // --idr     : klasik IDR GOP'a don
static bool        g_udpOnly     = false;     // --udp-only: sunucu TCP transportu reddeder
static bool        g_rssiAuto    = false;     // --rssi-bitrate : RSSI katmanlari
static bool        g_appsrcLeaky = false;     // appsrc leaky-type destegi (otomatik tespit)
static std::atomic<bool>     g_autoBitrate{false};     // --auto-bitrate : RTCP kayip kontrolcusu
static std::atomic<uint32_t> g_dropCount{0};           // encoder geride kaldiginda artar
static std::atomic<int>      g_rtcpKbps{0};            // RTCP kontrolcusunun onerisi (0 = henuz yok)
static std::atomic<int>      g_lastLossPermille{0};    // son RR kayip orani (binde)
static std::atomic<int64_t>  g_lastRssiMs{0};          // son RC_CHANNELS zamani (0 = hic gelmedi)

// RSSI% -> bitrate katmanlari. down: bu katmandan ASAGI dusme esigi,
// up: bu katmana YUKARI cikma esigi (histerezis). SAYILAR YER TUTUCU:
// kalibrasyon ucusunda "[LINK] rssi" ile bozulmanin basladigi degerlerden doldur.
struct Tier { int down; int up; int kbps; };
static std::vector<Tier> g_tiers = {
    { -1, -1, 1000 },   // 0  taban (her zaman erisilebilir)
    { 15, 22, 1500 },   // 1
    { 30, 37, 2000 },   // 2
    { 50, 57, 3000 },   // 3
    { 70, 77, 4000 },   // 4
    { 85, 92, 5000 },   // 5
};

static int64_t steadyMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Telemetry (OSD için basit)
static TelemetryData g_telemetry;
static std::mutex g_telemMutex;
static ClassicArduPilotOSD g_classicOSD(OSDSize::MEDIUM);

// ================== SIGNAL HANDLER ==================

void signalHandler(int)
{
    keepRunning = false;
    // accept()/recv() bloklarini cozmek icin FRAME socketlerini kapat
    int s = g_frameServerFd.exchange(-1);
    if (s >= 0) { shutdown(s, SHUT_RDWR); close(s); }
    int c = g_frameClientFd.exchange(-1);
    if (c >= 0) { shutdown(c, SHUT_RDWR); close(c); }
}

// ================== SERIAL OPEN =====================

int openSerial(const std::string &port, int baudrate)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Serial open failed: " << port << "  err=" << strerror(errno) << "\n";
        return -1;
    }

    termios tio {};
    if (tcgetattr(fd, &tio) != 0) {
        std::cerr << "tcgetattr failed\n";
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);

    speed_t speed;
    switch (baudrate) {
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:     speed = B115200; break;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cflag |= (CLOCAL | CREAD);

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        std::cerr << "tcsetattr failed\n";
        close(fd);
        return -1;
    }

    return fd;
}

// ============== MAVLINK UDP OPEN (mavlink-router) ==============
// Router Mode=Server: once biz bir paket yollayinca adresimizi ogrenir.
// connect() ile read()/write() seri gibi calisir (router'dan gelen filtrelenir).
int openMavlinkUDP(const std::string &host, int port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "UDP socket failed: " << strerror(errno) << "\n";
        return -1;
    }
    memset(&g_routerAddr, 0, sizeof(g_routerAddr));
    g_routerAddr.sin_family = AF_INET;
    g_routerAddr.sin_port   = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &g_routerAddr.sin_addr) <= 0) {
        std::cerr << "UDP bad host: " << host << "\n";
        close(fd);
        return -1;
    }
    if (connect(fd, (struct sockaddr*)&g_routerAddr, sizeof(g_routerAddr)) < 0) {
        std::cerr << "UDP connect failed: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }
    int flags = fcntl(fd, F_GETFL, 0);     // serial O_NONBLOCK ile ayni davranis
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

// Router'a heartbeat + stream istegi yolla.
//   - heartbeat: Mode=Server adresimizi ogrensin / route canli kalsin
//   - REQUEST_DATA_STREAM: ArduPilot USB'de stream'i otomatik gondermiyor;
//     RC + telemetri (OSD) icin acikca istiyoruz. target=0 (broadcast).
void routerKeepalive()
{
    if (mavlink_serial_fd < 0) return;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len;

    mavlink_msg_heartbeat_pack(254, 190, &msg,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    if (write(mavlink_serial_fd, buf, len) < 0) { /* EAGAIN normal */ }

    const uint8_t streams[] = {
        MAV_DATA_STREAM_RC_CHANNELS,      // RC (CH5/CH12/CH13)
        MAV_DATA_STREAM_EXTENDED_STATUS,  // SYS_STATUS/GPS (OSD)
        MAV_DATA_STREAM_EXTRA1,           // ATTITUDE (OSD)
        MAV_DATA_STREAM_EXTRA2,            // VFR_HUD (OSD)
        // [PATCH-RSSI] GLOBAL_POSITION_INT bu grupta;
        // istenmezse OSD irtifasi guncellenmez.
        MAV_DATA_STREAM_POSITION
    };
    for (uint8_t s : streams) {
        mavlink_msg_request_data_stream_pack(254, 190, &msg, 0, 0, s, 10, 1);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        if (write(mavlink_serial_fd, buf, len) < 0) { /* EAGAIN normal */ }
    }
}

// ============== TCP FRAME SERVER (Python beyin) ==============
// camera_bridge.cpp ile BIREBIR protokol. Sadece GONDERIR (komut almaz —
// ROI/track kontrolu MK15 RC'sinden, runtracker zaten okuyor).
int startFrameServer(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { std::cerr << "[FRAME] socket: " << strerror(errno) << "\n"; return -1; }
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;   // 0.0.0.0 — localhost da, agdaki PC de baglanir
    addr.sin_port        = htons(port);
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[FRAME] bind " << port << ": " << strerror(errno) << "\n";
        close(fd); return -1;
    }
    if (listen(fd, 1) < 0) {
        std::cerr << "[FRAME] listen: " << strerror(errno) << "\n";
        close(fd); return -1;
    }
    std::cout << "[FRAME] TCP port " << port << " dinleniyor (Python beyin)\n";
    return fd;
}

void frameServerThread(int port)
{
    int server_fd = startFrameServer(port);
    if (server_fd < 0) return;
    g_frameServerFd.store(server_fd);

    while (keepRunning.load()) {
        std::cout << "[FRAME] Client bekleniyor...\n";
        struct sockaddr_in caddr; socklen_t clen = sizeof(caddr);
        int cfd = accept(server_fd, (struct sockaddr*)&caddr, &clen);
        if (cfd < 0) { if (!keepRunning.load()) break; continue; }

        char ipbuf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &caddr.sin_addr, ipbuf, sizeof(ipbuf));
        std::cout << "[FRAME] Client baglandi: " << ipbuf << "\n";

        int yes = 1;
        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        int sndbuf = 32 * 1024;
        setsockopt(cfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
        g_frameClientFd.store(cfd);

        // Sadece disconnect tespiti (komut yok)
        char tmp[64];
        while (keepRunning.load()) {
            ssize_t n = recv(cfd, tmp, sizeof(tmp), 0);
            if (n <= 0) { std::cout << "[FRAME] Client ayrildi\n"; break; }
        }
        int old = g_frameClientFd.exchange(-1);
        if (old >= 0) close(old);
    }
    int s = g_frameServerFd.exchange(-1);
    if (s >= 0) close(s);
}

// FRAME mesajini client'a yolla (non-blocking; client yavassa drop)
bool sendFrameToClient(uint32_t fid, bool valid,
                       float cx, float cy, float bw, float bh,
                       float ang_x, float ang_y, const char* state)
{
    int fd = g_frameClientFd.load();
    if (fd < 0) return false;
    std::ostringstream oss;
    oss << "FRAME " << fid << " " << (valid ? 1 : 0) << " "
        << std::fixed << std::setprecision(5)
        << cx << " " << cy << " " << bw << " " << bh << " "
        << ang_x << " " << ang_y << " " << state << "\n";
    std::string s = oss.str();
    ssize_t n = send(fd, s.c_str(), s.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
    return n > 0;
}

// ================== MAVLINK RC THREAD ==============

void mavlinkRcThread()
{
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[1024];

    for (int i = 0; i < 16; ++i) rc_values[i] = 1500;

    auto lastRatePrint = std::chrono::steady_clock::now();
    auto lastMsgTime   = std::chrono::steady_clock::now();

    // UDP (router) modunda: adresimizi ogret + stream iste (ArduPilot USB'de otomatik gondermiyor)
    if (g_useUdp) routerKeepalive();
    auto lastKeepalive = std::chrono::steady_clock::now();

    while (keepRunning.load()) {
        // Router route'unu canli tut + stream'leri yeniden iste (idempotent)
        if (g_useUdp) {
            auto nowk = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(nowk - lastKeepalive).count() > 2000) {
                routerKeepalive();
                lastKeepalive = nowk;
            }
        }
        int n = read(mavlink_serial_fd, buf, sizeof(buf));
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        for (int i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                uint8_t mtype = msg.msgid;

                auto now   = std::chrono::steady_clock::now();
                auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastMsgTime).count();
                lastMsgTime = now;

                if (mtype == MAVLINK_MSG_ID_RC_CHANNELS) {
                    mavlink_rc_channels_t rc;
                    mavlink_msg_rc_channels_decode(&msg, &rc);
                    {   // [PATCH-RSSI] MAVLink2'de RSSI'yi
                        // RC_CHANNELS (msg 65) tasir; RC_CHANNELS_RAW degil.
                        std::lock_guard<std::mutex> tl(g_telemMutex);
                        g_telemetry.rssi = rc.rssi;
                    }
                    g_lastRssiMs = steadyMs();   // [EYLUL] RSSI tazeligi (2 s gelmezse link yok)

                    {
                        std::lock_guard<std::mutex> lock(rcMutex);
                        rc_values[0]  = rc.chan1_raw;
                        rc_values[1]  = rc.chan2_raw;
                        rc_values[2]  = rc.chan3_raw;
                        rc_values[3]  = rc.chan4_raw;
                        rc_values[4]  = rc.chan5_raw;
                        rc_values[5]  = rc.chan6_raw;
                        rc_values[6]  = rc.chan7_raw;
                        rc_values[7]  = rc.chan8_raw;
                        rc_values[8]  = rc.chan9_raw;
                        rc_values[9]  = rc.chan10_raw;
                        rc_values[10] = rc.chan11_raw;
                        rc_values[11] = rc.chan12_raw;
                        rc_values[12] = rc.chan13_raw;
                        rc_values[13] = rc.chan14_raw;
                        rc_values[14] = rc.chan15_raw;
                        rc_values[15] = rc.chan16_raw;
                    }

                    auto now2   = std::chrono::steady_clock::now();
                    auto dtRate = std::chrono::duration_cast<std::chrono::milliseconds>(now2 - lastRatePrint).count();
                    if (dtRate > 500) {
                        std::lock_guard<std::mutex> lock(rcMutex);
                        std::cout << "[RC RATE] dt=" << dt_ms << " ms, "
                                  << "ch8="  << rc_values[7]
                                  << " ch12=" << rc_values[11]
                                  << " ch13=" << rc_values[12] << std::endl;
                        lastRatePrint = now2;
                    }
                }
                else if (mtype == MAVLINK_MSG_ID_RC_CHANNELS_RAW) {
                    mavlink_rc_channels_raw_t rc;
                    mavlink_msg_rc_channels_raw_decode(&msg, &rc);
                    std::lock_guard<std::mutex> lock(rcMutex);
                    rc_values[0] = rc.chan1_raw;
                    rc_values[1] = rc.chan2_raw;
                    rc_values[2] = rc.chan3_raw;
                    rc_values[3] = rc.chan4_raw;
                    rc_values[4] = rc.chan5_raw;
                    rc_values[5] = rc.chan6_raw;
                    rc_values[6] = rc.chan7_raw;
                    rc_values[7] = rc.chan8_raw;
                    // [10 Haz] OSD sinyal gucu: RC_CHANNELS_RAW.rssi (cihaz-bagimli, 0-254; 255=gecersiz)
                    {
                        std::lock_guard<std::mutex> tlock(g_telemMutex);
                        g_telemetry.rssi = rc.rssi;
                    }
                    g_lastRssiMs = steadyMs();   // [EYLUL]
                }
                
                // ===== BASIT TELEMETRY (OSD için) =====
                else if (mtype == MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    // SADECE gerçek autopilot heartbeat'i. GCS heartbeat'leri
                    // (controller_orin sys255, runtracker sys254) custom_mode=0 +
                    // armed=0 taşır → OSD STAB/disarmed ile gerçek mod arası TİTRER.
                    // GCS'lerde autopilot == MAV_AUTOPILOT_INVALID, onları atla.
                    if (hb.autopilot != MAV_AUTOPILOT_INVALID) {
                        std::lock_guard<std::mutex> lock(g_telemMutex);
                        g_telemetry.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
                        switch (hb.custom_mode) {
                            case 0:  g_telemetry.flight_mode = "STAB"; break;
                            case 1:  g_telemetry.flight_mode = "ACRO"; break;
                            case 2:  g_telemetry.flight_mode = "ALTH"; break;
                            case 3:  g_telemetry.flight_mode = "AUTO"; break;
                            case 5:  g_telemetry.flight_mode = "LOIT"; break;
                            case 6:  g_telemetry.flight_mode = "RTL"; break;
                            case 9:  g_telemetry.flight_mode = "LAND"; break;
                            case 20: g_telemetry.flight_mode = "GNGP"; break;
                            default: g_telemetry.flight_mode = "MODE"; break;
                        }
                    }
                }
                else if (mtype == MAVLINK_MSG_ID_SYS_STATUS) {
                    mavlink_sys_status_t sys;
                    mavlink_msg_sys_status_decode(&msg, &sys);
                    std::lock_guard<std::mutex> lock(g_telemMutex);
                    g_telemetry.battery_voltage = sys.voltage_battery / 1000.0f;
                    g_telemetry.battery_remaining = sys.battery_remaining;
                }
                else if (mtype == MAVLINK_MSG_ID_GPS_RAW_INT) {
                    mavlink_gps_raw_int_t gps;
                    mavlink_msg_gps_raw_int_decode(&msg, &gps);
                    std::lock_guard<std::mutex> lock(g_telemMutex);
                    g_telemetry.gps_sat_count = gps.satellites_visible;
                    // [PATCH-RSSI] OSD GPS bloku icin gerekli alanlar
                    g_telemetry.gps_fix_type = gps.fix_type;
                    g_telemetry.gps_lat = gps.lat / 1e7;
                    g_telemetry.gps_lon = gps.lon / 1e7;
                }
                else if (mtype == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_global_position_int_t pos;
                    mavlink_msg_global_position_int_decode(&msg, &pos);
                    std::lock_guard<std::mutex> lock(g_telemMutex);
                    g_telemetry.altitude_rel = pos.relative_alt / 1000.0f;
                }
                else if (mtype == MAVLINK_MSG_ID_ATTITUDE) {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);
                    std::lock_guard<std::mutex> lock(g_telemMutex);
                    g_telemetry.roll = att.roll;
                    g_telemetry.pitch = att.pitch;
                    g_telemetry.yaw = att.yaw;
                }
            }
        }
    }
}

// =============== RC INTERPRETATION =================

struct RcControl {
    bool roi_visible;   // CH8  (latching buton): >1600 AC, <1350 KAPAT  [TERS CEVRILDI]
    bool tracker_start; // CH12 (yayli slider): ileri it  -> kilitle
    bool tracker_stop;  // CH12 (yayli slider): geri cek  -> birak
    int  resize_dir;    // CH13 (yayli slider): -1 kucult / +1 buyut (surekli), 0 yok
    int  bitrate_kbps;  // [EYLUL] --ch-bitrate N kanali: pilot tavani (kbps), -1 = yok
};

void readRcControl(RcControl &ctrl)
{
    // KANAL HARITASI (yeni sistem — degisken isimleri FIZIKSEL kanallarla ayni):
    //   CH8  = pencere ac/kapa   (latching buton)
    //   CH12 = kilitle/birak     (yayli slider — ucarken arkadan parmakla)
    //   CH13 = ROI boyut         (yayli slider)
    //   CH5 ARTIK UCUS MODU KANALI (FLTMODE_CH=5) — tracker ASLA okumaz!
    //   CH9 bosta.
    uint16_t ch8, ch12, ch13, chBr = 0;
    {
        std::lock_guard<std::mutex> lock(rcMutex);
        ch8  = rc_values[7];    // CH8  pencere
        ch12 = rc_values[11];   // CH12 kilit
        ch13 = rc_values[12];   // CH13 boyut
        if (g_chBitrate >= 1 && g_chBitrate <= 16) chBr = rc_values[g_chBitrate - 1];   // [EYLUL]
    }
    ctrl.bitrate_kbps = -1;     // [EYLUL] erken donus dahil her yolda tanimli

    // Pencere durumu (kalici) — GECERSIZ RC yolunda da dogru deger dönsün diye
    // korumadan ONCE tanimli olmali.
    static bool lastRoi = false;

    // GECERSIZ RC KORUMASI: RC gelmeden / dusunce degerler 0 olabilir.
    // Korumasiz kalirsa 0 degeri "geri cekildi" sanilip SAHTE BIRAK tetikler.
    // DIKKAT: cagiran taraf `RcControl rcCtrl;` ile ILKLENMEMIS struct veriyor
    // (satir ~999), o yuzden erken donuste DORT alani da doldurmak ZORUNDAYIZ.
    if (ch8 < 900 || ch12 < 900 || ch13 < 900) {
        ctrl.roi_visible   = lastRoi;   // pencere durumu korunur
        ctrl.tracker_start = false;
        ctrl.tracker_stop  = false;
        ctrl.resize_dir    = 0;
        return;
    }

    // --- Pencere ac/kapa (CH8) ---
    // TERS CEVRILDI: eski kod yayli slider icindi (<1350 ac). Latching butonda
    // basili=1950 / bos=1050 oldugu icin o mantik "basinca KAPANIR" demek olurdu.
    // Yeni: basili (>1600) -> AC, bos (<1350) -> KAPAT.
    // Acilista buton bos (1050) -> pencere KAPALI. Eski davranisla ayni.
    bool roi = lastRoi;

    if (ch8 > 1600)      roi = true;
    else if (ch8 < 1350) roi = false;

    ctrl.roi_visible = roi;
    lastRoi = roi;

    // --- ROI boyut (CH13, yayli slider) --- MANTIK DEGISMEDI
    static uint16_t lastCh13 = 1500;
    ctrl.resize_dir = 0;
    // Slider'i tuttugu surece buyut/kucult, birakinca ortaya doner
    if (ch13 < 1330) {
        ctrl.resize_dir = -1;  // surekli kucult
    } else if (ch13 > 1650) {
        ctrl.resize_dir = +1;  // surekli buyut
    }
    lastCh13 = ch13;

    // --- Kilitle / birak (CH12, yayli slider) --- KENAR TETIKLI
    // Yayli slider bu mantiga BUTONDAN IYI oturuyor:
    //   ileri it -> LOCK_HI gecilir -> KILITLE ; birak -> 1500'e doner, tetik YOK
    //   geri cek -> LOCK_LO gecilir -> BIRAK   ; birak -> 1500'e doner, tetik YOK
    // Esikler burada tek yerde — bench sonrasi daralt/genislet.
    // Not: kenar tetikli oldugu icin yanlislikla tetiklenmesi kotudur;
    //      bu yuzden CH13 resize esiklerinden (1330/1650) biraz daha genis.
    static const uint16_t LOCK_HI = 1650;   // eski deger 1700
    static const uint16_t LOCK_LO = 1350;   // eski deger 1300
    static uint16_t lastCh12 = 1500;
    ctrl.tracker_start = false;
    ctrl.tracker_stop  = false;
    if (ch12 > LOCK_HI && lastCh12 <= LOCK_HI) {
        ctrl.tracker_start = true;
    } else if (ch12 < LOCK_LO && lastCh12 >= LOCK_LO) {
        ctrl.tracker_stop = true;
    }
    lastCh12 = ch12;

    // --- [EYLUL] Bitrate tavani (--ch-bitrate N, varsayilan KAPALI) ---
    // 1050..1950 -> brMin..brMax, LOG olcek (dusuk tarafta ince ayar), 100 kbps adim.
    // Pot ise surekli; 3 konumlu switch ise brMin / sqrt(min*max) / brMax olur.
    // Otomatik modlar acikken bu deger TAVAN olarak min()'e girer; kapaliyken dogrudan bitrate.
    if (g_chBitrate >= 1 && chBr >= 900) {
        float t  = std::max(0.0f, std::min(1.0f, (chBr - 1050) / 900.0f));
        int   kb = (int)(g_brMin * std::pow((double)g_brMax / (double)g_brMin, (double)t));
        ctrl.bitrate_kbps = std::max(g_brMin, (kb / 100) * 100);
    }
}

// ================= RTSP OUTPUT (GStreamer) =================

// [EYLUL] x264enc "bitrate" PLAYING'de degistirilebilir (x264_encoder_reconfig, sonraki karede).
// Dogrulama: gst-inspect-1.0 x264enc | grep -A3 "  bitrate"  -> "changeable ... PLAYING"
void setEncoderBitrate(int kbps)
{
    kbps = std::max(g_brMin, std::min(kbps, g_brMax));
    std::lock_guard<std::mutex> lock(g_gstMutex);
    if (!g_x264 || kbps == g_curKbps) return;
    g_object_set(g_x264, "bitrate", (guint)kbps, NULL);
    g_curKbps = kbps;
    std::cout << "[RTSP] bitrate -> " << kbps << " kbps\n";
}

// [EYLUL] RSSI% -> bitrate katmani. Asagi: EMA esigi gecince HEMEN. Yukari: ust esik + 3 s.
// rssiPct < 0 -> gecersiz, katman degismez. Tablo: g_tiers (KALIBRASYON UCUSUNDAN doldur).
static int rssiTierKbps(float rssiPct)
{
    using clock = std::chrono::steady_clock;
    static int   tier = -1;
    static float ema  = 100.0f;
    static auto  lastChange = clock::now();
    if (g_tiers.empty()) return g_brMax;
    if (tier < 0) {   // baslangic: --bitrate'e esit/altindaki en yuksek katman (RSSI oradan oynatir)
        tier = 0;
        for (int i = 0; i < (int)g_tiers.size(); ++i)
            if (g_tiers[i].kbps <= g_launchKbps) tier = i;
    }
    if (rssiPct < 0.0f) return g_tiers[tier].kbps;

    ema += 0.2f * (rssiPct - ema);                 // 10 Hz RC_CHANNELS'ta ~0.5 s yumusatma
    auto now = clock::now();
    while (tier > 0 && ema < g_tiers[tier].down) { tier--; lastChange = now; }
    if (tier + 1 < (int)g_tiers.size() && ema >= g_tiers[tier + 1].up &&
        std::chrono::duration_cast<std::chrono::seconds>(now - lastChange).count() >= 3) {
        tier++; lastChange = now;
    }
    return g_tiers[tier].kbps;
}

// [EYLUL] Yeni client PLAY dediginde IDR zorla: intra-refresh modunda baska IDR gelmez,
// shared media'ya sonradan katilan client aksi halde temiz kareye 1 s'den gec kavusur.
static void forceKeyframe()
{
    std::lock_guard<std::mutex> lock(g_gstMutex);
    if (!g_x264) return;
    GstEvent *ev = gst_video_event_new_upstream_force_key_unit(GST_CLOCK_TIME_NONE, TRUE, 0);
    gst_element_send_event(g_x264, ev);
    std::cout << "[RTSP] IDR zorlandi (yeni client)\n";
}

static void client_play_request_cb(GstRTSPClient * /*client*/, GstRTSPContext * /*ctx*/, gpointer)
{
    forceKeyframe();
}

// [EYLUL] Her SETUP'ta client'in istedigi tasima:
//   "RTP/AVP/TCP;unicast;interleaved=0-1"   -> TCP: kesintide tampon birikir (= ILERI SARMA),
//                                              RTCP kayip hep 0 gorunur. --udp-only dene.
//   "RTP/AVP;unicast;client_port=5000-5001" -> UDP: istenen.
static void client_setup_request_cb(GstRTSPClient * /*client*/, GstRTSPContext *ctx, gpointer)
{
    gchar *val = nullptr;
    if (ctx && ctx->request &&
        gst_rtsp_message_get_header(ctx->request, GST_RTSP_HDR_TRANSPORT, &val, 0) == GST_RTSP_OK && val)
        std::cout << "[RTSP] SETUP Transport: " << val << "\n";
}

static void client_connected_cb(GstRTSPServer * /*server*/, GstRTSPClient *client, gpointer)
{
    g_signal_connect(client, "setup-request", (GCallback)client_setup_request_cb, nullptr);
    g_signal_connect(client, "play-request",  (GCallback)client_play_request_cb,  nullptr);
}

static void media_unprepared_cb(GstRTSPMedia * /*media*/, gpointer /*user_data*/)
{
    // Son client koptugunda media unprepare olur (reusable YOK -> media yok edilir).
    // appsrc/x264 temizligini push'taki FLUSHING yolu yapar; yeni baglantida configure tazeler.
    std::cout << "[RTSP] Media unprepared (son client koptu)\n";
    std::lock_guard<std::mutex> lock(g_gstMutex);
    if (g_rtpSession) { g_object_unref(g_rtpSession); g_rtpSession = nullptr; }
}

// [EYLUL] Pipeline hazir olunca RTP session olusur (RTCP Receiver Report istatistikleri buradan)
static void media_prepared_cb(GstRTSPMedia *media, gpointer /*user_data*/)
{
    std::lock_guard<std::mutex> lock(g_gstMutex);
    if (g_rtpSession) { g_object_unref(g_rtpSession); g_rtpSession = nullptr; }
    GstRTSPStream *st = gst_rtsp_media_get_stream(media, 0);
    if (st) g_rtpSession = gst_rtsp_stream_get_rtpsession(st);   // transfer-full: biz unref ederiz
    std::cout << "[RTSP] Media prepared, RTP session " << (g_rtpSession ? "alindi" : "YOK") << "\n";
}

static void media_configure_cb(GstRTSPMediaFactory * /*factory*/,
                               GstRTSPMedia *media,
                               gpointer /*user_data*/)
{
    std::lock_guard<std::mutex> lock(g_gstMutex);

    // RECONNECT FIX: bayat referanslar varsa TAZELE (erken-return YOK).
    // Erken-return + reusable kombinasyonu, QGC kapa-ac sonrasi kalici
    // goruntusuzluk yaratiyordu (kanit: mini_rtsp testi, 11 Haz).
    if (g_appsrc) {
        std::cout << "[RTSP] Bayat appsrc referansi birakildi (yeni prepare)\n";
        gst_object_unref(g_appsrc);
        g_appsrc = nullptr;
    }
    if (g_x264) { gst_object_unref(g_x264); g_x264 = nullptr; }

    GstElement *pipeline = gst_rtsp_media_get_element(media);
    g_appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(pipeline), "mysrc");
    g_x264   = gst_bin_get_by_name_recurse_up(GST_BIN(pipeline), "enc");
    gst_object_unref(pipeline);   // get_element ref dondurur; sizinti kapatildi

    g_pts_base_us = -1;           // PTS bu pipeline icin sifirdan
    g_curKbps     = g_launchKbps; // yeni encoder launch degeriyle basladi
    g_rtcpKbps    = 0;            // RTCP onerisi ilk RR'a kadar yok
    g_dropCount   = 0;

    // NOT: set_reusable(TRUE) BILEREK YOK - reconnect fix'in cekirdegi.
    // Dogru kopus logu: media'nin "unprepared" sinyali (factory'de boyle sinyal yok)
    g_signal_connect(media, "prepared",   (GCallback)media_prepared_cb,   nullptr);
    g_signal_connect(media, "unprepared", (GCallback)media_unprepared_cb, nullptr);

    std::cout << "[RTSP] Pipeline hazir: preset=" << g_preset
              << " profile=" << (g_mainProfile ? "main" : "baseline")
              << " intra-refresh=" << (g_intraRefresh ? "on" : "off")
              << " " << g_launchKbps << " kbps"
              << (g_x264 ? "" : "  (UYARI: x264 'enc' bulunamadi!)") << "\n";
}

// [EYLUL] Her 500 ms (GLib main loop thread'i): client'in son Receiver Report'undaki kayip
// oranina gore ONERI uretir (g_rtcpKbps); uygulamayi ana dongu min() ile yapar.
// Yeni RR gelmediyse (exthighestseq ayni) dokunmaz. Client hic RR yollamiyorsa
// [RTCP] satiri hic gelmez -> RSSI/CH9 ile devam (sorun degil).
static gboolean rtcp_tick(gpointer /*user_data*/)
{
    if (!g_autoBitrate.load()) return G_SOURCE_CONTINUE;

    guint fracLost = 0, extSeq = 0, rtt = 0;
    gboolean haveRb = FALSE;
    int cur = 0;
    static bool dumped = false, typeWarned = false;
    {
        std::lock_guard<std::mutex> lock(g_gstMutex);
        if (!g_rtpSession) return G_SOURCE_CONTINUE;
        cur = g_curKbps;
        GstStructure *stats = nullptr;
        g_object_get(g_rtpSession, "stats", &stats, NULL);
        if (!stats) return G_SOURCE_CONTINUE;
        const GValue *arr = gst_structure_get_value(stats, "source-stats");
        G_GNUC_BEGIN_IGNORE_DEPRECATIONS   // rtpsession "source-stats" hala GValueArray (deprecated ama kullanimda)
        if (arr && G_VALUE_HOLDS(arr, g_value_array_get_type())) {
            GValueArray *va = (GValueArray *)g_value_get_boxed(arr);
            for (guint i = 0; va && i < va->n_values; ++i) {
                const GstStructure *s = gst_value_get_structure(g_value_array_get_nth(va, i));
                if (!s) continue;
                gboolean internal = TRUE, rb = FALSE;
                gst_structure_get_boolean(s, "internal", &internal);
                gst_structure_get_boolean(s, "have-rb", &rb);
                if (!internal && rb) {   // uzak alici: bizim akisimiz hakkinda RR yollamis
                    gst_structure_get_uint(s, "rb-fractionlost",  &fracLost);
                    gst_structure_get_uint(s, "rb-exthighestseq", &extSeq);
                    gst_structure_get_uint(s, "rb-round-trip",    &rtt);
                    haveRb = TRUE;
                    if (!dumped) {   // alan adlarini bir kez gor (surume gore dogrula)
                        gchar *txt = gst_structure_to_string(s);
                        std::cout << "[RTCP] ilk RR: " << txt << "\n";
                        g_free(txt);
                        dumped = true;
                    }
                }
            }
        } else if (!typeWarned) {
            std::cout << "[RTCP] source-stats beklenen tipte degil (GStreamer surumu?) - otomatik mod pasif\n";
            typeWarned = true;
        }
        G_GNUC_END_IGNORE_DEPRECATIONS
        gst_structure_free(stats);
    }
    if (!haveRb) return G_SOURCE_CONTINUE;

    static guint lastSeq  = 0;
    static int   cleanRRs = 0;
    if (extSeq == lastSeq) return G_SOURCE_CONTINUE;   // yeni rapor yok
    lastSeq = extSeq;

    float loss = fracLost / 256.0f;                     // son RR araligindaki kayip orani
    g_lastLossPermille = (int)(loss * 1000.0f);
    int target = cur;
    if      (loss > 0.05f)    { target = (int)(cur * 0.70f);            cleanRRs = 0; }  // agir kayip: sert kis
    else if (loss > 0.01f)    { target = (int)(cur * 0.90f);            cleanRRs = 0; }  // hafif kayip: yumusak kis
    else if (++cleanRRs >= 3) { target = std::min(g_brMax, cur + 250); cleanRRs = 0; }  // 3 temiz RR: +250
    g_rtcpKbps = std::max(g_brMin, target);
    std::cout << "[RTCP] loss=" << loss * 100.0f << "%  rtt=" << rtt / 65.536f
              << " ms  oneri=" << g_rtcpKbps.load() << " kbps\n";
    return G_SOURCE_CONTINUE;
}

bool initRtspServer(int width, int height, int fps, int bitrateKbps)
{
    g_fps_n = fps;
    g_fps_d = 1;

    int argc = 0;
    char **argv = nullptr;
    gst_init(&argc, &argv);

    // [EYLUL] appsrc leaky-type var mi? (GStreamer >= 1.20 / JetPack 6)
    {
        GstElement *probe = gst_element_factory_make("appsrc", nullptr);
        g_appsrcLeaky = probe && g_object_class_find_property(G_OBJECT_GET_CLASS(probe), "leaky-type");
        if (probe) gst_object_unref(probe);
        std::cout << "[RTSP] appsrc leaky-type "
                  << (g_appsrcLeaky ? "VAR (>=1.20): en taze kare encoder'a" : "YOK: kaynakta atlama") << "\n";
    }

    g_main_loop = g_main_loop_new(nullptr, FALSE);

    GstRTSPServer *server = gst_rtsp_server_new();
    g_object_set(server,
                 "address", "0.0.0.0",
                 "service", "8554",
                 NULL);

    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

    std::ostringstream launch;
    launch
        // ===== appsrc: OpenCV'den I420 kare (videoconvert KALKTI) =====
        // block=false iken appsrc max-bytes'i asinca yine de kuyruga alir (sadece
        // enough-data sinyali) -> eski "max-bytes=0" SINIRSIZDI. Sinir:
        //   >=1.20 : leaky-type=downstream (eskiyi atar, en taze kare encoder'a)
        //   eski   : rtspPushFrame'deki level kontrolu (yeniyi atlar)
        << "appsrc name=mysrc is-live=true block=false format=time do-timestamp=false "
        << (g_appsrcLeaky ? "max-bytes=0 max-buffers=1 max-time=0 leaky-type=downstream "
                          : "max-bytes=0 ")
        << "caps=video/x-raw,format=I420,width=" << width
        << ",height=" << height
        << ",framerate=" << fps << "/1 "

        // ===== x264enc: dusuk gecikme, CBR =====
        // 10-Haz audit gecerli: profile/level/nal-hrd/min-keyint/weightp property DEGIL.
        << "! x264enc name=enc "
        << "tune=zerolatency "
        << "speed-preset=" << g_preset << " "   // ultrafast'ten cik: superfast/veryfast (drop sayaci 0 kaldigi surece)
        << "pass=cbr "
        << "bitrate=" << bitrateKbps << " "
        << "vbv-buf-capacity=100 "              // 100-200 ms dene; kucuk = duz bitrate
        << "intra-refresh=" << (g_intraRefresh ? "true" : "false") << " "   // IDR patlamasi yok: yuvarlanan intra
        << "key-int-max=" << fps << " "         // intra-refresh: tam tazeleme ~1 s; --idr: GOP 1 s (eskisi 0.5 s idi)
        << "bframes=0 "
        << "ref=1 "                             // intra-refresh icin zorunlu
        << "cabac=" << (g_mainProfile ? 1 : 0) << " "
        << "dct8x8=0 "
        << "threads=4 "                         // sliced: 4 slice -> kaybolan paket karenin en fazla 1/4'u
        << "sliced-threads=true "
        << "sync-lookahead=0 "
        << "rc-lookahead=0 "

        // ===== H.264 caps: AVC stream (Annex-B degil) =====
        << "! video/x-h264,profile=" << (g_mainProfile ? "main" : "baseline")
        << ",stream-format=avc,alignment=au "

        // ===== h264parse: SPS/PPS =====
        << "! h264parse config-interval=-1 "

        // ===== Kucuk queue. LEAKY DEGIL: sikistirilmis kare atmak referans zincirini bozar;
        //       kare eleme encoder ONCESINDE (appsrc) yapiliyor. =====
        << "! queue "
        << "max-size-buffers=5 "
        << "max-size-bytes=0 "
        << "max-size-time=0 "

        // ===== RTP payload (DIKKAT: name=pay0) =====
        << "! rtph264pay name=pay0 "
        << "pt=96 "
        << "config-interval=1 "                 // SPS/PPS her 1 s bant ici: intra-refresh'te SART
        << "mtu=1200 ";

    gst_rtsp_media_factory_set_launch(factory, launch.str().c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);  // SHARED: Tek pipeline, tum clientlar paylasir
    if (g_udpOnly)   // TCP isteyen client 461 alir; UDP'ye dusmuyorsa goruntu gelmez -> bayragi kaldir
        gst_rtsp_media_factory_set_protocols(factory, GST_RTSP_LOWER_TRANS_UDP);
    g_signal_connect(factory, "media-configure",
                     (GCallback)media_configure_cb, nullptr);

    gst_rtsp_mount_points_add_factory(mounts, "/main.264", factory);
    g_object_unref(mounts);

    // [EYLUL] client kancalari: SETUP transport logu + PLAY'de IDR
    g_signal_connect(server, "client-connected", (GCallback)client_connected_cb, nullptr);

    if (gst_rtsp_server_attach(server, NULL) == 0) {
        std::cerr << "Failed to attach RTSP server\n";
        return false;
    }

    // [EYLUL] RTCP kontrolcusu (default context = asagidaki main loop)
    g_timeout_add(500, rtcp_tick, nullptr);

    g_gstRunning = true;
    g_gstThread = std::thread([](){
        g_main_loop_run(g_main_loop);
        g_gstRunning = false;
    });

    std::cout << "=========================================\n";
    std::cout << "SIYI-COMPATIBLE RTSP Server Started\n";
    std::cout << "URL: rtsp://<ORIN_IP>:8554/main.264\n";
    std::cout << "---\n";
    std::cout << "  - H264 " << (g_mainProfile ? "Main (cabac)" : "Baseline") << " profile\n";
    std::cout << "  - preset " << g_preset << ", " << (g_intraRefresh ? "intra-refresh" : "IDR GOP")
              << " 1 s, CBR " << bitrateKbps << " kbps (bitrate araligi "
              << g_brMin << "-" << g_brMax << ")\n";
    std::cout << "  - transport: " << (g_udpOnly ? "UDP only" : "client secer (SETUP loguna bak)") << "\n";
    std::cout << "  - bitrate kontrol: "
              << (g_rssiAuto ? "RSSI " : "") << (g_autoBitrate.load() ? "RTCP " : "")
              << (g_chBitrate > 0 ? ("CH" + std::to_string(g_chBitrate) + " ") : "")
              << ((!g_rssiAuto && !g_autoBitrate.load() && g_chBitrate <= 0) ? "SABIT" : "") << "\n";
    std::cout << "=========================================\n";

    return true;
}

void rtspPushFrame(const cv::Mat &frameIn)
{
    std::lock_guard<std::mutex> lock(g_gstMutex);
    if (!g_appsrc) return;

    // [EYLUL] ENCODER GERIDE MI? appsrc kuyrugunda hala bir kare bekliyorsa:
    //   eski GStreamer -> bu kareyi ATLA (gecikme 1 kareyi asamaz)
    //   >=1.20 leaky  -> yine PUSH ET, appsrc kuyruktaki ESKI kareyi atar (en taze kazanir)
    // Sayac her durumda: "[RTSP] encoder geride" = preset fazla agir / termal throttle.
    const guint64 oneFrame = (guint64)frameIn.cols * frameIn.rows * 3 / 2;   // I420
    guint64 queued = gst_app_src_get_current_level_bytes(GST_APP_SRC(g_appsrc));
    if (queued >= oneFrame) {
        uint32_t d = ++g_dropCount;
        // Baglanti basinda (preroll) birkac drop NORMAL; sayac [LINK] satirinda surekli ARTIYORSA
        // encoder yetismiyor -> preset'i bir kademe hizlandir / termal throttle'a bak.
        if ((d % 30) == 0)
            std::cout << "[RTSP] encoder geride, kare " << (g_appsrcLeaky ? "eskisi atildi" : "atlandi")
                      << " (toplam " << d << ")\n";
        if (!g_appsrcLeaky) return;
    }

    cv::Mat bgr = frameIn;
    if (bgr.type() != CV_8UC3) cv::cvtColor(frameIn, bgr, cv::COLOR_GRAY2BGR);

    // [EYLUL] BGR -> I420 burada (NEON'lu OpenCV): pipeline'daki videoconvert kalkti,
    // GstBuffer'a 3 yerine 1.5 byte/px kopyalaniyor. (w,h cift olmali - main() garanti eder)
    cv::Mat i420;
    cv::cvtColor(bgr, i420, cv::COLOR_BGR2YUV_I420);   // (h*3/2) x w, CV_8UC1, surekli

    const gsize size = i420.total() * i420.elemSize();
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    std::memcpy(map.data, i420.data, size);
    gst_buffer_unmap(buffer, &map);

    // Monotonik saat bazli PTS (10-Haz fix): RTP zamani ile RTCP duvar saati
    // ayni hizda akar -> uzun oturumda clock-skew kaynakli resync/donma kalkar.
    gint64 now_us = g_get_monotonic_time();
    if (g_pts_base_us < 0) g_pts_base_us = now_us;
    GstClockTime pts = (GstClockTime)(now_us - g_pts_base_us) * 1000ull;

    GST_BUFFER_PTS(buffer)      = pts;
    GST_BUFFER_DTS(buffer)      = pts;   // bframes=0 -> DTS=PTS guvenli
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(GST_SECOND, 1, g_fps_n);

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(g_appsrc), buffer);
    if (ret != GST_FLOW_OK) {
        // Client disconnect veya pipeline hatasi
        if (ret == GST_FLOW_FLUSHING || ret == GST_FLOW_EOS) {
            // Pipeline kapanmis, referanslari temizle
            gst_object_unref(g_appsrc);
            g_appsrc = nullptr;
            if (g_x264) { gst_object_unref(g_x264); g_x264 = nullptr; }
        }
    }
}

void shutdownRtsp()
{
    {
        std::lock_guard<std::mutex> lock(g_gstMutex);
        if (g_appsrc) {
            gst_app_src_end_of_stream(GST_APP_SRC(g_appsrc));
            gst_object_unref(g_appsrc);
            g_appsrc = nullptr;
        }
        if (g_x264)       { gst_object_unref(g_x264);      g_x264 = nullptr; }
        if (g_rtpSession) { g_object_unref(g_rtpSession);  g_rtpSession = nullptr; }
    }

    if (g_main_loop) {
        g_main_loop_quit(g_main_loop);
    }

    if (g_gstThread.joinable()) {
        g_gstThread.join();
    }

    if (g_main_loop) {
        g_main_loop_unref(g_main_loop);
        g_main_loop = nullptr;
    }
}

// ====================== MAIN =======================

int main(int argc, char** argv)
{
    std::string rtspUrl    = "rtsp://192.168.144.25:8554/main.264";
    std::string serialPort = "/dev/ttyACM0";
    int baudrate           = 115200;
    std::string mavlinkUdp = "127.0.0.1:14550";   // mavlink-router endpoint (varsayilan)
    int framePort          = 9999;                 // Python beyin TCP FRAME portu

    // output resolution ayarlarÄ±
    int outWidth  = 1280;  // default
    int outHeight = 720;   // default
    int outBitrateKbps = 1700;  // default (--bitrate ile degistir)

#ifdef WITH_ORTRACK
    applyQualityManagerCfg();   // [HYB] configs/orin-camera-quality-768.yaml
#endif

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--rtsp") && i+1 < argc) {
            rtspUrl = argv[++i];
        } else if (!strcmp(argv[i], "--mavlink-udp") && i+1 < argc) {
            mavlinkUdp = argv[++i];
            g_useUdp = true;
        } else if (!strcmp(argv[i], "--serial") && i+1 < argc) {
            serialPort = argv[++i];
            g_useUdp = false;          // DOGRUDAN seri (router YOKKEN). Router acikken KULLANMA!
        } else if (!strcmp(argv[i], "--baud") && i+1 < argc) {
            baudrate = std::stoi(argv[++i]);
        } else if (!strcmp(argv[i], "--frame-port") && i+1 < argc) {
            framePort = std::stoi(argv[++i]);
        } else if (!strcmp(argv[i], "--hfov") && i+1 < argc) {
            CAMERA_HFOV = std::stof(argv[++i]);
        } else if (!strcmp(argv[i], "--vfov") && i+1 < argc) {
            CAMERA_VFOV = std::stof(argv[++i]);
        } else if (!strcmp(argv[i], "--width") && i+1 < argc) {
            outWidth = std::stoi(argv[++i]);
        } else if (!strcmp(argv[i], "--height") && i+1 < argc) {
            outHeight = std::stoi(argv[++i]);
        } else if (!strcmp(argv[i], "--bitrate") && i+1 < argc) {
            outBitrateKbps = std::stoi(argv[++i]);
        // ---- [EYLUL] video / link kontrol ----
        } else if (!strcmp(argv[i], "--preset") && i+1 < argc) {
            g_preset = argv[++i];                 // ultrafast | superfast | veryfast | faster
        } else if (!strcmp(argv[i], "--main")) {
            g_mainProfile = true;                 // cabac=1 + profile=main (SIYI FPV kabul ederse)
        } else if (!strcmp(argv[i], "--idr")) {
            g_intraRefresh = false;               // klasik IDR GOP (intra-refresh sorun cikarirsa)
        } else if (!strcmp(argv[i], "--udp-only")) {
            g_udpOnly = true;                     // TCP transport isteyen client reddedilir
        } else if (!strcmp(argv[i], "--br-min") && i+1 < argc) {
            g_brMin = std::stoi(argv[++i]);
        } else if (!strcmp(argv[i], "--br-max") && i+1 < argc) {
            g_brMax = std::stoi(argv[++i]);
        } else if (!strcmp(argv[i], "--ch-bitrate") && i+1 < argc) {
            g_chBitrate = std::stoi(argv[++i]);   // pilot tavani kanali (orn. 9), 0 = kapali
        } else if (!strcmp(argv[i], "--rssi-bitrate")) {
            g_rssiAuto = true;                    // RSSI katmanlari (g_tiers)
        } else if (!strcmp(argv[i], "--auto-bitrate")) {
            g_autoBitrate = true;                 // RTCP kayip kontrolcusu
        // ---- [1080] isleme/yayin ayrisimi ----
        } else if ((!strcmp(argv[i], "--out-width")) && i+1 < argc) {
            outWidth = std::stoi(argv[++i]);
        } else if ((!strcmp(argv[i], "--out-height")) && i+1 < argc) {
            outHeight = std::stoi(argv[++i]);
        } else if (!strcmp(argv[i], "--proc-out")) {
            g_procAtOut = true;                   // ESKI davranis: yayin cozunurlugunde isle
        // ---- [ORT/HYB] tracker secimi ----
        } else if (!strcmp(argv[i], "--ortrack")) {
            g_useOrtrack = true;
        } else if (!strcmp(argv[i], "--hybrid")) {
            g_hybrid = true;
        } else if (!strcmp(argv[i], "--hybrid-kcf")) {
            g_hybridKcf = true;
        } else if (!strcmp(argv[i], "--detect-only")) {
            g_detectOnly = true;              // tracker YOK, sadece YOLO kutusu
        } else if (!strcmp(argv[i], "--det-hold") && i+1 < argc) {
            g_detHoldFrames = std::max(1, std::stoi(argv[++i]));
        } else if (!strcmp(argv[i], "--ortrack-engine") && i+1 < argc) {
            g_ortEngine = argv[++i];
        } else if (!strcmp(argv[i], "--yolo-engine") && i+1 < argc) {
            g_yoloEngine = argv[++i];
        } else if (!strcmp(argv[i], "--ortrack-min-score") && i+1 < argc) {
            g_ortMinScore = std::stof(argv[++i]);
        } else if (!strcmp(argv[i], "--ortrack-min-box") && i+1 < argc) {
            g_ortMinBox = std::stod(argv[++i]);
        } else if (!strcmp(argv[i], "--ortrack-lost") && i+1 < argc) {
            g_ortLostLimit = std::max(1, std::stoi(argv[++i]));
        } else if (!strcmp(argv[i], "--ortrack-verbose")) {
            g_ortVerbose = true;
        } else if (!strcmp(argv[i], "--yolo-conf") && i+1 < argc) {
            g_yoloConf = std::stof(argv[++i]);
        } else if (!strcmp(argv[i], "--yolo-iou") && i+1 < argc) {
            g_yoloIoU = std::stof(argv[++i]);
        } else if (!strcmp(argv[i], "--box-smooth") && i+1 < argc) {
            g_boxSmooth = std::max(0, std::stoi(argv[++i]));
        } else if (!strcmp(argv[i], "--track-csv") && i+1 < argc) {
            g_trackCsv = argv[++i];
        } else if (!strcmp(argv[i], "--hyb-draw-dets")) {
            g_hybDrawDets = true;
        } else if (!strcmp(argv[i], "--hyb-legacy")) {
#ifdef WITH_ORTRACK
            applyLegacyManagerCfg();
#endif
        } else if (!strcmp(argv[i], "--hyb-search-interval") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_mgrCfg.search_interval = std::max(1, std::stoi(argv[++i]));
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--hyb-verify-interval") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_mgrCfg.verify_interval = std::max(0, std::stoi(argv[++i]));
            g_verifyIvSet = true;
#else
            ++i;
#endif
        // ---- [HYB] manager ayar yuzeyi ----
        // verify_miss_patience: kilitliyken KAC ardisik desteksiz dogrulamada
        // birakilir. Varsayilan 3 upstream'den; YOLO recall'u dusukse (sizin
        // olcumunuzde 934 cagrida 21 kutu) 3 cok serttir, DOGRU kilit de duser.
        } else if (!strcmp(argv[i], "--hyb-verify-miss-patience") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_mgrCfg.verify_miss_patience = std::max(1, std::stoi(argv[++i]));
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--hyb-low-score") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_mgrCfg.tracker_low_score = std::stod(argv[++i]);
            g_lowSet = true;
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--hyb-recover-score") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_mgrCfg.tracker_recover_score = std::stod(argv[++i]);
            g_recoverSet = true;
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--hyb-accept-conf") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_mgrCfg.detection_accept_confidence = std::stod(argv[++i]);
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--hyb-max-growth") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_mgrCfg.max_box_growth = std::stod(argv[++i]);   // 0 = kapali
#else
            ++i;
#endif
        // ---- [LOCK] --hybrid-kcf ayarlari ----
        } else if (!strcmp(argv[i], "--kcf-lock-window") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_lockCfg.lockWindow = std::max(1, std::stoi(argv[++i]));
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--kcf-lock-hits") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_lockCfg.lockHits = std::max(1, std::stoi(argv[++i]));
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--kcf-invalid-limit") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_lockCfg.invalidLimit = std::max(1, std::stoi(argv[++i]));
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--kcf-outside-limit") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_lockCfg.outsideLimit = std::max(1, std::stoi(argv[++i]));
#else
            ++i;
#endif
        // Varlik kontrolu: VARSAYILAN KAPALI. 0'dan buyuk verilirse
        // N karede bir tespit kosar ve karede HIC drone yoksa sayar.
        } else if (!strcmp(argv[i], "--kcf-presence-check") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_lockCfg.presencePeriod = std::max(0, std::stoi(argv[++i]));
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--kcf-presence-misses") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_lockCfg.presenceMisses = std::max(1, std::stoi(argv[++i]));
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--kcf-growth-trigger") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_lockCfg.growthTrigger = std::stod(argv[++i]);
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--kcf-growth-reset") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_lockCfg.growthReset = std::stod(argv[++i]);
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--kcf-growth-min-px") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_lockCfg.growthMinPx = std::stod(argv[++i]);
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--hyb-refresh-template")) {
#ifdef WITH_ORTRACK
            // YOLO dogrulamasi basarili oldugunda tracker'i YOLO'nun kutusundan
            // YENIDEN KUR. KCF'te olcek tek yonlu bir cirnata gibidir: kutu
            // buyur ama kuculmez (model arka plani ogrenir, kuculmek tepeyi
            // dusurur). Yenileme bu birikimi her dogrulamada sifirlar.
            g_mgrCfg.refresh_template = true;
#endif
        } else if (!strcmp(argv[i], "--hyb-confirm-hits") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_mgrCfg.confirmation_hits = std::max(1, std::stoi(argv[++i]));
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--hyb-low-patience") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_mgrCfg.tracker_low_patience = std::max(1, std::stoi(argv[++i]));
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--hyb-miss-patience") && i+1 < argc) {
#ifdef WITH_ORTRACK
            g_mgrCfg.verification_miss_patience = std::max(1, std::stoi(argv[++i]));
#else
            ++i;
#endif
        } else if (!strcmp(argv[i], "--help")) {
            std::cout <<
                "Usage: " << argv[0]
                << " [--rtsp URL] [--mavlink-udp HOST:PORT] [--frame-port N]\n"
                << "          [--hfov DEG] [--vfov DEG] [--width W] [--height H] [--bitrate KBPS]\n"
                << "          [--serial DEV --baud BPS]  (router YOKKEN dogrudan seri)\n"
                << "  Video/link ([EYLUL]):\n"
                << "          [--preset superfast|veryfast|ultrafast] [--main] [--idr] [--udp-only]\n"
                << "          [--br-min KBPS] [--br-max KBPS] [--ch-bitrate N] [--rssi-bitrate] [--auto-bitrate]\n"
                << "  bitrate = min(RSSI katmani, RTCP onerisi, CHn tavani); hicbiri yoksa --bitrate sabit.\n"
                << "  Cozunurluk ([1080]):\n"
                << "          --width/--height veya --out-width/--out-height = YAYIN boyutu\n"
                << "          isleme boyutu KAYNAKTAN gelir (1080p). --proc-out ile eski davranis.\n"
                << "  Tracker ([ORT/HYB]):\n"
                << "          [--ortrack]                  manuel ROI + ORTrack (TensorRT)\n"
                << "          [--hybrid]                   YOLO tespit + ORTrack takip (OTOMATIK)\n"
                << "          [--hybrid-kcf]               YOLO tespit + KCF takip (bir kez kilitle, karisma)\n"
                << "          [--detect-only]              SADECE YOLO kutusu, tracker YOK (tanilama)\n"
                << "          [--det-hold N]               tespit kaybolunca kutu N kare tutulur (vars 15)\n"
                << "          [--kcf-lock-hits N] [--kcf-lock-window N]   vars 20/30\n"
                << "          [--kcf-invalid-limit N] [--kcf-outside-limit N]  vars 30/60\n"
                << "          [--kcf-presence-check N]     N karede 1 varlik kontrolu (0 = KAPALI)\n"
                << "          [--kcf-presence-misses N]    ust uste kac bos kontrolde duser (vars 3)\n"
                << "          [--kcf-growth-trigger S] [--kcf-growth-reset S] [--kcf-growth-min-px N]\n"
                << "          [--ortrack-engine YOL]       vars: ../../models/ORTrack_ep0300-fp16.engine\n"
                << "          [--yolo-engine YOL]          vars: ../../models/UAV-YOLOv11m-768.engine\n"
                << "          [--ortrack-min-score S]      Hann tepe esigi, 0 = kapali\n"
                << "          [--ortrack-min-box N]        kutu tabani (vars 4.0; eski 10.0)\n"
                << "          [--ortrack-lost N]           MANUEL modda N gecersiz kare -> kilit duser\n"
                << "          [--yolo-conf S] [--yolo-iou S]\n"
                << "          [--hyb-legacy]               eski manager parametreleri\n"
                << "          [--hyb-search-interval N] [--hyb-verify-interval N]\n"
                << "          [--hyb-verify-miss-patience N] [--hyb-miss-patience N]\n"
                << "          [--hyb-low-score S] [--hyb-recover-score S] [--hyb-low-patience N]\n"
                << "          [--hyb-accept-conf S] [--hyb-max-growth S] [--hyb-confirm-hits N]\n"
                << "          [--hyb-refresh-template]     dogrulamada tracker'i YOLO kutusundan yenile\n"
                << "          [--hyb-draw-dets]            tum YOLO kutularini ciz (tanilama)\n"
                << "  Olcum / yumusatma:\n"
                << "          [--box-smooth N]             kutu w/h icin N kareli MEDYAN (0 = kapali)\n"
                << "          [--track-csv YOL]            kare kare CSV (ham VE yumusatilmis)\n"
                << "          [--ortrack-verbose]\n"
                << "  Bayrak YOKSA KCF calisir.\n"
                "Example (mavlink-router ile):\n"
                "  " << argv[0]
                << " --rtsp rtsp://192.168.144.25:8554/main.264"
                << " --mavlink-udp 127.0.0.1:14550 --frame-port 9999\n"
                << "  --width 1920 --height 1080 --bitrate 3000 --preset superfast\n"
                << "  (kalibrasyondan sonra) --rssi-bitrate --auto-bitrate --br-min 800 --br-max 5000\n";
            return 0;
        }
    }
    if (g_brMin < 200)     g_brMin = 200;
    if (g_brMax < g_brMin) g_brMax = g_brMin;

    std::cout << "=== RTSP + MAVLink TRACKER (RTSP OUT + TCP FRAME) ===\n";
    std::cout << "Tracker : " << (g_detectOnly ? "DETECT-ONLY (sadece YOLO, tracker YOK)"
                                : g_hybridKcf ? "HYBRID-KCF (YOLO + KCF)"
                                : g_hybrid   ? "HYBRID (YOLO + ORTrack)"
                                : g_useOrtrack ? "ORTRACK (manuel ROI)" : "KCF") << "\n";
#ifdef WITH_ORTRACK
    if (g_hybrid || g_hybridKcf || g_useOrtrack || g_detectOnly) {
        if (g_useOrtrack || g_hybrid)
            std::cout << "  ortrack : " << g_ortEngine << "  (" << ortrack::tensorrtBuildInfo() << ")\n";
        if (g_hybrid || g_hybridKcf || g_detectOnly)
            std::cout << "  yolo    : " << g_yoloEngine << "\n";
        std::cout << "  min-box=" << g_ortMinBox
                  << "  min-score=" << g_ortMinScore << (g_ortMinScore > 0.0f ? "" : " (kapali)")
                  << "  lost-limit=" << g_ortLostLimit << "\n";
        if (g_hybridKcf) {
            std::cout << "  kilit   : " << g_lockCfg.lockHits << "/" << g_lockCfg.lockWindow
                      << " tespit -> KCF kurulur, SONRA SADECE KCF (tespit durur)\n"
                      << "            kilidi dusuren: " << g_lockCfg.invalidLimit
                      << " gecersiz kare | " << g_lockCfg.outsideLimit
                      << " kare guvenli alan disi";
            if (g_lockCfg.presencePeriod > 0)
                std::cout << " | varlik: " << g_lockCfg.presencePeriod << " karede 1, "
                          << g_lockCfg.presenceMisses << " bos";
            else
                std::cout << " | varlik kontrolu KAPALI";
            std::cout << "\n            buyume  : >=" << g_lockCfg.growthMinPx << "px iken x"
                      << g_lockCfg.growthTrigger << " asilirsa x" << g_lockCfg.growthReset
                      << "'e cekilir (ref guncellenir)\n"
                      << "            CH5 bu modda KULLANILMIYOR\n";
        }
        if (g_hybrid)
            std::cout << "  manager : accept=" << g_mgrCfg.detection_accept_confidence
                      << " low=" << g_mgrCfg.tracker_low_score
                      << " recover=" << g_mgrCfg.tracker_recover_score
                      << " growth=" << g_mgrCfg.max_box_growth
                      << " search_iv=" << g_mgrCfg.search_interval
                      << " verify_iv=" << g_mgrCfg.verify_interval
                      << " verify_miss=" << g_mgrCfg.verify_miss_patience
                      << " low_pat=" << g_mgrCfg.tracker_low_patience
                      << " miss_pat=" << g_mgrCfg.verification_miss_patience
                      << " hits=" << g_mgrCfg.confirmation_hits
                      << " refresh=" << (g_mgrCfg.refresh_template ? 1 : 0) << "\n";
    }
#else
    if (g_hybrid || g_hybridKcf || g_useOrtrack || g_detectOnly)
        std::cout << "  !!! Bu ikili WITH_ORTRACK=OFF ile derlendi — KCF kullanilacak.\n";
#endif
    std::cout << "RTSP IN  : " << rtspUrl    << "\n";
    if (g_useUdp)
        std::cout << "MAVLink  : UDP " << mavlinkUdp << " (mavlink-router)\n";
    else
        std::cout << "MAVLink  : Serial " << serialPort << " @ " << baudrate
                  << "  (DOGRUDAN — router KAPALI olmali!)\n";
    std::cout << "FRAME    : TCP port " << framePort << " (Python beyin)\n";
    std::cout << "FOV      : HFOV=" << CAMERA_HFOV << "  VFOV=" << CAMERA_VFOV << "\n";
    std::cout << "Out size : " << outWidth << "x" << outHeight << "\n";
    std::cout << "Video    : preset=" << g_preset << " " << (g_mainProfile ? "main" : "baseline")
              << " " << (g_intraRefresh ? "intra-refresh" : "idr-gop")
              << " bitrate=" << outBitrateKbps << " [" << g_brMin << ".." << g_brMax << "]"
              << (g_rssiAuto ? " +rssi" : "") << (g_autoBitrate.load() ? " +rtcp" : "")
              << (g_chBitrate > 0 ? " +ch" + std::to_string(g_chBitrate) : "")
              << (g_udpOnly ? " udp-only" : "") << "\n\n";

    std::signal(SIGINT, signalHandler);

    if (g_useUdp) {
        std::string host = "127.0.0.1"; int port = 14550;
        size_t colon = mavlinkUdp.find(':');
        if (colon != std::string::npos) {
            host = mavlinkUdp.substr(0, colon);
            port = std::stoi(mavlinkUdp.substr(colon + 1));
        }
        mavlink_serial_fd = openMavlinkUDP(host, port);
        if (mavlink_serial_fd < 0)
            std::cerr << "UDP acilamadi, RC/telemetri yok!\n";
        else
            std::cout << "v MAVLink UDP baglandi: " << host << ":" << port << "\n";
    } else {
        mavlink_serial_fd = openSerial(serialPort, baudrate);
        if (mavlink_serial_fd < 0)
            std::cerr << "Serial acilamadi, RC yok!\n";
        else
            std::cout << "v MAVLink Serial: " << serialPort << "\n";
    }

    std::thread rcThread;
    if (mavlink_serial_fd >= 0) {
        rcThread = std::thread(mavlinkRcThread);
    }

    // TCP FRAME server (Python beyne bbox yayinlar)
    std::thread frameThread(frameServerThread, framePort);

    // RTSP CAMERA INPUT
    
    // DONANIM DECODE (nvv4l2decoder + nvvidconv): Python appsink testinde
    // dogrulandi -> "FRAME GELDI (720,1280,3)". Yazilim avdec_h265 yerine GPU
    // decode -> dusuk latency + CPU rahat. Onceki not-negotiated hatalari
    // fpsdisplaysink/autovideosink (headless display yok) yuzundendi; appsink
    // ve fakesink ile donanim decode sorunsuz calisiyor (Test 3/4 ile kanitli).
    // nvv4l2decoder -> NV12/NVMM -> nvvidconv -> BGRx (sistem mem) -> videoconvert -> BGR.
    std::string gstIn =
    "rtspsrc location=" + rtspUrl + " protocols=udp latency=0 ! "
    "rtph265depay ! h265parse ! nvv4l2decoder ! "
    "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
    "appsink caps=video/x-raw,format=BGR sync=false max-buffers=1 drop=true";

    cv::VideoCapture cap(gstIn, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "RTSP (GStreamer) acilmadi: " << gstIn << "\n";
        keepRunning = false;
    }
    // NOT: cap.set(CAP_PROP_BUFFERSIZE) KALDIRILDI. appsink'te zaten
    // max-buffers=1 drop=true var (ayni isi pipeline icinde yapar). Bu set
    // cagrisi OpenCV-GStreamer backend'inde appsink kurulduktan SONRA pipeline'i
    // yeniden negotiate etmeye calisip 'not-linked/unhandled property' ile
    // bozuyordu (Python testinde bu satir yoktu, frame sorunsuz geldi).
    
//    cv::VideoCapture cap(rtspUrl);
//    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
//    if (!cap.isOpened()) {
//        std::cerr << "RTSP aÃ§Ä±lmadÄ±: " << rtspUrl << "\n";
//        keepRunning = false;
//    }

    cv::Mat frame;
    // İlk frame icin RETRY: RTSP/UDP akisi kurulmasi birkac saniye surebilir
    // (gst-launch'ta da "Redistribute latency" ~saniyelerce surer). Tek read()
    // ile pes ETME; akis hazir olana kadar 5sn boyunca 100ms araliklarla dene.
    if (keepRunning) {
        bool gotFirst = false;
        for (int attempt = 0; attempt < 50 && keepRunning; ++attempt) {  // 50 x 100ms = 5sn
            if (cap.read(frame) && !frame.empty()) { gotFirst = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!gotFirst) {
            std::cerr << "RTSP ilk frame alinamadi (5sn timeout) — kamera akisi/baglanti?\n";
            keepRunning = false;
        }
    }

    if (!keepRunning) {
        // TEMIZLIK DUZELTMESI:
        // Eskiden burada SADECE rcThread join ediliyordu; frameThread joinable
        // kalip yok ediliyordu -> std::terminate -> "Aborted (core dumped)".
        // Kamera acilamadiginda hata mesaji cekirdek dokumune bogulyordu.
        //
        // frameThread accept()'te BLOKE oldugu icin join() sonsuza kadar
        // bekler; once soketi kapatip accept'i uyandiriyoruz.
        {
            int fd = g_frameServerFd.exchange(-1);
            if (fd >= 0) { shutdown(fd, SHUT_RDWR); close(fd); }
        }
        if (frameThread.joinable()) frameThread.join();
        if (rcThread.joinable()) rcThread.join();
        if (mavlink_serial_fd >= 0) close(mavlink_serial_fd);
        return -1;
    }

    int srcW = frame.cols;
    int srcH = frame.rows;

    // EÄŸer width/height 0 girilirse, source boyuta eÅŸitle
    if (outWidth <= 0 || outHeight <= 0) {
        outWidth  = srcW;
        outHeight = srcH;
    }

    outWidth  &= ~1;   // [EYLUL] I420 icin cift boyut sart
    outHeight &= ~1;

    std::cout << "Source size: " << srcW   << "x" << srcH   << "\n";
    std::cout << "Using size : " << outWidth << "x" << outHeight << "\n";

    int outFps         = 30;

    g_launchKbps = outBitrateKbps;   // [EYLUL] configure_cb bunu g_curKbps'e yazar
    if (!initRtspServer(outWidth, outHeight, outFps, outBitrateKbps)) {
        std::cerr << "RTSP server Ã§Ä±kÄ±ÅŸÄ± yok.\n";
    }

    // KCF Tracker
    // =================================================================
    // [ORT/HYB] TRACKER KURULUMU
    // Oncelik: --hybrid > --ortrack > KCF. Her basarisizlik bir ALT
    // secenege duser; ucus motor dosyasi yuzunden IPTAL OLMAZ.
    // =================================================================
    std::unique_ptr<TrackerBackend> tracker;
    long long hybFrameIdx  = 0;
    bool      hybPrevHadBox = false;
    std::vector<hybrid::Detection> detLast;   // [DET] son tespit listesi
    int       detAge = 9999;                  // kac karedir yeni sonuc yok
#ifdef WITH_ORTRACK
    std::unique_ptr<ortrack::OrtrackTracker> hybOrt;
    std::unique_ptr<KcfTrackerHandle>        hybKcf;
    std::unique_ptr<hybrid::YoloDetector>    hybYolo;
    std::unique_ptr<hybrid::TargetManager>   hybMgr;
    std::unique_ptr<slock::SimpleLock>       slk;      // --hybrid-kcf
    std::unique_ptr<slock::AsyncDetector>    slkDet;   // --hybrid-kcf
    TrackerBackend* hybStats = nullptr;   // rozet / log / CSV (hangi backend olursa)

    // ---- YOLO: her iki hibrit mod da ister
    if (g_hybrid || g_hybridKcf || g_detectOnly) {
        hybrid::YoloOptions y;
        y.enginePath = g_yoloEngine;
        y.confidence = g_yoloConf;
        y.nmsIoU     = g_yoloIoU;
        y.verbose    = g_ortVerbose;
        std::string err;
        hybYolo = hybrid::makeYoloTensorRT(y, err);
        if (!hybYolo)
            std::cerr << "\n[HYB] YOLO BASLATILAMADI:\n       " << err << "\n";
    }

    // ---- ORTrack: --ortrack (manuel) veya --hybrid.
    //      --hybrid-kcf ISTEMEZ: o modda ORTrack motoru hic yuklenmez.
    if (g_useOrtrack || g_hybrid) {
        ortrack::TrtOptions o;
        o.enginePath = g_ortEngine;
        o.minScore   = g_ortMinScore;
        o.minBoxSize = g_ortMinBox;
        o.verbose    = g_ortVerbose;
        std::string err;
        hybOrt = ortrack::makeTensorRTTracker(o, err);
        if (!hybOrt)
            std::cerr << "\n[ORT] ORTrack BASLATILAMADI:\n       " << err << "\n";
    }

    // ---- Bagla. Her basarisizlik bir ALT secenege duser; ucus iptal olmaz.
    if (g_detectOnly && hybYolo) {
        slkDet.reset(new slock::AsyncDetector(hybYolo.get()));
        std::cout << "[DET] SADECE TESPIT modu: YOLO(" << hybYolo->inputSize()
                  << "), tracker YOK. Ham dedektor performansini gormek icin.\n";
    } else if (g_hybridKcf && hybYolo) {
        hybKcf   = makeKcfTracker(g_ortMinBox);
        hybStats = hybKcf.get();
        slk.reset(new slock::SimpleLock(g_lockCfg));
        slkDet.reset(new slock::AsyncDetector(hybYolo.get()));   // AYRI THREAD
        std::cout << "[LOCK] hazir: YOLO(" << hybYolo->inputSize()
                  << ") + KCF, tespit AYRI THREAD'de.\n";
    } else if (g_hybrid && hybYolo && hybOrt) {
        hybStats = hybOrt.get();
        hybMgr.reset(new hybrid::TargetManager(hybOrt.get(), g_mgrCfg));
        std::cout << "[HYB] hibrit hazir: YOLO(" << hybYolo->inputSize()
                  << ") + ORTrack, otomatik yakalama.\n";
    } else if (hybOrt) {
        if (g_hybrid) std::cerr << "[HYB] YOLO yok -> manuel ORTrack moduna dusuluyor.\n";
        tracker.reset(hybOrt.release());
        std::cout << "[ORT] ORTrack hazir (manuel ROI).\n";
    }
    if (!hybMgr && !tracker && !(g_detectOnly && slkDet)) {
        if (g_hybrid || g_hybridKcf || g_useOrtrack || g_detectOnly)
            std::cerr << "[ORT] manuel KCF moduna dusuluyor.\n";
        tracker = makeKcfTracker(g_ortMinBox);
    }
#else
    if (g_hybrid || g_hybridKcf || g_useOrtrack)
        std::cerr << "[ORT] --ortrack/--hybrid istendi ama ikili WITH_ORTRACK=OFF; KCF kullanilacak.\n";
    tracker = makeKcfTracker();
#endif

    bool trackingActive = false;
    bool roiVisible     = false;
    int  lostStreak     = 0;   // [ORT] ardisik gecersiz kare (KCF yolunda HEP 0)

    // [SMOOTH/CSV]
    MedianWin smoothW, smoothH;
    smoothW.setN(g_boxSmooth); smoothH.setN(g_boxSmooth);
    long long frameCounter = 0;
    std::ofstream trackCsv;
    if (!g_trackCsv.empty()) {
        trackCsv.open(g_trackCsv);
        if (trackCsv) {
            trackCsv << "frame,t_ms,state,valid,cx,cy,w_raw,h_raw,w_out,h_out,"
                        "score,trk_ms,det_ms,box_cx,box_cy\n";
            std::cout << "[CSV] kare kare kayit: " << g_trackCsv << "\n";
        } else {
            std::cerr << "[CSV] acilamadi: " << g_trackCsv << "\n";
        }
    }
    if (g_boxSmooth > 1)
        std::cout << "Yumusatma : kutu w/h medyan penceresi " << g_boxSmooth << " kare\n";

    // -----------------------------------------------------------------
    // [1080] ROI olcegi. roiSize=50 / MIN=20 / MAX=150 ve CH12'nin +4 adimi
    // 720p pikseliydi. 1080p'de ayni sayilar FOV'un 1/1.5'ini kaplar -> kutu
    // goze kucuk gelir, CH12 yavaslar. k ile olceklenir.
    // -----------------------------------------------------------------
    const int    procW = g_procAtOut ? outWidth  : srcW;
    const int    procH = g_procAtOut ? outHeight : srcH;
    const double roiK  = (double)procH / 720.0;
    const int roiBase      = std::max(8, (int)lround(50.0  * roiK));
    const int MIN_ROI_SIZE = std::max(6, (int)lround(20.0  * roiK));
    const int MAX_ROI_SIZE = std::max(roiBase, (int)lround(150.0 * roiK));
    const int roiStep      = std::max(1, (int)lround(4.0   * roiK));
    int roiSize = roiBase;
    cv::Rect roiRect;

    auto updateCenterRoi = [&](int w, int h){
        int half = roiSize / 2;
        roiRect  = cv::Rect(w / 2 - half, h / 2 - half, roiSize, roiSize);
    };
    updateCenterRoi(procW, procH);

    std::cout << "Isleme    : " << procW << "x" << procH
              << (g_procAtOut ? "  (--proc-out: yayin cozunurlugunde)" : "  (kaynak cozunurlugu)")
              << "\n";
    std::cout << "ROI       : baslangic " << roiBase << "  min " << MIN_ROI_SIZE
              << "  max " << MAX_ROI_SIZE << "  adim " << roiStep << "\n";


    std::cout << "=== RC Kontrolleri ===\n";
    std::cout << "CH13 <1350 : center box AÇ (MAVİ)\n";
    std::cout << "CH13 >1600 : center box KAPAT\n";
    std::cout << "CH12 <1330 : box sürekli küçült (basılı tut)\n";
    std::cout << "CH12 >1650 : box sürekli büyüt (basılı tut, max 150x150)\n";
    std::cout << "CH5  low   : tracker DUR\n";
    std::cout << "CH5  high  : tracker BAŞLAT (kutu TURUNCU olur)\n";
    std::cout << "ROI Başlangıç: 50x50 (mavi)\n";
    std::cout << "Ctrl+C     : çıkış\n";

    while (keepRunning.load()) {
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Frame okunamadÄ± (RTSP kesildi?).\n";
            break;
        }


        // =============================================================
        // [1080] ISLEME KARESI — kaynak cozunurlugu, KUCULTME YOK.
        // Kazanc ORTrack'te: arama kirpmasi (sqrt(w*h)*4) native pikselden
        // alinir. 720p'de 30 px hedef 120 px kirpilip 256'ya 2.13x BUYUTULUR
        // (saf interpolasyon); 1080p'de ayni hedef 45 px, 180 px kirpilir,
        // 1.42x -> gercek doku. YOLO tam kare tespitte kazanc YOKTUR:
        // 1280x720 ve 1920x1080 letterbox sonrasi IKISI DE 768x432 olur.
        // --proc-out ile eski davranisa (yayin cozunurlugunde isleme) donulur.
        // =============================================================
        // [OLCUM] Gercek kare periyodu: onceki dongu basindan bu ana kadar.
        // Tum isi kapsar. drops=0 olmasi fps'in yerinde oldugunu KANITLAMAZ:
        // capture yavaslarsa appsrc'ye daha az kare gider, encoder rahat
        // yetisir, sayac 0 kalir. Asil olcu bu.
        {
            static long long prevLoopNs = 0;
            const long long tNow = steadyNs();
            if (prevLoopNs) {
                const double dtMs = (double)(tNow - prevLoopNs) / 1e6;
                ++g_loopFrames;
                g_loopSumMs += dtMs;
                if (dtMs > g_loopMaxMs) g_loopMaxMs = dtMs;
                if (dtMs > 40.0) ++g_loopSlow;   // 30 fps butcesi 33.3 ms
            }
            prevLoopNs = tNow;
        }

        cv::Mat procBuf;
        if (g_procAtOut && (frame.cols != outWidth || frame.rows != outHeight))
            cv::resize(frame, procBuf, cv::Size(outWidth, outHeight), 0, 0, cv::INTER_AREA);
        cv::Mat& proc = procBuf.empty() ? frame : procBuf;
        const int pw = proc.cols, ph = proc.rows;


        // Crosshair
     //   cv::drawMarker(display,
     //                  cv::Point(w / 2, h / 2),
     //                  cv::Scalar(0, 255, 0),
     //                  cv::MARKER_CROSS, 20, 2);

        // RC
        RcControl rcCtrl;
        readRcControl(rcCtrl);

        if (!trackingActive) {
            if (rcCtrl.roi_visible && !roiVisible) {
                roiVisible = true;
                roiSize = roiBase;   // [1080] k ile olcekli taban  // Her açılışta 50x50'den başla
                updateCenterRoi(pw, ph);
                std::cout << "ROI visible (CH13) - Size: \n";
            } else if (!rcCtrl.roi_visible && roiVisible) {
                roiVisible = false;
                std::cout << "ROI hidden (CH13)\n";
            }
        }

        if (!trackingActive && roiVisible && rcCtrl.resize_dir != 0) {
            static auto lastResizeTime = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastResizeTime).count();
            
            // 150ms'de 5 piksel artış/azalış (saniyede ~33 piksel)
            if (elapsed >= 60) {
                roiSize += rcCtrl.resize_dir * roiStep;   // [1080] k ile olcekli
                roiSize = std::max(MIN_ROI_SIZE, std::min(roiSize, MAX_ROI_SIZE));
                updateCenterRoi(pw, ph);
                lastResizeTime = now;
                std::cout << "ROI size: " << roiSize << "x" << roiSize << "\n";
            }
        }

        // ===== FRAME (Python beyin) — bu frame'in raporu =====
        // fcx/fcy/fbw/fbh NORMALIZE (0..1). Cozunurluk degisimi PID kazancini
        // bozmaz; ham piksel gonderilmeye baslanirsa her cozunurluk bir kazanc
        // degisimi olur. Boyle TUTULMALI.
        bool        frameValid = false;
        const char* frameState = "idle";
        float fcx = 0.5f, fcy = 0.5f, fbw = 0.0f, fbh = 0.0f;

        // =============================================================
        // [ORT/HYB] TAKIP — tum hesap "proc" (isleme) koordinatinda.
        // Cizim ERTELENIR: display asagida uretilir, kutular S() ile tasinir.
        // =============================================================
        bool  drawSafe = false;
        bool  drawTrk  = false;  cv::Rect  trkBoxP;
        bool  drawRoi  = false;
        bool  drawCand = false;  cv::Rect  candBoxP;
        cv::Scalar trkColor(255, 0, 0);
        std::vector<cv::Rect> detBoxesP;
        float  curScore = -1.0f;
        double curTrkMs = 0.0;

        // ---- guvenli alan (oransal, cozunurlukten BAGIMSIZ)
        const float safeMarginRatio = 0.075f;      // (100-85)/2
        const int   marginX = (int)(pw * safeMarginRatio);
        const int   marginY = (int)(ph * safeMarginRatio);
        const int   minX = marginX, maxX = pw - marginX;
        const int   minY = marginY, maxY = ph - marginY;

#ifdef WITH_ORTRACK
        if (g_detectOnly && slkDet) {
            // ===== SADECE TESPIT: tracker YOK =====
            // YOLO ayri thread'de surekli kosar; sonuc gelene kadar son liste
            // cizilmeye devam eder (aksi halde kutu yanip soner). g_detHoldFrames
            // kare boyunca yeni sonuc gelmezse kutu kaybolur.
            drawSafe = true;
            if (!slkDet->busy()) slkDet->request(proc);

            std::vector<hybrid::Detection> dets;
            if (slkDet->poll(dets)) {
                g_hybDetMs = slkDet->lastMs();
                ++g_hybDetCalls;
                g_hybDetRaw += (long long)dets.size();
                detLast.swap(dets);
                detAge = 0;
            } else if (detAge < 100000) {
                ++detAge;
            }
            if (detAge > g_detHoldFrames) detLast.clear();

            // En guvenli tespit "hedef"; digerleri ince yesil.
            const hybrid::Detection* best = nullptr;
            for (const hybrid::Detection& d : detLast) {
                if (!best || d.score > best->score) best = &d;
                detBoxesP.push_back(d.box.toRect());
            }
            curScore = best ? (float)best->score : -1.0f;
            curTrkMs = 0.0;

            if (best) {
                trkBoxP = best->box.toRect();
                drawTrk = true;
                const int bcx = trkBoxP.x + trkBoxP.width  / 2;
                const int bcy = trkBoxP.y + trkBoxP.height / 2;
                if (bcx < minX || bcx > maxX || bcy < minY || bcy > maxY) {
                    frameValid = false;
                    frameState = "lost";
                } else {
                    frameValid = true;
                    frameState = "tracking";
                    fcx = (float)bcx / (float)pw;
                    fcy = (float)bcy / (float)ph;
                    fbw = (float)trkBoxP.width  / (float)pw;
                    fbh = (float)trkBoxP.height / (float)ph;
                }
            }
            if (!best && hybPrevHadBox) frameState = "lost";
            hybPrevHadBox = (best != nullptr);

            std::ostringstream b;
            b << "DETECT n=" << detLast.size();
            if (best) b << std::fixed << std::setprecision(2) << "  c=" << best->score;
            b << std::fixed << std::setprecision(0) << "  det=" << g_hybDetMs << "ms";
            g_badge = b.str();
            g_badgeActive = (best != nullptr);
        } else if (slk) {
            // ===== HIBRIT-KCF: bir kez kilitle, karisma =====
            // Kilit KENDILIGINDEN dusmez. Tespit AYRI THREAD'de; ana dongu
            // YOLO'yu HIC beklemez, bu yuzden kilitteyken kare periyodu sabit.
            drawSafe = true;

            {
                // CH5 BU MODDA KULLANILMIYOR (ucus modu anahtari). Kilidi
                // dusuren tek yollar: gecersiz kutu / guvenli alan disi /
                // (opsiyonel) varlik kontrolu.
                // 1) KCF (sadece kilitteyken)
                bool tok = false; hybrid::BoxF tb{};
                if (slk->state() == slock::State::LOCK && hybKcf) {
                    float tsc = -1.f, traw = -1.f;
                    tok = hybKcf->track(proc, tb, tsc, traw);
                    curScore = tsc;
                    curTrkMs = hybKcf->lastMs();
                }
                slk->onTrackerResult(tok, tb);

                // 2) Tespit istiyorsak ve worker bossa gonder (BEKLEMEDEN)
                if (slk->detectionWanted() && !slkDet->busy()) slkDet->request(proc);

                // 3) Hazir sonuc varsa isle (gec gelmesi SORUN DEGIL)
                std::vector<hybrid::Detection> dets;
                if (slkDet->poll(dets)) {
                    g_hybDetMs = slkDet->lastMs();
                    ++g_hybDetCalls;
                    g_hybDetRaw += (long long)dets.size();
                    if (g_hybDrawDets)
                        for (const hybrid::Detection& d : dets)
                            detBoxesP.push_back(d.box.toRect());
                    slk->onDetections(dets);
                }

                // 4) Kilit kurulacak mi (ilk kilit veya daha iyi hedefe gecis)
                hybrid::BoxF ib{};
                if (slk->pendingInit(ib)) slk->initDone(hybKcf->initialize(proc, ib));

                if (!slk->lastEvent().empty()) {
                    std::cout << "[LOCK] " << slock::stateName(slk->state())
                              << " : " << slk->lastEvent() << "\n";
                    slk->clearEvent();
                }

                // 5) FRAME + cizim
                if (slk->hasBox()) {
                    trkBoxP = slk->box().toRect();
                    drawTrk = true;
                    // Kontrol sirasinda RENK DEGISTIRMIYORUZ: 50 karede bir
                    // 5 kare turuncuya donmek gorsel olarak rahatsiz ediyor.
                    // Kontrol gostergesi rozette ("KONTROL").
                    trkColor = cv::Scalar(255, 0, 0);
                    const int bcx = trkBoxP.x + trkBoxP.width  / 2;
                    const int bcy = trkBoxP.y + trkBoxP.height / 2;
                    const bool inSafe = !(bcx < minX || bcx > maxX ||
                                          bcy < minY || bcy > maxY);
                    slk->onUsable(inSafe);
                    if (!inSafe) {
                        // Beyne "kullanma" denir; kilit HEMEN dusurulmez.
                        // Ama outsideLimit kare disarida kalirsa makine
                        // aramaya doner (kenara park etmeyi onler).
                        frameValid = false;
                        frameState = "lost";
                    } else {
                        frameValid = true;
                        frameState = "tracking";
                        fcx = (float)bcx / (float)pw;
                        fcy = (float)bcy / (float)ph;
                        fbw = (float)trkBoxP.width  / (float)pw;
                        fbh = (float)trkBoxP.height / (float)ph;
                    }
                } else if (slk->hasCand()) {
                    candBoxP = slk->cand().toRect();
                    drawCand = true;
                }
                if (!slk->hasBox() && hybPrevHadBox) frameState = "lost";
                hybPrevHadBox = slk->hasBox();

                std::ostringstream b;
                b << "KCF-LOCK " << slock::stateName(slk->state());
                if (slk->state() == slock::State::LOCK) {
                    if (curScore >= 0.0f)
                        b << std::fixed << std::setprecision(2) << " s=" << curScore;
                    b << std::fixed << std::setprecision(0)
                      << "  trk=" << curTrkMs << "ms"
                      << "  " << (slk->lockFrames() / 30) << "s";
                    if (slk->clamps() > 0) b << "  clamp" << slk->clamps();
                } else {
                    b << " " << slk->hits() << "/" << slk->window();
                }
                g_badge = b.str();
                g_badgeActive = slk->hasBox();
            }
        } else if (hybMgr) {
            // ===================== HIBRIT: YOLO + ORTrack =====================
            // Akis pipeline.py ile AYNI:
            //   begin_frame -> detection_due -> detect -> apply_detection -> finish_frame
            drawSafe = true;

            // [HYB] PILOT ANA ANAHTARI — CH5 dusuk = kilidi birak + yakalamayi
            // duraklat. Hedefi manager seciyor; pilotun yanlis kilidi reddetme
            // yolu OLMAK ZORUNDA. YOLO recall'u dusuk (olcumunde 934 cagrinin
            // 21'i), yanlis kilit gercek bir olasilik.
            // RC gecersizken (tezgah, FC yok) tracker_stop false gelir -> hibrit
            // calisir; ucusta CH5 her zaman gecerlidir.
            if (rcCtrl.tracker_stop) {
                if (hybMgr->hasBox() || hybMgr->hasCandidate()) {
                    hybMgr->reset("pilot_release");
                    std::cout << "[HYB] PILOT BIRAKTI (CH5 dusuk) - yakalama duraklatildi\n";
                }
                hybPrevHadBox = false;
                g_badge = "HYBRID beklemede (CH5)";
                g_badgeActive = false;
                goto hybDone;          // manager'i bu karede hic surme
            }

            {
            const long long capNs = steadyNs();
            float hsc = -1.f, hraw = -1.f;
            hybMgr->beginFrame(proc, hybFrameIdx, capNs, &hsc, &hraw);

            // g_hybDetMs BILEREK sifirlanmiyor: 1 Hz log tespit YAPILMAYAN
            // bir kareye denk gelince "det=" satirdan dusuyordu.
            if (hybMgr->detectionDue()) {
                const int gen = hybMgr->requestDetection();
                std::vector<hybrid::Detection> dets;
                if (hybYolo->detect(proc, dets)) {
                    g_hybDetMs = hybYolo->lastMs();
                    ++g_hybDetCalls;
                    g_hybDetRaw += (long long)dets.size();
                    if (g_hybDrawDets)
                        for (const hybrid::Detection& d : dets) detBoxesP.push_back(d.box.toRect());
                    const long long doneNs = steadyNs();
                    hybMgr->applyDetection(proc, hybFrameIdx, capNs, hybFrameIdx, capNs,
                                           gen, doneNs, steadyNs(), dets);
                }
            }
            hybMgr->finishFrame(capNs);
            ++hybFrameIdx;

            curScore = hybMgr->haveFiltered() ? (float)hybMgr->filteredScore() : hsc;
            curTrkMs = hybStats->lastMs();
            if (hybMgr->hasBox()) {
                trkBoxP = hybMgr->box().toRect();
                drawTrk = true;
                // zayif (dusuk skor / dogrulama bekliyor) -> turuncu, saglam -> mavi
                trkColor = hybMgr->weak() ? cv::Scalar(0, 165, 255) : cv::Scalar(255, 0, 0);

                const int bcx = trkBoxP.x + trkBoxP.width  / 2;
                const int bcy = trkBoxP.y + trkBoxP.height / 2;
                if (bcx < minX || bcx > maxX || bcy < minY || bcy > maxY) {
                    // MANUEL moddan FARK: kilidi DUSURMUYORUZ. Manager kendi kayip
                    // mantigina sahip; guvenli alandan cikmak burada sadece beyne
                    // "simdilik kullanma" demek. Hedef geri girerse takip surer.
                    frameValid = false;
                    frameState = "lost";
                } else {
                    frameValid = true;
                    frameState = "tracking";
                    fcx = (float)bcx / (float)pw;
                    fcy = (float)bcy / (float)ph;
                    fbw = (float)trkBoxP.width  / (float)pw;
                    fbh = (float)trkBoxP.height / (float)ph;
                }
            } else if (hybMgr->hasCandidate()) {
                candBoxP = hybMgr->candidate().toRect();
                drawCand = true;
            }
            if (!hybMgr->hasBox() && hybPrevHadBox) frameState = "lost";
            hybPrevHadBox = hybMgr->hasBox();

            for (const hybrid::Event& e : hybMgr->events())
                std::cout << "[HYB] " << e.from << " -> " << e.to << " (" << e.reason << ")\n";

            {   // rozet
                std::ostringstream b;
                b << (g_hybridKcf ? "HYBRID-KCF " : "HYBRID ")
                  << hybrid::stateName(hybMgr->state());
                if (hybMgr->hasBox() && hybMgr->haveFiltered())
                    b << std::fixed << std::setprecision(2) << " s=" << hybMgr->filteredScore();
                b << std::fixed << std::setprecision(0)
                  << "  trk=" << hybStats->lastMs() << "ms";
                if (g_hybDetMs > 0.0) b << " det=" << g_hybDetMs << "ms";
                g_badge = b.str();
                g_badgeActive = hybMgr->hasBox();
            }
            }
            hybDone: ;
        } else
#endif
        {
            // ===================== MANUEL: RC + ROI (KCF veya ORTrack) ========
            if (rcCtrl.tracker_start) {
                if (roiVisible && !trackingActive) {
                    if (tracker->init(proc, roiRect)) {
                        trackingActive = true;
                        lostStreak     = 0;
                        std::cout << "TRACKING STARTED (" << tracker->name() << ")\n";
                    } else {
                        std::cout << "TRACKING START FAILED (" << tracker->name()
                                  << ") - ROI gecersiz\n";
                    }
                }
            }
            if (rcCtrl.tracker_stop) {
                if (trackingActive) {
                    trackingActive = false;
                    lostStreak     = 0;
                    tracker->reset();
                    roiVisible     = true;
                    roiSize = roiBase;
                    updateCenterRoi(pw, ph);
                    std::cout << "TRACKING STOPPED - ROI reset\n";
                }
            }

            if (trackingActive) {
                // [ORT] KCF her zaman true doner -> lostStreak 0 kalir -> eski davranis.
                cv::Rect trackedBox;
                const bool frameOk = tracker->update(proc, trackedBox);
                curScore = tracker->score();
                curTrkMs = tracker->lastMs();
                if (frameOk) lostStreak = 0; else ++lostStreak;
                const bool lockLost = (lostStreak >= g_ortLostLimit);

                const int bcx = trackedBox.x + trackedBox.width  / 2;
                const int bcy = trackedBox.y + trackedBox.height / 2;
                drawSafe = true;

                if (lockLost || bcx < minX || bcx > maxX || bcy < minY || bcy > maxY) {
                    trackingActive = false;
                    lostStreak = 0;
                    tracker->reset();
                    roiVisible = true;
                    updateCenterRoi(pw, ph);
                    frameState = "lost";
                    std::cout << "TRACKING AUTO-STOPPED: "
                              << (lockLost ? "tracker gecersiz kare siniri asildi"
                                           : "hedef guvenli alandan cikti (85%)")
                              << "  pos=(" << bcx << "," << bcy << ")"
                              << "  skor=" << tracker->score() << "\n";
                } else {
                    drawTrk = true;
                    trkBoxP = trackedBox;
                    frameValid = true;
                    frameState = "tracking";
                    fcx = (float)bcx / (float)pw;
                    fcy = (float)bcy / (float)ph;
                    fbw = (float)trackedBox.width  / (float)pw;
                    fbh = (float)trackedBox.height / (float)ph;
                }
            } else if (roiVisible) {
                drawRoi = true;
            }

            {   // rozet
                std::ostringstream b;
                b << tracker->name();
                if (trackingActive) {
                    const float sc = tracker->score();
                    if (sc >= 0.0f) b << std::fixed << std::setprecision(2) << "  s=" << sc;
                    b << std::fixed << std::setprecision(1) << "  " << tracker->lastMs() << "ms";
                    if (lostStreak > 0) b << "  miss" << lostStreak;
                }
                g_badge = b.str();
                g_badgeActive = trackingActive;
            }
        }

        // =============================================================
        // [SMOOTH] Kutu boyutu medyan filtresi.
        // SADECE w/h yumusatilir; merkez (fcx/fcy) HAM birakilir — merkez
        // skor tepesinden gelir ve farkli bir kafadir, boyut kadar oynamaz.
        // CSV'ye HEM ham HEM yumusatilmis yazilir, hicbir sey gizlenmez.
        // Tracker'in KENDI kutusu dokunulmadan kalir: yumusatilmis kutuyu
        // geri beslemek arama kirpmasini degistirir ve dogrulanmis yoldan
        // saptirirdi. Burasi sadece CIKIS tarafi.
        // =============================================================
        // box_cx/box_cy: kutunun GERCEK merkezi, gecerlilikten BAGIMSIZ.
        // fcx/fcy guvenli alan reddinde 0.5'te kaliyordu; kutunun nerede
        // oldugu loglardan GORULEMIYORDU.
        double rawBoxW = 0.0, rawBoxH = 0.0, rawBoxCx = -1.0, rawBoxCy = -1.0;
        if (drawTrk) {
            rawBoxW = trkBoxP.width; rawBoxH = trkBoxP.height;
            rawBoxCx = (trkBoxP.x + trkBoxP.width  / 2.0) / (double)pw;
            rawBoxCy = (trkBoxP.y + trkBoxP.height / 2.0) / (double)ph;
        }
        if (g_boxSmooth > 1) {
            if (drawTrk && frameValid) {
                const double sw = smoothW.push(rawBoxW);
                const double sh = smoothH.push(rawBoxH);
                const int bcx = trkBoxP.x + trkBoxP.width  / 2;
                const int bcy = trkBoxP.y + trkBoxP.height / 2;
                trkBoxP = cv::Rect((int)lround(bcx - sw / 2.0), (int)lround(bcy - sh / 2.0),
                                   std::max(1, (int)lround(sw)), std::max(1, (int)lround(sh)));
                fbw = (float)(sw / (double)pw);
                fbh = (float)(sh / (double)ph);
            } else if (!drawTrk) {
                smoothW.clear(); smoothH.clear();   // kilit yokken pencereyi bosalt
            }
        }

        if (trackCsv.is_open()) {
            trackCsv << frameCounter << ',' << steadyMs() << ',' << frameState << ','
                     << (frameValid ? 1 : 0) << ',' << fcx << ',' << fcy << ','
                     << (rawBoxW / (double)pw) << ',' << (rawBoxH / (double)ph) << ','
                     << fbw << ',' << fbh << ',' << curScore << ','
                     << curTrkMs << ',' << g_hybDetMs << ','
                     << rawBoxCx << ',' << rawBoxCy << '\n';
        }
        ++frameCounter;

        // =============================================================
        // [1080] YAYIN KARESI — isleme bitti, simdi kucult ve CIZ
        // =============================================================
        cv::Mat display;
        if (pw != outWidth || ph != outHeight) {
            // INTER_AREA: alan ortalamasi yuksek frekansli gurultuyu siler,
            // encoder ayni bitrate'te daha temiz goruntu uretir.
            cv::resize(proc, display, cv::Size(outWidth, outHeight), 0, 0, cv::INTER_AREA);
        } else {
            // proc == frame (capture buffer). Uzerine cizmek YASAK -> kopya sart.
            display = proc.clone();
        }

        const double sx = (double)outWidth  / (double)pw;
        const double sy = (double)outHeight / (double)ph;
        // proc -> display. 14 px taban: 1080p'de 20 px kilit 720p ekranda 13 px'e
        // duser, pilot goremez ama sistem takip ediyordur. Gorunurlugu koordinat
        // dogrulugundan AYIRIR (kutu ciziminde buyur, FRAME'de degil).
        auto S = [&](const cv::Rect& r) {
            const int rw = std::max(14, (int)lround(r.width  * sx));
            const int rh = std::max(14, (int)lround(r.height * sy));
            const int cx = (int)lround((r.x + r.width  / 2.0) * sx);
            const int cy = (int)lround((r.y + r.height / 2.0) * sy);
            return cv::Rect(cx - rw / 2, cy - rh / 2, rw, rh);
        };

        if (drawSafe) {
            const int dmx = (int)(outWidth  * safeMarginRatio);
            const int dmy = (int)(outHeight * safeMarginRatio);
            cv::rectangle(display,
                          cv::Rect(dmx, dmy, outWidth - 2 * dmx, outHeight - 2 * dmy),
                          cv::Scalar(100, 100, 100), 1, cv::LINE_4);
        }
        for (const cv::Rect& d : detBoxesP)
            cv::rectangle(display, S(d), cv::Scalar(0, 220, 0), 1, cv::LINE_AA);
        if (drawCand) cv::rectangle(display, S(candBoxP), cv::Scalar(0, 255, 255), 2);
        if (drawTrk) {
            const cv::Rect db = S(trkBoxP);
            cv::rectangle(display, db, trkColor, 4);
            cv::circle(display, cv::Point(db.x + db.width / 2, db.y + db.height / 2),
                       4, trkColor, -1);
        }
        if (drawRoi) cv::rectangle(display, S(roiRect), cv::Scalar(40, 40, 255), 4);


        // ===== FRAME mesajini Python beyne gonder =====
        {
            float ang_x = (fcx - 0.5f) * CAMERA_HFOV;
            float ang_y = (fcy - 0.5f) * CAMERA_VFOV;
            sendFrameToClient(g_frameId++, frameValid, fcx, fcy, fbw, fbh,
                              ang_x, ang_y, frameState);
        }

        // ===== [EYLUL] BITRATE KARARI: min(RSSI katmani, RTCP onerisi, CHn tavani) =====
        // RSSI%: ham 0-254 -> %, 255 = gecersiz. 2 s RC_CHANNELS gelmediyse link yok -> taban.
        // Hic gelmediyse (bench, FC yok) -1 -> katman degismez (ust katmanda kalir).
        float rssiPct = -1.0f;
        {
            int raw;
            { std::lock_guard<std::mutex> lock(g_telemMutex); raw = (int)g_telemetry.rssi; }
            int64_t lastMs = g_lastRssiMs.load();
            if (lastMs == 0)                          rssiPct = -1.0f;
            else if (steadyMs() - lastMs > 2000)      rssiPct = 0.0f;
            else if (raw >= 0 && raw < 255)           rssiPct = raw * 100.0f / 254.0f;
        }
        {
            bool anyCtrl = g_rssiAuto || g_autoBitrate.load() || rcCtrl.bitrate_kbps > 0;
            if (anyCtrl) {
                int want = g_brMax;
                if (g_rssiAuto)                                    want = std::min(want, rssiTierKbps(rssiPct));
                if (g_autoBitrate.load() && g_rtcpKbps.load() > 0) want = std::min(want, g_rtcpKbps.load());
                if (rcCtrl.bitrate_kbps > 0)                       want = std::min(want, rcCtrl.bitrate_kbps);
                setEncoderBitrate(want);   // degismediyse no-op
            }
        }

        // ===== [EYLUL] 1 Hz kalibrasyon satiri (rssi / uygulanan kbps / drop / son RTCP kayip) =====
        {
            static int64_t lastLogMs = 0;
            int64_t nowMs = steadyMs();
            if (nowMs - lastLogMs >= 1000) {
                lastLogMs = nowMs;
                std::ostringstream ls;
                ls << std::fixed << std::setprecision(1)
                   << "[LINK] rssi=" << rssiPct << "% kbps=" << g_curKbps
                   << " drops=" << g_dropCount.load()
                   << " loss=" << g_lastLossPermille.load() / 10.0f << "%\n";
                std::cout << ls.str();
            }
        }

        // ===== [ORT/HYB] 1 Hz tracker satiri =====
        // bboxA BUYUMUYORSA hedefe yaklasilmiyor demektir — sahte kilit
        // ilk 2-3 saniyede buradan yakalanir.
        {
            static int64_t lastTrkMs = 0;
            int64_t nowMs = steadyMs();
            if (nowMs - lastTrkMs >= 1000) {
                lastTrkMs = nowMs;
                std::ostringstream ts;
                const double avgMs = g_loopFrames ? (g_loopSumMs / (double)g_loopFrames) : 0.0;
                ts << std::fixed << "[TRK] " << g_badge
                   << std::setprecision(1)
                   << " fps=" << (avgMs > 0.0 ? 1000.0 / avgMs : 0.0)
                   << " dt=" << avgMs << "/" << g_loopMaxMs << "ms"
                   << " slow=" << g_loopSlow
                   << " valid=" << (frameValid ? 1 : 0)
                   << std::setprecision(5) << " bboxA=" << (fbw * fbh);
                g_loopFrames = 0; g_loopSumMs = 0.0; g_loopMaxMs = 0.0; g_loopSlow = 0;
#ifdef WITH_ORTRACK
                if (g_detectOnly)
                    ts << " st=detect n=" << detLast.size()
                       << " age=" << detAge
                       << " yolo=" << g_hybDetCalls << "/" << g_hybDetRaw;
                else if (slk)
                    ts << " st=" << slock::stateName(slk->state())
                       << " hits=" << slk->hits() << "/" << slk->window()
                       << " lockf=" << slk->lockFrames()
                       << " clamp=" << slk->clamps()
                       << " pmiss=" << slk->presenceMiss()
                       << " yolo=" << g_hybDetCalls << "/" << g_hybDetRaw;
                else if (hybMgr)
                    ts << " st=" << hybrid::stateName(hybMgr->state())
                       << " hits=" << hybMgr->hits()
                       << " low=" << hybMgr->lowCount()
                       << " miss=" << hybMgr->misses()
                       << " unver=" << hybMgr->unverified()
                       << " gen=" << hybMgr->generation()
                       << " rej=" << hybMgr->rejected()
                       << " yolo=" << g_hybDetCalls << "/" << g_hybDetRaw;
                else
#endif
                    ts << " miss=" << lostStreak;
                ts << "\n";
                g_hybDetCalls = 0; g_hybDetRaw = 0;
                std::cout << ts.str();
            }
        }

        // ===== OSD OVERLAY =====
        TelemetryData telem_copy;
        {
            std::lock_guard<std::mutex> lock(g_telemMutex);
            telem_copy = g_telemetry;
        }
        // Otonom gostergesi: CH6 HIGH (>1700) ise mod yaninda "(OTONOM)".
        // controller_guided ile AYNI sinyal (CH6 kapisi). FC modundan bagimsiz —
        // firmware GUIDED_NOGPS'i reddetse bile pilot niyetini dogru gosterir.
        {
            std::lock_guard<std::mutex> lock(rcMutex);
            telem_copy.autonomous = (rc_values[5] > 1700);
        }
        g_classicOSD.draw(display, telem_copy);

        // ===== [ORT/HYB] Tracker rozeti — SOL UST =====
        // OSD'de drawTopLeft KAPALI (classic_ardupilot_osd.h), bolge bos.
        // OSD'den SONRA cizilir. MK15/QGC'de KCF ile ayni sekilde gorunur.
        {
            std::ostringstream bs;
            bs << std::fixed << std::setprecision(2) << g_badge;
            const std::string bt = bs.str();
            cv::putText(display, bt, cv::Point(12, 27), cv::FONT_HERSHEY_SIMPLEX,
                        0.6, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
            cv::putText(display, bt, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        g_badgeActive ? cv::Scalar(0, 255, 255) : cv::Scalar(200, 200, 200),
                        2, cv::LINE_AA);
        }

        rtspPushFrame(display);
    }

    keepRunning = false;

    shutdownRtsp();

    // FRAME server kapat (accept/recv unblock + temiz cikis)
    {
        int c = g_frameClientFd.exchange(-1);
        if (c >= 0) { shutdown(c, SHUT_RDWR); close(c); }
        int s = g_frameServerFd.exchange(-1);
        if (s >= 0) { shutdown(s, SHUT_RDWR); close(s); }
    }
    if (frameThread.joinable()) frameThread.join();

    if (rcThread.joinable()) rcThread.join();
    if (mavlink_serial_fd >= 0) close(mavlink_serial_fd);

    return 0;
}
