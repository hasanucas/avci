// =====================================================================
// runtracker.cpp — HUNTER gercek donanim surumu
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

// KCF tracker
#include "kcftracker.hpp"

// MAVLink
#include "../mavlink/common/mavlink.h"

// GStreamer RTSP
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>

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

// Kamera FOV (ang_x/ang_y derece donusumu) — SIYI gercek degerine gore --hfov/--vfov ayarla
static float CAMERA_HFOV = 80.0f;
static float CAMERA_VFOV = 50.534f;

// RTSP OUT globals
static GstElement *g_appsrc = nullptr;
static GMainLoop *g_main_loop = nullptr;
static std::thread g_gstThread;
static std::mutex g_gstMutex;
static std::atomic<bool> g_gstRunning{false};
static GstClockTime g_timestamp = 0;
static guint g_fps_n = 30, g_fps_d = 1;

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
        MAV_DATA_STREAM_RC_CHANNELS,      // RC (CH5/CH12/CH13) + RSSI
        MAV_DATA_STREAM_EXTENDED_STATUS,  // SYS_STATUS/GPS_RAW_INT (OSD)
        MAV_DATA_STREAM_EXTRA1,           // ATTITUDE (OSD)
        MAV_DATA_STREAM_EXTRA2,           // VFR_HUD (OSD)
        // [Tem] GLOBAL_POSITION_INT bu grupta. Istenmezse OSD irtifasi
        // baska bir GCS stream istemedikce guncellenmez.
        MAV_DATA_STREAM_POSITION          // GLOBAL_POSITION_INT (irtifa)
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
                    // [Tem] RSSI: MAVLink2'de ArduPilot RC_CHANNELS (msg 65)
                    // yollar, RC_CHANNELS_RAW'i DEGIL. Eskiden rssi sadece RAW
                    // dalinda okunuyordu -> OSD'de hep 0 kaliyordu.
                    // Kaynak: RSSI_TYPE=2, RSSI_CHANNEL=14 (SIYI RSSI kanali).
                    {
                        std::lock_guard<std::mutex> tl(g_telemMutex);
                        g_telemetry.rssi = rc.rssi;
                    }

                    auto now2   = std::chrono::steady_clock::now();
                    auto dtRate = std::chrono::duration_cast<std::chrono::milliseconds>(now2 - lastRatePrint).count();
                    if (dtRate > 500) {
                        std::lock_guard<std::mutex> lock(rcMutex);
                        std::cout << "[RC RATE] dt=" << dt_ms << " ms, "
                                  << "ch5="  << rc_values[4]
                                  << " ch12=" << rc_values[11]
                                  << " ch13=" << rc_values[12] << std::endl;
                        lastRatePrint = now2;
                    }
                }
                else if (mtype == MAVLINK_MSG_ID_RC_CHANNELS_RAW) {
                    mavlink_rc_channels_raw_t rc;
                    mavlink_msg_rc_channels_raw_decode(&msg, &rc);
                    {
                        std::lock_guard<std::mutex> lock(rcMutex);
                        rc_values[0] = rc.chan1_raw;
                        rc_values[1] = rc.chan2_raw;
                        rc_values[2] = rc.chan3_raw;
                        rc_values[3] = rc.chan4_raw;
                        rc_values[4] = rc.chan5_raw;
                        rc_values[5] = rc.chan6_raw;
                        rc_values[6] = rc.chan7_raw;
                        rc_values[7] = rc.chan8_raw;
                    }
                    // [Tem] MAVLink1 fallback — RSSI.
                    // rcMutex birakildiktan SONRA aliniyor: ic ice kilit yok.
                    {
                        std::lock_guard<std::mutex> tl(g_telemMutex);
                        g_telemetry.rssi = rc.rssi;
                    }
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
                    // [Tem] OSD GPS bloku icin gerekli alanlar
                    g_telemetry.gps_fix_type = gps.fix_type;   // 0-1 yok,2=2D,3=3D,4=DGPS,5+=RTK
                    g_telemetry.gps_lat = gps.lat / 1e7;       // degE7 -> derece
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
    bool roi_visible;   // CH13: <1350 aç, >1600 kapat
    bool tracker_start; // CH5 rising >1700
    bool tracker_stop;  // CH5 falling <1300
    int  resize_dir;    // CH12: -1 küçült (continuous), +1 büyüt (continuous), 0 yok
};

void readRcControl(RcControl &ctrl)
{
    uint16_t ch5, ch12, ch13;
    {
        std::lock_guard<std::mutex> lock(rcMutex);
        ch5  = rc_values[4];    // CH5
        // CH12/CH13 mapping swap:
        ch12 = rc_values[12];   // FÄ°ZÄ°KSEL CH13 â†’ boyut
        ch13 = rc_values[11];   // FÄ°ZÄ°KSEL CH12 â†’ ROI ON/OFF
    }

    static bool lastRoi = false;
    bool roi = lastRoi;

    if (ch13 < 1350)      roi = true;
    else if (ch13 > 1600) roi = false;

    ctrl.roi_visible = roi;
    lastRoi = roi;

    static uint16_t lastCh12 = 1500;
    ctrl.resize_dir = 0;
    // Continuous resize: tuşa basılı tuttuğu sürece büyüt/küçült
    if (ch12 < 1330) {
        ctrl.resize_dir = -1;  // Sürekli küçült
    } else if (ch12 > 1650) {
        ctrl.resize_dir = +1;  // Sürekli büyüt
    }
    lastCh12 = ch12;

    static uint16_t lastCh5 = 1500;
    ctrl.tracker_start = false;
    ctrl.tracker_stop  = false;
    if (ch5 > 1700 && lastCh5 <= 1700) {
        ctrl.tracker_start = true;
    } else if (ch5 < 1300 && lastCh5 >= 1300) {
        ctrl.tracker_stop = true;
    }
    lastCh5 = ch5;
}

// ================= RTSP OUTPUT (GStreamer) =================

static void media_configure_cb(GstRTSPMediaFactory *factory,
                               GstRTSPMedia *media,
                               gpointer /*user_data*/)
{
    std::lock_guard<std::mutex> lock(g_gstMutex);
    
    // ✅ ÇÖZÜM: Eğer appsrc zaten varsa, yeni client paylaşacak
    if (g_appsrc) {
        std::cout << "[RTSP] ✓ Yeni client bağlandı, mevcut stream paylaşılıyor\n";
        return;
    }
    
    GstElement *pipeline = gst_rtsp_media_get_element(media);
    GstElement *appsrc = gst_bin_get_by_name_recurse_up(
        GST_BIN(pipeline), "mysrc");
    
    g_appsrc = appsrc;
    g_timestamp = 0;
    
    // ✅ Media'yı reusable yap - aynı pipeline tüm clientlar için
    gst_rtsp_media_set_reusable(media, TRUE);
    
    std::cout << "[RTSP] ✓ Pipeline ilk kez oluşturuldu\n";
}

static void media_unprepared_cb(GstRTSPMedia *media, gpointer /*user_data*/)
{
    // ✅ SHARED pipeline kullanıldığında bu callback çalışmamalı
    // Eğer çalışırsa bile appsrc'yi temizleme - başka clientlar hala bağlı olabilir
    std::cout << "[RTSP] Client disconnected (pipeline shared, appsrc korunuyor)\n";
}

bool initRtspServer(int width, int height, int fps, int bitrateKbps)
{
    g_fps_n = fps;
    g_fps_d = 1;

    int argc = 0;
    char **argv = nullptr;
    gst_init(&argc, &argv);

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
        // ===== APPsrc: KCF'ten gelen BGR frame'ler =====
        << "appsrc name=mysrc is-live=true block=false format=time "
        << "do-timestamp=true max-bytes=0 "
        << "caps=video/x-raw,format=BGR,width=" << width
        << ",height=" << height
        << ",framerate=" << fps << "/1 "

        // BGR -> I420
        << "! videoconvert "
        << "! video/x-raw,format=I420 "

        // ===== x264enc: dÃ¼ÅŸÃ¼k gecikme + baseline profil =====
        << "! x264enc "

        // Profil
        << "profile=baseline "
        << "level=3.1 "

        // Performans / dÃ¼ÅŸÃ¼k gecikme
        << "tune=zerolatency "
        << "speed-preset=ultrafast "

        // CBR
        << "nal-hrd=cbr "
        << "pass=cbr "
        << "bitrate=" << bitrateKbps << " "
        << "vbv-buf-capacity=" << bitrateKbps << " "

        // Keyframe aralÄ±ÄŸÄ± (fps ile aynÄ±)
        << "key-int-max=" << fps << " "
        << "min-keyint="   << fps << " "

        // Baseline gereksinimleri
        << "bframes=0 "
        << "cabac=0 "
        << "dct8x8=0 "
        << "ref=1 "
        << "weightp=0 "

        // Multi-thread
        << "threads=4 "
        << "sliced-threads=true "

        // Ek gecikmeyi kapat
        << "sync-lookahead=0 "
        << "rc-lookahead=0 "

        // ===== H.264 caps: AVC stream (Annex-B deÄŸil) =====
        << "! video/x-h264,profile=baseline,stream-format=avc,alignment=au "

        // ===== h264parse: SPS/PPS =====
        << "! h264parse config-interval=1 "

        // ===== KÃ¼Ã§Ã¼k queue: donma yerine frame drop =====
        << "! queue "
        << "max-size-buffers=1 "
        << "max-size-bytes=0 "
        << "max-size-time=0 "
        << "leaky=2 "

        // ===== RTP payload (DÄ°KKAT: name=pay0) =====
        << "! rtph264pay name=pay0 "
        << "pt=96 "
        << "config-interval=1 "
        << "mtu=1200 ";


    gst_rtsp_media_factory_set_launch(factory, launch.str().c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);  // ✅ SHARED: Tek pipeline, tüm clientlar paylaşır
    g_signal_connect(factory, "media-configure",
                     (GCallback)media_configure_cb, nullptr);
    g_signal_connect(factory, "media-unprepared",
                     (GCallback)media_unprepared_cb, nullptr);

    gst_rtsp_mount_points_add_factory(mounts, "/main.264", factory);
    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server, NULL) == 0) {
        std::cerr << "Failed to attach RTSP server\n";
        return false;
    }

    g_gstRunning = true;
    g_gstThread = std::thread([](){
        g_main_loop_run(g_main_loop);
        g_gstRunning = false;
    });

    std::cout << "=========================================\n";
    std::cout << "SIYI-COMPATIBLE RTSP Server Started\n";
    std::cout << "URL: rtsp://<ORIN_IP>:8554/main.264\n";
    std::cout << "---\n";
    std::cout << "Optimizations:\n";
    std::cout << "  â€¢ H264 Baseline Profile (SÄ±yÄ± FPV compatible)\n";
    std::cout << "  â€¢ RTSP Path: /main.264 (matches camera)\n";
    std::cout << "  â€¢ Multi-threading: 4 cores\n";
    std::cout << "  â€¢ CBR encoding: " << bitrateKbps << " kbps\n";
    std::cout << "  â€¢ Keyframe interval: 15 frames (0.6s @ 25fps)\n";
    std::cout << "  â€¢ SPS/PPS: Every keyframe\n";
    std::cout << "  â€¢ Latency: Minimal (1 frame buffer)\n";
    std::cout << "=========================================\n";

    return true;
}

void rtspPushFrame(const cv::Mat &frameBGR)
{
    std::lock_guard<std::mutex> lock(g_gstMutex);
    if (!g_appsrc) return;

    cv::Mat bgr;
    if (frameBGR.type() == CV_8UC3) {
        bgr = frameBGR;
    } else {
        cv::cvtColor(frameBGR, bgr, cv::COLOR_GRAY2BGR);
    }

    int size = static_cast<int>(bgr.total() * bgr.elemSize());
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    std::memcpy(map.data, bgr.data, size);
    gst_buffer_unmap(buffer, &map);

    GstClockTime duration = gst_util_uint64_scale_int(GST_SECOND, 1, g_fps_n);
    GST_BUFFER_PTS(buffer) = g_timestamp;
    GST_BUFFER_DTS(buffer) = g_timestamp;
    GST_BUFFER_DURATION(buffer) = duration;
    g_timestamp += duration;

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(g_appsrc), buffer);
    if (ret != GST_FLOW_OK) {
        // Client disconnect veya pipeline hatası
        if (ret == GST_FLOW_FLUSHING || ret == GST_FLOW_EOS) {
            // Pipeline kapanmış, referansı temizle
            gst_object_unref(g_appsrc);
            g_appsrc = nullptr;
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
        } else if (!strcmp(argv[i], "--help")) {
            std::cout <<
                "Usage: " << argv[0]
                << " [--rtsp URL] [--mavlink-udp HOST:PORT] [--frame-port N]\n"
                << "          [--hfov DEG] [--vfov DEG] [--width W] [--height H]\n"
                << "          [--serial DEV --baud BPS]  (router YOKKEN dogrudan seri)\n"
                "Example (mavlink-router ile):\n"
                "  " << argv[0]
                << " --rtsp rtsp://192.168.144.25:8554/main.264"
                << " --mavlink-udp 127.0.0.1:14550 --frame-port 9999\n";
            return 0;
        }
    }

    std::cout << "=== RTSP + MAVLink KCF TRACKER (RTSP OUT + TCP FRAME) ===\n";
    std::cout << "RTSP IN  : " << rtspUrl    << "\n";
    if (g_useUdp)
        std::cout << "MAVLink  : UDP " << mavlinkUdp << " (mavlink-router)\n";
    else
        std::cout << "MAVLink  : Serial " << serialPort << " @ " << baudrate
                  << "  (DOGRUDAN — router KAPALI olmali!)\n";
    std::cout << "FRAME    : TCP port " << framePort << " (Python beyin)\n";
    std::cout << "FOV      : HFOV=" << CAMERA_HFOV << "  VFOV=" << CAMERA_VFOV << "\n";
    std::cout << "Out size : " << outWidth << "x" << outHeight << "\n\n";

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
    
    std::string gstIn =
    "rtspsrc location=" + rtspUrl + " protocols=tcp latency=0 ! "
    "rtph265depay ! h265parse ! "
    "nvv4l2decoder enable-max-performance=1 ! "
    "nvvidconv ! video/x-raw,format=BGRx ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink sync=false max-buffers=1 drop=true";

    cv::VideoCapture cap(gstIn, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "RTSP (GStreamer/NVDEC) aÃ§Ä±lmadÄ±: " << gstIn << "\n";
        keepRunning = false;
    }
    
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
//    cv::VideoCapture cap(rtspUrl);
//    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
//    if (!cap.isOpened()) {
//        std::cerr << "RTSP aÃ§Ä±lmadÄ±: " << rtspUrl << "\n";
//        keepRunning = false;
//    }

    cv::Mat frame;
    if (keepRunning && !cap.read(frame)) {
        std::cerr << "RTSP ilk frame alÄ±namadÄ±\n";
        keepRunning = false;
    }

    if (!keepRunning) {
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

    std::cout << "Source size: " << srcW   << "x" << srcH   << "\n";
    std::cout << "Using size : " << outWidth << "x" << outHeight << "\n";

    int outFps         = 30;
    int outBitrateKbps = 2000;

    if (!initRtspServer(outWidth, outHeight, outFps, outBitrateKbps)) {
        std::cerr << "RTSP server Ã§Ä±kÄ±ÅŸÄ± yok.\n";
    }

    // KCF Tracker
    KCFTracker tracker(true, true, true, false);
    bool trackingActive = false;
    bool roiVisible     = false;

    // ROI başlangıç: 50x50, maksimum: 150x150
    int roiSize = 50;
    const int MIN_ROI_SIZE = 20;
    const int MAX_ROI_SIZE = 150;
    cv::Rect roiRect;

    auto updateCenterRoi = [&](int w, int h){
        int cx   = w / 2;
        int cy   = h / 2;
        int half = roiSize / 2;
        roiRect  = cv::Rect(cx - half, cy - half, roiSize, roiSize);
    };
    updateCenterRoi(outWidth, outHeight);

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

        cv::Mat proc;
        if (frame.cols != outWidth || frame.rows != outHeight) {
            cv::resize(frame, proc, cv::Size(outWidth, outHeight));
        } else {
            proc = frame;
        }

        cv::Mat display = proc.clone();

        int w = display.cols;
        int h = display.rows;

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
                roiSize = 50;  // Her açılışta 50x50'den başla
                updateCenterRoi(w, h);
                std::cout << "ROI visible (CH13) - Size: 50x50\n";
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
                roiSize += rcCtrl.resize_dir * 4;
                roiSize = std::max(MIN_ROI_SIZE, std::min(roiSize, MAX_ROI_SIZE));
                updateCenterRoi(w, h);
                lastResizeTime = now;
                std::cout << "ROI size: " << roiSize << "x" << roiSize << "\n";
            }
        }

        if (rcCtrl.tracker_start) {
            if (roiVisible && !trackingActive) {
                tracker.init(roiRect, display);
                trackingActive = true;
                std::cout << "TRACKING STARTED\n";
            }
        }
        if (rcCtrl.tracker_stop) {
            if (trackingActive) {
                trackingActive = false;
                roiVisible     = true;
                roiSize = 50;  // Stop'ta 50x50'ye dön
                updateCenterRoi(w, h);
                std::cout << "TRACKING STOPPED - ROI reset to 50x50\n";
            }
        }

        // ===== FRAME (Python beyin) — bu frame'in raporu =====
        bool        frameValid = false;
        const char* frameState = "idle";
        float fcx = 0.5f, fcy = 0.5f, fbw = 0.0f, fbh = 0.0f;

        if (trackingActive) {
            cv::Rect trackedBox = tracker.update(display);
            
            // Tracked box merkezini hesapla
            int boxCenterX = trackedBox.x + trackedBox.width / 2;
            int boxCenterY = trackedBox.y + trackedBox.height / 2;
            
            // Ekranın %85'lik güvenli alanının sınırlarını hesapla
            // %15 boşluk = her yandan %7.5
            float safeMarginRatio = 0.075f; // (100-85)/2 = 7.5%
            int marginX = static_cast<int>(w * safeMarginRatio);
            int marginY = static_cast<int>(h * safeMarginRatio);
            int minX = marginX;
            int maxX = w - marginX;
            int minY = marginY;
            int maxY = h - marginY;
            
            // Güvenli alanı çiz (ince gri çerçeve - görsel referans için)
            cv::Rect safeZone(minX, minY, maxX - minX, maxY - minY);
            cv::rectangle(display, safeZone, cv::Scalar(100, 100, 100), 1, cv::LINE_4);
            
            // Merkez güvenli alan dışına çıktı mı kontrol et
            if (boxCenterX < minX || boxCenterX > maxX || 
                boxCenterY < minY || boxCenterY > maxY) {
                // Tracking otomatik durdur
                trackingActive = false;
                roiVisible = true;
                updateCenterRoi(w, h);
                frameState = "lost";   // bu frame'de hedef kayboldu (bbox_valid=0)
                std::cout << "⚠️  TRACKING AUTO-STOPPED: Target left safe zone (85%)\n";
                std::cout << "    Target pos: (" << boxCenterX << "," << boxCenterY << ")\n";
                std::cout << "    ROI reset to center. Ready for new track.\n";
            } else {
                // Normal tracking devam
                // Turuncu: BGR(0, 165, 255)
                cv::rectangle(display, trackedBox, cv::Scalar(255, 0, 0), 4);
                
                // Merkez noktasını işaretle (turuncu)
                cv::circle(display, cv::Point(boxCenterX, boxCenterY), 
                          4, cv::Scalar(255, 0, 0), -1);

                // FRAME raporu: aktif takip (normalize bbox)
                frameValid = true;
                frameState = "tracking";
                fcx = (float)boxCenterX     / (float)w;
                fcy = (float)boxCenterY     / (float)h;
                fbw = (float)trackedBox.width  / (float)w;
                fbh = (float)trackedBox.height / (float)h;
            }
        } else if (roiVisible) {
            // Mavi: BGR(255, 0, 0)
            cv::rectangle(display, roiRect, cv::Scalar(40, 40, 255), 4);
        }

        // ===== FRAME mesajini Python beyne gonder =====
        {
            float ang_x = (fcx - 0.5f) * CAMERA_HFOV;
            float ang_y = (fcy - 0.5f) * CAMERA_VFOV;
            sendFrameToClient(g_frameId++, frameValid, fcx, fcy, fbw, fbh,
                              ang_x, ang_y, frameState);
        }

        // ===== OSD OVERLAY =====
        TelemetryData telem_copy;
        {
            std::lock_guard<std::mutex> lock(g_telemMutex);
            telem_copy = g_telemetry;
        }
        g_classicOSD.draw(display, telem_copy);

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
