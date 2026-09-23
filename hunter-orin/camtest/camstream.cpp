// camstream.cpp — USB kamera (C270) -> OpenCV overlay -> RTSP (MK15 test)
//
// Kullanim:
//   ./camstream                      # /dev/video0, MJPEG 1280x720
//   ./camstream "<gst pipeline>"     # istege bagli: ... ! video/x-raw,format=BGR ! appsink
//
// Kumanda URL: rtsp://192.168.144.50:<PORT><MOUNT>

#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <glib-unix.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

// ---- AYARLAR ---------------------------------------------------------------
// PORT ve MOUNT runtracker'dakiyle ayni olsun -> MK15'te hicbir sey degismez
static const char* PORT  = "8554";
static const char* MOUNT = "/main.264";
static const int   W = 1280, H = 720, FPS = 30;

// Encoder: runtracker'da MK15 ile calistigi kanitlanmis satir varsa buraya onu koy
static const char* ENC_PIPE =
    "( appsrc name=src "
    "! queue max-size-buffers=2 leaky=downstream "
    "! videoconvert ! video/x-raw,format=I420 "
    "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2500 key-int-max=30 "
    "! video/x-h264,profile=baseline "
    "! rtph264pay name=pay0 pt=96 config-interval=1 )";
// ----------------------------------------------------------------------------

static std::mutex        g_mtx;
static GstElement*       g_appsrc = nullptr;   // istemci bagliyken dolu
static std::atomic<bool> g_run{true};
static GMainLoop*        g_loop = nullptr;

static void on_unprepared(GstRTSPMedia*, gpointer) {
    std::lock_guard<std::mutex> lk(g_mtx);
    if (g_appsrc) { gst_object_unref(g_appsrc); g_appsrc = nullptr; }
    g_print("[rtsp] istemci kalmadi, yayin durdu\n");
}

static void on_media_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer) {
    GstElement* bin = gst_rtsp_media_get_element(media);
    GstElement* src = gst_bin_get_by_name_recurse_up(GST_BIN(bin), "src");
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width", G_TYPE_INT, W, "height", G_TYPE_INT, H,
        "framerate", GST_TYPE_FRACTION, FPS, 1, NULL);
    g_object_set(src, "caps", caps, "format", GST_FORMAT_TIME,
                 "is-live", TRUE, "do-timestamp", TRUE, NULL);
    gst_caps_unref(caps);
    g_signal_connect(media, "unprepared", G_CALLBACK(on_unprepared), NULL);
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        if (g_appsrc) gst_object_unref(g_appsrc);
        g_appsrc = src;
    }
    gst_object_unref(bin);
    g_print("[rtsp] istemci baglandi, yayin basladi\n");
}

static void draw_overlay(cv::Mat& img, long n, double fps) {
    const cv::Point c(img.cols / 2, img.rows / 2);
    const cv::Scalar green(0, 255, 0), white(255, 255, 255), black(0, 0, 0);

    // Nisangah (ortasi bos arti) + kutu
    cv::line(img, c + cv::Point(-40, 0), c + cv::Point(-10, 0), green, 2);
    cv::line(img, c + cv::Point( 10, 0), c + cv::Point( 40, 0), green, 2);
    cv::line(img, c + cv::Point(0, -40), c + cv::Point(0, -10), green, 2);
    cv::line(img, c + cv::Point(0,  10), c + cv::Point(0,  40), green, 2);
    cv::rectangle(img, cv::Rect(c.x - 100, c.y - 75, 200, 150), green, 2);

    // Yazilar (siyah golge + beyaz)
    char buf[64];
    auto text = [&](const char* s, cv::Point p, double sc) {
        cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, sc, black, 5, cv::LINE_AA);
        cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, sc, white, 2, cv::LINE_AA);
    };
    text("HUNTER TEST", {20, 45}, 1.2);
    snprintf(buf, sizeof buf, "FPS %.1f   frame %ld", fps, n);
    text(buf, {20, img.rows - 20}, 0.8);
}

static void push_frame(const cv::Mat& img) {
    std::lock_guard<std::mutex> lk(g_mtx);
    if (!g_appsrc) return;                          // kimse izlemiyor
    const gsize size = img.total() * img.elemSize();
    GstBuffer* buf = gst_buffer_new_allocate(NULL, size, NULL);
    gst_buffer_fill(buf, 0, img.data, size);
    GstFlowReturn ret;
    g_signal_emit_by_name(g_appsrc, "push-buffer", buf, &ret);
    gst_buffer_unref(buf);
}

static void capture_loop(std::string cam_pipe) {
    cv::VideoCapture cap;
    if (cam_pipe.empty()) {
        cap.open(0, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, W);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, H);
        cap.set(cv::CAP_PROP_FPS, FPS);
    } else {
        cap.open(cam_pipe, cv::CAP_GSTREAMER);
    }
    if (!cap.isOpened()) {
        fprintf(stderr, "[cam] kamera acilamadi\n");
        g_main_loop_quit(g_loop);
        return;
    }

    cv::Mat frame;
    long n = 0;
    int cnt = 0;
    double fps = 0.0;
    auto t0 = std::chrono::steady_clock::now();

    while (g_run) {
        if (!cap.read(frame) || frame.empty()) {
            fprintf(stderr, "[cam] kare okunamadi\n");
            break;
        }
        if (n == 0) g_print("[cam] ilk kare geldi: %dx%d\n", frame.cols, frame.rows);
        if (frame.cols != W || frame.rows != H) cv::resize(frame, frame, cv::Size(W, H));

        ++cnt;
        const double dt = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (dt >= 1.0) { fps = cnt / dt; cnt = 0; t0 = std::chrono::steady_clock::now(); }

        draw_overlay(frame, n++, fps);
        push_frame(frame);
    }
    g_main_loop_quit(g_loop);
}

static gboolean on_sigint(gpointer) {
    g_run = false;
    g_main_loop_quit(g_loop);
    return G_SOURCE_REMOVE;
}

int main(int argc, char** argv) {
    gst_init(&argc, &argv);
    const std::string cam_pipe = argc > 1 ? argv[1] : "";

    GstRTSPServer* server = gst_rtsp_server_new();
    gst_rtsp_server_set_service(server, PORT);
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, ENC_PIPE);
    gst_rtsp_media_factory_set_shared(factory, TRUE);   // MK15 + laptop ayni anda izleyebilir
    g_signal_connect(factory, "media-configure", G_CALLBACK(on_media_configure), NULL);
    gst_rtsp_mount_points_add_factory(mounts, MOUNT, factory);
    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server, NULL) == 0) {
        fprintf(stderr, "[rtsp] port %s acilamadi (runtracker calisiyor olabilir)\n", PORT);
        return 1;
    }
    g_print("[rtsp] hazir: rtsp://<orin-ip>:%s%s\n", PORT, MOUNT);

    g_loop = g_main_loop_new(NULL, FALSE);
    g_unix_signal_add(SIGINT, on_sigint, NULL);
    std::thread cam(capture_loop, cam_pipe);
    g_main_loop_run(g_loop);

    g_run = false;
    cam.join();
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        if (g_appsrc) { gst_object_unref(g_appsrc); g_appsrc = nullptr; }
    }
    g_main_loop_unref(g_loop);
    g_object_unref(server);
    return 0;
}
