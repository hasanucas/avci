// =====================================================================
// yo_log.hpp — [YO] --yolo-ortrack performans logu (ucus sonrasi analiz).
//
// --yo-log DIR verilince DIR icine:
//   frames.csv  her islenen kare: durum, kutu, skorlar, sayaclar, tarama,
//               duzeltme, dongu suresi + telemetri (roll/pitch/yaw/irtifa,
//               CH6, ucus modu, arm, rssi)
//   events.csv  durum gecisleri (+ kayip kanitlari), duzeltmeler, catch-up,
//               reddedilen taramalar, pilot bekletme/devam
//   scans.csv   her biten tarama: kaynak kare, yas, sure, karo/infer/merge
//               ms, kutu sayilari, TUM tespitler
//   config.txt  calisma parametreleri (manager + SAHI + motorlar)
//
// Yazim tamponlu; diske saniyede bir flush (kare basina flush YOK -> I/O
// takilmasi ana donguyu durdurmaz). SIGINT ile duzgun kapanir; ani guc
// kesilmesinde en fazla ~1 s kaybolur.
// =====================================================================
#pragma once

#include "yo_manager.hpp"
#include "yo_sahi.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace yo {

struct FrameRow {
    long long frame = 0;
    long long tMs = 0;            // steady_clock ms (events/scans ile ayni eksen)
    double    posMs = NAN;        // GStreamer konumu (ham kayitla hizalama icin, yaklasik)
    double    loopMs = NAN;       // onceki kareden bu kareye dongu periyodu
    bool      hold = false;       // CH12 ile beklemede
    const char* state = "idle";
    const char* wire  = "idle";   // FRAME'e giden durum
    bool      valid = false;
    bool      hasBox = false; BoxF box{};
    double    cx = 0.5, cy = 0.5, w = 0.0, h = 0.0;   // FRAME (normalize)
    bool      tracked = false, trackValid = false;
    double    score = NAN, raw = NAN, filt = NAN;
    bool      weak = false;
    int       low = 0, misses = 0, unver = 0, gen = 0, hits = 0;
    long long rejected = 0;
    double    unsupMs = NAN, lockMs = NAN, trkMs = NAN;
    bool      hasCand = false; BoxF cand{};
    // bu karede biten tarama
    bool      scanDone = false, scanOk = false, scanApplied = false;
    long long scanSrc = -1;
    double    scanAgeMs = NAN, scanMs = NAN;
    int       tiles = 0, before = 0, after = 0;
    ApplyInfo apply;
    long long inflight = -1;
    // telemetri
    double    roll = NAN, pitch = NAN, yaw = NAN, alt = NAN;
    int       ch6 = 0, armed = 0, rssi = -1;
    std::string mode;
};

class RunLog {
public:
    bool open(const std::string& dir, const std::string& configText)
    {
        dir_ = dir;
        frames_.open(dir + "/frames.csv");
        events_.open(dir + "/events.csv");
        scans_.open(dir + "/scans.csv");
        std::ofstream cfg(dir + "/config.txt");
        if (!frames_ || !events_ || !scans_ || !cfg) return false;
        cfg << configText;
        frames_ << "frame,t_ms,pos_ms,loop_ms,hold,state,wire,valid,x1,y1,x2,y2,cx,cy,w,h,"
                   "tracked,trk_valid,score,raw,filt,weak,low,misses,unver,gen,hits,rejected,"
                   "unsup_ms,lock_ms,trk_ms,cand,cx1,cy1,cx2,cy2,"
                   "scan_done,scan_ok,scan_applied,scan_src,scan_age_ms,scan_ms,tiles,dets_before,dets_after,"
                   "verified,missed,acquired,candidate,corrected,match_iou,match_score,corr_dx,corr_dy,inflight,"
                   "roll,pitch,yaw,alt,ch6,mode,armed,rssi\n";
        events_ << "frame,t_ms,type,from,to,reason,src,to_frame,dx,dy,value,generation,"
                   "ev_filtered,ev_low,ev_misses,ev_unverified,ev_unsup_ms,ev_det_avg,ev_samples,ev_low_ms,ev_box_scale\n";
        scans_ << "frame,t_ms,src,age_ms,ok,applied,total_ms,tile_ms,infer_ms,merge_ms,tiles,before,after,error,dets\n";
        frames_ << std::setprecision(10);
        events_ << std::setprecision(10);
        scans_  << std::setprecision(10);
        return true;
    }
    bool isOpen() const { return frames_.is_open(); }
    const std::string& dir() const { return dir_; }

    void frame(const FrameRow& r)
    {
        auto b = [&](bool has, const BoxF& x) {
            if (has) frames_ << x.x1 << ',' << x.y1 << ',' << x.x2 << ',' << x.y2;
            else     frames_ << ",,,";
        };
        frames_ << r.frame << ',' << r.tMs << ',' << n(r.posMs) << ',' << n(r.loopMs) << ','
                << (int)r.hold << ',' << r.state << ',' << r.wire << ',' << (int)r.valid << ',';
        b(r.hasBox, r.box);
        frames_ << ',' << r.cx << ',' << r.cy << ',' << r.w << ',' << r.h << ','
                << (int)r.tracked << ',' << (int)r.trackValid << ',' << n(r.score) << ',' << n(r.raw) << ','
                << n(r.filt) << ',' << (int)r.weak << ',' << r.low << ',' << r.misses << ',' << r.unver << ','
                << r.gen << ',' << r.hits << ',' << r.rejected << ','
                << n(r.unsupMs) << ',' << n(r.lockMs) << ',' << n(r.trkMs) << ',' << (int)r.hasCand << ',';
        b(r.hasCand, r.cand);
        frames_ << ',' << (int)r.scanDone << ',' << (int)r.scanOk << ',' << (int)r.scanApplied << ','
                << r.scanSrc << ',' << n(r.scanAgeMs) << ',' << n(r.scanMs) << ','
                << r.tiles << ',' << r.before << ',' << r.after << ','
                << (int)r.apply.verified << ',' << (int)r.apply.missed << ',' << (int)r.apply.acquired << ','
                << (int)r.apply.candidate << ',' << (int)r.apply.corrected << ','
                << n(r.apply.matchIoU) << ',' << n(r.apply.matchScore) << ','
                << (r.apply.corrected ? r.apply.dx : 0.0) << ',' << (r.apply.corrected ? r.apply.dy : 0.0) << ','
                << r.inflight << ','
                << n(r.roll) << ',' << n(r.pitch) << ',' << n(r.yaw) << ',' << n(r.alt) << ','
                << r.ch6 << ',' << r.mode << ',' << r.armed << ',' << r.rssi << '\n';
    }

    void event(long long frame, long long tMs, const Event& e)
    {
        const char t[2] = {e.type, 0};
        events_ << frame << ',' << tMs << ',' << t << ',' << e.from << ',' << e.to << ',' << e.reason << ','
                << e.srcFrame << ',' << e.toFrame << ',' << n(e.dx) << ',' << n(e.dy) << ',' << n(e.value) << ','
                << e.generation << ',';
        if (e.hasEvidence)
            events_ << n(e.filtered) << ',' << e.low << ',' << e.misses << ',' << e.unverified << ','
                    << n(e.unsupportedMs) << ',' << n(e.detAvg) << ',' << e.samples << ','
                    << n(e.lowMs) << ',' << n(e.boxScale) << '\n';
        else
            events_ << ",,,,,,,,\n";
    }

    // Pilot bekletme/devam vb. manager disi olaylar
    void note(long long frame, long long tMs, const std::string& what)
    {
        events_ << frame << ',' << tMs << ",P,,," << what << ",-1,-1,,,,-1,,,,,,,,,\n";
    }

    void scan(long long frame, long long tMs, long long src, double ageMs, bool ok, bool applied,
              const ScanStats& s, const std::string& error, const std::vector<Detection>& dets)
    {
        scans_ << frame << ',' << tMs << ',' << src << ',' << n(ageMs) << ',' << (int)ok << ',' << (int)applied << ','
               << s.totalMs << ',' << s.tileMsSum << ',' << s.inferMsSum << ',' << s.mergeMs << ','
               << s.tiles << ',' << s.before << ',' << s.after << ',' << clean(error) << ',';
        for (size_t i = 0; i < dets.size(); ++i) {
            const Detection& d = dets[i];
            if (i) scans_ << ';';
            scans_ << std::fixed << std::setprecision(1) << d.box.x1 << ':' << d.box.y1 << ':'
                   << d.box.x2 << ':' << d.box.y2 << ':' << std::setprecision(4) << d.score << ':' << d.classId;
            scans_ << std::defaultfloat << std::setprecision(10);
        }
        scans_ << '\n';
    }

    void flushIfDue(long long nowMs)
    {
        if (nowMs - lastFlush_ < 1000) return;
        lastFlush_ = nowMs;
        frames_.flush(); events_.flush(); scans_.flush();
    }
    void close() { if (frames_.is_open()) { frames_.close(); events_.close(); scans_.close(); } }
    ~RunLog() { close(); }

private:
    static std::string n(double v) {
        if (!std::isfinite(v)) return "";
        std::ostringstream o; o << std::setprecision(10) << v; return o.str();
    }
    static std::string clean(const std::string& s) {
        std::string o = s;
        for (char& c : o) if (c == ',' || c == '\n' || c == '\r') c = ' ';
        return o;
    }
    std::string dir_;
    std::ofstream frames_, events_, scans_;
    long long lastFlush_ = 0;
};

} // namespace yo
