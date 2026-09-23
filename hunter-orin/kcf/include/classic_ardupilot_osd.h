// classic_ardupilot_osd.h
// FPV Style - Noktalı Pitch Ladder + Heading Tape Compass
// Görsellerdeki gibi tam FPV simulator tarzı

#ifndef CLASSIC_ARDUPILOT_OSD_H
#define CLASSIC_ARDUPILOT_OSD_H

#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <vector>
#include <chrono>      // [Tem] blinkOn() icin

// ==================== TELEMETRY DATA ====================
struct TelemetryData {
    float battery_voltage = 0.0f;
    float battery_current = 0.0f;
    int battery_remaining = 0;
    float battery_consumed_mah = 0.0f;
    
    int gps_sat_count = 0;
    int gps_fix_type = 0;
    float gps_hdop = 99.0f;
    double gps_lat = 0.0;
    double gps_lon = 0.0;
    
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    
    float altitude_msl = 0.0f;
    float altitude_rel = 0.0f;
    float ground_speed = 0.0f;
    float air_speed = 0.0f;
    float climb_rate = 0.0f;
    
    std::string flight_mode = "UNKNOWN";
    bool armed = false;
    bool autonomous = false;   // CH6 HIGH -> otonom; OSD'de mod yaninda "(OTONOM)" gosterilir
    
    double home_lat = 0.0;
    double home_lon = 0.0;
    bool home_set = false;
    
    int rssi = 0;
    
    std::string fc_identifier = "CUAV_V5P";
    std::string fc_version = "4.5.0";
    
    uint64_t last_update_ms = 0;
};

// ==================== OSD SIZE ====================
enum class OSDSize {
    SMALL,
    MEDIUM,
    LARGE
};

// ==================== FPV OSD CLASS ====================
class ClassicArduPilotOSD {
public:
    ClassicArduPilotOSD(OSDSize size = OSDSize::MEDIUM) {
        color_text = cv::Scalar(255, 255, 255);
        // [Tem] Uyari renkleri (BGR)
        color_ok   = cv::Scalar(0, 255, 0);     // yesil
        color_warn = cv::Scalar(0, 180, 255);   // turuncu
        color_crit = cv::Scalar(0, 0, 255);     // kirmizi
        color_shadow = cv::Scalar(0, 0, 0);
        color_horizon = cv::Scalar(0, 255, 0);  // Yeşil
        
        font = cv::FONT_HERSHEY_SIMPLEX;
        setSize(size);
    }
    
    void setSize(OSDSize size) {
        current_size = size;
        
        switch(size) {
            case OSDSize::SMALL:
                font_scale = 0.4;
                thickness = 1;
                shadow_thickness = 2;
                spacing = 20;
                break;
                
            case OSDSize::MEDIUM:
                font_scale = 0.9;        // 0.6 × 1.5 = 0.9
                thickness = 3;           // 2 × 1.5 = 3
                shadow_thickness = 5;    // 3 × 1.5 = 4.5 ≈ 5
                spacing = 42;            // 28 × 1.5 = 42
                break;
                
            case OSDSize::LARGE:
                font_scale = 0.8;
                thickness = 3;
                shadow_thickness = 4;
                spacing = 35;
                break;
        }
    }
    
    void draw(cv::Mat& frame, const TelemetryData& telem) {
        int w = frame.cols;
        int h = frame.rows;
        
        // Telemetri bilgileri
        // [10 Haz] Sol-ust blok (RSSI + uydu sayisi) kaldirildi - istege gore
        // drawTopLeft(frame, 10, 25, telem);
        drawTopRight(frame, w - 100, 25, telem);
        drawTopCenter(frame, w / 2, 25, telem);
        
        // HEADING TAPE (üstte yön göstergesi)
        drawHeadingTape(frame, w / 2, 60, telem);
        
        // MERKEZ HORIZON + Noktalı Pitch Ladder
        drawFPVHorizon(frame, w / 2, h / 2, telem);
        
        // ROLL INDICATOR
        drawRollIndicator(frame, w / 2, 100, telem);
        
        // [10 Haz] Sol-alt blok (CUAV_V5P + LAT/LON + voltaj + %) tamamen kaldirildi
        // drawBottom(frame, 10, h - 80, telem);
        
        // MERKEZ NOKTA
        drawCenterDot(frame, w / 2, h / 2);
    }
    
private:
    OSDSize current_size;
    cv::Scalar color_text, color_shadow, color_horizon;
    cv::Scalar color_ok, color_warn, color_crit;

    // [Tem] Batarya esikleri (hucre basina, LiPo). 8S -> 30.0V / 28.0V
    static constexpr int   CELLS         = 8;
    static constexpr float V_CELL_OK     = 3.75f;
    static constexpr float V_CELL_WARN   = 3.50f;
    // [Tem] RSSI yuzde esikleri
    static constexpr float RSSI_PCT_OK   = 50.0f;
    static constexpr float RSSI_PCT_CRIT = 22.0f;

    // ~2 Hz kare dalga — kritik uyarilarin yanip sonmesi icin
    bool blinkOn() const {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now().time_since_epoch()).count();
        return ((ms / 250) % 2) == 0;
    }

    cv::Scalar rssiColor(float pct) const {
        if (pct >  RSSI_PCT_OK)   return color_ok;
        if (pct >= RSSI_PCT_CRIT) return color_warn;
        return color_crit;
    }

    cv::Scalar batteryColor(float v) const {
        if (v <= 0.0f) return color_text;
        float pc = v / (float)CELLS;
        if (pc >= V_CELL_OK)   return color_ok;
        if (pc >= V_CELL_WARN) return color_warn;
        return color_crit;
    }
    int font;
    double font_scale;
    int thickness, shadow_thickness;
    int spacing;
    
    void drawOSDText(cv::Mat& frame, const std::string& text, int x, int y,
                     bool centered = false, cv::Scalar col = cv::Scalar(-1,-1,-1)) {
        int baseline;
        cv::Size textSize = cv::getTextSize(text, font, font_scale, thickness, &baseline);
        
        if (centered) {
            x -= textSize.width / 2;
        }
        
        if (col[0] < 0) col = color_text;   // varsayilan beyaz
        cv::putText(frame, text, cv::Point(x + 2, y + 2), 
                    font, font_scale, color_shadow, shadow_thickness);
        cv::putText(frame, text, cv::Point(x, y), 
                    font, font_scale, col, thickness);
    }

    // [Tem] Metin genisligi — hizalama hesaplari icin
    int textWidth(const std::string& text) {
        int baseline;
        return cv::getTextSize(text, font, font_scale, thickness, &baseline).width;
    }

    // [Tem] Sag kenara hizali metin (LAT/LON gibi uzun satirlar tasmasin)
    void drawOSDTextRight(cv::Mat& frame, const std::string& text, int right_x, int y,
                          cv::Scalar col = cv::Scalar(-1,-1,-1)) {
        drawOSDText(frame, text, right_x - textWidth(text), y, false, col);
    }
    
    void drawTopLeft(cv::Mat& frame, int x, int y, const TelemetryData& telem) {
        std::ostringstream ss;
        ss << (telem.rssi < 100 ? " " : "") << telem.rssi;
        drawOSDText(frame, ss.str(), x, y);
        
        ss.str("");
        ss << (telem.gps_sat_count < 10 ? " " : "") << telem.gps_sat_count;
        drawOSDText(frame, ss.str(), x, y + spacing);
    }
    
    // Sinyal sembolu: artan yukseklikte 4 cubuk (golge + beyaz)
    // [Tem] level: 0.0-1.0 (negatif = veri yok). Cubuklar seviyeye gore dolar
    // ve rengi metinle ayni olur -> tek bakista okunur.
    void drawSignalIcon(cv::Mat& frame, int x, int y_baseline,
                        float level, cv::Scalar col) {
        const int bw = 4, gap = 3;          // cubuk genisligi / aralik
        const int heights[4] = {6, 11, 16, 21};
        int filled = (level < 0.0f) ? 0 : (int)lround(level * 4.0f);
        for (int i = 0; i < 4; ++i) {
            int bx = x + i * (bw + gap);
            int top = y_baseline - heights[i];
            // golge
            cv::rectangle(frame, cv::Point(bx + 1, top + 1),
                          cv::Point(bx + bw + 1, y_baseline + 1),
                          color_shadow, cv::FILLED);
            if (i < filled) {
                cv::rectangle(frame, cv::Point(bx, top),
                              cv::Point(bx + bw, y_baseline), col, cv::FILLED);
            } else {
                cv::rectangle(frame, cv::Point(bx, top),
                              cv::Point(bx + bw, y_baseline),
                              cv::Scalar(140,140,140), 1);
            }
        }
    }

    // [Tem] Sag sutun. ARM/DARM ust basliga (drawTopCenter) tasindi;
    // bu yuzden tum satirlar bir kademe yukari kaydi -> irtifa en ustte.
    // Tum satirlar sag kenara hizali.
    void drawTopRight(cv::Mat& frame, int x, int y, const TelemetryData& telem) {
        std::ostringstream ss;
        const int right = frame.cols - 10;

        // 1) Irtifa (bagil)
        ss << std::fixed << std::setprecision(0) << telem.altitude_rel << "m";
        drawOSDTextRight(frame, ss.str(), right, y);

        // 2) RSSI + sinyal sembolu.
        //    Kaynak: RC_CHANNELS.rssi (0-254; 255 = gecersiz/raporlanmiyor).
        //    FC tarafi: RSSI_TYPE=2, RSSI_CHANNEL=14 (SIYI RSSI kanali).
        //    Renk: >%50 yesil, %22-50 turuncu, <%22 kirmizi + yanip soner.
        int yr = y + spacing;
        if (telem.rssi > 0 && telem.rssi != 255) {
            float level = telem.rssi / 254.0f;
            float pct   = level * 100.0f;
            cv::Scalar col = rssiColor(pct);
            if (pct < RSSI_PCT_CRIT && !blinkOn()) col = color_shadow;

            ss.str("");
            ss << telem.rssi << " " << std::fixed << std::setprecision(0) << pct << "%";
            int tx = right - textWidth(ss.str());
            drawOSDText(frame, ss.str(), tx, yr, false, col);
            drawSignalIcon(frame, tx - 38, yr, level, col);
        } else {
            std::string dash = "--";
            int tx = right - textWidth(dash);
            drawOSDText(frame, dash, tx, yr, false, color_warn);
            drawSignalIcon(frame, tx - 38, yr, -1.0f, color_warn);
        }

        // 3) Batarya — hucre basina gerilime gore renk kodlu.
        //    8S: >=30.0V yesil, 28.0-30.0V turuncu, <28.0V kirmizi + yanip soner.
        //    NOT: tam gazda anlik sarkma normaldir; esigi hover voltajina gore yorumla.
        cv::Scalar bcol = batteryColor(telem.battery_voltage);
        if (telem.battery_voltage > 0.0f &&
            telem.battery_voltage / (float)CELLS < V_CELL_WARN && !blinkOn()) {
            bcol = color_shadow;
        }
        ss.str("");
        ss << std::fixed << std::setprecision(1) << telem.battery_voltage << "V";
        drawOSDTextRight(frame, ss.str(), right, y + spacing * 2, bcol);

        // 4) GPS: uydu sayisi + fix tipi
        //    (0-1 = fix yok, 2=2D, 3=3D, 4=DGPS, 5+=RTK)
        const char* fix_str = "NOFIX";
        if      (telem.gps_fix_type >= 5) fix_str = "RTK";
        else if (telem.gps_fix_type == 4) fix_str = "DGPS";
        else if (telem.gps_fix_type == 3) fix_str = "3D";
        else if (telem.gps_fix_type == 2) fix_str = "2D";
        cv::Scalar gcol = (telem.gps_fix_type >= 3) ? color_text : color_warn;
        ss.str("");
        ss << telem.gps_sat_count << "SAT " << fix_str;
        drawOSDTextRight(frame, ss.str(), right, y + spacing * 3, gcol);

        // 5-6) LAT / LON — ham GPS_RAW_INT (degE7/1e7). Fix yokken 0 gelir.
        ss.str("");
        ss << "LAT " << std::fixed << std::setprecision(6) << telem.gps_lat;
        drawOSDTextRight(frame, ss.str(), right, y + spacing * 4);
        ss.str("");
        ss << "LON " << std::fixed << std::setprecision(6) << telem.gps_lon;
        drawOSDTextRight(frame, ss.str(), right, y + spacing * 5);
    }
    
    // [Tem] Ust orta: ucus modu (+OTONOM) ve ARM/DARM BIRLIKTE ortalanir.
    void drawTopCenter(cv::Mat& frame, int cx, int y, const TelemetryData& telem) {
        std::string label = telem.flight_mode;
        if (telem.autonomous) label += " (OTONOM)";
        const std::string arm = telem.armed ? "ARM" : "DARM";
        const int gap = 30;
        int mw = textWidth(label);
        int aw = textWidth(arm);
        int x0 = cx - (mw + gap + aw) / 2;
        drawOSDText(frame, label, x0, y);
        drawOSDText(frame, arm, x0 + mw + gap, y, false,
                    telem.armed ? color_ok : color_text);
    }
    
    void drawBottom(cv::Mat& frame, int x, int y, const TelemetryData& telem) {
        std::ostringstream ss;
        
        if (telem.gps_lat != 0.0 || telem.gps_lon != 0.0) {
            ss << "LAT" << std::fixed << std::setprecision(1) << std::abs(telem.gps_lat);
            drawOSDText(frame, ss.str(), x, y);
            
            ss.str("");
            ss << "LON" << std::fixed << std::setprecision(1) << std::abs(telem.gps_lon);
            drawOSDText(frame, ss.str(), x + 200, y);
        }
        
        drawOSDText(frame, telem.fc_identifier, x, y + spacing);
        
        ss.str("");
        ss << std::fixed << std::setprecision(1) << telem.battery_voltage << "V";
        drawOSDText(frame, ss.str(), x, y + spacing * 2);
        
        if (telem.battery_remaining > 0) {
            ss.str("");
            ss << telem.battery_remaining << "%";
            drawOSDText(frame, ss.str(), x + 100, y + spacing * 2);
        }
    }
    
    // ============ HEADING TAPE (YÖN GÖSTERGESİ) ============
    void drawHeadingTape(cv::Mat& frame, int cx, int y, const TelemetryData& telem) {
        float heading_deg = telem.yaw * 180.0f / M_PI;
        if (heading_deg < 0) heading_deg += 360.0f;
        
        int tape_width = 450;  // 300 × 1.5 = 450
        int tick_spacing = 30; // 20 × 1.5 = 30
        
        // Arka plan (siyah bant - şeffaf YOK)
	cv::rectangle(frame, 
		     cv::Point(cx - tape_width/2, y - 20),
		     cv::Point(cx + tape_width/2, y + 10),
		     cv::Scalar(0, 0, 0), -1);

	// Çerçeve
	cv::rectangle(frame, 
		     cv::Point(cx - tape_width/2, y - 20),
		     cv::Point(cx + tape_width/2, y + 10),
		     cv::Scalar(100, 100, 100), 1);
		     
		     
        // Yön sembolleri ve dereceleri
        struct HeadingMark {
            float angle;
            std::string label;
        };
        
        std::vector<HeadingMark> marks = {
            {0,   "N"},
            {45,  "NE"},
            {90,  "E"},
            {135, "SE"},
            {180, "S"},
            {225, "SW"},
            {270, "W"},
            {315, "NW"}
        };
        
        // Her 15° için küçük işaret
        for (int angle = 0; angle < 360; angle += 15) {
            float diff = angle - heading_deg;
            
            // -180 ile +180 arası normalize et
            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;
            
            // Ekran üzerinde pozisyon
            int screen_x = cx + static_cast<int>(diff * tick_spacing / 15.0f);
            
            // Görünür alanda mı?
            if (screen_x < cx - tape_width/2 || screen_x > cx + tape_width/2) {
                continue;
            }
            
            // Ana yönler (N, E, S, W) için uzun işaret ve etiket
            bool is_major = false;
            std::string label = "";
            
            for (auto& mark : marks) {
                if (std::abs(angle - mark.angle) < 1) {
                    is_major = true;
                    label = mark.label;
                    break;
                }
            }
            
            if (is_major) {
                // Uzun işaret
                cv::line(frame, cv::Point(screen_x, y - 15), 
                        cv::Point(screen_x, y - 5), 
                        color_text, 2);
                
                // Etiket
                drawOSDText(frame, label, screen_x, y, true);
            } else {
                // Kısa işaret (her 15°)
                cv::line(frame, cv::Point(screen_x, y - 12), 
                        cv::Point(screen_x, y - 8), 
                        color_text, 1);
            }
        }
        
        // Merkez göstergesi (üçgen - mevcut yön)
        std::vector<cv::Point> triangle;
        triangle.push_back(cv::Point(cx, y + 12));      // Alt uç
        triangle.push_back(cv::Point(cx - 8, y + 5));   // Sol
        triangle.push_back(cv::Point(cx + 8, y + 5));   // Sağ
        
        cv::fillPoly(frame, triangle, color_horizon);
        cv::polylines(frame, triangle, true, color_text, 1);
        
        // Mevcut derece (merkez altında)
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(0) << heading_deg;  // [10 Haz] derece sembolu kaldirildi (Hershey ASCII -> "??" basiyordu)
        drawOSDText(frame, ss.str(), cx, y + 30, true);
    }
    
    // ============ FPV HORIZON + NOKTALI PITCH LADDER ============
    void drawFPVHorizon(cv::Mat& frame, int cx, int cy, const TelemetryData& telem) {
        float roll_deg = telem.roll * 180.0f / M_PI;
        float pitch_deg = telem.pitch * 180.0f / M_PI;
        
        int pitch_offset = static_cast<int>(pitch_deg * -3.0f);  // [10 Haz] isaret terslendi (pitch yonu duzeltmesi)
        
        float cos_r = std::cos(roll_deg * M_PI / 180.0f);
        float sin_r = std::sin(roll_deg * M_PI / 180.0f);
        
        // ===== ANA YEŞİL HORİZON ÇİZGİSİ (Merkez) =====
        int line_length = 120;  // 80 × 1.5 = 120
        int line_gap = 60;      // 40 × 1.5 = 60
        
        // Sol çizgi
        int left_x1 = -line_gap - line_length;
        int left_x2 = -line_gap;
        int left_y = -pitch_offset;
        
        int rlx1 = cx + static_cast<int>(left_x1 * cos_r - left_y * sin_r);
        int rly1 = cy + static_cast<int>(left_x1 * sin_r + left_y * cos_r);
        int rlx2 = cx + static_cast<int>(left_x2 * cos_r - left_y * sin_r);
        int rly2 = cy + static_cast<int>(left_x2 * sin_r + left_y * cos_r);
        
        cv::line(frame, cv::Point(rlx1 + 1, rly1 + 1), cv::Point(rlx2 + 1, rly2 + 1), 
                color_shadow, 2);
        cv::line(frame, cv::Point(rlx1, rly1), cv::Point(rlx2, rly2), 
                color_horizon, 2);
        
        // Sağ çizgi
        int right_x1 = line_gap;
        int right_x2 = line_gap + line_length;
        int right_y = -pitch_offset;
        
        int rrx1 = cx + static_cast<int>(right_x1 * cos_r - right_y * sin_r);
        int rry1 = cy + static_cast<int>(right_x1 * sin_r + right_y * cos_r);
        int rrx2 = cx + static_cast<int>(right_x2 * cos_r - right_y * sin_r);
        int rry2 = cy + static_cast<int>(right_x2 * sin_r + right_y * cos_r);
        
        cv::line(frame, cv::Point(rrx1 + 1, rry1 + 1), cv::Point(rrx2 + 1, rry2 + 1), 
                color_shadow, 2);
        cv::line(frame, cv::Point(rrx1, rry1), cv::Point(rrx2, rry2), 
                color_horizon, 2);
        
        // ===== NOKTALI PITCH LADDER (Kenarlarda - SABİT) =====
        // Sabit referans çizgileri (hareket etmez)
        // Her 45 piksel aralıkla noktalı dikey çizgiler
        for (int y_offset = -135; y_offset <= 135; y_offset += 45) {  // 1.5x scaled
            // SOL TARAF - Sabit noktalı dikey çizgi
            drawStaticDottedLine(frame, cx - 225, cy + y_offset);  // -150 × 1.5 = -225
            
            // SAĞ TARAF - Sabit noktalı dikey çizgi  
            drawStaticDottedLine(frame, cx + 225, cy + y_offset);  // +150 × 1.5 = +225
        }
    }
    
    // Sabit noktalı dikey çizgi (hareket etmez)
    void drawStaticDottedLine(cv::Mat& frame, int x, int y_center) {
        int dot_spacing = 8;  // Noktalar arası boşluk
        int dot_size = 1;     // Nokta boyutu (İNCE)
        int num_dots = 5;     // Yukarı ve aşağı nokta sayısı
        
        // Dikey çizgi için noktalar (sabit pozisyon)
        for (int i = -num_dots; i <= num_dots; i++) {
            int dot_y = y_center + i * dot_spacing;
            
            // İnce nokta çiz (gölge + beyaz)
            cv::circle(frame, cv::Point(x + 1, dot_y + 1), 
                      dot_size, color_shadow, -1);
            cv::circle(frame, cv::Point(x, dot_y), 
                      dot_size, color_text, -1);
        }
    }
    
    // ============ ROLL INDICATOR (DISABLED) ============
    void drawRollIndicator(cv::Mat& frame, int cx, int y, const TelemetryData& telem) {
        // Roll indicator devre dışı (yarım çember ve yeşil üçgen kaldırıldı)
        // İsterseniz tekrar aktif etmek için bu kısmın yorumunu kaldırın
        
        /*
        float roll_deg = telem.roll * 180.0f / M_PI;
        
        int arc_radius = 90;
        int tick_length = 12;
        
        // Yay
        cv::ellipse(frame, cv::Point(cx, y + arc_radius), 
                   cv::Size(arc_radius, arc_radius),
                   0, 180, 360,
                   color_shadow, 2);
        cv::ellipse(frame, cv::Point(cx, y + arc_radius), 
                   cv::Size(arc_radius, arc_radius),
                   0, 180, 360,
                   color_text, 1);
        
        // İşaretler
        for (int angle = -60; angle <= 60; angle += 15) {
            float rad = (90 + angle) * M_PI / 180.0f;
            
            int x1 = cx + static_cast<int>(arc_radius * std::cos(rad));
            int y1 = (y + arc_radius) + static_cast<int>(arc_radius * std::sin(rad));
            
            int tick_len = (angle == 0) ? tick_length + 4 : tick_length;
            
            int x2 = cx + static_cast<int>((arc_radius + tick_len) * std::cos(rad));
            int y2 = (y + arc_radius) + static_cast<int>((arc_radius + tick_len) * std::sin(rad));
            
            cv::line(frame, cv::Point(x1 + 1, y1 + 1), cv::Point(x2 + 1, y2 + 1), 
                    color_shadow, 2);
            cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), 
                    color_text, 1);
        }
        
        // Roll göstergesi (üçgen)
        float roll_rad = (90 + roll_deg) * M_PI / 180.0f;
        int ind_x = cx + static_cast<int>((arc_radius + 12) * std::cos(roll_rad));
        int ind_y = (y + arc_radius) + static_cast<int>((arc_radius + 12) * std::sin(roll_rad));
        
        std::vector<cv::Point> triangle;
        triangle.push_back(cv::Point(ind_x, ind_y));
        
        float perp_rad = roll_rad + M_PI / 2;
        triangle.push_back(cv::Point(
            ind_x + static_cast<int>(6 * std::cos(perp_rad) - 10 * std::cos(roll_rad)),
            ind_y + static_cast<int>(6 * std::sin(perp_rad) - 10 * std::sin(roll_rad))
        ));
        triangle.push_back(cv::Point(
            ind_x + static_cast<int>(-6 * std::cos(perp_rad) - 10 * std::cos(roll_rad)),
            ind_y + static_cast<int>(-6 * std::sin(perp_rad) - 10 * std::sin(roll_rad))
        ));
        
        cv::fillPoly(frame, triangle, color_horizon);
        cv::polylines(frame, triangle, true, color_text, 1);
        */
    }
    
    // ============ MERKEZ NOKTA ============
    void drawCenterDot(cv::Mat& frame, int cx, int cy) {
        cv::circle(frame, cv::Point(cx, cy), 2, color_shadow, -1);
        cv::circle(frame, cv::Point(cx, cy), 2, color_horizon, -1);
    }
};

#endif // CLASSIC_ARDUPILOT_OSD_H
