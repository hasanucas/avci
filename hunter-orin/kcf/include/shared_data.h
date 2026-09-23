// shared_data.h
// Tracking data structure for Unix socket IPC
// Shared between runtracker.cpp and motor_control.cpp

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <cstdint>

// Tracking state enum
enum class TrackingState : uint8_t {
    NO_TRACKING = 0,      // Tracker kapalı, kullanıcı kontrolünde
    TRACKING_ACTIVE = 1,  // Tracker aktif, hedef takip ediliyor
    TRACKING_FAULT = 2    // Tracker aktifti ama hedef kayboldu
};

// Tracking data packet (36 bytes, aligned)
struct TrackingData {
    // State information
    TrackingState state;          // 1 byte
    uint8_t padding1[3];          // 3 byte (alignment to 4)
    
    // Normalized coordinates (-1.0 to +1.0, center = 0.0)
    float norm_x;                 // 4 bytes - Yaw ekseni
    float norm_y;                 // 4 bytes - Pitch ekseni
    
    // Angular offsets (degrees)
    float angle_yaw;              // 4 bytes - Sağ(+) / Sol(-)
    float angle_pitch;            // 4 bytes - Yukarı(+) / Aşağı(-)
    
    // Tracking quality metrics
    float confidence;             // 4 bytes - KCF tracker confidence [0.0-1.0]
    uint32_t frame_id;           // 4 bytes - Frame counter
    
    // Timestamp (microseconds since epoch)
    uint64_t timestamp_us;        // 8 bytes
    
    // Constructor with defaults
    TrackingData() 
        : state(TrackingState::NO_TRACKING)
        , norm_x(0.0f)
        , norm_y(0.0f)
        , angle_yaw(0.0f)
        , angle_pitch(0.0f)
        , confidence(0.0f)
        , frame_id(0)
        , timestamp_us(0)
    {
        padding1[0] = padding1[1] = padding1[2] = 0;
    }
};

#endif // SHARED_DATA_H
