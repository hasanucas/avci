// =====================================================================
// ortrack_trt.hpp — ORTrack TensorRT backend'inin DIS yuzu.
//
// Bu baslik BILEREK NvInfer.h / cuda_runtime_api.h ICERMEZ. Boylece
// runtracker.cpp TensorRT basliklarina bagimli olmaz; tum TRT kodu
// ortrack_trt.cpp icinde tek bir derleme biriminde kalir.
// WITH_ORTRACK kapaliyken bu dosya hic derlenmez, runtracker KCF ile calisir.
// =====================================================================
#pragma once

#include "tracker_backend.hpp"
#include "target_manager.hpp"   // hybrid::ManagedTracker
#include <memory>
#include <string>

namespace ortrack {

struct TrtOptions {
    std::string enginePath;        // ORTrack_ep0300-fp16.engine
    int   device      = 0;         // CUDA cihazi
    float minScore    = 0.0f;      // Hann'li tepe esigi; 0 = KAPALI (varsayilan)
    double minBoxSize = 10.0;      // upstream taban. HIBRIT varsayilani 4.0:
                                   // orin-camera-quality-768.yaml notu — uzaklasan
                                   // hedefte kutu 10x10'a cakilip tracker gercek
                                   // olcegi kaybediyor, sonra arka plana atliyor.
    bool  verbose     = false;     // TRT info loglari
    int   warmupIters = 3;         // ilk karede tokezlemeyi onler
};

// ORTrack IKI arayuzu birden uygular:
//   TrackerBackend        -> manuel ROI modu (runtracker ana dongusu)
//   hybrid::ManagedTracker -> hibrit mod (TargetManager surer)
// Ayni nesne, ayni durum. reset() her iki tabanda da ayni imzada oldugu icin
// tek override ikisini birden karsilar.
struct OrtrackTracker : public TrackerBackend, public hybrid::ManagedTracker {
    virtual ~OrtrackTracker() {}
};

// Basarisizlikta nullptr doner ve err doldurulur (insan okunur, cozum onerili).
std::unique_ptr<OrtrackTracker> makeTensorRTTracker(const TrtOptions& opt, std::string& err);

// Derleme bilgisi (basligin acilisinda yazdirilir).
const char* tensorrtBuildInfo();

} // namespace ortrack
