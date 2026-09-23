cmake_minimum_required(VERSION 3.10)
project(kcfTracker)

# ============================================================================
# HUNTER Orin — runtracker: KCF | ORTrack (TensorRT) + RTSP + MAVLink + FRAME
#
# DEGISIKLIKLER (10 Eylul):
#   1) pkg-config listesine gstreamer-video-1.0 (force-key-unit event) ve
#      gstreamer-rtsp-1.0 (SETUP header okuma) EKLENDI. Bunlar olmadan
#      runtracker.cpp'nin 9 Eylul surumu LINK OLMAZ:
#        "undefined reference to gst_rtsp_message_get_header"
#   2) WITH_ORTRACK secenegi: ACIK ise ORTrack TensorRT backend'i derlenir.
#      TensorRT/CUDA bulunamazsa OTOMATIK KAPANIR ve ikili yine derlenir
#      (sadece KCF yapar). Yani bu CMakeLists TensorRT'siz makinede de calisir.
#   3) test_ortrack_offline hedefi: ORTrack on/son islemesini Python
#      referansina karsi dogrular. TensorRT/CUDA/kamera/FC GEREKTIRMEZ.
#
# Kullanim:
#   mkdir -p build && cd build
#   cmake .. && make -j4                 # ORTrack varsa otomatik acilir
#   cmake .. -DWITH_ORTRACK=OFF && make  # sadece KCF
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ORTrack yuvarlama davranisi IEEE ties-to-even'e bagli (std::nearbyint).
# -ffast-math bunu bozar -> Python referansiyla ayrisir. ASLA eklemeyin.
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

option(WITH_ORTRACK "ORTrack (TensorRT) tracker backend'ini derle" ON)

# --- OpenCV (JetPack ile gelir) ---
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# --- GStreamer (RTSP in/out) ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
    gstreamer-1.0
    gstreamer-rtsp-server-1.0
    gstreamer-rtsp-1.0          # [10 Eyl] SETUP transport header
    gstreamer-app-1.0
    gstreamer-video-1.0         # [10 Eyl] force-key-unit event
)
include_directories(${GST_INCLUDE_DIRS})
link_directories(${GST_LIBRARY_DIRS})

# --- OpenMP (KCF HOG hizlandirma) ---
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# --- Include yollari ---
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../mavlink)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(LIBRARIES ${OpenCV_LIBRARIES})
if(UNIX)
    list(APPEND LIBRARIES pthread stdc++fs)
endif()

# ============================================================================
# TensorRT + CUDA arama (find_package(CUDAToolkit) KULLANILMIYOR: cmake 3.17+
# gerektiriyor, burada 3.10 tabani korunuyor)
# ============================================================================
set(ORTRACK_SOURCES "")
set(ORTRACK_LIBS "")
set(ORTRACK_ENABLED OFF)

if(WITH_ORTRACK)
    find_path(TENSORRT_INCLUDE_DIR NvInfer.h
        HINTS
            /usr/include/aarch64-linux-gnu
            /usr/include/x86_64-linux-gnu
            /usr/local/tensorrt/include
            /usr/local/TensorRT/include
            /opt/nvidia/tensorrt/include
            ENV TENSORRT_DIR
        PATH_SUFFIXES include)
    find_library(TENSORRT_NVINFER nvinfer
        HINTS
            /usr/lib/aarch64-linux-gnu
            /usr/lib/x86_64-linux-gnu
            /usr/local/tensorrt/lib
            ENV TENSORRT_DIR
        PATH_SUFFIXES lib lib64)

    find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
        HINTS /usr/local/cuda/include /usr/local/cuda/targets/aarch64-linux/include
              /usr/include ENV CUDA_HOME
        PATH_SUFFIXES include)
    find_library(CUDA_CUDART cudart
        HINTS /usr/local/cuda/lib64 /usr/local/cuda/targets/aarch64-linux/lib
              /usr/lib/aarch64-linux-gnu ENV CUDA_HOME
        PATH_SUFFIXES lib lib64)

    if(TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER AND CUDA_INCLUDE_DIR AND CUDA_CUDART)
        set(ORTRACK_ENABLED ON)
        set(ORTRACK_SOURCES ortrack_trt.cpp yolo_trt.cpp)
        set(ORTRACK_LIBS ${TENSORRT_NVINFER} ${CUDA_CUDART})
        include_directories(${TENSORRT_INCLUDE_DIR} ${CUDA_INCLUDE_DIR})
    else()
        message(WARNING
            "WITH_ORTRACK=ON ama TensorRT/CUDA bulunamadi -> ORTrack KAPATILDI.\n"
            "  NvInfer.h  : ${TENSORRT_INCLUDE_DIR}\n"
            "  libnvinfer : ${TENSORRT_NVINFER}\n"
            "  cuda_runtime_api.h : ${CUDA_INCLUDE_DIR}\n"
            "  libcudart  : ${CUDA_CUDART}\n"
            "  Ikili yine derlenecek ama sadece KCF yapacak.")
    endif()
endif()

# --- KCF kutuphanesi ---
file(GLOB SOURCES src/*.cpp)
add_library(kcf SHARED ${SOURCES})
target_link_libraries(kcf PUBLIC ${LIBRARIES})
target_include_directories(kcf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# --- Ana program ---
add_executable(kcfTracker runtracker.cpp ${ORTRACK_SOURCES})
target_link_libraries(kcfTracker PUBLIC kcf ${GST_LIBRARIES} ${ORTRACK_LIBS} pthread)
target_include_directories(kcfTracker PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR})
if(ORTRACK_ENABLED)
    target_compile_definitions(kcfTracker PRIVATE WITH_ORTRACK=1)
endif()

# --- ESKI goruntu aktarim programi (sade KCF + RTSP restream, ORTrack YOK) ---
# runtracker1.cpp: manuel ROI + KCF + RTSP in/out + MAVLink + frame TCP.
# Yeni kcfTracker ile YAN YANA derlenir; ayri binary oldugu icin main() cakismasi olmaz.
# Calistirma:
#   ./kcfTrackerLegacy --rtsp rtsp://192.168.144.25:8554/main.264 \
#                      --mavlink-udp 127.0.0.1:14550 --frame-port 9999 --bitrate 3000
add_executable(kcfTrackerLegacy runtracker1.cpp)
target_link_libraries(kcfTrackerLegacy PUBLIC kcf ${GST_LIBRARIES} pthread)
target_include_directories(kcfTrackerLegacy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR})

# --- Offline regresyon testleri (TensorRT/CUDA/kamera/FC GEREKMEZ) ---
#   make tests  &&  ctest        ya da elle:
#   ./test_ortrack_offline ../tests/golden.txt
#   ./test_target_manager  ../tests/manager_golden.txt
#   ./test_yolo_offline    ../tests/yolo_golden.txt
enable_testing()
add_custom_target(tests)
foreach(T test_ortrack_offline test_target_manager test_yolo_offline test_simple_lock)
    add_executable(${T} EXCLUDE_FROM_ALL tests/${T}.cpp)
    target_link_libraries(${T} PRIVATE ${OpenCV_LIBRARIES} pthread)
    target_include_directories(${T} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    add_dependencies(tests ${T})
endforeach()
add_test(NAME ortrack_core   COMMAND test_ortrack_offline ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden.txt)
add_test(NAME target_manager COMMAND test_target_manager  ${CMAKE_CURRENT_SOURCE_DIR}/tests/manager_golden.txt)
add_test(NAME yolo_core      COMMAND test_yolo_offline    ${CMAKE_CURRENT_SOURCE_DIR}/tests/yolo_golden.txt)
add_test(NAME simple_lock    COMMAND test_simple_lock)

message(STATUS "")
message(STATUS "===== HUNTER Orin build =====")
message(STATUS "  OpenCV    : ${OpenCV_VERSION}")
message(STATUS "  GStreamer : ${GST_VERSION}")
message(STATUS "  Target    : kcfTracker (runtracker.cpp)")
message(STATUS "  Target    : kcfTrackerLegacy (runtracker1.cpp, sade KCF+RTSP)")
if(ORTRACK_ENABLED)
    message(STATUS "  ORTrack   : ACIK  (nvinfer=${TENSORRT_NVINFER})")
else()
    message(STATUS "  ORTrack   : KAPALI (sadece KCF)")
endif()
message(STATUS "  Test      : make tests && ctest --output-on-failure")
message(STATUS "============================")
