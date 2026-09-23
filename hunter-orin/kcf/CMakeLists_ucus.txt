cmake_minimum_required(VERSION 3.10)
project(kcfTracker)

# ============================================================================
# HUNTER Orin — gercek donanim KCF tracker + RTSP + MAVLink(UDP) + TCP FRAME
# Gazebo BAGIMLILIGI KALDIRILDI (runtracker Gazebo kullanmiyor — RTSP'den okur).
# Bagimliliklar: OpenCV, GStreamer (RTSP), OpenMP, MAVLink (header-only), pthread.
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- OpenCV (JetPack ile gelir, 4.10 test edildi) ---
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# --- GStreamer (RTSP in/out) — pkg-config ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
    gstreamer-1.0
    gstreamer-rtsp-server-1.0
    gstreamer-app-1.0
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
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../mavlink)   # MAVLink (header-only)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})              # shared_data.h vb.

# --- Kutuphaneler ---
set(LIBRARIES ${OpenCV_LIBRARIES})
if(UNIX)
    list(APPEND LIBRARIES pthread stdc++fs)
endif()

# --- KCF tracker kutuphanesi (src/*.cpp) ---
file(GLOB SOURCES src/*.cpp)
add_library(kcf SHARED ${SOURCES})
target_link_libraries(kcf PUBLIC ${LIBRARIES})
target_include_directories(kcf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# --- Ana program: kcfTracker (runtracker.cpp) ---
add_executable(kcfTracker runtracker.cpp)
target_link_libraries(kcfTracker
    PUBLIC
    kcf
    ${GST_LIBRARIES}
    pthread
)
target_include_directories(kcfTracker PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

message(STATUS "")
message(STATUS "===== HUNTER Orin build =====")
message(STATUS "  OpenCV    : ${OpenCV_VERSION}")
message(STATUS "  GStreamer : ${GST_VERSION}")
message(STATUS "  Target    : kcfTracker (runtracker.cpp)")
message(STATUS "  (Gazebo ve motor_control KALDIRILDI)")
message(STATUS "============================")
