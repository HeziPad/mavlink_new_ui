cmake_minimum_required(VERSION 3.16)

project(mavlink_gcs_ui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_AUTOMOC ON)

option(GCS_WITH_GSTREAMER "Build the UDP H.264 video receiver (needs GStreamer dev packages)" ON)

find_package(Threads REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Network)
find_package(PkgConfig)
if(GCS_WITH_GSTREAMER AND PkgConfig_FOUND)
    pkg_check_modules(GST IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
endif()

function(gcs_warnings target)
    if(NOT MSVC)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

# ---- core + link: plain C++17, no Qt, unit-tested ---------------------------
set(GCS_LINK_SOURCES
    src/link/command_sender.cpp
    src/link/mavlink_client.cpp
    src/link/mavlink_globals.cpp
    src/link/mission_transfer.cpp
)

add_library(gcs_core STATIC
    src/core/geo.cpp
    src/core/health.cpp
    src/core/mission.cpp
    src/core/orbit_trim.cpp
    src/core/plane_modes.cpp
    src/core/vehicle_state.cpp
    ${GCS_LINK_SOURCES}
)
target_include_directories(gcs_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
# SYSTEM: the generated MAVLink headers are not ours to keep -Wpedantic clean.
target_include_directories(gcs_core SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/c_library_v2)
target_link_libraries(gcs_core PUBLIC Threads::Threads)
gcs_warnings(gcs_core)
if(NOT MSVC)
    set_source_files_properties(${GCS_LINK_SOURCES} PROPERTIES
        COMPILE_OPTIONS "-Wno-address-of-packed-member;-Wno-pedantic")
endif()

# ---- application --------------------------------------------------------------
add_executable(mavlink_gcs_ui
    src/main.cpp
    src/ui/alerts_panel.cpp
    src/ui/attitude_ball.cpp
    src/ui/camera_view.cpp
    src/ui/control_dock.cpp
    src/ui/fx_overlay.cpp
    src/ui/main_window.cpp
    src/ui/orbit_panel.cpp
    src/ui/map_view.cpp
    src/ui/plan_strip.cpp
    src/ui/telemetry_bar.cpp
    src/ui/theme.cpp
    src/ui/settings_screen.cpp
    src/ui/tile_cache.cpp
    src/ui/video_receiver.cpp
    src/ui/view_host.cpp
    src/ui/units.cpp
    src/ui/waypoint_panel.cpp
)
target_link_libraries(mavlink_gcs_ui PRIVATE gcs_core Qt6::Widgets Qt6::Network)
gcs_warnings(mavlink_gcs_ui)
if(GST_FOUND)
    target_link_libraries(mavlink_gcs_ui PRIVATE PkgConfig::GST)
    target_compile_definitions(mavlink_gcs_ui PRIVATE GCS_HAVE_GSTREAMER=1)
    message(STATUS "Video: GStreamer ${GST_gstreamer-1.0_VERSION} found, H.264 receiver enabled")
else()
    message(STATUS "Video: GStreamer not found, camera view shows the synthetic HUD only")
endif()

# ---- tests ----------------------------------------------------------------------
enable_testing()
foreach(name geo mission_builder mission_transfer command_sender lift_rotors health orbit_trim)
    add_executable(test_${name} tests/test_${name}.cpp)
    target_link_libraries(test_${name} PRIVATE gcs_core)
    gcs_warnings(test_${name})
    if(NOT MSVC)
        target_compile_options(test_${name} PRIVATE -Wno-address-of-packed-member)
    endif()
    add_test(NAME ${name} COMMAND test_${name})
endforeach()
