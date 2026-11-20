cmake_minimum_required(VERSION 3.16)

project(mavlink_gcs_ui VERSION 0.6.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GCS_WITH_GSTREAMER "Build the UDP H.264 video receiver (needs GStreamer dev packages)" ON)

find_package(Threads REQUIRED)
find_package(ZLIB)
if(NOT ZLIB_FOUND)
    # Windows has no system zlib: build a static one
    message(STATUS "zlib: not found, building 1.3.1 from source")
    include(FetchContent)
    FetchContent_Declare(zlib
        URL https://github.com/madler/zlib/releases/download/v1.3.1/zlib-1.3.1.tar.gz
        URL_HASH SHA256=9a93b2b7dfdac77ceba5a558a580e74667dd6fede4585b91eefb60f03b72df23
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    set(SKIP_INSTALL_ALL ON CACHE BOOL "" FORCE)
    set(ZLIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(zlib)
    target_include_directories(zlibstatic PUBLIC ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
    add_library(ZLIB::ZLIB ALIAS zlibstatic)
endif()
find_package(Qt6 REQUIRED COMPONENTS Widgets Network)
set(CMAKE_AUTOMOC ON)  # after zlib: only our targets need moc
find_package(PkgConfig)
if(GCS_WITH_GSTREAMER AND PkgConfig_FOUND)
    pkg_check_modules(GST IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
endif()

function(gcs_warnings target)
    if(MSVC)
        # /utf-8: the sources are UTF-8 (°, ◀ ▶ ...); /external: quiet SYSTEM headers
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus /external:W0)
        target_compile_definitions(${target} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

# ---- core + link: plain C++17, no Qt, unit-tested ---------------------------
set(GCS_LINK_SOURCES
    src/link/command_sender.cpp
    src/link/mavlink_client.cpp
    src/link/mavlink_globals.cpp
    src/link/mission_transfer.cpp
    src/payload/payload_manager.cpp
)

add_library(gcs_core STATIC
    src/core/frame_layout.cpp
    src/core/geo.cpp
    src/core/gzip.cpp
    src/core/health.cpp
    src/core/altitude.cpp
    src/core/coords.cpp
    src/core/endurance.cpp
    src/core/flight_status.cpp
    src/core/hud_config.cpp
    src/core/display_round.cpp
    src/core/mission.cpp
    src/core/mvt.cpp
    src/core/orbit_trim.cpp
    src/core/plane_modes.cpp
    src/core/pmtiles.cpp
    src/core/preflight.cpp
    src/core/speed_mode.cpp
    src/core/descent_speed.cpp
    src/core/tiles.cpp
    src/core/vehicle_state.cpp
    src/payload/adapter.cpp
    src/payload/adapters/nextvision.cpp
    src/payload/icd.cpp
    src/payload/icd_codec.cpp
    src/payload/json.cpp
    src/link/serial_transport.cpp
    src/link/udp_transport.cpp
    ${GCS_LINK_SOURCES}
)
target_include_directories(gcs_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
# SYSTEM: the generated MAVLink headers are not ours to keep -Wpedantic clean.
target_include_directories(gcs_core SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/c_library_v2)
target_link_libraries(gcs_core PUBLIC Threads::Threads ZLIB::ZLIB)
if(WIN32)
    target_link_libraries(gcs_core PUBLIC ws2_32 setupapi advapi32)  # sockets, serial port list
endif()
gcs_warnings(gcs_core)
if(NOT MSVC)
    set_source_files_properties(${GCS_LINK_SOURCES} PROPERTIES
        COMPILE_OPTIONS "-Wno-address-of-packed-member;-Wno-pedantic")
endif()

# ---- application --------------------------------------------------------------
add_executable(mavlink_gcs_ui
    src/main.cpp
    src/ui/alerts_panel.cpp
    src/ui/app_icon.cpp
    src/ui/attitude_ball.cpp
    src/ui/camera_view.cpp
    src/ui/control_dock.cpp
    src/ui/elevation_cache.cpp
    src/ui/entry_pad.cpp
    src/ui/fx_overlay.cpp
    src/ui/hud_panel.cpp
    src/ui/main_window.cpp
    src/ui/orbit_panel.cpp
    src/ui/payload_panel.cpp
    src/ui/map_view.cpp
    src/ui/offline_downloader.cpp
    src/ui/offline_store.cpp
    src/ui/plan_library.cpp
    src/ui/plan_library_panel.cpp
    src/ui/plan_strip.cpp
    src/ui/pmtiles_extract.cpp
    src/ui/preflight_art.cpp
    src/ui/preflight_runner.cpp
    src/ui/preflight_screen.cpp
    src/ui/telemetry_bar.cpp
    src/ui/theme.cpp
    src/ui/settings_screen.cpp
    src/ui/speed_panel.cpp
    src/ui/tile_cache.cpp
    src/ui/vector_map.cpp
    src/ui/video_receiver.cpp
    src/ui/view_host.cpp
    src/ui/units.cpp
    src/ui/waypoint_panel.cpp
    src/ui/wind_gauge.cpp
)
target_link_libraries(mavlink_gcs_ui PRIVATE gcs_core Qt6::Widgets Qt6::Network)
# payload ICDs: found in the source tree when running from a build directory
target_compile_definitions(mavlink_gcs_ui PRIVATE GCS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
gcs_warnings(mavlink_gcs_ui)
# bundled fonts: the same look and text metrics on every OS (theme::loadFonts)
qt_add_resources(mavlink_gcs_ui fonts
    PREFIX /fonts
    BASE resources/fonts
    FILES resources/fonts/DejaVuSans.ttf resources/fonts/DejaVuSans-Bold.ttf
          resources/fonts/DejaVuSansMono.ttf resources/fonts/DejaVuSansMono-Bold.ttf)
if(WIN32)
    set_target_properties(mavlink_gcs_ui PROPERTIES WIN32_EXECUTABLE ON)
    target_sources(mavlink_gcs_ui PRIVATE resources/windows/mavlink_gcs_ui.rc)
elseif(APPLE)
    set(GCS_ICNS ${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/mavlink_gcs_ui.icns)
    set_source_files_properties(${GCS_ICNS} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    target_sources(mavlink_gcs_ui PRIVATE ${GCS_ICNS})
    set_target_properties(mavlink_gcs_ui PROPERTIES
        MACOSX_BUNDLE ON
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/resources/macos/Info.plist.in
        MACOSX_BUNDLE_BUNDLE_NAME "QuadPlane GCS"
        MACOSX_BUNDLE_GUI_IDENTIFIER io.github.mavlink-gcs-ui
        MACOSX_BUNDLE_ICON_FILE mavlink_gcs_ui.icns
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
endif()
if(GST_FOUND)
    target_link_libraries(mavlink_gcs_ui PRIVATE PkgConfig::GST)
    target_compile_definitions(mavlink_gcs_ui PRIVATE GCS_HAVE_GSTREAMER=1)
    message(STATUS "Video: GStreamer ${GST_gstreamer-1.0_VERSION} found, H.264 receiver enabled")
else()
    message(STATUS "Video: GStreamer not found, camera view shows the synthetic HUD only")
endif()

# ---- tests ----------------------------------------------------------------------
enable_testing()
foreach(name geo altitude coords display_round speed_mode descent_speed endurance flight_status hud_config mission_builder mission_transfer command_sender lift_rotors health orbit_trim preflight offline_map transport)
    add_executable(test_${name} tests/test_${name}.cpp)
    target_link_libraries(test_${name} PRIVATE gcs_core)
    gcs_warnings(test_${name})
    if(NOT MSVC)
        target_compile_options(test_${name} PRIVATE -Wno-address-of-packed-member)
    endif()
    add_test(NAME ${name} COMMAND test_${name})
endforeach()
# payload ICD engine: reads icds/*.json from the source tree
add_executable(test_payload tests/test_payload.cpp)
target_link_libraries(test_payload PRIVATE gcs_core)
target_compile_definitions(test_payload PRIVATE GCS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
gcs_warnings(test_payload)
if(NOT MSVC)
    target_compile_options(test_payload PRIVATE -Wno-address-of-packed-member -Wno-pedantic)
endif()
add_test(NAME payload COMMAND test_payload)

# saved plans: the plan files (QtCore only)
add_executable(test_plan_library tests/test_plan_library.cpp src/ui/plan_library.cpp)
target_link_libraries(test_plan_library PRIVATE gcs_core Qt6::Core)
gcs_warnings(test_plan_library)
add_test(NAME plan_library COMMAND test_plan_library)

# ---- install & packages -----------------------------------------------------------
# Linux: binary + desktop entry + icons. Windows / macOS: a self-contained
# folder / .app with the Qt libraries next to it (windeployqt / macdeployqt).
include(GNUInstallDirs)
if(WIN32 OR APPLE)
    if(Qt6_VERSION VERSION_LESS 6.5)
        message(WARNING "Qt ${Qt6_VERSION}: packaging on Windows/macOS needs Qt 6.5 or newer")
    else()
        install(TARGETS mavlink_gcs_ui BUNDLE DESTINATION . RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
        qt_generate_deploy_app_script(TARGET mavlink_gcs_ui OUTPUT_SCRIPT gcs_deploy NO_UNSUPPORTED_PLATFORM_ERROR)
        install(SCRIPT ${gcs_deploy})
        # payload ICDs next to the binary (Windows) / in the bundle (macOS)
        if(APPLE)
            install(DIRECTORY icds/ DESTINATION mavlink_gcs_ui.app/Contents/Resources/icds FILES_MATCHING PATTERN "*.json")
        else()
            install(DIRECTORY icds/ DESTINATION ${CMAKE_INSTALL_BINDIR}/icds FILES_MATCHING PATTERN "*.json")
        endif()
    endif()
else()
    install(TARGETS mavlink_gcs_ui RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(FILES resources/mavlink_gcs_ui.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications)
    install(DIRECTORY icds/ DESTINATION ${CMAKE_INSTALL_DATADIR}/mavlink_gcs_ui/icds FILES_MATCHING PATTERN "*.json")
    foreach(size 16 24 32 48 64 128 256 512)
        install(FILES resources/icons/mavlink_gcs_ui_${size}.png
                DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/${size}x${size}/apps
                RENAME mavlink_gcs_ui.png)
    endforeach()
endif()

set(CPACK_PACKAGE_NAME mavlink_gcs_ui)
set(CPACK_PACKAGE_VENDOR "mavlink_gcs_ui")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
if(WIN32)
    set(CPACK_GENERATOR ZIP)
elseif(APPLE)
    set(CPACK_GENERATOR DragNDrop)
else()
    set(CPACK_GENERATOR TGZ)
endif()
include(CPack)
