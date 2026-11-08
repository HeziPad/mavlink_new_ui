cmake_minimum_required(VERSION 3.16)

project(mavlink_gcs_ui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(mavlink_gcs_ui
    src/main.cpp
    src/mavlink_receiver.cpp
    src/main_window.cpp
)

target_include_directories(mavlink_gcs_ui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include/c_library_v2
)

target_link_libraries(mavlink_gcs_ui PRIVATE Qt6::Widgets)

if(MSVC)
    target_compile_options(mavlink_gcs_ui PRIVATE /W4)
else()
    target_compile_options(mavlink_gcs_ui PRIVATE -Wall -Wextra -Wpedantic)
    set_source_files_properties(src/mavlink_receiver.cpp PROPERTIES
        COMPILE_OPTIONS "-Wno-address-of-packed-member;-Wno-pedantic")
endif()
