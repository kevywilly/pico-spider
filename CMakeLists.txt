cmake_minimum_required(VERSION 3.19)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_spider C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

add_executable(pico_spider main.cpp src/pca9685.h model.cpp src/leg.h src/body.h src/structs.h src/joint.h)

target_include_directories(pico_spider
        PUBLIC
        /Users/kevywilly/Documents/Projects/Robotics/libraries/ik-library/src
        )

# Initalise the SDK
pico_sdk_init()

target_link_libraries(pico_spider pico_stdlib hardware_i2c)

target_sources(pico_spider
        PRIVATE
        model.cpp
        )

pico_enable_stdio_usb(pico_spider 1)

pico_add_extra_outputs(pico_spider)
