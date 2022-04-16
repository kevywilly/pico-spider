cmake_minimum_required(VERSION 3.19)

include(
        $ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake
        )

project(pico_spider C CXX ASM)
#set(CMAKE_C_STANDARD 11)
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall" )

#set(CMAKE_C_STANDARD 99)

add_executable(pico_spider main.cpp src/pca9685.h model.cpp src/model.h)

target_include_directories(pico_spider
        PUBLIC
        /Users/kevywilly/Documents/Projects/Robotics/c++/libraries/IKLibrary/src
        $ENV{PICO_TFLITE_PATH}/src/
        $ENV{PICO_TFLITE_PATH}/src/third_party/cmsis/CMSIS/DSP/Include
        $ENV{PICO_TFLITE_PATH}/src/third_party/ruy
        $ENV{PICO_TFLITE_PATH}/src/third_party/gemmlowp
        $ENV{PICO_TFLITE_PATH}/src/third_party/kissfft
        $ENV{PICO_TFLITE_PATH}/src/third_party/flatbuffers
        $ENV{PICO_TFLITE_PATH}/src/third_party/cmsis/CMSIS/Core/Include
        $ENV{PICO_TFLITE_PATH}/src/third_party/cmsis
        $ENV{PICO_TFLITE_PATH}/src/third_party/flatbuffers/include
        $ENV{PICO_TFLITE_PATH}/src/third_party/cmsis/CMSIS/NN/Include
        )

# Initalise the SDK
pico_sdk_init()

#include_directories(
#        $ENV{PICO_TFLITE_PATH}
#)



target_compile_definitions(
        pico_spider
        PUBLIC
        COMPILE_DEFINITIONS TF_LITE_DISABLE_X86_NEON=1
        COMPILE_DEFINITIONS TF_LITE_STATIC_MEMORY=1
        COMPILE_DEFINITIONS CMSIS_NN=1
)

set_target_properties(
        pico_spider
        PROPERTIES
        COMPILE_FLAGS -fno-rtti
        COMPILE_FLAGS -fno-exceptions
        COMPILE_FLAGS -fno-threadsafe-statics
        COMPILE_FLAGS -nostdlib
)

target_link_libraries(pico_spider pico_stdlib hardware_i2c)

target_sources(pico_spider
        PRIVATE
        model.cpp

        PUBLIC
        $ENV{PICO_TFLITE_PATH}/src/tensorflow/lite/micro/
        )

pico_enable_stdio_usb(pico_spider 1)

pico_add_extra_outputs(pico_spider)