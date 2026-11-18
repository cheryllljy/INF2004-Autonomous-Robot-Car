cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use the installed Pico SDK specified by PICO_SDK_PATH.
if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR
        "PICO_SDK_PATH is not set."
    )
endif()

set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}" CACHE PATH
    "Path to Raspberry Pi Pico SDK"
)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(buddy4_imu_test C CXX ASM)

pico_sdk_init()

add_executable(buddy4_imu_test
    main.c
    ../src/imu.c
)

target_include_directories(buddy4_imu_test PRIVATE
    ../include
)

target_link_libraries(buddy4_imu_test PRIVATE
    pico_stdlib
    hardware_i2c
)

target_compile_options(buddy4_imu_test PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wshadow
    -Wcast-qual
    -Wmissing-prototypes
    -Wstrict-prototypes
    -Wundef
    -fno-common
)

pico_enable_stdio_usb(buddy4_imu_test 1)
pico_enable_stdio_uart(buddy4_imu_test 0)

pico_add_extra_outputs(buddy4_imu_test)