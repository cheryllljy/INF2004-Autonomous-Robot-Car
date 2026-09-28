cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")
include(../pico_sdk_import.cmake)

project(buddy4_imu_test C CXX ASM)
pico_sdk_init()

add_executable(buddy4_imu_test
    main.c
    ../src/imu.c
)
target_include_directories(buddy4_imu_test PRIVATE ../include)
target_link_libraries(buddy4_imu_test PRIVATE pico_stdlib hardware_i2c)
target_compile_options(buddy4_imu_test PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wcast-qual
    -Wmissing-prototypes -Wstrict-prototypes -Wundef -fno-common
)
pico_enable_stdio_usb(buddy4_imu_test 1)
pico_enable_stdio_uart(buddy4_imu_test 0)
pico_add_extra_outputs(buddy4_imu_test)
