# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# ---------------------------------------------------------------------
# Buddy 5: adaptive ultrasonic scanning and obstacle profiling
# Raspberry Pi Pico W on Cytron Robo Pico
#
# Two build configurations:
#
#   -DUSE_MTK3=OFF   (default)  Pico SDK + the bring-up shim in shim/.
#                               Flashable today. src/main.c gives you a
#                               USB serial menu of bench tests. Use
#                               this to prove the hardware works before
#                               the kernel is integrated.
#
#   -DUSE_MTK3=ON               Real micro T-Kernel 3.0. src/usermain.c
#                               is the entry point and the shim is
#                               excluded. This is the graded build.
#
# The four files in src/ that make up the subsystem are identical in
# both. Only the entry point and the tk/tkernel.h include path change.
# ---------------------------------------------------------------------

cmake_minimum_required(VERSION 3.13)

# Must come before project(). Copy pico_sdk_import.cmake from the SDK:
#   cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
include(pico_sdk_import.cmake)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(buddy5_obstacle C CXX ASM)

pico_sdk_init()

option(USE_MTK3 "Build against micro T-Kernel 3.0 instead of the shim" OFF)
set(MTK3_PATH "" CACHE PATH "Path to the micro T-Kernel 3.0 BSP")

# ---------------------------------------------------------------------
# The subsystem itself. These four files never change between builds.
# ---------------------------------------------------------------------

set(OBSTACLE_SOURCES
    src/servo.c
    src/ultrasonic.c
    src/obstacle.c
    src/obstacle_geometry.c
)

set(BUDDY1_SOURCES
    src/wifi.c
    src/mqtt.c
)

add_executable(buddy5 ${OBSTACLE_SOURCES} ${BUDDY1_SOURCES})

target_include_directories(buddy5 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# ---------------------------------------------------------------------
# Entry point and kernel
# ---------------------------------------------------------------------

if(USE_MTK3)
    if(NOT MTK3_PATH)
        message(FATAL_ERROR
            "USE_MTK3=ON needs -DMTK3_PATH=/path/to/mtkernel_3")
    endif()

    message(STATUS "Buddy 5: building against micro T-Kernel 3.0")

    target_sources(buddy5 PRIVATE src/usermain.c)

    # The BSP supplies <tk/tkernel.h> and <tm/tmonitor.h>. Adjust these
    # two paths to match your BSP layout, then check the porting notes
    # in README_BUILD.md before the first run.
    target_include_directories(buddy5 PRIVATE
        ${MTK3_PATH}/include
        ${MTK3_PATH}/config
    )

    target_link_libraries(buddy5 PRIVATE mtkernel_3)
else()
    message(STATUS "Buddy 5: building with the bring-up shim (no RTOS)")

    target_sources(buddy5 PRIVATE
        src/main.c
        src/tk_shim.c
    )

    # shim/ holds tk/tkernel.h. Because it is only on the include path
    # in this configuration, src/obstacle.c and src/ultrasonic.c need
    # no #ifdef at all.
    target_include_directories(buddy5 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/shim
    )
endif()

# ---------------------------------------------------------------------
# Pico SDK libraries
# ---------------------------------------------------------------------

target_link_libraries(buddy5 PRIVATE
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_clocks
    hardware_timer
    hardware_sync
)

# ---------------------------------------------------------------------
# Buddy 1: WiFi communication libraries
# ---------------------------------------------------------------------

target_link_libraries(buddy5 PRIVATE
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
)

# ---------------------------------------------------------------------
# Warnings. The subsystem is expected to build clean under all of
# these; if a warning appears, fix the code rather than the flag.
# ---------------------------------------------------------------------

target_compile_options(buddy5 PRIVATE
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

# Uncomment once everything is clean, to keep it that way.
# target_compile_options(buddy5 PRIVATE -Werror)

# Size matters on a resource-constrained target. -Og keeps the
# debugger usable while you are bringing the sensor up; switch to -Os
# for the demonstration build and re-measure.
target_compile_options(buddy5 PRIVATE $<$<CONFIG:Debug>:-Og>)
target_compile_options(buddy5 PRIVATE $<$<CONFIG:Release>:-Os>)

# ---------------------------------------------------------------------
# Console. USB CDC is convenient on the bench; UART survives a USB
# replug and does not stall when nothing is listening, which matters
# once the car is actually driving.
# ---------------------------------------------------------------------

pico_enable_stdio_usb(buddy5 1)
pico_enable_stdio_uart(buddy5 0)

pico_add_extra_outputs(buddy5)

# ---------------------------------------------------------------------
# make host-tests: run the laptop-side suite from this build tree.
# It uses the system compiler, not the ARM one, so it is a plain
# custom target rather than a CMake executable.
# ---------------------------------------------------------------------

add_custom_target(host-tests
    COMMAND ${CMAKE_COMMAND} -E echo "Running host tests..."
    COMMAND make -C ${CMAKE_CURRENT_LIST_DIR}/test
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    USES_TERMINAL
)
