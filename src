# =====================================================================
# INF2004 Autonomous Robot Car
# Buddy 5: adaptive ultrasonic scanning and obstacle profiling
# Raspberry Pi Pico W on Cytron Robo Pico
#
# IF YOU USE THE VS CODE "RASPBERRY PI PICO" EXTENSION
# ---------------------------------------------------
# The extension generates its own CMakeLists.txt with version numbers
# pinned to YOUR installed SDK and toolchain. Those are machine
# specific. Do not copy mine.
#
#   1. Keep everything the extension generated, from line 1 down to
#      and including pico_sdk_init().
#   2. Delete its add_executable() / target_link_libraries() lines.
#   3. Paste everything below the "BUDDY 5 STARTS HERE" marker.
#
# The top block here is an EXAMPLE for command-line builds only.
#
# Build configurations:
#   -DUSE_MTK3=OFF (default)  Pico SDK + bring-up shim. Flash today;
#                             src/main.c gives a USB serial test menu.
#   -DUSE_MTK3=ON             Real micro T-Kernel 3.0 (graded build).
#                             src/usermain.c is the entry point.
# =====================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
# EXAMPLE VALUES. If the extension made your CMakeLists, keep ITS
# numbers - they must match what is installed under ~/.pico-sdk.
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico_w CACHE STRING "Board type")

# The VS Code extension generates this file. Without the extension:
#   cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
include(pico_sdk_import.cmake)

project(INF2004_Autonomous_Robot_Car C CXX ASM)

pico_sdk_init()

# =====================================================================
# ===================== BUDDY 5 STARTS HERE ===========================
# =====================================================================

option(USE_MTK3 "Build against micro T-Kernel 3.0 instead of the shim" OFF)
set(MTK3_PATH "" CACHE PATH "Path to the micro T-Kernel 3.0 BSP")

# The four subsystem files. Identical in both builds; the only files
# that ship in the integrated robot.
set(OBSTACLE_SOURCES
    src/servo.c
    src/ultrasonic.c
    src/obstacle.c
    src/obstacle_geometry.c
)

add_executable(buddy5 ${OBSTACLE_SOURCES})

target_include_directories(buddy5 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

if(USE_MTK3)
    if(NOT MTK3_PATH)
        message(FATAL_ERROR
            "USE_MTK3=ON needs -DMTK3_PATH=/path/to/mtkernel_3")
    endif()

    message(STATUS "Buddy 5: building against micro T-Kernel 3.0")

    target_sources(buddy5 PRIVATE src/usermain.c)

    # Adjust to your BSP layout. See porting notes in README_BUILD.md.
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

    # shim/ provides tk/tkernel.h, so obstacle.c and ultrasonic.c need
    # no #ifdef.
    target_include_directories(buddy5 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/shim
    )
endif()

target_link_libraries(buddy5 PRIVATE
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_clocks
    hardware_timer
    hardware_sync
    hardware_irq
)

# Subsystem builds clean under all of these. Fix code, not flags.
target_compile_options(buddy5 PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wcast-qual
    -Wmissing-prototypes -Wstrict-prototypes -Wundef -fno-common
)

# target_compile_options(buddy5 PRIVATE -Werror)

target_compile_options(buddy5 PRIVATE $<$<CONFIG:Debug>:-Og>)
target_compile_options(buddy5 PRIVATE $<$<CONFIG:Release>:-Os>)

# USB CDC for the bench. Switch to UART once the car drives.
pico_enable_stdio_usb(buddy5 1)
pico_enable_stdio_uart(buddy5 0)

pico_add_extra_outputs(buddy5)

# Host tests use the PC compiler: cmake --build build --target host-tests
add_custom_target(host-tests
    COMMAND make -C ${CMAKE_CURRENT_LIST_DIR}/test
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    USES_TERMINAL
)
