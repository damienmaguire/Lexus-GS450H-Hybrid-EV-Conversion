# GS450h BTH spoof -- Raspberry Pi Pico 2 (RP2350) firmware.
# All settings of the car-tested build are the defaults in src/config.h,
# so a plain build reproduces the car build. See README.md.
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350 CACHE STRING "Platform")

include(pico_sdk_import.cmake)

project(gs450h_bth_spoof C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Source order is kept from the development build so the linked image matches.
add_executable(gs450h_bth_spoof
    src/main.c          # core 0 BTH sender, core 1 CAN loop, current sampling
    src/flash_log.c     # ring log, meta records, USB dump
    src/bmw_pack.c      # BMW module CAN decoding, cell guard, pack totals
    src/mcp2518fd.c     # MCP2518FD CAN controller driver
)

target_include_directories(gs450h_bth_spoof PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

target_link_libraries(gs450h_bth_spoof
    pico_stdlib
    pico_multicore
    hardware_uart
    hardware_adc
    hardware_gpio
    hardware_spi
    hardware_sync
    hardware_flash
    pico_flash
)

# BTH uses UART0, so stdio goes to USB only.
pico_enable_stdio_usb(gs450h_bth_spoof 1)
pico_enable_stdio_uart(gs450h_bth_spoof 0)

# USB product string. Kept as in the car-tested build.
target_compile_definitions(gs450h_bth_spoof PRIVATE
    USBD_PRODUCT="GS450h BTH v4m"
)

# Optional overrides for experiments, e.g. -DBTH_DEFS="RUN_DB_DA=10;AUX_V_FIXED=13.0f".
# Leave empty for the car build.
if(BTH_DEFS)
    target_compile_definitions(gs450h_bth_spoof PRIVATE ${BTH_DEFS})
endif()

pico_add_extra_outputs(gs450h_bth_spoof)
