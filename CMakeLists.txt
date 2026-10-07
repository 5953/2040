cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(stm_programmer C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# FatFs: 从 fatfs/source/ 目录引用 (GitHub Actions 会自动下载)
add_library(fatfs STATIC
    fatfs/source/ff.c
    fatfs/source/ffsystem.c
    fatfs/source/ffunicode.c
)
# 项目根目录在前, 这样自定义 ffconf.h 优先于 FatFs 自带的
target_include_directories(fatfs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/fatfs/source
)

add_executable(stm_programmer
    main.c
    oled_display.c
    oled_font_cn.c
    chip_database.c
    protocol_and_menu.c
    usb_msc.c
    fatfs_diskio.c
)

pico_enable_stdio_usb(stm_programmer 0)
pico_enable_stdio_uart(stm_programmer 1)

pico_add_extra_outputs(stm_programmer)

target_include_directories(stm_programmer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(stm_programmer
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_gpio
    hardware_flash
    hardware_sync
    hardware_clocks
    tinyusb_device
    fatfs
)