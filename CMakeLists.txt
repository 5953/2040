cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(stm_programmer C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ===== 自动查找 FatFs (来自 TinyUSB 库) =====
# Pico SDK 下载后, TinyUSB 在 lib/tinyusb/, FatFs 在 lib/tinyusb/lib/fatfs/source/
set(FATFS_SOURCE_DIR "")

# 在 Pico SDK 的 TinyUSB 里找
file(GLOB FATFS_CANDIDATES
    "${PICO_SDK_PATH}/lib/tinyusb/lib/fatfs/source/ff.c"
    "${PICO_SDK_PATH}/lib/tinyusb/lib/fatfs/ff.c"
    "${CMAKE_CURRENT_BINARY_DIR}/_pico_sdk/pico-sdk/lib/tinyusb/lib/fatfs/source/ff.c"
)
if(FATFS_CANDIDATES)
    list(GET FATFS_CANDIDATES 0 FIRST_MATCH)
    get_filename_component(FATFS_SOURCE_DIR "${FIRST_MATCH}" DIRECTORY)
    message(STATUS "FatFs found in TinyUSB: ${FATFS_SOURCE_DIR}")
endif()

# 如果没找到, 单独下载 TinyUSB (只需要 fatfs 部分)
if(NOT FATFS_SOURCE_DIR)
    message(STATUS "FatFs not in TinyUSB, downloading...")
    include(FetchContent)
    FetchContent_Declare(
        tinyusb_fatfs
        GIT_REPOSITORY https://github.com/hathach/tinyusb.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_SUBMODULES ""
    )
    FetchContent_Populate(tinyusb_fatfs)

    file(GLOB FATFS_CANDIDATES2
        "${tinyusb_fatfs_SOURCE_DIR}/lib/fatfs/source/ff.c"
        "${tinyusb_fatfs_SOURCE_DIR}/lib/fatfs/ff.c"
    )
    if(FATFS_CANDIDATES2)
        list(GET FATFS_CANDIDATES2 0 FIRST_MATCH2)
        get_filename_component(FATFS_SOURCE_DIR "${FIRST_MATCH2}" DIRECTORY)
        message(STATUS "FatFs found in downloaded TinyUSB: ${FATFS_SOURCE_DIR}")
    endif()
endif()

if(NOT FATFS_SOURCE_DIR)
    message(FATAL_ERROR "FatFs not found! 请检查 Pico SDK 是否正确下载")
endif()

message(STATUS "Using FatFs: ${FATFS_SOURCE_DIR}")

# ===== 强制打开 FF_USE_MKFS =====
# SDK 自带的 ffconf.h 里 FF_USE_MKFS 硬编码为 0, 命令行 -DFF_USE_MKFS=1 会被它覆盖,
# 导致 f_mkfs 根本没被编译进 ff.c.o, 链接时报 undefined reference.
# 解决办法: 在 CMake 配置阶段直接把 ffconf.h 里的 0 改成 1 (幂等, 重复跑不会重复改).
set(FATFS_FFCONF "${FATFS_SOURCE_DIR}/ffconf.h")
if(EXISTS "${FATFS_FFCONF}")
    file(READ "${FATFS_FFCONF}" _ffconf_content)
    if(_ffconf_content MATCHES "#define[ \t]+FF_USE_MKFS[ \t]+1")
        message(STATUS "ffconf.h already has FF_USE_MKFS=1, skip patch")
    elseif(_ffconf_content MATCHES "#define[ \t]+FF_USE_MKFS[ \t]+0")
        string(REGEX REPLACE
            "#define[ \t]+FF_USE_MKFS[ \t]+0"
            "#define FF_USE_MKFS   1"
            _ffconf_content "${_ffconf_content}")
        file(WRITE "${FATFS_FFCONF}" "${_ffconf_content}")
        message(STATUS "Patched ffconf.h: FF_USE_MKFS -> 1")
    else()
        message(WARNING "无法在 ffconf.h 中找到 FF_USE_MKFS 定义, 请手动检查 ${FATFS_FFCONF}")
    endif()
else()
    message(WARNING "未找到 ffconf.h: ${FATFS_FFCONF}")
endif()

file(GLOB FATFS_SOURCES
    "${FATFS_SOURCE_DIR}/ff.c"
    "${FATFS_SOURCE_DIR}/ffsystem.c"
    "${FATFS_SOURCE_DIR}/ffunicode.c"
)

add_library(fatfs STATIC ${FATFS_SOURCES})
target_include_directories(fatfs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${FATFS_SOURCE_DIR}
)

# ===== 主程序 =====
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
