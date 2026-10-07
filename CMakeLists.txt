cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(stm_programmer C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(stm_programmer
    main.c                    # 主程序: 按键 / 配置 / 主循环
    oled_display.c            # OLED 显示模块 (绘图 + 中英文文字)
    oled_font_cn.c            # 汉字点阵 (tools/gen_font.py 自动生成)
    chip_database.c           # 芯片数据库 (在这里添加新型号)
    protocol_and_menu.c       # SWIM/SWD 协议 + 菜单 + 烧录流程
)

pico_enable_stdio_usb(stm_programmer 1)
pico_enable_stdio_uart(stm_programmer 0)

pico_add_extra_outputs(stm_programmer)

target_link_libraries(stm_programmer
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_gpio
    hardware_flash
    hardware_sync
    hardware_clocks
)
