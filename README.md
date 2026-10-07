# STM 离线烧录器 (RP2040-Zero)

树莓派自动编译: GitHub → Actions → Run workflow → 下载 stm_programmer-uf2。

## 文件说明
| 文件 | 作用 |
|---|---|
| `main.c` | 按键、CRC、配置、系统初始化、主循环 |
| `oled_display.c/.h` | **OLED 显示模块** (I2C 驱动、绘图、UTF-8 中英文文字), 主代码只调用 `oled_*` 函数 |
| `oled_font_cn.c` | 汉字点阵, **自动生成, 不要手改** |
| `tools/gen_font.py` | 汉字点阵生成脚本 |
| `chip_database.c/.h` | **芯片数据库**, 在 `CHIP_TABLE` 里加一行就能新增型号 |
| `protocol_and_menu.c` | SWIM/SWD 协议、菜单、烧录流程、连接稳定性保护 |
| `stm_programmer.h` | 公共定义 (引脚、结构体、错误码) |

## 新增汉字后
菜单/提示里写了新汉字, 在工程根目录运行一次:

    python3 tools/gen_font.py

会重新生成 `oled_font_cn.c` (需要 `pip install pillow` 和系统里有中文字体; Windows 会自动用宋体)。
把生成后的 `oled_font_cn.c` 一起提交到 GitHub, Actions 编译时不需要 Python。
加 `--preview a.png` 可以输出点阵预览图。

## 新增芯片
编辑 `chip_database.c`:

    STM32_CHIP("型号", DEV_ID, Flash(KB), RAM(KB), 页大小(字节)),
    STM8_CHIP ("型号", ID,     Flash(KB), RAM(字节), 擦写块(字节)),

## 屏幕参数 (oled_display.h)
- 屏幕上下颠倒: `OLED_ROTATE_180` 改为 1
- 屏幕是 1.3 寸 SH1106: `OLED_COLUMN_OFFSET` 改为 2
- 引脚: `OLED_PIN_SDA` / `OLED_PIN_SCL`, 地址 `OLED_I2C_ADDR`
