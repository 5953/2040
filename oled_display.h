// oled_display.h - OLED 显示模块 (SSD1315 / SSD1306 / SH1106, I2C, 128x64)
//
// 使用方法:
//   oled_init();                         // 初始化 I2C + 屏幕
//   oled_clear();                        // 清空显存
//   oled_show_string(0, 0, "STM烧录器"); // 中英文混排 (UTF-8)
//   oled_refresh();                      // 把显存一次性送到屏幕
//
// 汉字点阵由 tools/gen_font.py 扫描源码自动生成 (oled_font_cn.c),
// 新增汉字后重新运行一次脚本即可, 不需要手写点阵。
#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

//=============================================================================
// 可配置项 (可在包含本头文件之前 #define 覆盖)
//=============================================================================
#ifndef OLED_WIDTH
#define OLED_WIDTH          128
#endif
#ifndef OLED_HEIGHT
#define OLED_HEIGHT         64
#endif
#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR       0x3C
#endif
#ifndef OLED_I2C_BAUD
#define OLED_I2C_BAUD       400000
#endif
#ifndef OLED_PIN_SDA
#define OLED_PIN_SDA        28
#endif
#ifndef OLED_PIN_SCL
#define OLED_PIN_SCL        29
#endif

// SSD1306/SSD1315 填 0; SH1106 的 132 列控制器填 2
#ifndef OLED_COLUMN_OFFSET
#define OLED_COLUMN_OFFSET  0
#endif
// 屏幕装反了(上下左右颠倒)时改成 1
#ifndef OLED_ROTATE_180
#define OLED_ROTATE_180     0
#endif

// 文字单元高度: 汉字 12x12, ASCII 6x8 (在 12 像素行内垂直居中)
#define OLED_CN_SIZE        12
#define OLED_ASCII_W        6
#define OLED_TEXT_HEIGHT    12

//=============================================================================
// 基础接口
//=============================================================================
bool oled_init(void);                    // 返回 false 表示总线上找不到屏幕
bool oled_is_present(void);              // 最近一次通信是否正常
void oled_clear(void);                   // 清空显存 (不会立刻刷新屏幕)
bool oled_refresh(void);                 // 显存 -> 屏幕, 失败时会自动尝试重新初始化
void oled_set_brightness(uint8_t value); // 0~255
void oled_set_display_on(bool on);

//=============================================================================
// 绘图
//=============================================================================
void oled_draw_pixel(int x, int y, bool on);
void oled_draw_hline(int x, int y, int w);
void oled_draw_vline(int x, int y, int h);
void oled_draw_rect(int x, int y, int w, int h, bool fill);
void oled_invert_rect(int x, int y, int w, int h);   // 反色 (可用来做选中高亮)

//=============================================================================
// 文字 (UTF-8, 中英文混排)
//=============================================================================
int  oled_text_width(const char *str);                              // 像素宽度
int  oled_show_string(int x, int y, const char *str);               // 返回结束 x 坐标
int  oled_show_string_clip(int x, int y, int max_w, const char *str); // 超宽自动截断
int  oled_show_string_center(int y, const char *str);               // 水平居中
void oled_show_number(int x, int y, uint32_t num);
void oled_show_progress(int x, int y, int w, int h, uint32_t value, uint32_t total);
void oled_show_percent(int x, int y, uint32_t value, uint32_t total);
void oled_show_speed(int x, int y, uint32_t bytes_per_sec);

//=============================================================================
// 汉字字库接口 (由 oled_font_cn.c 提供, 自动生成)
//=============================================================================
// 按 Unicode 查找 12x12 点阵; 每行 2 字节, 高位在前, 仅使用每行的高 12 位
const uint8_t *oled_font_cn_find(uint32_t unicode);

#endif // OLED_DISPLAY_H
