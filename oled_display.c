// oled_display.c - OLED 显示模块实现
#include "oled_display.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define OLED_I2C_PORT       i2c0
#define OLED_PAGES          (OLED_HEIGHT / 8)
#define OLED_I2C_TIMEOUT_US 20000    // 单次 I2C 事务超时, 防止屏幕掉线时卡死主循环
#define OLED_RETRY_MS       1000     // 通信失败后, 最短每隔多久尝试重新初始化一次

//=============================================================================
// 内部状态
//=============================================================================
static uint8_t  s_buf[OLED_WIDTH * OLED_PAGES];   // 显存: 每页 128 字节, bit0 在上
static bool     s_present = false;
static uint8_t  s_brightness = 0xCF;
static uint32_t s_last_retry_ms = 0;

//=============================================================================
// 6x8 ASCII 字体 (0x20 ~ 0x7E, 每个字符 6 列, 每列 bit0 在上)
//=============================================================================
static const uint8_t font_6x8[95][6] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5F, 0x00, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00, 0x00}, {0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00}, {0x23, 0x13, 0x08, 0x64, 0x62, 0x00},
    {0x36, 0x49, 0x56, 0x20, 0x50, 0x00}, {0x00, 0x08, 0x07, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00, 0x00}, {0x00, 0x41, 0x22, 0x1C, 0x00, 0x00},
    {0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 0x00}, {0x08, 0x08, 0x3E, 0x08, 0x08, 0x00},
    {0x00, 0x80, 0x70, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08, 0x00},
    {0x00, 0x00, 0x60, 0x60, 0x00, 0x00}, {0x20, 0x10, 0x08, 0x04, 0x02, 0x00},
    {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00}, {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00},
    {0x72, 0x49, 0x49, 0x49, 0x46, 0x00}, {0x21, 0x41, 0x49, 0x4D, 0x33, 0x00},
    {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00}, {0x27, 0x45, 0x45, 0x45, 0x39, 0x00},
    {0x3C, 0x4A, 0x49, 0x49, 0x31, 0x00}, {0x41, 0x21, 0x11, 0x09, 0x07, 0x00},
    {0x36, 0x49, 0x49, 0x49, 0x36, 0x00}, {0x46, 0x49, 0x49, 0x29, 0x1E, 0x00},
    {0x00, 0x00, 0x14, 0x00, 0x00, 0x00}, {0x00, 0x40, 0x34, 0x00, 0x00, 0x00},
    {0x00, 0x08, 0x14, 0x22, 0x41, 0x00}, {0x14, 0x14, 0x14, 0x14, 0x14, 0x00},
    {0x00, 0x41, 0x22, 0x14, 0x08, 0x00}, {0x02, 0x01, 0x59, 0x09, 0x06, 0x00},
    {0x3E, 0x41, 0x5D, 0x59, 0x4E, 0x00}, {0x7C, 0x12, 0x11, 0x12, 0x7C, 0x00},
    {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00}, {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00},
    {0x7F, 0x41, 0x41, 0x41, 0x3E, 0x00}, {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00},
    {0x7F, 0x09, 0x09, 0x09, 0x01, 0x00}, {0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00},
    {0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00}, {0x00, 0x41, 0x7F, 0x41, 0x00, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01, 0x00}, {0x7F, 0x08, 0x14, 0x22, 0x41, 0x00},
    {0x7F, 0x40, 0x40, 0x40, 0x40, 0x00}, {0x7F, 0x02, 0x1C, 0x02, 0x7F, 0x00},
    {0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00}, {0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00},
    {0x7F, 0x09, 0x09, 0x09, 0x06, 0x00}, {0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00},
    {0x7F, 0x09, 0x19, 0x29, 0x46, 0x00}, {0x26, 0x49, 0x49, 0x49, 0x32, 0x00},
    {0x03, 0x01, 0x7F, 0x01, 0x03, 0x00}, {0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00},
    {0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00}, {0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00},
    {0x63, 0x14, 0x08, 0x14, 0x63, 0x00}, {0x03, 0x04, 0x78, 0x04, 0x03, 0x00},
    {0x61, 0x59, 0x49, 0x4D, 0x43, 0x00}, {0x00, 0x7F, 0x41, 0x41, 0x41, 0x00},
    {0x02, 0x04, 0x08, 0x10, 0x20, 0x00}, {0x00, 0x41, 0x41, 0x41, 0x7F, 0x00},
    {0x04, 0x02, 0x01, 0x02, 0x04, 0x00}, {0x40, 0x40, 0x40, 0x40, 0x40, 0x00},
    {0x00, 0x03, 0x07, 0x08, 0x00, 0x00}, {0x20, 0x54, 0x54, 0x78, 0x40, 0x00},
    {0x7F, 0x28, 0x44, 0x44, 0x38, 0x00}, {0x38, 0x44, 0x44, 0x44, 0x28, 0x00},
    {0x38, 0x44, 0x44, 0x28, 0x7F, 0x00}, {0x38, 0x54, 0x54, 0x54, 0x18, 0x00},
    {0x00, 0x08, 0x7E, 0x09, 0x02, 0x00}, {0x18, 0xA4, 0xA4, 0x9C, 0x78, 0x00},
    {0x7F, 0x08, 0x04, 0x04, 0x78, 0x00}, {0x00, 0x44, 0x7D, 0x40, 0x00, 0x00},
    {0x20, 0x40, 0x40, 0x3D, 0x00, 0x00}, {0x7F, 0x10, 0x28, 0x44, 0x00, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00, 0x00}, {0x7C, 0x04, 0x78, 0x04, 0x78, 0x00},
    {0x7C, 0x08, 0x04, 0x04, 0x78, 0x00}, {0x38, 0x44, 0x44, 0x44, 0x38, 0x00},
    {0xFC, 0x18, 0x24, 0x24, 0x18, 0x00}, {0x18, 0x24, 0x24, 0x18, 0xFC, 0x00},
    {0x7C, 0x08, 0x04, 0x04, 0x08, 0x00}, {0x48, 0x54, 0x54, 0x54, 0x24, 0x00},
    {0x04, 0x04, 0x3F, 0x44, 0x24, 0x00}, {0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00},
    {0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00}, {0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00},
    {0x44, 0x28, 0x10, 0x28, 0x44, 0x00}, {0x4C, 0x90, 0x90, 0x90, 0x7C, 0x00},
    {0x44, 0x64, 0x54, 0x4C, 0x44, 0x00}, {0x00, 0x08, 0x36, 0x41, 0x00, 0x00},
    {0x00, 0x00, 0x77, 0x00, 0x00, 0x00}, {0x00, 0x41, 0x36, 0x08, 0x00, 0x00},
    {0x02, 0x01, 0x02, 0x04, 0x02, 0x00},
};

//=============================================================================
// 底层 I2C
//=============================================================================
// 一次事务里发送 [控制字节 + 若干数据]: 控制字节 0x00 = 后面全是命令, 0x40 = 后面全是显示数据
static bool oled_write(uint8_t control, const uint8_t *data, size_t len) {
    uint8_t pkt[1 + OLED_WIDTH];
    if (len > OLED_WIDTH) return false;
    pkt[0] = control;
    memcpy(&pkt[1], data, len);
    int ret = i2c_write_timeout_us(OLED_I2C_PORT, OLED_I2C_ADDR, pkt, len + 1, false,
                                   OLED_I2C_TIMEOUT_US);
    return ret == (int)(len + 1);
}

static bool oled_cmds(const uint8_t *cmds, size_t len) {
    return oled_write(0x00, cmds, len);
}

// 屏幕控制器初始化序列
static bool oled_hw_init(void) {
    const uint8_t init_seq[] = {
        0xAE,             // 关显示
        0xD5, 0x80,       // 时钟分频
        0xA8, OLED_HEIGHT - 1,  // 多路复用比
        0xD3, 0x00,       // 显示偏移
        0x40,             // 起始行
        0x8D, 0x14,       // 打开电荷泵 (内部升压, 必须)
        0x20, 0x02,       // 页地址模式
#if OLED_ROTATE_180
        0xA0, 0xC0,
#else
        0xA1, 0xC8,
#endif
        0xDA, (OLED_HEIGHT == 64) ? 0x12 : 0x02,   // COM 引脚配置
        0x81, s_brightness,
        0xD9, 0xF1,       // 预充电
        0xDB, 0x40,       // VCOMH
        0xA4,             // 按显存内容显示
        0xA6,             // 正常(非反色)显示
        0xAF              // 开显示
    };
    return oled_cmds(init_seq, sizeof(init_seq));
}

//=============================================================================
// 基础接口
//=============================================================================
bool oled_init(void) {
    i2c_init(OLED_I2C_PORT, OLED_I2C_BAUD);
    gpio_set_function(OLED_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(OLED_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_PIN_SDA);
    gpio_pull_up(OLED_PIN_SCL);

    sleep_ms(100);                       // 等屏幕上电稳定
    memset(s_buf, 0, sizeof(s_buf));
    s_present = oled_hw_init();
    if (s_present) oled_refresh();
    return s_present;
}

bool oled_is_present(void) { return s_present; }

void oled_clear(void) { memset(s_buf, 0, sizeof(s_buf)); }

bool oled_refresh(void) {
    // 上次通信失败: 限速重试初始化 (排线接触不良/屏幕被拔插后自动恢复)
    if (!s_present) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - s_last_retry_ms < OLED_RETRY_MS) return false;
        s_last_retry_ms = now;
        if (!oled_hw_init()) return false;
        s_present = true;
    }

    for (int page = 0; page < OLED_PAGES; page++) {
        const uint8_t col = OLED_COLUMN_OFFSET;
        const uint8_t addr_cmd[3] = {
            (uint8_t)(0xB0 + page),            // 页地址
            (uint8_t)(0x00 | (col & 0x0F)),    // 列地址低 4 位
            (uint8_t)(0x10 | (col >> 4))       // 列地址高 4 位
        };
        if (!oled_cmds(addr_cmd, sizeof(addr_cmd)) ||
            !oled_write(0x40, &s_buf[page * OLED_WIDTH], OLED_WIDTH)) {
            s_present = false;
            return false;
        }
    }
    return true;
}

void oled_set_brightness(uint8_t value) {
    s_brightness = value;
    if (!s_present) return;
    const uint8_t cmd[2] = { 0x81, value };
    if (!oled_cmds(cmd, sizeof(cmd))) s_present = false;
}

void oled_set_display_on(bool on) {
    if (!s_present) return;
    const uint8_t cmd[1] = { on ? 0xAF : 0xAE };
    if (!oled_cmds(cmd, sizeof(cmd))) s_present = false;
}

//=============================================================================
// 绘图
//=============================================================================
void oled_draw_pixel(int x, int y, bool on) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    uint8_t *p = &s_buf[(y >> 3) * OLED_WIDTH + x];
    if (on) *p |= (uint8_t)(1u << (y & 7));
    else    *p &= (uint8_t)~(1u << (y & 7));
}

void oled_draw_hline(int x, int y, int w) {
    for (int i = 0; i < w; i++) oled_draw_pixel(x + i, y, true);
}

void oled_draw_vline(int x, int y, int h) {
    for (int i = 0; i < h; i++) oled_draw_pixel(x, y + i, true);
}

void oled_draw_rect(int x, int y, int w, int h, bool fill) {
    if (w <= 0 || h <= 0) return;
    if (fill) {
        for (int i = 0; i < h; i++) oled_draw_hline(x, y + i, w);
    } else {
        oled_draw_hline(x, y, w);
        oled_draw_hline(x, y + h - 1, w);
        oled_draw_vline(x, y, h);
        oled_draw_vline(x + w - 1, y, h);
    }
}

void oled_invert_rect(int x, int y, int w, int h) {
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            int px = x + i, py = y + j;
            if (px < 0 || px >= OLED_WIDTH || py < 0 || py >= OLED_HEIGHT) continue;
            s_buf[(py >> 3) * OLED_WIDTH + px] ^= (uint8_t)(1u << (py & 7));
        }
    }
}

//=============================================================================
// 文字
//=============================================================================
// 解码一个 UTF-8 字符, 返回字节数 (至少 1, 遇到非法字节按 1 字节跳过)
static int utf8_next(const char *s, uint32_t *cp) {
    const uint8_t c = (uint8_t)s[0];
    if (c < 0x80) { *cp = c; return 1; }
    int n = 0;
    uint32_t v = 0;
    if      ((c & 0xE0) == 0xC0) { n = 2; v = c & 0x1F; }
    else if ((c & 0xF0) == 0xE0) { n = 3; v = c & 0x0F; }
    else if ((c & 0xF8) == 0xF0) { n = 4; v = c & 0x07; }
    else { *cp = 0xFFFD; return 1; }
    for (int i = 1; i < n; i++) {
        if (((uint8_t)s[i] & 0xC0) != 0x80) { *cp = 0xFFFD; return i; }  // 截断的序列
        v = (v << 6) | ((uint8_t)s[i] & 0x3F);
    }
    *cp = v;
    return n;
}

static int glyph_width(uint32_t cp) {
    return (cp >= 0x20 && cp <= 0x7E) ? OLED_ASCII_W : OLED_CN_SIZE;
}

// 画一个字符, 返回它占的宽度
static int draw_glyph(int x, int y, uint32_t cp) {
    if (cp >= 0x20 && cp <= 0x7E) {
        const uint8_t *g = font_6x8[cp - 0x20];
        const int yo = y + (OLED_TEXT_HEIGHT - 8) / 2;     // 在 12 像素行里垂直居中
        for (int col = 0; col < OLED_ASCII_W; col++)
            for (int row = 0; row < 8; row++)
                if (g[col] & (1u << row)) oled_draw_pixel(x + col, yo + row, true);
        return OLED_ASCII_W;
    }

    const uint8_t *g = oled_font_cn_find(cp);
    if (!g) {                                              // 字库里没有: 画空心方框占位
        oled_draw_rect(x, y, OLED_CN_SIZE - 1, OLED_CN_SIZE - 1, false);
        return OLED_CN_SIZE;
    }
    for (int row = 0; row < OLED_CN_SIZE; row++) {
        const uint16_t bits = (uint16_t)((g[row * 2] << 8) | g[row * 2 + 1]);
        for (int col = 0; col < OLED_CN_SIZE; col++)
            if (bits & (0x8000u >> col)) oled_draw_pixel(x + col, y + row, true);
    }
    return OLED_CN_SIZE;
}

int oled_text_width(const char *str) {
    int w = 0;
    while (*str) {
        uint32_t cp;
        str += utf8_next(str, &cp);
        w += glyph_width(cp);
    }
    return w;
}

int oled_show_string_clip(int x, int y, int max_w, const char *str) {
    int pos = x;
    const int limit = (max_w > 0 && x + max_w < OLED_WIDTH) ? x + max_w : OLED_WIDTH;
    while (*str) {
        uint32_t cp;
        int n = utf8_next(str, &cp);
        int w = glyph_width(cp);
        if (pos + w > limit) break;                        // 放不下就整字丢弃, 不会画出半个字
        pos += draw_glyph(pos, y, cp);
        str += n;
    }
    return pos;
}

int oled_show_string(int x, int y, const char *str) {
    return oled_show_string_clip(x, y, 0, str);
}

int oled_show_string_center(int y, const char *str) {
    int w = oled_text_width(str);
    int x = (OLED_WIDTH - w) / 2;
    return oled_show_string(x < 0 ? 0 : x, y, str);
}

void oled_show_number(int x, int y, uint32_t num) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)num);
    oled_show_string(x, y, buf);
}

void oled_show_progress(int x, int y, int w, int h, uint32_t value, uint32_t total) {
    oled_draw_rect(x, y, w, h, false);
    if (total == 0 || w < 3 || h < 3) return;
    if (value > total) value = total;
    int fill = (int)((uint64_t)value * (uint32_t)(w - 2) / total);
    if (fill > 0) oled_draw_rect(x + 1, y + 1, fill, h - 2, true);
}

void oled_show_percent(int x, int y, uint32_t value, uint32_t total) {
    char buf[8];
    uint32_t pct = (total > 0) ? (uint32_t)((uint64_t)value * 100 / total) : 0;
    if (pct > 100) pct = 100;
    snprintf(buf, sizeof(buf), "%lu%%", (unsigned long)pct);
    oled_show_string(x, y, buf);
}

void oled_show_speed(int x, int y, uint32_t bytes_per_sec) {
    char buf[16];
    if (bytes_per_sec >= 1024) snprintf(buf, sizeof(buf), "%luK/s", (unsigned long)(bytes_per_sec / 1024));
    else                       snprintf(buf, sizeof(buf), "%luB/s", (unsigned long)bytes_per_sec);
    oled_show_string(x, y, buf);
}
