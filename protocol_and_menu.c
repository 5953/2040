// protocol_and_menu.c - 协议实现和完整菜单系统 (STM8 优化版)
// 修复:
//   [SWIM] 入场序列按 UM0470 重写
//   [SWIM] 位时序/读写位重写
//   [SWIM] Flash 解锁地址 PUKR(0x5062), 密钥 0x56/0xAE, 检查 PUL 位
//   [SWIM] Flash 就绪检查 IAPSR.EOP(0x04)
//   [SWIM] Flash 擦除 CR2/NCR2 写两个寄存器
//   [SWD]  读位采样时机修正 + dir_input 关内部上下拉
//   [SWD]  初始化先低速, 读完 IDCODE 再恢复
//   [DET]  优先尝试 SWIM (适合刷 STM8)
#include "stm_programmer.h"
#include "chip_database.h"
#include "ff.h"

extern programmer_state_t g_state;
extern menu_state_t g_menu;
extern uint8_t g_stream_buffer[];

static chip_type_t g_link_type = CHIP_TYPE_UNKNOWN;
static uint16_t    g_link_dev_id = 0;
static uint32_t    g_link_dp_idcode = 0;

static void menu_draw_progress(void);

//=============================================================================
// SWIM协议实现（STM8）— 按 UM0470 重写
//=============================================================================

// --- SWIM 时序参数 ---
#define SWIM_CLK_SLOW_US        22      // 复位后慢速: 22us/bit (45kHz)

// SWIM / STM8 Flash 寄存器地址
#define SWIM_CSR_ADDR           0x7F80
#define FLASH_IAPSR_ADDR        0x505F
#define FLASH_CR2_ADDR          0x505B
#define FLASH_NCR2_ADDR         0x505C
#define FLASH_PUKR_ADDR         0x5062  // 程序 Flash 解锁寄存器
#define FLASH_DUKR_ADDR         0x5064  // 数据 EEPROM 解锁寄存器

// IAPSR 位定义
#define IAPSR_HALT              0x80
#define IAPSR_PUL               0x08    // 程序存储器解锁
#define IAPSR_EOP               0x04    // 操作完成
#define IAPSR_DUL               0x02    // 数据 EEPROM 解锁

static inline void swim_high(void)  { gpio_put(PIN_SWIM_SWDIO, 1); }
static inline void swim_low(void)   { gpio_put(PIN_SWIM_SWDIO, 0); }
static inline bool swim_read(void)  { return gpio_get(PIN_SWIM_SWDIO); }

static inline void swim_output(void) {
    gpio_set_dir(PIN_SWIM_SWDIO, GPIO_OUT);
}

static inline void swim_input(void) {
    gpio_set_dir(PIN_SWIM_SWDIO, GPIO_IN);
    gpio_disable_pulls(PIN_SWIM_SWDIO);  // 关内部上下拉, 避免干扰 target 驱动
}

// --- 入场序列 (UM0470) ---
// NRST 拉低 → SWIM 低 16 周期 → 4 组 (高 4 周期 + 低 4 周期) → 释放
static bool swim_entry_sequence(void) {
    swim_output();

    gpio_put(PIN_NRST, 0);
    sleep_us(100);                          // 让芯片进入复位状态

    // 1) SWIM 保持低 16 个 SWIM 时钟周期
    swim_low();
    sleep_us(16 * SWIM_CLK_SLOW_US);        // ≈ 352us

    // 2) 4 组「高 4 周期 + 低 4 周期」
    for (int i = 0; i < 4; i++) {
        swim_high();
        sleep_us(4 * SWIM_CLK_SLOW_US);
        swim_low();
        sleep_us(4 * SWIM_CLK_SLOW_US);
    }

    // 3) 释放 SWIM 并保持
    swim_high();
    sleep_us(6 * SWIM_CLK_SLOW_US);

    // 4) 释放 NRST
    gpio_put(PIN_NRST, 1);
    sleep_us(200);
    return true;
}

// --- 写位: 1 = 低1/4 + 高3/4, 0 = 高1/4 + 低3/4 ---
static void swim_write_bit(bool bit) {
    swim_output();
    if (bit) {
        swim_low();  sleep_us(1);
        swim_high(); sleep_us(3);
    } else {
        swim_high(); sleep_us(1);
        swim_low();  sleep_us(3);
    }
}

// --- 读位: 主机释放线, 在位的 T/2 处采样 ---
// 调用前请确保采样点已对齐 (调用方负责 sleep_us 补足)
static bool swim_read_bit(void) {
    swim_input();
    sleep_us(2);
    return swim_read();
}

// 等待起始位的下降沿. 起始位为 0, 高位 1/4 后在 T/4 处下降
// 返回时已经 sleep 到 T/2, 可直接开始读位
static bool swim_wait_start_bit(void) {
    swim_input();

    uint32_t timeout = 100000;
    while (swim_read() && --timeout);       // 等线拉低
    if (!timeout) return false;

    sleep_us(1);                            // 从 T/4 走到 T/2
    return true;
}

// --- 写字节: 起始位(0) + 8 数据位(LSB first) + 奇偶位, 等 ACK ---
static bool swim_write_byte(uint8_t data) {
    swim_write_bit(0);                      // 起始位

    uint8_t parity = data;
    parity ^= parity >> 4;
    parity ^= parity >> 2;
    parity ^= parity >> 1;
    parity &= 1;

    for (int i = 0; i < 8; i++)
        swim_write_bit((data >> i) & 1);

    swim_write_bit(parity);

    // ACK: target 把线拉低一小段时间
    swim_input();
    sleep_us(3);
    uint32_t t = 1000;
    while (swim_read() && --t);             // 等 ACK 低
    if (!t) return false;
    t = 1000;
    while (!swim_read() && --t);            // 等 ACK 恢复高
    if (!t) return false;

    swim_output();
    swim_high();
    return true;
}

// --- 读字节: 等起始位 → 读 8 数据位 + 1 奇偶位 ---
static bool swim_read_byte(uint8_t *data) {
    if (!swim_wait_start_bit()) return false;  // 采样点 = 起始位 T/2

    // 起始位本身应该为 0
    if (swim_read_bit()) return false;

    uint8_t value = 0;
    for (int i = 0; i < 8; i++) {
        // 每位整周期 T=4us. swim_read_bit() 内部 sleep 2us,
        // 从 T/2 跳到下一位的 T/2 还差 2us
        sleep_us(2);
        if (swim_read_bit()) value |= (1 << i);  // LSB first
    }

    // 奇偶位
    sleep_us(2);
    swim_read_bit();                        // 丢弃奇偶校验

    swim_output();
    swim_high();

    *data = value;
    return true;
}

static bool swim_read_memory(uint32_t addr, uint8_t *data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        uint8_t retry = 3;
        bool ok = false;
        while (retry-- && !ok) {
            ok  = swim_write_byte(0x01);    // READ 命令
            ok &= swim_write_byte((addr >> 16) & 0xFF);
            ok &= swim_write_byte((addr >> 8)  & 0xFF);
            ok &= swim_write_byte(addr & 0xFF);
            ok &= swim_read_byte(&data[i]);
        }
        if (!ok) return false;
        addr++;
    }
    return true;
}

static bool swim_write_memory(uint32_t addr, const uint8_t *data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        uint8_t retry = 3;
        bool ok = false;
        while (retry-- && !ok) {
            ok  = swim_write_byte(0x02);    // WRITE 命令
            ok &= swim_write_byte((addr >> 16) & 0xFF);
            ok &= swim_write_byte((addr >> 8)  & 0xFF);
            ok &= swim_write_byte(addr & 0xFF);
            ok &= swim_write_byte(data[i]);
        }
        if (!ok) return false;
        addr++;
    }
    return true;
}

static bool swim_init(void) {
    if (!swim_entry_sequence()) return false;

    uint8_t csr;
    if (!swim_read_memory(SWIM_CSR_ADDR, &csr, 1)) {
        printf("[SWIM] CSR read FAIL\n");
        return false;
    }
    printf("[SWIM] CSR = 0x%02X\n", csr);
    return true;
}

static bool swim_read_device_id(uint16_t *device_id) {
    if (!swim_init()) return false;

    uint8_t id[2];
    if (!swim_read_memory(0x004850, id, 2)) {
        printf("[SWIM] DeviceID read FAIL\n");
        return false;
    }

    *device_id = (id[0] << 8) | id[1];
    printf("[SWIM] DeviceID = 0x%04X\n", *device_id);

    if (*device_id == 0x0000 || *device_id == 0xFFFF) return false;
    return true;
}

// --- 解锁程序 Flash (PUKR, 0x5062, 密钥 0x56/0xAE) ---
static bool swim_unlock_flash(void) {
    printf("[FLASH] unlock PUKR\n");

    uint8_t key1 = 0x56;
    uint8_t key2 = 0xAE;
    if (!swim_write_memory(FLASH_PUKR_ADDR, &key1, 1)) {
        printf("[FLASH] key1 write FAIL\n");
        return false;
    }
    if (!swim_write_memory(FLASH_PUKR_ADDR, &key2, 1)) {
        printf("[FLASH] key2 write FAIL\n");
        return false;
    }
    sleep_ms(10);

    uint8_t iapsr;
    if (!swim_read_memory(FLASH_IAPSR_ADDR, &iapsr, 1)) {
        printf("[FLASH] IAPSR read FAIL\n");
        return false;
    }
    printf("[FLASH] IAPSR = 0x%02X (PUL=%d)\n", iapsr, !!(iapsr & IAPSR_PUL));
    return (iapsr & IAPSR_PUL) != 0;
}

// --- 等待 Flash 操作完成 (IAPSR.EOP) ---
static bool swim_wait_flash_ready(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms) {
        uint8_t iapsr;
        if (!swim_read_memory(FLASH_IAPSR_ADDR, &iapsr, 1)) return false;
        if ((iapsr & IAPSR_EOP) != 0) return true;
        sleep_ms(1);
    }
    printf("[FLASH] wait ready TIMEOUT\n");
    return false;
}

// --- 全片擦除 ---
static bool swim_erase_chip(void) {
    if (!swim_unlock_flash()) return false;

    // 写 CR2 / NCR2 使能块擦除
    uint8_t cr2  = 0x02;
    uint8_t ncr2 = (uint8_t)~0x02;
    if (!swim_write_memory(FLASH_CR2_ADDR,  &cr2,  1)) {
        printf("[FLASH] CR2 write FAIL\n");
        return false;
    }
    if (!swim_write_memory(FLASH_NCR2_ADDR, &ncr2, 1)) {
        printf("[FLASH] NCR2 write FAIL\n");
        return false;
    }

    // 向块内任意地址写一个字节触发擦除
    uint8_t dummy = 0x00;
    if (!swim_write_memory(0x8000, &dummy, 1)) {
        printf("[FLASH] erase trigger FAIL\n");
        return false;
    }

    return swim_wait_flash_ready(5000);
}

//=============================================================================
// SWD协议实现（STM32）— 优化版
//=============================================================================

static volatile uint32_t g_swd_delay_cycles = 15;

void swd_set_speed(swd_speed_t speed) {
    switch (speed) {
        case SWD_SPEED_LOW:    g_swd_delay_cycles = 50; break;
        case SWD_SPEED_MEDIUM: g_swd_delay_cycles = 15; break;
        case SWD_SPEED_HIGH:   g_swd_delay_cycles = 6;  break;
    }
}

static inline void swd_delay(void) {
    for (volatile uint32_t i = 0; i < g_swd_delay_cycles; i++) {
        __asm volatile("nop");
    }
}

static inline void swd_dir_input(void) {
    gpio_set_dir(PIN_SWIM_SWDIO, GPIO_IN);
    gpio_disable_pulls(PIN_SWIM_SWDIO);
}

static inline void swd_dir_output(void) {
    gpio_set_dir(PIN_SWIM_SWDIO, GPIO_OUT);
}

static inline void swd_clock_low(void) {
    gpio_put(PIN_SWCLK, 0);
    swd_delay();
}

static inline void swd_clock_high(void) {
    gpio_put(PIN_SWCLK, 1);
    swd_delay();
}

static inline void swd_clock(void) {
    swd_clock_low();
    swd_clock_high();
}

static void swd_write_bit(bool bit) {
    swd_dir_output();
    gpio_put(PIN_SWIM_SWDIO, bit);
    swd_delay();
    swd_clock_low();
    swd_clock_high();
    gpio_put(PIN_SWCLK, 0);         // 补一个下降沿, 保证周期完整
    swd_delay();
}

static bool swd_read_bit(void) {
    swd_dir_input();
    swd_clock_low();
    swd_clock_high();               // 上升沿, target 驱动数据
    bool bit = gpio_get(PIN_SWIM_SWDIO);
    gpio_put(PIN_SWCLK, 0);
    swd_delay();
    return bit;
}

static bool calc_parity(uint32_t data) {
    data ^= data >> 16;
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return data & 1;
}

static void swd_line_reset(void) {
    swd_dir_output();
    gpio_set_dir(PIN_SWCLK, GPIO_OUT);

    gpio_put(PIN_SWIM_SWDIO, 1);
    for (int i = 0; i < 60; i++) swd_clock();

    uint16_t sync = 0xE79E;
    for (int i = 0; i < 16; i++) {
        swd_write_bit((sync >> i) & 1);
    }

    gpio_put(PIN_SWIM_SWDIO, 1);
    for (int i = 0; i < 60; i++) swd_clock();

    gpio_put(PIN_SWIM_SWDIO, 0);
    for (int i = 0; i < 8; i++) swd_clock();

    gpio_put(PIN_SWCLK, 0);
}

// WAIT 重试 + turnaround
static bool swd_transfer(bool ap, bool read, uint8_t addr, uint32_t *data) {
    uint8_t request = 0x81;
    request |= (ap   ? 0x02 : 0x00);
    request |= (read ? 0x04 : 0x00);
    request |= ((addr & 0x0C) << 1);

    bool parity = 0;
    parity ^= ap;
    parity ^= read;
    parity ^= ((addr >> 2) & 1);
    parity ^= ((addr >> 3) & 1);
    request |= (parity ? 0x20 : 0x00);

    swd_dir_output();
    for (int i = 0; i < 8; i++) {
        swd_write_bit((request >> i) & 1);
    }

    swd_dir_input();
    swd_clock();                    // turnaround

    uint8_t ack = 0;
    for (int i = 0; i < 3; i++) {
        if (swd_read_bit()) ack |= (1 << i);
    }

    if (ack == SWD_ACK_WAIT) {
        for (int i = 0; i < 8; i++) swd_clock();
        swd_dir_output();
        static int s_retry = 0;
        if (s_retry < 100) {
            s_retry++;
            bool result = swd_transfer(ap, read, addr, data);
            s_retry--;
            return result;
        }
        return false;
    }

    if (ack != SWD_ACK_OK) {
        swd_dir_output();
        swd_line_reset();
        return false;
    }

    if (read) {
        uint32_t value = 0;
        for (int i = 0; i < 32; i++) {
            if (swd_read_bit()) value |= (1UL << i);
        }
        bool par = swd_read_bit();
        if (par != calc_parity(value)) {
            swd_dir_output();
            swd_clock();
            return false;
        }
        *data = value;
        swd_dir_output();
        swd_clock();
    } else {
        swd_dir_output();
        swd_clock();                // turnaround

        uint32_t value = *data;
        for (int i = 0; i < 32; i++) {
            swd_write_bit((value >> i) & 1);
        }
        swd_write_bit(calc_parity(value));
    }

    gpio_put(PIN_SWIM_SWDIO, 0);
    for (int i = 0; i < 8; i++) swd_clock();

    return true;
}

static bool swd_init(void) {
    // 先切到低速, 保证复位后能可靠通信
    uint32_t saved = g_swd_delay_cycles;
    g_swd_delay_cycles = 100;       // ≈ 750ns 半周期, ~660kHz

    gpio_put(PIN_NRST, 0);
    sleep_ms(20);
    gpio_put(PIN_NRST, 1);
    sleep_ms(50);                   // STM32 上电到 SWD 可用需要时间

    printf("[SWD] line reset\n");
    swd_line_reset();

    uint32_t idcode = 0;
    if (!swd_transfer(false, true, DP_IDCODE, &idcode)) {
        printf("[SWD] IDCODE read FAIL\n");
        g_swd_delay_cycles = saved;
        return false;
    }
    printf("[SWD] IDCODE = 0x%08lX\n", (unsigned long)idcode);

    if (idcode == 0 || idcode == 0xFFFFFFFF) {
        g_swd_delay_cycles = saved;
        return false;
    }
    g_link_dp_idcode = idcode;

    uint32_t abort = 0x1E;
    swd_transfer(false, false, DP_ABORT, &abort);

    uint32_t ctrl = 0x50000000;
    if (!swd_transfer(false, false, DP_CTRL_STAT, &ctrl)) {
        g_swd_delay_cycles = saved;
        return false;
    }
    sleep_ms(10);
    ctrl = 0x50000000;
    swd_transfer(false, false, DP_CTRL_STAT, &ctrl);

    g_swd_delay_cycles = saved;
    return true;
}

static bool swd_read_device_id(uint16_t *device_id) {
    if (!swd_init()) return false;

    uint32_t select = 0x00000000;
    if (!swd_transfer(false, false, DP_SELECT, &select)) return false;

    uint32_t csw = 0x61000002;
    if (!swd_transfer(true, false, AP_CSW, &csw)) return false;

    uint32_t tar = 0xE0042000;
    if (!swd_transfer(true, false, AP_TAR, &tar)) return false;

    uint32_t dummy = 0;
    if (!swd_transfer(true, true, AP_DRW, &dummy)) return false;

    uint32_t idcode = 0;
    if (!swd_transfer(false, true, DP_RDBUFF, &idcode)) return false;

    *device_id = idcode & 0x0FFF;
    if (*device_id == 0x0000 || *device_id == 0x0FFF) return false;
    return true;
}

static bool swd_read_memory(uint32_t addr, uint8_t *data, uint32_t len) {
    uint32_t select = 0x00000000;
    if (!swd_transfer(false, false, DP_SELECT, &select)) return false;

    uint32_t csw = 0x61000002;
    if (!swd_transfer(true, false, AP_CSW, &csw)) return false;

    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t tar = addr + i;
        if (!swd_transfer(true, false, AP_TAR, &tar)) return false;
        uint32_t dummy = 0;
        if (!swd_transfer(true, true, AP_DRW, &dummy)) return false;
        uint32_t value = 0;
        if (!swd_transfer(false, true, DP_RDBUFF, &value)) return false;
        uint32_t copy_len = (len - i) < 4 ? (len - i) : 4;
        memcpy(data + i, &value, copy_len);
    }
    return true;
}

static bool swd_write_memory(uint32_t addr, const uint8_t *data, uint32_t len) {
    uint32_t select = 0x00000000;
    if (!swd_transfer(false, false, DP_SELECT, &select)) return false;

    uint32_t csw = 0x61000002;
    if (!swd_transfer(true, false, AP_CSW, &csw)) return false;

    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t tar = addr + i;
        if (!swd_transfer(true, false, AP_TAR, &tar)) return false;
        uint32_t value = 0xFFFFFFFF;
        uint32_t copy_len = (len - i) < 4 ? (len - i) : 4;
        memcpy(&value, data + i, copy_len);
        if (!swd_transfer(true, false, AP_DRW, &value)) return false;
    }
    return true;
}

static bool stm32_flash_unlock(programmer_state_t *state) {
    uint32_t base = state->detected_chip.flash_reg_base;
    uint32_t key1 = FLASH_KEY1;
    if (!swd_write_memory(base + FLASH_KEYR_OFFSET, (uint8_t*)&key1, 4)) return false;
    uint32_t key2 = FLASH_KEY2;
    if (!swd_write_memory(base + FLASH_KEYR_OFFSET, (uint8_t*)&key2, 4)) return false;
    sleep_ms(10);

    uint32_t sr;
    if (!swd_read_memory(base + FLASH_SR_OFFSET, (uint8_t*)&sr, 4)) return false;
    if (base == STM32_F4_FLASH_BASE) {
        return (sr & 0x02) == 0;
    }
    return (sr & 0x80) == 0;
}

static bool stm32_wait_flash_ready(programmer_state_t *state, uint32_t timeout_ms) {
    uint32_t base = state->detected_chip.flash_reg_base;
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms) {
        uint32_t sr;
        if (!swd_read_memory(base + FLASH_SR_OFFSET, (uint8_t*)&sr, 4)) return false;
        if ((sr & 0x01) == 0) return true;
        sleep_ms(1);
    }
    return false;
}

static bool stm32_mass_erase(programmer_state_t *state) {
    uint32_t base = state->detected_chip.flash_reg_base;
    bool is_f4 = (base == STM32_F4_FLASH_BASE);

    if (!stm32_flash_unlock(state)) return false;
    if (!stm32_wait_flash_ready(state, 1000)) return false;

    if (is_f4) {
        uint32_t cr = 0x00000004;
        if (!swd_write_memory(base + FLASH_CR_OFFSET, (uint8_t*)&cr, 4)) return false;
        cr = 0x00010004;
        if (!swd_write_memory(base + FLASH_CR_OFFSET, (uint8_t*)&cr, 4)) return false;
    } else {
        uint32_t cr = 0x00000004;
        if (!swd_write_memory(base + FLASH_CR_OFFSET, (uint8_t*)&cr, 4)) return false;
        cr = 0x00000044;
        if (!swd_write_memory(base + FLASH_CR_OFFSET, (uint8_t*)&cr, 4)) return false;
    }

    return stm32_wait_flash_ready(state, 30000);
}

//=============================================================================
// Intel HEX解析 (不变)
//=============================================================================

static bool hex_parse_line(const char *line, hex_record_t *record) {
    if (line[0] != ':') return false;
    char buf[3] = {0};

    buf[0] = line[1]; buf[1] = line[2];
    record->count = strtoul(buf, NULL, 16);

    buf[0] = line[3]; buf[1] = line[4];
    uint8_t addr_h = strtoul(buf, NULL, 16);
    buf[0] = line[5]; buf[1] = line[6];
    uint8_t addr_l = strtoul(buf, NULL, 16);
    record->addr = (addr_h << 8) | addr_l;

    buf[0] = line[7]; buf[1] = line[8];
    record->type = strtoul(buf, NULL, 16);

    for (int i = 0; i < record->count; i++) {
        buf[0] = line[9 + i * 2];
        buf[1] = line[10 + i * 2];
        record->data[i] = strtoul(buf, NULL, 16);
    }

    uint8_t checksum = 0;
    for (int i = 1; i < 9 + record->count * 2; i += 2) {
        buf[0] = line[i];
        buf[1] = line[i + 1];
        checksum += strtoul(buf, NULL, 16);
    }
    buf[0] = line[9 + record->count * 2];
    buf[1] = line[10 + record->count * 2];
    uint8_t file_checksum = strtoul(buf, NULL, 16);

    return ((checksum + file_checksum) & 0xFF) == 0;
}

static bool hex_to_bin(const char *hex_text, uint32_t hex_size,
                      uint8_t **bin_data, uint32_t *bin_size, uint32_t *base_addr) {
    const char *p = hex_text;
    char line[600];
    hex_record_t record = {0};
    uint32_t ext_addr = 0;
    uint32_t min_addr = 0xFFFFFFFF;
    uint32_t max_addr = 0;

    uint8_t *temp_buffer = malloc(512 * 1024);
    if (!temp_buffer) return false;
    memset(temp_buffer, 0xFF, 512 * 1024);

    while (*p) {
        int i = 0;
        while (*p && *p != '\n' && *p != '\r' && i < sizeof(line) - 1) {
            line[i++] = *p++;
        }
        line[i] = '\0';
        while (*p == '\n' || *p == '\r') p++;

        if (line[0] != ':') continue;

        if (!hex_parse_line(line, &record)) {
            free(temp_buffer);
            return false;
        }

        switch (record.type) {
            case 0x00: {
                uint32_t addr = ext_addr + record.addr;
                if (addr < min_addr) min_addr = addr;
                if (addr + record.count > max_addr) max_addr = addr + record.count;
                if (addr < 512 * 1024) {
                    memcpy(temp_buffer + addr, record.data, record.count);
                }
                break;
            }
            case 0x01:
                goto parse_done;
            case 0x04:
                ext_addr = ((uint32_t)record.data[0] << 24) |
                          ((uint32_t)record.data[1] << 16);
                break;
            case 0x05:
                break;
            default:
                free(temp_buffer);
                return false;
        }
    }

parse_done:
    if (max_addr <= min_addr || max_addr - min_addr > 512 * 1024) {
        free(temp_buffer);
        return false;
    }

    *base_addr = min_addr;
    *bin_size = max_addr - min_addr;
    *bin_data = malloc(*bin_size);

    if (!*bin_data) {
        free(temp_buffer);
        return false;
    }

    memcpy(*bin_data, temp_buffer + min_addr, *bin_size);
    free(temp_buffer);
    return true;
}

//=============================================================================
// 进度页刷新 / 状态提示 (不变)
//=============================================================================

static void prog_ui_update(bool force) {
    static uint32_t last_ms = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (!force && now - last_ms < 150) return;
    last_ms = now;
    menu_draw_progress();
}

static void prog_set_message(programmer_state_t *state, const char *msg) {
    snprintf(state->prog_message, sizeof(state->prog_message), "%s", msg);
    prog_ui_update(true);
}

static void prog_fail(programmer_state_t *state, const char *msg) {
    state->prog_state = PROG_STATE_ERROR;
    if (state->error_code != ERR_CONNECTION_UNSTABLE) {
        snprintf(state->prog_message, sizeof(state->prog_message), "%s", msg);
    }
    prog_ui_update(true);
}

static void prog_detect_fail(programmer_state_t *state) {
    const char *msg = "检测失败";
    switch (state->error_code) {
        case ERR_CONNECTION_UNSTABLE: msg = "连接不稳定,请插紧"; break;
        case ERR_NO_CHIP_DETECTED:    msg = "未检测到芯片";       break;
        case ERR_UNKNOWN_CHIP:        msg = "芯片型号未收录";     break;
        default: break;
    }
    state->prog_state = PROG_STATE_ERROR;
    snprintf(state->prog_message, sizeof(state->prog_message), "%s", msg);
    prog_ui_update(true);
}

//=============================================================================
// 芯片检测 + 连接稳定性保护
// STM8 优先 (适合刷 STM8)
//=============================================================================

bool programmer_detect_chip(chip_type_t *type, uint16_t *device_id) {
    const interface_mode_t mode = g_state.config.interface_mode;

    // STM8 (SWIM) 优先
    if (mode != INTERFACE_SWD) {
        printf("[DET] try SWIM...\n");
        if (swim_read_device_id(device_id)) {
            *type = CHIP_TYPE_STM8;
            g_link_type = *type;
            g_link_dev_id = *device_id;
            printf("[DET] SWIM OK, id=0x%04X\n", *device_id);
            return true;
        }
    }
    if (mode != INTERFACE_SWIM) {
        printf("[DET] try SWD...\n");
        if (swd_read_device_id(device_id)) {
            *type = CHIP_TYPE_STM32;
            g_link_type = *type;
            g_link_dev_id = *device_id;
            printf("[DET] SWD OK, id=0x%04X\n", *device_id);
            return true;
        }
    }
    g_link_type = CHIP_TYPE_UNKNOWN;
    printf("[DET] not detected\n");
    return false;
}

bool programmer_link_alive(programmer_state_t *state) {
    (void)state;
    if (g_link_type == CHIP_TYPE_STM8) {
        uint8_t id[2];
        if (!swim_read_memory(0x004850, id, 2)) return false;
        uint16_t v = (uint16_t)((id[0] << 8) | id[1]);
        return v != 0x0000 && v != 0xFFFF && v == g_link_dev_id;
    }
    if (g_link_type == CHIP_TYPE_STM32) {
        uint32_t idcode = 0;
        if (!swd_transfer(false, true, DP_IDCODE, &idcode)) return false;
        return idcode == g_link_dp_idcode;
    }
    return false;
}

static bool link_guard(programmer_state_t *state, const char *stage) {
    if (programmer_link_alive(state)) return true;
    state->error_code = ERR_CONNECTION_UNSTABLE;
    state->prog_state = PROG_STATE_ERROR;
    snprintf(state->prog_message, sizeof(state->prog_message), "连接中断:%s", stage);
    prog_ui_update(true);
    return false;
}

static int programmer_stability_check(chip_type_t *detected_type, uint16_t *device_id) {
    chip_type_t first_type = CHIP_TYPE_UNKNOWN;
    uint16_t first_id = 0;

    for (int i = 0; i < STABILITY_CHECK_TIMES; i++) {
        chip_type_t t;
        uint16_t id;
        bool ok = programmer_detect_chip(&t, &id);

        if (i == 0) {
            if (!ok) return ERR_NO_CHIP_DETECTED;
            first_type = t;
            first_id = id;
        } else if (!ok || t != first_type || id != first_id) {
            return ERR_CONNECTION_UNSTABLE;
        }
        if (i + 1 < STABILITY_CHECK_TIMES) sleep_ms(STABILITY_CHECK_GAP_MS);
    }

    *detected_type = first_type;
    *device_id = first_id;
    return ERR_OK;
}

bool programmer_auto_detect(programmer_state_t *state) {
    chip_type_t type;
    uint16_t device_id;

    if (state->config.stability_check_enabled) {
        int err = programmer_stability_check(&type, &device_id);
        if (err != ERR_OK) {
            state->error_code = err;
            return false;
        }
    } else {
        if (!programmer_detect_chip(&type, &device_id)) {
            state->error_code = ERR_NO_CHIP_DETECTED;
            return false;
        }
    }

    chip_info_t *found = chip_db_find_by_id(device_id, type);

    if (!found && type == CHIP_TYPE_STM8 &&
        state->selected_chip.valid && state->selected_chip.type == CHIP_TYPE_STM8) {
        found = &state->selected_chip;
    }

    if (found) {
        chip_info_t copy = *found;
        state->detected_chip = copy;
        state->chip_detected = true;
        state->error_code = ERR_OK;
        return true;
    }

    if (state->config.allow_unknown_chip) {
        snprintf(state->detected_chip.name, sizeof(state->detected_chip.name),
                "UNK_%04X", device_id);
        state->detected_chip.device_id = device_id;
        state->detected_chip.type = type;
        state->detected_chip.flash_addr = (type == CHIP_TYPE_STM32) ? 0x08000000 : 0x8000;
        state->detected_chip.flash_size = 64 * 1024;
        state->detected_chip.page_size = 1024;
        state->detected_chip.flash_reg_base = STM32_F1_FLASH_BASE;
        state->detected_chip.valid = true;
        state->chip_detected = true;
        state->error_code = ERR_OK;
        return true;
    }

    state->error_code = ERR_UNKNOWN_CHIP;
    return false;
}

bool programmer_erase_chip(programmer_state_t *state) {
    if (!link_guard(state, "擦除前")) return false;

    bool ok = false;
    if (state->detected_chip.type == CHIP_TYPE_STM8) {
        ok = swim_erase_chip();
    } else if (state->detected_chip.type == CHIP_TYPE_STM32) {
        ok = stm32_mass_erase(state);
    }
    if (!ok) return false;

    return link_guard(state, "擦除后");
}

#define LINK_CHECK_BLOCK 256

static void prog_update_speed(programmer_state_t *state, uint32_t offset, uint32_t total_size) {
    state->prog_progress = offset;
    state->prog_total = total_size;
    uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - state->start_time;
    if (elapsed > 0) state->prog_speed = (uint32_t)((uint64_t)offset * 1000 / elapsed);
    prog_ui_update(false);
}

static bool programmer_write_flash_stream(programmer_state_t *state,
                                         const uint8_t *data, uint32_t addr,
                                         uint32_t total_size) {
    uint32_t offset = 0;
    const bool is_stm8 = (state->detected_chip.type == CHIP_TYPE_STM8);
    const uint32_t block_size = is_stm8 ? STM8_WRITE_BLOCK : STM32_WRITE_BLOCK;

    state->start_time = to_ms_since_boot(get_absolute_time());

    if (!link_guard(state, "写入前")) return false;

    if (is_stm8) {
        if (!swim_unlock_flash()) {
            printf("[WRITE] unlock FAIL\n");
            state->error_code = ERR_WRITE_FAILED;
            return false;
        }
    } else {
        if (!stm32_flash_unlock(state)) {
            state->error_code = ERR_WRITE_FAILED;
            return false;
        }
    }

    while (offset < total_size) {
        uint32_t write_size = (total_size - offset) > block_size ? block_size : (total_size - offset);

        bool success = is_stm8 ? swim_write_memory(addr + offset, data + offset, write_size)
                               : swd_write_memory(addr + offset, data + offset, write_size);
        if (!success) {
            state->error_code = ERR_WRITE_FAILED;
            printf("[WRITE] offset 0x%lX FAIL\n", (unsigned long)offset);
            return false;
        }

        if (!link_guard(state, "写入")) return false;

        offset += write_size;
        prog_update_speed(state, offset, total_size);
    }
    return true;
}

static bool programmer_read_flash_stream(programmer_state_t *state,
                                        uint8_t *data, uint32_t addr,
                                        uint32_t total_size) {
    uint32_t offset = 0;
    const bool is_stm8 = (state->detected_chip.type == CHIP_TYPE_STM8);

    state->start_time = to_ms_since_boot(get_absolute_time());

    if (!link_guard(state, "读取前")) return false;

    while (offset < total_size) {
        uint32_t read_size = (total_size - offset) > LINK_CHECK_BLOCK ? LINK_CHECK_BLOCK : (total_size - offset);

        bool success = is_stm8 ? swim_read_memory(addr + offset, data + offset, read_size)
                               : swd_read_memory(addr + offset, data + offset, read_size);
        if (!success) {
            state->error_code = ERR_READ_FAILED;
            return false;
        }

        if (!link_guard(state, "读取")) return false;

        offset += read_size;
        prog_update_speed(state, offset, total_size);
    }
    return true;
}

static bool programmer_verify_flash(programmer_state_t *state,
                                   const uint8_t *data, uint32_t addr,
                                   uint32_t total_size) {
    uint32_t offset = 0;
    const bool is_stm8 = (state->detected_chip.type == CHIP_TYPE_STM8);

    state->start_time = to_ms_since_boot(get_absolute_time());

    if (!link_guard(state, "校验前")) return false;

    while (offset < total_size) {
        uint32_t verify_size = (total_size - offset) > LINK_CHECK_BLOCK ? LINK_CHECK_BLOCK : (total_size - offset);

        bool success = is_stm8 ? swim_read_memory(addr + offset, g_stream_buffer, verify_size)
                               : swd_read_memory(addr + offset, g_stream_buffer, verify_size);
        if (!success) {
            state->error_code = ERR_READ_FAILED;
            return false;
        }

        if (!link_guard(state, "校验")) return false;

        if (memcmp(data + offset, g_stream_buffer, verify_size) != 0) {
            state->error_code = ERR_VERIFY_FAILED;
            return false;
        }

        offset += verify_size;
        prog_update_speed(state, offset, total_size);
    }
    return true;
}

//=============================================================================
// 文件系统 (FatFs)
//=============================================================================

static FATFS g_fatfs;
static FIL   g_fil;

bool fs_init(void) {
    FRESULT fr = f_mount(&g_fatfs, "", 1);
    if (fr == FR_NO_FILESYSTEM) {
        static BYTE work_buf[4096];
        MKFS_PARM parm;
        memset(&parm, 0, sizeof(parm));
        parm.fmt = FM_FAT;
        fr = f_mkfs("", &parm, work_buf, sizeof(work_buf));
        if (fr != FR_OK) return false;
        fr = f_mount(&g_fatfs, "", 1);
    }
    return (fr == FR_OK);
}

bool fs_list_files(file_info_t *files, int *count, int max_count) {
    *count = 0;
    DIR dir;
    FILINFO info;
    if (f_opendir(&dir, "") != FR_OK) return false;

    while (f_readdir(&dir, &info) == FR_OK && info.fname[0] && *count < max_count) {
        if (!(info.fattrib & AM_DIR)) {
            strncpy(files[*count].filename, info.fname, MAX_FILENAME_LEN - 1);
            files[*count].filename[MAX_FILENAME_LEN - 1] = 0;
            files[*count].size = info.fsize;
            const char *ext = strrchr(info.fname, '.');
            if (ext) {
                if (strcasecmp(ext, ".hex") == 0) files[*count].type = FILE_TYPE_HEX;
                else if (strcasecmp(ext, ".bin") == 0) files[*count].type = FILE_TYPE_BIN;
                else files[*count].type = FILE_TYPE_BIN;
            } else {
                files[*count].type = FILE_TYPE_BIN;
            }
            files[*count].valid = true;
            (*count)++;
        }
    }
    f_closedir(&dir);
    return true;
}

bool fs_read_file(const char *filename, uint8_t **buffer, uint32_t *size) {
    if (f_open(&g_fil, filename, FA_READ) != FR_OK) return false;

    *size = f_size(&g_fil);
    *buffer = malloc(*size);
    if (!*buffer) { f_close(&g_fil); return false; }

    UINT br;
    FRESULT fr = f_read(&g_fil, *buffer, *size, &br);
    f_close(&g_fil);
    return (fr == FR_OK && br == *size);
}

bool fs_write_file(const char *filename, const uint8_t *data,
                   uint32_t size, file_type_t type) {
    (void)type;
    if (f_open(&g_fil, filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return false;
    UINT bw;
    FRESULT fr = f_write(&g_fil, data, size, &bw);
    f_close(&g_fil);
    return (fr == FR_OK && bw == size);
}

bool fs_delete_file(const char *filename) {
    return (f_unlink(filename) == FR_OK);
}

//=============================================================================
// 完整烧录流程
//=============================================================================

bool programmer_full_process(programmer_state_t *state, const char *firmware_file) {
    state->error_code = ERR_OK;
    state->prog_progress = 0;
    state->prog_total = 100;

    state->prog_state = PROG_STATE_CONNECTING;
    prog_set_message(state, "连接中...");

    state->prog_state = PROG_STATE_DETECTING;
    prog_set_message(state, "检测中...");

    if (!programmer_auto_detect(state)) {
        prog_detect_fail(state);
        return false;
    }

    {
        char found_msg[48];
        snprintf(found_msg, sizeof(found_msg), "检测到:%s", state->detected_chip.name);
        prog_set_message(state, found_msg);
    }
    sleep_ms(500);

    state->prog_state = PROG_STATE_LOADING;
    prog_set_message(state, "加载固件...");

    uint8_t *file_buffer = NULL;
    uint32_t file_size = 0;

    if (!fs_read_file(firmware_file, &file_buffer, &file_size)) {
        state->error_code = ERR_FILE_NOT_FOUND;
        prog_fail(state, "文件未找到");
        return false;
    }

    uint8_t *bin_data = NULL;
    uint32_t bin_size = 0;
    uint32_t base_addr = 0;

    if (strstr(firmware_file, ".HEX") || strstr(firmware_file, ".hex")) {
        prog_set_message(state, "解析HEX...");

        if (!hex_to_bin((char *)file_buffer, file_size, &bin_data, &bin_size, &base_addr)) {
            free(file_buffer);
            state->error_code = ERR_PARSE_HEX;
            prog_fail(state, "HEX解析失败");
            return false;
        }

        free(file_buffer);
        file_buffer = bin_data;
        file_size = bin_size;
    } else {
        base_addr = state->detected_chip.flash_addr;
    }

    if (state->config.backup_before_program) {
        state->prog_state = PROG_STATE_BACKING_UP;
        prog_set_message(state, "备份中...");

        uint8_t *backup_buffer = malloc(state->detected_chip.flash_size);
        if (backup_buffer) {
            if (programmer_read_flash_stream(state, backup_buffer,
                                           state->detected_chip.flash_addr,
                                           state->detected_chip.flash_size)) {
                char backup_name[MAX_FILENAME_LEN];
                snprintf(backup_name, sizeof(backup_name), "BAK_%s_%lu.BIN",
                        state->detected_chip.name,
                        to_ms_since_boot(get_absolute_time()));
                fs_write_file(backup_name, backup_buffer,
                            state->detected_chip.flash_size, FILE_TYPE_BIN);
            }
            free(backup_buffer);
        }
    }

    if (state->config.auto_erase) {
        state->prog_state = PROG_STATE_ERASING;
        prog_set_message(state, "擦除中...");

        if (!programmer_erase_chip(state)) {
            free(file_buffer);
            if (state->error_code != ERR_CONNECTION_UNSTABLE) state->error_code = ERR_ERASE_FAILED;
            prog_fail(state, "擦除失败");
            return false;
        }
    }

    state->prog_state = PROG_STATE_WRITING;
    prog_set_message(state, "烧录中...");
    state->prog_progress = 0;
    state->prog_total = file_size;

    if (!programmer_write_flash_stream(state, file_buffer, base_addr, file_size)) {
        free(file_buffer);
        prog_fail(state, "烧录失败");
        return false;
    }

    if (state->config.verify_after_program) {
        state->prog_state = PROG_STATE_VERIFYING;
        prog_set_message(state, "校验中...");
        state->prog_progress = 0;

        if (!programmer_verify_flash(state, file_buffer, base_addr, file_size)) {
            free(file_buffer);
            prog_fail(state, "校验失败");
            return false;
        }
    }

    free(file_buffer);

    state->prog_state = PROG_STATE_SUCCESS;
    state->prog_progress = state->prog_total;
    prog_set_message(state, "烧录成功!");

    return true;
}

//=============================================================================
// 完整菜单系统
//=============================================================================

static const char *main_menu_items[] = {
    "开始烧录",
    "读取备份",
    "擦除芯片",
    "固件管理",
    "选择芯片",
    "系统设置"
};
#define MAIN_MENU_COUNT 6

static const char *read_menu_items[] = {
    "读取全部",
    "读取范围",
    "备份列表",
    "芯片信息"
};
#define READ_MENU_COUNT 4

static const char *firmware_menu_items[] = {
    "固件列表",
    "删除文件",
    "文件信息"
};
#define FIRMWARE_MENU_COUNT 3

static const char *settings_menu_items[] = {
    "烧录模式",
    "接口模式",
    "稳定检测",
    "允许未知",
    "自动校验",
    "自动备份",
    "自动擦除",
    "SWD速度",
    "屏幕亮度",
    "恢复默认"
};
#define SETTINGS_MENU_COUNT 10

static const char *program_mode_str[] = {"自动", "手动"};
static const char *interface_mode_str[] = {"自动", "SWIM", "SWD"};
static const char *swd_speed_str[] = {"低速", "中速", "高速"};
static const char *bool_str[] = {"关闭", "开启"};

static void menu_draw_scrollbar(int count, int scroll) {
    if (count <= MENU_ITEMS_PER_PAGE) return;
    int area = OLED_HEIGHT - MENU_START_Y;
    int bar_height = area * MENU_ITEMS_PER_PAGE / count;
    if (bar_height < 4) bar_height = 4;
    int bar_pos = (area - bar_height) * scroll / (count - MENU_ITEMS_PER_PAGE);
    oled_draw_rect(OLED_WIDTH - 2, MENU_START_Y + bar_pos, 2, bar_height, true);
}

static void menu_draw_header(const char *title) {
    oled_show_string(5, 0, title);
    oled_draw_hline(0, 13, OLED_WIDTH);
}

static void menu_draw_items(const char **items, int count, int selected, int scroll) {
    int y = MENU_START_Y;
    for (int i = 0; i < MENU_ITEMS_PER_PAGE && (scroll + i) < count; i++) {
        int index = scroll + i;
        if (index == selected) oled_show_string(0, y, ">");
        oled_show_string_clip(10, y, OLED_WIDTH - 14, items[index]);
        y += MENU_LINE_HEIGHT;
    }
    menu_draw_scrollbar(count, scroll);
}

static void menu_draw_main(void) {
    oled_clear();
    menu_draw_header("STM烧录器");
    menu_draw_items(main_menu_items, MAIN_MENU_COUNT, g_menu.selected_item, g_menu.scroll_offset);
    if (g_state.chip_connected) oled_show_string(OLED_WIDTH - 40, 0, "已连接");
    oled_refresh();
}

static void menu_draw_read(void) {
    oled_clear();
    menu_draw_header("读取备份");
    menu_draw_items(read_menu_items, READ_MENU_COUNT, g_menu.selected_item, g_menu.scroll_offset);
    oled_refresh();
}

static void menu_draw_firmware(void) {
    oled_clear();
    menu_draw_header("固件管理");
    menu_draw_items(firmware_menu_items, FIRMWARE_MENU_COUNT, g_menu.selected_item, g_menu.scroll_offset);
    oled_refresh();
}

static void menu_draw_settings(void) {
    oled_clear();
    menu_draw_header("系统设置");

    int y = MENU_START_Y;
    for (int i = 0; i < MENU_ITEMS_PER_PAGE && (g_menu.scroll_offset + i) < SETTINGS_MENU_COUNT; i++) {
        int index = g_menu.scroll_offset + i;

        if (index == g_menu.selected_item) oled_show_string(0, y, ">");
        oled_show_string(10, y, settings_menu_items[index]);

        char bright_str[8];
        const char *value_str = "";
        switch (index) {
            case 0: value_str = program_mode_str[g_state.config.program_mode]; break;
            case 1: value_str = interface_mode_str[g_state.config.interface_mode]; break;
            case 2: value_str = bool_str[g_state.config.stability_check_enabled]; break;
            case 3: value_str = bool_str[g_state.config.allow_unknown_chip]; break;
            case 4: value_str = bool_str[g_state.config.verify_after_program]; break;
            case 5: value_str = bool_str[g_state.config.backup_before_program]; break;
            case 6: value_str = bool_str[g_state.config.auto_erase]; break;
            case 7: value_str = swd_speed_str[g_state.config.swd_speed]; break;
            case 8:
                snprintf(bright_str, sizeof(bright_str), "%d", g_state.config.brightness);
                value_str = bright_str;
                break;
        }
        oled_show_string(OLED_WIDTH - 6 - oled_text_width(value_str), y, value_str);
        y += MENU_LINE_HEIGHT;
    }

    menu_draw_scrollbar(SETTINGS_MENU_COUNT, g_menu.scroll_offset);
    oled_refresh();
}

static void menu_draw_chip_select(void) {
    oled_clear();
    menu_draw_header("选择芯片");

    int y = MENU_START_Y;
    for (int i = 0; i < MENU_ITEMS_PER_PAGE && (g_menu.scroll_offset + i) < g_chip_db_count; i++) {
        int index = g_menu.scroll_offset + i;
        if (index == g_menu.selected_item) oled_show_string(0, y, ">");
        oled_show_string_clip(10, y, OLED_WIDTH - 24, g_chip_db[index].name);
        if (strcmp(g_chip_db[index].name, g_state.selected_chip.name) == 0) {
            oled_show_string(OLED_WIDTH - 12, y, "*");
        }
        y += MENU_LINE_HEIGHT;
    }

    menu_draw_scrollbar(g_chip_db_count, g_menu.scroll_offset);
    oled_refresh();
}

static void menu_draw_firmware_select(void) {
    oled_clear();
    menu_draw_header("选择固件");

    int y = MENU_START_Y;
    for (int i = 0; i < MENU_ITEMS_PER_PAGE && (g_menu.scroll_offset + i) < g_state.file_count; i++) {
        int index = g_menu.scroll_offset + i;
        if (index == g_menu.selected_item) oled_show_string(0, y, ">");
        oled_show_string_clip(10, y, OLED_WIDTH - 14, g_state.file_list[index].filename);
        y += MENU_LINE_HEIGHT;
    }

    menu_draw_scrollbar(g_state.file_count, g_menu.scroll_offset);
    oled_refresh();
}

static void menu_draw_progress(void) {
    oled_clear();

    oled_show_string_clip(0, 0, 80, g_state.detected_chip.name);
    if (g_state.error_code != 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "错误:%d", g_state.error_code);
        oled_show_string(OLED_WIDTH - oled_text_width(buf), 0, buf);
    }

    oled_show_string_clip(0, 13, OLED_WIDTH, g_state.prog_message);

    const bool finished = (g_state.prog_state == PROG_STATE_SUCCESS ||
                           g_state.prog_state == PROG_STATE_ERROR);

    if (g_state.prog_total > 0) {
        oled_show_progress(5, 27, OLED_WIDTH - 10, 10, g_state.prog_progress, g_state.prog_total);
        oled_show_percent(OLED_WIDTH / 2 - 12, 38, g_state.prog_progress, g_state.prog_total);
    }

    if (finished) {
        oled_show_string_center(51, "按任意键返回");
    } else if (g_state.prog_total > 0) {
        oled_show_speed(5, 52, g_state.prog_speed);
        char buf[32];
        snprintf(buf, sizeof(buf), "%lu/%lu",
                 (unsigned long)g_state.prog_progress, (unsigned long)g_state.prog_total);
        oled_show_string(OLED_WIDTH - 5 - oled_text_width(buf), 52, buf);
    }

    oled_refresh();
}

static void menu_draw_confirm(const char *message) {
    oled_clear();
    oled_show_string_center(12, message);

    oled_show_string(14, 40, g_menu.selected_item == 0 ? ">" : " ");
    oled_show_string(24, 40, "确认");
    oled_show_string(68, 40, g_menu.selected_item == 1 ? ">" : " ");
    oled_show_string(78, 40, "取消");

    oled_refresh();
}

static void menu_draw_info(void) {
    oled_clear();
    menu_draw_header(g_menu.info_title);

    int y = MENU_START_Y;
    const char *line = g_menu.info_message;
    while (*line && y < OLED_HEIGHT - 12) {
        char buf[64];
        int i = 0;
        while (*line && *line != '\n' && i < (int)sizeof(buf) - 1) buf[i++] = *line++;
        buf[i] = 0;
        if (*line == '\n') line++;
        oled_show_string_clip(5, y, OLED_WIDTH - 5, buf);
        y += MENU_LINE_HEIGHT;
    }

    oled_show_string_center(OLED_HEIGHT - 12, "确认");
    oled_refresh();
}

//=============================================================================
// 菜单导航
//=============================================================================

static void menu_navigate(int item_count, key_event_t key) {
    if (item_count <= 0) return;
    if (key != KEY_UP && key != KEY_DOWN) return;

    if (key == KEY_UP) {
        g_menu.selected_item = (g_menu.selected_item > 0) ? g_menu.selected_item - 1 : item_count - 1;
    } else {
        g_menu.selected_item = (g_menu.selected_item < item_count - 1) ? g_menu.selected_item + 1 : 0;
    }

    if (g_menu.selected_item < g_menu.scroll_offset) {
        g_menu.scroll_offset = g_menu.selected_item;
    } else if (g_menu.selected_item >= g_menu.scroll_offset + MENU_ITEMS_PER_PAGE) {
        g_menu.scroll_offset = g_menu.selected_item - MENU_ITEMS_PER_PAGE + 1;
    }
    if (item_count <= MENU_ITEMS_PER_PAGE) g_menu.scroll_offset = 0;

    g_menu.need_refresh = true;
}

static void menu_goto(menu_state_t *menu, menu_id_t id) {
    menu->previous_menu = menu->current_menu;
    menu->current_menu = id;
    menu->selected_item = 0;
    menu->scroll_offset = 0;
    menu->need_refresh = true;
}

static void menu_show_info(menu_state_t *menu, const char *title, const char *message) {
    snprintf(menu->info_title, sizeof(menu->info_title), "%s", title);
    snprintf(menu->info_message, sizeof(menu->info_message), "%s", message);
    menu_goto(menu, MENU_INFO);
}

static void menu_start_job(menu_state_t *menu, programmer_state_t *state, int mode, int file_index) {
    menu->job_mode = mode;
    menu->job_file_index = file_index;
    menu_goto(menu, MENU_PROGRAM_PROGRESS);
    state->prog_state = PROG_STATE_IDLE;
    state->error_code = ERR_OK;
    state->prog_progress = 0;
    state->prog_total = 0;
    state->prog_speed = 0;
    state->prog_message[0] = 0;
}

void menu_system_init(void) {
    memset(&g_menu, 0, sizeof(g_menu));
    g_menu.current_menu = MENU_MAIN;
    g_menu.previous_menu = MENU_MAIN;
    g_menu.need_refresh = true;
}

static void menu_render(menu_state_t *menu) {
    if (!menu->need_refresh) return;
    menu->need_refresh = false;

    switch (menu->current_menu) {
        case MENU_MAIN:             menu_draw_main(); break;
        case MENU_READ_BACKUP:      menu_draw_read(); break;
        case MENU_CHIP_ERASE:
            oled_clear();
            menu_draw_header("擦除芯片");
            oled_show_string_center(30, "按确认键执行");
            oled_refresh();
            break;
        case MENU_FIRMWARE_MGMT:    menu_draw_firmware(); break;
        case MENU_SETTINGS:         menu_draw_settings(); break;
        case MENU_CHIP_SELECT:      menu_draw_chip_select(); break;
        case MENU_FIRMWARE_SELECT:  menu_draw_firmware_select(); break;
        case MENU_PROGRAM_PROGRESS: menu_draw_progress(); break;
        case MENU_CONFIRM:          menu_draw_confirm("确认擦除芯片?"); break;
        case MENU_INFO:             menu_draw_info(); break;
        default: break;
    }
}

void menu_system_process(menu_state_t *menu, key_event_t key, programmer_state_t *state) {
    switch (menu->current_menu) {
        case MENU_MAIN:
            menu_navigate(MAIN_MENU_COUNT, key);
            if (key == KEY_OK) {
                switch (menu->selected_item) {
                    case 0:
                        if (state->file_count > 0) {
                            menu_goto(menu, MENU_FIRMWARE_SELECT);
                        } else {
                            menu_show_info(menu, "提示", "没有可用固件\n请先添加固件文件");
                        }
                        break;
                    case 1: menu_goto(menu, MENU_READ_BACKUP); break;
                    case 2: menu_goto(menu, MENU_CHIP_ERASE); break;
                    case 3: menu_goto(menu, MENU_FIRMWARE_MGMT); break;
                    case 4: {
                        int cur = chip_db_find_index_by_name(state->selected_chip.name);
                        menu_goto(menu, MENU_CHIP_SELECT);
                        if (cur > 0) {
                            menu->selected_item = cur;
                            if (cur >= MENU_ITEMS_PER_PAGE) menu->scroll_offset = cur - MENU_ITEMS_PER_PAGE + 1;
                        }
                        break;
                    }
                    case 5: menu_goto(menu, MENU_SETTINGS); break;
                }
            }
            break;

        case MENU_READ_BACKUP:
            menu_navigate(READ_MENU_COUNT, key);
            if (key == KEY_BACK) {
                menu_goto(menu, MENU_MAIN);
            } else if (key == KEY_OK) {
                switch (menu->selected_item) {
                    case 0: menu_start_job(menu, state, 1, 0); break;
                    case 1: menu_show_info(menu, "提示", "该功能暂未实现"); break;
                    case 2: {
                        char msg[64];
                        snprintf(msg, sizeof(msg), "共有 %d 个文件", state->file_count);
                        menu_show_info(menu, "备份列表", msg);
                        break;
                    }
                    case 3:
                        if (programmer_auto_detect(state)) {
                            char msg[96];
                            snprintf(msg, sizeof(msg), "%s\nFlash:%luK\nRAM:%luK",
                                     state->detected_chip.name,
                                     (unsigned long)(state->detected_chip.flash_size / 1024),
                                     (unsigned long)(state->detected_chip.ram_size / 1024));
                            menu_show_info(menu, "芯片信息", msg);
                        } else {
                            const char *why = "未检测到芯片";
                            if (state->error_code == ERR_CONNECTION_UNSTABLE) why = "连接不稳定\n请插紧排针";
                            else if (state->error_code == ERR_UNKNOWN_CHIP)   why = "芯片型号未收录\n请加入数据库";
                            menu_show_info(menu, "错误", why);
                        }
                        break;
                }
            }
            break;

        case MENU_CHIP_ERASE:
            if (key == KEY_BACK) {
                menu_goto(menu, MENU_MAIN);
            } else if (key == KEY_OK) {
                menu_goto(menu, MENU_CONFIRM);
            }
            break;

        case MENU_FIRMWARE_MGMT:
            menu_navigate(FIRMWARE_MENU_COUNT, key);
            if (key == KEY_BACK) {
                menu_goto(menu, MENU_MAIN);
            } else if (key == KEY_OK) {
                switch (menu->selected_item) {
                    case 0:
                        if (state->file_count > 0) menu_goto(menu, MENU_FIRMWARE_SELECT);
                        else menu_show_info(menu, "提示", "没有固件文件");
                        break;
                    case 1: menu_show_info(menu, "提示", "该功能暂未实现"); break;
                    case 2: {
                        char msg[64];
                        snprintf(msg, sizeof(msg), "共有 %d 个文件", state->file_count);
                        menu_show_info(menu, "固件文件", msg);
                        break;
                    }
                }
            }
            break;

        case MENU_SETTINGS:
            menu_navigate(SETTINGS_MENU_COUNT, key);
            if (key == KEY_BACK) {
                menu_goto(menu, MENU_MAIN);
            } else if (key == KEY_OK) {
                switch (menu->selected_item) {
                    case 0: state->config.program_mode = (state->config.program_mode + 1) % 2; break;
                    case 1: state->config.interface_mode = (state->config.interface_mode + 1) % 3; break;
                    case 2: state->config.stability_check_enabled = !state->config.stability_check_enabled; break;
                    case 3: state->config.allow_unknown_chip = !state->config.allow_unknown_chip; break;
                    case 4: state->config.verify_after_program = !state->config.verify_after_program; break;
                    case 5: state->config.backup_before_program = !state->config.backup_before_program; break;
                    case 6: state->config.auto_erase = !state->config.auto_erase; break;
                    case 7:
                        state->config.swd_speed = (swd_speed_t)((state->config.swd_speed + 1) % 3);
                        swd_set_speed(state->config.swd_speed);
                        break;
                    case 8:
                        state->config.brightness += 50;
                        if (state->config.brightness > 250) state->config.brightness = 50;
                        oled_set_brightness(state->config.brightness);
                        break;
                    case 9:
                        config_set_default(&state->config);
                        oled_set_brightness(state->config.brightness);
                        swd_set_speed(state->config.swd_speed);
                        config_save(&state->config);
                        menu_show_info(menu, "提示", "已恢复默认设置");
                        goto settings_done;
                }
                config_save(&state->config);
                menu->need_refresh = true;
            }
            settings_done:
            break;

        case MENU_CHIP_SELECT:
            menu_navigate(g_chip_db_count, key);
            if (key == KEY_BACK) {
                menu_goto(menu, MENU_MAIN);
            } else if (key == KEY_OK && g_chip_db_count > 0) {
                int sel = menu->selected_item;
                state->selected_chip = g_chip_db[sel];
                state->config.default_chip_index = sel;
                config_save(&state->config);
                char msg[64];
                snprintf(msg, sizeof(msg), "已选择:\n%s", g_chip_db[sel].name);
                menu_show_info(menu, "选择芯片", msg);
                menu->previous_menu = MENU_MAIN;
            }
            break;

        case MENU_FIRMWARE_SELECT:
            menu_navigate(state->file_count, key);
            if (key == KEY_BACK) {
                menu_id_t back = menu->previous_menu;
                menu_goto(menu, back);
            } else if (key == KEY_OK && state->file_count > 0) {
                int sel = menu->selected_item;
                if (menu->previous_menu == MENU_MAIN) {
                    menu_start_job(menu, state, 0, sel);
                } else {
                    char msg[96];
                    snprintf(msg, sizeof(msg), "%s\n大小:%lu字节",
                             state->file_list[sel].filename,
                             (unsigned long)state->file_list[sel].size);
                    menu_show_info(menu, "文件信息", msg);
                    menu->previous_menu = MENU_FIRMWARE_MGMT;
                }
            }
            break;

        case MENU_PROGRAM_PROGRESS:
            if (key != KEY_NONE &&
                (state->prog_state == PROG_STATE_SUCCESS || state->prog_state == PROG_STATE_ERROR)) {
                state->prog_state = PROG_STATE_IDLE;
                state->error_code = ERR_OK;
                menu_goto(menu, MENU_MAIN);
            }
            break;

        case MENU_CONFIRM:
            if (key == KEY_UP || key == KEY_DOWN) {
                menu->selected_item = !menu->selected_item;
                menu->need_refresh = true;
            } else if (key == KEY_OK) {
                if (menu->selected_item == 0) {
                    menu_start_job(menu, state, 2, 0);
                } else {
                    menu_goto(menu, MENU_CHIP_ERASE);
                }
            } else if (key == KEY_BACK) {
                menu_goto(menu, MENU_CHIP_ERASE);
            }
            break;

        case MENU_INFO:
            if (key == KEY_OK || key == KEY_BACK) {
                menu_id_t back = menu->previous_menu;
                menu_goto(menu, back);
            }
            break;

        default:
            break;
    }

    menu_render(menu);
}

//=============================================================================
// 任务执行
//=============================================================================
void programmer_task_run(menu_state_t *menu, programmer_state_t *state) {
    if (menu->current_menu != MENU_PROGRAM_PROGRESS || state->prog_state != PROG_STATE_IDLE) return;

    state->error_code = ERR_OK;
    state->prog_progress = 0;
    state->prog_total = 0;
    state->prog_speed = 0;

    switch (menu->job_mode) {
        case 0: {
            int idx = menu->job_file_index;
            if (idx < 0 || idx >= state->file_count) {
                state->error_code = ERR_FILE_NOT_FOUND;
                prog_fail(state, "文件未找到");
                break;
            }
            programmer_full_process(state, state->file_list[idx].filename);
            break;
        }

        case 1: {
            state->prog_state = PROG_STATE_DETECTING;
            prog_set_message(state, "检测中...");
            if (!programmer_auto_detect(state)) {
                prog_detect_fail(state);
                break;
            }

            const uint32_t size = state->detected_chip.flash_size;
            if (state->read_buffer && state->read_buffer_size != size) {
                free(state->read_buffer);
                state->read_buffer = NULL;
            }
            if (!state->read_buffer) {
                state->read_buffer = malloc(size);
                state->read_buffer_size = state->read_buffer ? size : 0;
            }
            if (!state->read_buffer) {
                state->error_code = ERR_FS_ERROR;
                prog_fail(state, "内存不足");
                break;
            }

            state->prog_state = PROG_STATE_READING;
            prog_set_message(state, "读取中...");
            if (!programmer_read_flash_stream(state, state->read_buffer,
                                              state->detected_chip.flash_addr, size)) {
                prog_fail(state, "读取失败");
                break;
            }

            char filename[MAX_FILENAME_LEN];
            snprintf(filename, sizeof(filename), "READ_%s_%lu.BIN",
                     state->detected_chip.name, (unsigned long)to_ms_since_boot(get_absolute_time()));
            if (!fs_write_file(filename, state->read_buffer, size, FILE_TYPE_BIN)) {
                state->error_code = ERR_FS_ERROR;
                prog_fail(state, "保存失败");
                break;
            }
            fs_list_files(state->file_list, &state->file_count, MAX_FILES);

            state->prog_state = PROG_STATE_SUCCESS;
            state->prog_progress = state->prog_total;
            prog_set_message(state, "读取成功!");
            break;
        }

        case 2: {
            state->prog_state = PROG_STATE_DETECTING;
            prog_set_message(state, "检测中...");
            if (!programmer_auto_detect(state)) {
                prog_detect_fail(state);
                break;
            }

            state->prog_state = PROG_STATE_ERASING;
            prog_set_message(state, "擦除中...");
            if (!programmer_erase_chip(state)) {
                if (state->error_code != ERR_CONNECTION_UNSTABLE) state->error_code = ERR_ERASE_FAILED;
                prog_fail(state, "擦除失败");
                break;
            }
            state->prog_state = PROG_STATE_SUCCESS;
            prog_set_message(state, "擦除成功!");
            break;
        }

        default:
            state->error_code = ERR_INVALID_PARAM;
            prog_fail(state, "未知任务");
            break;
    }

    if (state->prog_state != PROG_STATE_SUCCESS && state->prog_state != PROG_STATE_ERROR) {
        state->prog_state = PROG_STATE_ERROR;
    }
    prog_ui_update(true);
}
