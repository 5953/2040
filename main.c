// main.c - 主程序 (修复版)
#include "stm_programmer.h"
#include "chip_database.h"

//=============================================================================
// 全局变量
//=============================================================================
programmer_state_t g_state = {0};
menu_state_t g_menu = {0};
chip_info_t g_chip_db[MAX_CHIP_DB] = {0};
int g_chip_db_count = 0;
uint8_t g_stream_buffer[STREAM_BUFFER_SIZE] __attribute__((aligned(4)));
firmware_info_t g_firmware_info = {0};

//=============================================================================
// 按键处理
//=============================================================================
static const int         KEY_PINS[4]   = { PIN_KEY1,  PIN_KEY2,  PIN_KEY3,   PIN_KEY4 };
static const key_event_t  KEY_EVENTS[4] = { KEY_BACK,  KEY_OK,    KEY_DOWN,   KEY_UP   };

static uint32_t g_key_last_time[4] = {0};
static bool g_key_last_state[4] = {false};
static uint32_t g_key_press_time[4] = {0};

void keys_init(void) {
    memset(g_key_last_time, 0, sizeof(g_key_last_time));
    memset(g_key_last_state, 0, sizeof(g_key_last_state));
    memset(g_key_press_time, 0, sizeof(g_key_press_time));
}

static bool key_read(int pin) {
    return !gpio_get(pin);
}

key_event_t keys_scan(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    for (int i = 0; i < 4; i++) {
        bool current = key_read(KEY_PINS[i]);
        if (current && !g_key_last_state[i]) {
            if (now - g_key_last_time[i] < KEY_DEBOUNCE_MS) continue;
            g_key_press_time[i] = now;
            g_key_last_state[i] = true;
            g_key_last_time[i] = now;
            return KEY_EVENTS[i];
        } else if (current && g_key_last_state[i]) {
            uint32_t hold_time = now - g_key_press_time[i];
            if (hold_time > KEY_REPEAT_MS) {
                uint32_t repeat_interval = (hold_time > 2 * KEY_REPEAT_MS) ? KEY_REPEAT_FAST_MS : KEY_REPEAT_MS;
                if (now - g_key_last_time[i] > repeat_interval) {
                    g_key_last_time[i] = now;
                    return KEY_EVENTS[i];
                }
            }
        } else if (!current && g_key_last_state[i]) {
            g_key_last_state[i] = false;
        }
    }
    return KEY_NONE;
}

//=============================================================================
// CRC32
//=============================================================================
static uint32_t crc32_table[256];
static bool crc32_table_initialized = false;

void crc32_init(void) {
    if (crc32_table_initialized) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
        crc32_table[i] = crc;
    }
    crc32_table_initialized = true;
}

uint32_t crc32_calculate(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ crc32_table[index];
    }
    return ~crc;
}

//=============================================================================
// 配置管理
//=============================================================================
#define CONFIG_MAGIC 0x53544D50
#define CONFIG_VERSION 1

void config_set_default(system_config_t *config) {
    memset(config, 0, sizeof(system_config_t));
    config->magic = CONFIG_MAGIC;
    config->version = CONFIG_VERSION;
    config->program_mode = PROGRAM_MODE_AUTO;
    config->interface_mode = INTERFACE_AUTO;
    config->swd_speed = SWD_SPEED_MEDIUM;
    config->stability_check_enabled = true;
    config->allow_unknown_chip = false;
    config->verify_after_program = true;
    config->backup_before_program = false;
    config->auto_erase = true;
    config->default_chip_index = -1;
    strcpy(config->default_firmware, "");
    config->brightness = 200;
    config->screen_auto_off = false;
    config->screen_timeout = 60;
}

bool config_load(system_config_t *config) {
    const uint8_t *flash_config = (const uint8_t *)(XIP_BASE + CONFIG_OFFSET);
    memcpy(config, flash_config, sizeof(system_config_t));

    if (config->magic != CONFIG_MAGIC) {
        config_set_default(config);
        return false;
    }

    uint32_t saved_crc = config->crc32;
    config->crc32 = 0;
    uint32_t calc_crc = crc32_calculate((uint8_t *)config, sizeof(system_config_t));
    config->crc32 = saved_crc;

    if (calc_crc != saved_crc) {
        config_set_default(config);
        return false;
    }
    return true;
}

bool config_save(system_config_t *config) {
    config->crc32 = 0;
    config->crc32 = crc32_calculate((uint8_t *)config, sizeof(system_config_t));
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(CONFIG_OFFSET, CONFIG_SIZE);
    flash_range_program(CONFIG_OFFSET, (uint8_t *)config, sizeof(system_config_t));
    restore_interrupts(ints);
    return true;
}

//=============================================================================
// 硬件初始化
//=============================================================================
static void hardware_init(void) {
    stdio_init_all();

    const int key_pins[] = { PIN_KEY1, PIN_KEY2, PIN_KEY3, PIN_KEY4 };
    for (int i = 0; i < 4; i++) {
        gpio_init(key_pins[i]);
        gpio_set_dir(key_pins[i], GPIO_IN);
        gpio_pull_up(key_pins[i]);
    }

    gpio_init(PIN_SWIM_SWDIO);
    gpio_init(PIN_SWCLK);
    gpio_init(PIN_NRST);
    gpio_set_dir(PIN_SWIM_SWDIO, GPIO_OUT);
    gpio_set_dir(PIN_SWCLK, GPIO_OUT);
    gpio_set_dir(PIN_NRST, GPIO_OUT);
    gpio_put(PIN_NRST, 1);
    gpio_put(PIN_SWIM_SWDIO, 1);
    gpio_put(PIN_SWCLK, 0);
}

//=============================================================================
// 系统初始化
//=============================================================================
static bool system_init(void) {
    if (!oled_init()) {
        printf("[OLED] 未检测到屏幕\n");
    }

    oled_clear();
    oled_show_string_center(18, "STM烧录器");
    oled_show_string_center(36, "初始化中...");
    oled_refresh();

    crc32_init();
    keys_init();
    chip_db_init();

    if (!config_load(&g_state.config)) {
        config_set_default(&g_state.config);
        config_save(&g_state.config);
    }

    swd_set_speed(g_state.config.swd_speed);

    oled_set_brightness(g_state.config.brightness);

    int idx = g_state.config.default_chip_index;
    if (idx < 0 || idx >= g_chip_db_count) idx = chip_db_first_index_of_type(CHIP_TYPE_STM8);
    if (idx >= 0) g_state.selected_chip = g_chip_db[idx];

    if (!fs_init()) {
    oled_clear();
    oled_show_string_center(26, "文件系统初始化失败");
    oled_refresh();
    sleep_ms(2000);
    g_state.file_count = 0;
  } else {
    log_init();    // ← 加这一行, 开启日志
    log_write("=== STM Programmer boot ===\n");
    log_write("[SYS] clock = %lu Hz\n", (unsigned long)clock_get_hz(clk_sys));

    fs_list_files(g_state.file_list, &g_state.file_count, MAX_FILES);
    log_write("[FS] %d files\n", g_state.file_count);
}

    menu_system_init();
    sleep_ms(800);
    return true;
}

//=============================================================================
// 主循环
//=============================================================================
static void main_loop(void) {
    uint32_t last_link_check = 0;

    while (1) {
        usb_task();

        key_event_t key = keys_scan();
        menu_system_process(&g_menu, key, &g_state);

        if (!usb_is_writing()) {
            programmer_task_run(&g_menu, &g_state);
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (g_menu.current_menu == MENU_MAIN && now - last_link_check > 2000) {
            last_link_check = now;
            bool connected;
            if (g_state.chip_connected) {
                connected = programmer_link_alive(&g_state);
            } else {
                chip_type_t type;
                uint16_t id;
                connected = programmer_detect_chip(&type, &id);
            }
            if (connected != g_state.chip_connected) {
                g_state.chip_connected = connected;
                g_menu.need_refresh = true;
            }
        }

        sleep_ms(5);
    }
}

//=============================================================================
// 主函数
//=============================================================================
int main(void) {
    hardware_init();
    usb_init();
    system_init();
    main_loop();
    return 0;
}
