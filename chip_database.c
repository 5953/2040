// chip_database.c - 芯片数据库 (修复版: 添加 flash_reg_base)
#include "chip_database.h"

#define STM8_CHIP(name, id, flash_kb, ram_bytes, block)                              \
    { name, (id), 0xFFFF, CHIP_TYPE_STM8, 0x8000, (flash_kb) * 1024u,                \
      (ram_bytes), (block), (block), true, 0 }

#define STM32_CHIP(name, id, flash_kb, ram_kb, page, reg_base)                        \
    { name, (id), 0x0FFF, CHIP_TYPE_STM32, 0x08000000, (flash_kb) * 1024u,           \
      (ram_kb) * 1024u, (page), (page), true, (reg_base) }

static const chip_info_t CHIP_TABLE[] = {
    STM8_CHIP ("STM8S003F3P6",   0x5344,    8,   1024,   64),
    STM8_CHIP ("STM8S103F3P6",   0x5348,    8,   1024,   64),
    STM8_CHIP ("STM8S105K4T6",   0x5358,   16,   2048,  128),
    STM8_CHIP ("STM8S207C8T6",   0x5378,   64,   6144,  128),
    STM8_CHIP ("STM8L151C6T6",   0x5367,   32,   2048,  128),

    STM32_CHIP("STM32F103C8T6",  0x410,    64,   20,   1024, STM32_F1_FLASH_BASE),
    STM32_CHIP("STM32F030F4P6",  0x444,    16,    4,   1024, STM32_F1_FLASH_BASE),
    STM32_CHIP("STM32F401CCU6",  0x423,   256,   64,  16384, STM32_F4_FLASH_BASE),
    STM32_CHIP("STM32G030F6P6",  0x466,    32,    8,   2048, STM32_G0_FLASH_BASE),
    STM32_CHIP("STM32F407VGT6",  0x413,  1024,  192,  16384, STM32_F4_FLASH_BASE),
};

#define CHIP_TABLE_COUNT ((int)(sizeof(CHIP_TABLE) / sizeof(CHIP_TABLE[0])))

_Static_assert(sizeof(CHIP_TABLE) / sizeof(CHIP_TABLE[0]) <= MAX_CHIP_DB,
               "芯片数据库条目超过 MAX_CHIP_DB");

void chip_db_init(void) {
    g_chip_db_count = 0;
    for (int i = 0; i < CHIP_TABLE_COUNT && g_chip_db_count < MAX_CHIP_DB; i++) {
        g_chip_db[g_chip_db_count++] = CHIP_TABLE[i];
    }
}

chip_info_t *chip_db_find_by_id(uint16_t device_id, chip_type_t type) {
    for (int i = 0; i < g_chip_db_count; i++) {
        chip_info_t *c = &g_chip_db[i];
        if (c->valid && c->type == type && (device_id & c->id_mask) == c->device_id) {
            return c;
        }
    }
    return NULL;
}

int chip_db_find_index_by_name(const char *name) {
    for (int i = 0; i < g_chip_db_count; i++) {
        if (g_chip_db[i].valid && strcmp(g_chip_db[i].name, name) == 0) return i;
    }
    return -1;
}

chip_info_t *chip_db_find_by_name(const char *name) {
    int i = chip_db_find_index_by_name(name);
    return (i >= 0) ? &g_chip_db[i] : NULL;
}

int chip_db_first_index_of_type(chip_type_t type) {
    for (int i = 0; i < g_chip_db_count; i++) {
        if (g_chip_db[i].valid && g_chip_db[i].type == type) return i;
    }
    return -1;
}