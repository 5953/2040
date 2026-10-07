// chip_database.c - 芯片数据库 (可自行添加型号)
//
// ┌─ 如何添加一款新芯片 ───────────────────────────────────────────────┐
// │ 在下面 CHIP_TABLE 里加一行即可, 不用动其他文件:                   │
// │   STM8_CHIP ("型号", ID, Flash(KB), RAM(字节), 擦写块大小(字节))   │
// │   STM32_CHIP("型号", DEV_ID, Flash(KB), RAM(KB), 页/扇区大小(字节))│
// │ 表里最多 MAX_CHIP_DB (128) 条; 菜单里"选择芯片"按此表顺序显示。    │
// └────────────────────────────────────────────────────────────────────┘
//
// 注意:
//  * STM32 的 DEV_ID 来自内核调试寄存器 DBGMCU_IDCODE[11:0] (SWD 读取), 同一系列
//    不同容量的型号常共用一个 ID (例如 F103 的 C6/C8/CB 都是 0x410),
//    自动识别时只会命中表里排在前面的那一条。
//  * STM8 没有 STM32 那样统一的设备 ID 寄存器, 下面 STM8 的 ID 值沿用了原工程里的
//    数值, 尚未用真实芯片验证。识别不到 ID 时, 程序会改用"选择芯片"里选定的型号。
#include "chip_database.h"

#define STM8_CHIP(name, id, flash_kb, ram_bytes, block)                              \
    { name, (id), 0xFFFF, CHIP_TYPE_STM8, 0x8000, (flash_kb) * 1024u,                \
      (ram_bytes), (block), (block), true }

#define STM32_CHIP(name, id, flash_kb, ram_kb, page)                                 \
    { name, (id), 0x0FFF, CHIP_TYPE_STM32, 0x08000000, (flash_kb) * 1024u,           \
      (ram_kb) * 1024u, (page), (page), true }

static const chip_info_t CHIP_TABLE[] = {
    //            型号              ID      Flash  RAM    块/页
    // ---- STM8 系列 (SWIM) ----
    STM8_CHIP ("STM8S003F3P6",   0x5344,    8,   1024,   64),   // 最常用: 8KB Flash, 1KB RAM, 128B EEPROM
    STM8_CHIP ("STM8S103F3P6",   0x5348,    8,   1024,   64),
    STM8_CHIP ("STM8S105K4T6",   0x5358,   16,   2048,  128),
    STM8_CHIP ("STM8S207C8T6",   0x5378,   64,   6144,  128),
    STM8_CHIP ("STM8L151C6T6",   0x5367,   32,   2048,  128),

    // ---- STM32 系列 (SWD) ----
    STM32_CHIP("STM32F103C8T6",  0x410,    64,   20,   1024),   // 很多实物是 128KB, 同 ID 无法区分
    STM32_CHIP("STM32F030F4P6",  0x444,    16,    4,   1024),
    STM32_CHIP("STM32F401CCU6",  0x423,   256,   64,  16384),   // 前 4 个扇区 16KB, 此处填首扇区大小
    STM32_CHIP("STM32G030F6P6",  0x466,    32,    8,   2048),
    STM32_CHIP("STM32F407VGT6",  0x413,  1024,  192,  16384),

    // ---- 在这里继续添加 ----
};

#define CHIP_TABLE_COUNT ((int)(sizeof(CHIP_TABLE) / sizeof(CHIP_TABLE[0])))

_Static_assert(sizeof(CHIP_TABLE) / sizeof(CHIP_TABLE[0]) <= MAX_CHIP_DB,
               "芯片数据库条目超过 MAX_CHIP_DB, 请调大 stm_programmer.h 里的 MAX_CHIP_DB");

void chip_db_init(void) {
    g_chip_db_count = 0;
    for (int i = 0; i < CHIP_TABLE_COUNT && g_chip_db_count < MAX_CHIP_DB; i++) {
        // 当前最终版只启用 STM8；STM32/SWD 保留数据库但暂不加入菜单。
        if (CHIP_TABLE[i].type != CHIP_TYPE_STM8) continue;
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
