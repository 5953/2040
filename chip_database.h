// chip_database.h - 芯片数据库接口
// 想新增芯片型号: 只需要改 chip_database.c 里的 CHIP_TABLE, 照着已有的格式加一行。
#ifndef CHIP_DATABASE_H
#define CHIP_DATABASE_H

#include "stm_programmer.h"

void        chip_db_init(void);                                    // 把数据表装入 g_chip_db
chip_info_t *chip_db_find_by_id(uint16_t device_id, chip_type_t type);
chip_info_t *chip_db_find_by_name(const char *name);               // 按型号名精确查找
int         chip_db_find_index_by_name(const char *name);          // 返回下标, 找不到 -1
int         chip_db_first_index_of_type(chip_type_t type);         // 该系列的第一个型号, 没有返回 -1

#endif
