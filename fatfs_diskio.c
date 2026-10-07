// fatfs_diskio.c — FatFs 磁盘 IO (对接 RP2040 Flash)
#include "ff.h"
#include "diskio.h"
#include "stm_programmer.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#define SECTOR_SIZE 512
static uint8_t s_buf[4096];

DSTATUS disk_initialize(BYTE drv) {
    return (drv == 0) ? 0 : STA_NOINIT;
}

DSTATUS disk_status(BYTE drv) {
    return (drv == 0) ? 0 : STA_NOINIT;
}

DRESULT disk_read(BYTE drv, BYTE* buf, LBA_t sector, UINT count) {
    if (drv != 0) return RES_NOTRDY;
    uint32_t addr = XIP_BASE + FILESYSTEM_OFFSET + (uint32_t)sector * SECTOR_SIZE;
    memcpy(buf, (const void*)addr, (size_t)count * SECTOR_SIZE);
    return RES_OK;
}

DRESULT disk_write(BYTE drv, const BYTE* buf, LBA_t sector, UINT count) {
    if (drv != 0) return RES_NOTRDY;
    if (count == 0) return RES_PARERR;

    uint32_t total = (uint32_t)count * SECTOR_SIZE;
    uint32_t flash_off = FILESYSTEM_OFFSET + (uint32_t)sector * SECTOR_SIZE;
    uint32_t written = 0;

    while (total > 0) {
        uint32_t block_base = flash_off & ~4095UL;
        uint32_t block_off  = flash_off - block_base;
        uint32_t chunk = (total > (4096 - block_off)) ? (4096 - block_off) : total;

        memcpy(s_buf, (const void*)(XIP_BASE + block_base), 4096);
        memcpy(s_buf + block_off, buf + written, chunk);

        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(block_base, 4096);
        flash_range_program(block_base, s_buf, 4096);
        restore_interrupts(ints);

        // ===== 读回验证 =====
        __dsb();   // 确保内存序
        const uint8_t *verify = (const uint8_t *)(XIP_BASE + block_base + block_off);
        for (uint32_t i = 0; i < chunk; i++) {
            if (verify[i] != buf[written + i]) {
                return RES_ERROR;   // ← 关键: 让 FatFs 知道写失败
            }
        }

        written   += chunk;
        flash_off += chunk;
        total     -= chunk;
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE drv, BYTE cmd, void* buf) {
    if (drv != 0) return RES_NOTRDY;
    switch (cmd) {
        case CTRL_SYNC:        return RES_OK;
        case GET_SECTOR_COUNT: *(LBA_t*)buf = FILESYSTEM_SIZE / SECTOR_SIZE; return RES_OK;
        case GET_SECTOR_SIZE:  *(WORD*)buf = SECTOR_SIZE; return RES_OK;
        case GET_BLOCK_SIZE:   *(DWORD*)buf = 8; return RES_OK;
        default:               return RES_PARERR;
    }
}

DWORD get_fattime(void) {
    return ((DWORD)(2024 - 1980) << 25) | (1 << 21) | (1 << 16);
}
