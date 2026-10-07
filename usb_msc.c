// usb_msc.c — USB 大容量存储 (U盘)
#include "stm_programmer.h"
#include "tusb.h"

#define STORAGE_OFFSET     FILESYSTEM_OFFSET
#define STORAGE_SIZE       FILESYSTEM_SIZE
#define SECTOR_SIZE        512
#define STORAGE_SECTORS    (STORAGE_SIZE / SECTOR_SIZE)

static bool s_usb_writing = false;
static uint8_t s_erase_buf[4096];

// ===== 设备描述符 =====
static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x2E8A,
    .idProduct          = 0x000B,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1
};

// ===== 配置描述符 (MSC only) =====
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)
#define EPNUM_MSC_OUT     0x01
#define EPNUM_MSC_IN      0x81

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1,1,0, CONFIG_TOTAL_LEN, 0, 0x80, 100),
    TUD_MSC_DESCRIPTOR(0, 3, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

// ===== 字符串描述符 =====
static const char* const desc_string[] = {
    "",
    "5953",
    "STM Programmer",
    "00000001",
};

static uint16_t _desc_str[32];

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint16_t chr;
    if (index == 0) {
        _desc_str[0] = (TUSB_DESC_STRING << 8) | 4;
        _desc_str[1] = 0x0409;
        return _desc_str;
    }
    if (index >= sizeof(desc_string)/sizeof(desc_string[0])) return NULL;

    const char* str = desc_string[index];
    uint8_t len = 1;
    while ((chr = *str++) && len < 31) {
        _desc_str[len++] = chr;
    }
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (len * 2);
    return _desc_str;
}

// ===== USB 初始化 =====
void usb_init(void) {
    tusb_init();
}

void usb_task(void) {
    tud_task();
}

bool usb_is_writing(void) { return s_usb_writing; }
void usb_clear_writing(void) { s_usb_writing = false; }

// ===== MSC 回调 =====

uint8_t tud_msc_get_maxlun_cb(void) {
    return 1;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           void* buffer, uint32_t bufsize) {
    (void)lun; (void)offset;
    if (lba + (bufsize / SECTOR_SIZE) > STORAGE_SECTORS) return -1;
    uint32_t flash_addr = XIP_BASE + STORAGE_OFFSET + lba * SECTOR_SIZE;
    memcpy(buffer, (const void*)flash_addr, bufsize);
    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                            uint8_t* buffer, uint32_t bufsize) {
    (void)lun; (void)offset;
    if (lba + (bufsize / SECTOR_SIZE) > STORAGE_SECTORS) return -1;

    s_usb_writing = true;

    uint32_t flash_off = STORAGE_OFFSET + lba * SECTOR_SIZE;
    uint32_t remaining = bufsize;

    while (remaining > 0) {
        uint32_t block_base = flash_off & ~4095UL;
        uint32_t block_off  = flash_off - block_base;
        uint32_t chunk = (remaining > (4096 - block_off)) ? (4096 - block_off) : remaining;

        memcpy(s_erase_buf, (const void*)(XIP_BASE + block_base), 4096);
        memcpy(s_erase_buf + block_off, buffer, chunk);

        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(block_base, 4096);
        flash_range_program(block_base, s_erase_buf, 4096);
        restore_interrupts(ints);

        buffer    += chunk;
        flash_off += chunk;
        remaining -= chunk;
    }

    return (int32_t)bufsize;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size) {
    (void)lun;
    *block_count = STORAGE_SECTORS;
    *block_size  = SECTOR_SIZE;
}

bool tud_msc_is_writable_cb(uint8_t lun) {
    (void)lun;
    return true;
}

bool tud_msc_is_ready_cb(uint8_t lun) {
    (void)lun;
    return true;
}

void tud_msc_flush_cb(uint8_t lun) {
    (void)lun;
    s_usb_writing = false;
}

void tud_msc_clear_feature_cb(uint8_t lun, uint8_t feature) {
    (void)lun; (void)feature;
}

int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16],
                          void* buffer, uint16_t bufsize) {
    (void)lun; (void)scsi_cmd; (void)buffer; (void)bufsize;
    return 0;
}
