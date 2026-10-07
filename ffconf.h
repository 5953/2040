// ffconf.h — FatFs 配置 (适配 RP2040)
// 如果用的是 FatFs R0.15, 把 FFCONF_DEF 改成你下载版本里的值
#define FFCONF_DEF 86606

#define FF_FS_READONLY      0
#define FF_FS_EXFAT         0
#define FF_FS_NORTC         1
#define FF_NORTC_MON        1
#define FF_NORTC_MDAY      1
#define FF_NORTC_YEAR       2024
#define FF_FS_NOFSINFO      0
#define FF_USE_TRIM         0

#define FF_LFN_UNICODE      0
#define FF_MAX_LFN         255
#define FF_LFN_BUF         255
#define FF_SFN_BUF         12
#define FF_STRF_ENCODE     3
#define FF_CODE_PAGE       437
#define FF_USE_LFN         1

#define FF_VOLUMES          1
#define FF_STR_VOLUME_ID    0
#define FF_MULTI_PARTITION  0
#define FF_MIN_SS           512
#define FF_MAX_SS           512
#define FF_LBA64            0

#define FF_FS_TINY          0
#define FF_USE_FASTSEEK     0
#define FF_USE_FIND         0
#define FF_USE_MKFS         1
#define FF_USE_FORWARD      0

#define FF_CODE_PAGE 437