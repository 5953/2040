// usb_storage.c
// 1MB internal Flash -> USB Mass Storage (FAT12)
// 同时提供烧录器内部的简单 FAT12 文件读写接口。
#include "stm_programmer.h"
#include <ctype.h>

#define FAT_SECTOR_SIZE 512u
#define FAT_RESERVED_SECTORS 1u
#define FAT_COUNT 2u
#define FAT_ROOT_ENTRIES 32u
#define FAT_ROOT_SECTORS ((FAT_ROOT_ENTRIES * 32u + FAT_SECTOR_SIZE - 1u) / FAT_SECTOR_SIZE)
#define FAT_SECTORS 3u
#define FAT_DATA_START (FAT_RESERVED_SECTORS + FAT_COUNT * FAT_SECTORS + FAT_ROOT_SECTORS)
#define FAT_CLUSTER_SECTORS 2u
#define FAT_CLUSTER_SIZE (FAT_CLUSTER_SECTORS * FAT_SECTOR_SIZE)
#define FAT_FIRST_CLUSTER 2u
#define FAT_MAX_CLUSTERS ((USB_DISK_SECTOR_COUNT - FAT_DATA_START) / FAT_CLUSTER_SECTORS)

static uint8_t sector_buf[FAT_SECTOR_SIZE];
static uint8_t block_buf[4096] __attribute__((aligned(4)));
static bool g_usb_ready = false;
static bool g_msc_ejected = false;

static inline const uint8_t *flash_ptr(uint32_t off) {
    return (const uint8_t *)(XIP_BASE + USB_DISK_OFFSET + off);
}

static void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p+2, (uint16_t)(v >> 16)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t get32(const uint8_t *p) { return get16(p) | ((uint32_t)get16(p+2) << 16); }

static bool raw_read_sector(uint32_t lba, void *buf) {
    if (lba >= USB_DISK_SECTOR_COUNT) return false;
    memcpy(buf, flash_ptr(lba * FAT_SECTOR_SIZE), FAT_SECTOR_SIZE);
    return true;
}

static bool raw_write_sector(uint32_t lba, const void *buf) {
    if (lba >= USB_DISK_SECTOR_COUNT) return false;
    uint32_t off = lba * FAT_SECTOR_SIZE;
    uint32_t erase_off = off & ~0xFFFu;
    uint32_t within = off & 0xFFFu;
    memcpy(block_buf, flash_ptr(erase_off), sizeof(block_buf));
    memcpy(block_buf + within, buf, FAT_SECTOR_SIZE);
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(USB_DISK_OFFSET + erase_off, 4096);
    flash_range_program(USB_DISK_OFFSET + erase_off, block_buf, 4096);
    restore_interrupts(ints);
    return true;
}

static bool volume_valid(void) {
    uint8_t b[FAT_SECTOR_SIZE];
    raw_read_sector(0, b);
    return b[510] == 0x55 && b[511] == 0xAA &&
           get16(&b[11]) == FAT_SECTOR_SIZE && b[13] == FAT_CLUSTER_SECTORS &&
           b[16] == FAT_COUNT && get16(&b[17]) == FAT_ROOT_ENTRIES &&
           get16(&b[22]) == FAT_SECTORS && get16(&b[19]) == USB_DISK_SECTOR_COUNT;
}

static void fat_set(uint8_t *fat, uint16_t cluster, uint16_t value) {
    uint32_t pos = cluster + cluster / 2;
    if (cluster & 1) {
        fat[pos] = (fat[pos] & 0x0F) | ((value << 4) & 0xF0);
        fat[pos+1] = (uint8_t)(value >> 4);
    } else {
        fat[pos] = (uint8_t)value;
        fat[pos+1] = (fat[pos+1] & 0xF0) | (uint8_t)(value >> 8);
    }
}

static uint16_t fat_get(const uint8_t *fat, uint16_t cluster) {
    uint32_t pos = cluster + cluster / 2;
    if (cluster & 1) return (uint16_t)(((fat[pos] >> 4) | (fat[pos+1] << 4)) & 0x0FFF);
    return (uint16_t)(fat[pos] | ((fat[pos+1] & 0x0F) << 8));
}

static uint32_t cluster_lba(uint16_t cluster) {
    return FAT_DATA_START + ((uint32_t)(cluster - FAT_FIRST_CLUSTER) * FAT_CLUSTER_SECTORS);
}

static bool format_volume(void) {
    uint8_t bs[FAT_SECTOR_SIZE];
    uint8_t fat[FAT_SECTORS * FAT_SECTOR_SIZE];
    uint8_t root[FAT_ROOT_SECTORS * FAT_SECTOR_SIZE];
    memset(bs, 0, sizeof(bs)); memset(fat, 0, sizeof(fat)); memset(root, 0, sizeof(root));

    bs[0]=0xEB; bs[1]=0x3C; bs[2]=0x90; memcpy(&bs[3], "STMPRG  ", 8);
    put16(&bs[11], FAT_SECTOR_SIZE); bs[13]=FAT_CLUSTER_SECTORS; bs[14]=1;
    bs[16]=FAT_COUNT; put16(&bs[17], FAT_ROOT_ENTRIES); put16(&bs[19], USB_DISK_SECTOR_COUNT);
    bs[21]=0xF8; put16(&bs[22], FAT_SECTORS); put16(&bs[24], 1); put16(&bs[26], 1);
    put32(&bs[28], 0); put32(&bs[32], 0); bs[36]=0; bs[38]=0x29; put32(&bs[39], 0x2040A001);
    memcpy(&bs[43], "STM-PROG   ", 11); memcpy(&bs[54], "FAT12   ", 8); bs[510]=0x55; bs[511]=0xAA;
    fat_set(fat, 0, 0xFF8); fat_set(fat, 1, 0xFFF);

    // Write all initial sectors. Each call preserves its 4K erase block.
    raw_write_sector(0, bs);
    for (uint32_t s=0; s<FAT_SECTORS; s++) raw_write_sector(FAT_RESERVED_SECTORS+s, fat+s*FAT_SECTOR_SIZE);
    for (uint32_t f=1; f<FAT_COUNT; f++) {
        uint32_t base=FAT_RESERVED_SECTORS+f*FAT_SECTORS;
        for (uint32_t s=0;s<FAT_SECTORS;s++) raw_write_sector(base+s, fat+s*FAT_SECTOR_SIZE);
    }
    for (uint32_t s=0;s<FAT_ROOT_SECTORS;s++) raw_write_sector(FAT_RESERVED_SECTORS+FAT_COUNT*FAT_SECTORS+s, root+s*FAT_SECTOR_SIZE);
    return volume_valid();
}

void usb_storage_init(void) {
    g_usb_ready = volume_valid();
    if (!g_usb_ready) g_usb_ready = format_volume();
    tusb_init();
}

void usb_storage_task(void) { tud_task(); }
bool usb_storage_format_if_needed(void) {
    if (volume_valid()) return true;
    g_usb_ready = format_volume();
    return g_usb_ready;
}

// ---------------------------------------------------------------------------
// FAT12 application-side file access
// ---------------------------------------------------------------------------
static bool read_fat(uint8_t *fat) {
    for (uint32_t i=0;i<FAT_SECTORS;i++) if (!raw_read_sector(1+i, fat+i*FAT_SECTOR_SIZE)) return false;
    return true;
}
static bool write_fat(const uint8_t *fat) {
    for (uint32_t copy=0;copy<FAT_COUNT;copy++) {
        for (uint32_t i=0;i<FAT_SECTORS;i++) if (!raw_write_sector(FAT_RESERVED_SECTORS+copy*FAT_SECTORS+i, fat+i*FAT_SECTOR_SIZE)) return false;
    }
    return true;
}
static bool read_root(uint8_t *root) {
    for (uint32_t i=0;i<FAT_ROOT_SECTORS;i++) if (!raw_read_sector(FAT_RESERVED_SECTORS+FAT_COUNT*FAT_SECTORS+i, root+i*FAT_SECTOR_SIZE)) return false;
    return true;
}
static bool write_root(const uint8_t *root) {
    for (uint32_t i=0;i<FAT_ROOT_SECTORS;i++) if (!raw_write_sector(FAT_RESERVED_SECTORS+FAT_COUNT*FAT_SECTORS+i, root+i*FAT_SECTOR_SIZE)) return false;
    return true;
}

static void short_name_to_text(const uint8_t *e, char *out, size_t cap) {
    char base[9], ext[4]; memcpy(base,e,8); base[8]=0; memcpy(ext,e+8,3); ext[3]=0;
    int n=8; while(n>0 && base[n-1]==' ') base[--n]=0;
    int m=3; while(m>0 && ext[m-1]==' ') ext[--m]=0;
    if (m) snprintf(out,cap,"%s.%s",base,ext); else snprintf(out,cap,"%s",base);
}
static void lfn_part(const uint8_t *e, char *out, int max) {
    int n=0; const int pos[][2]={{1,10},{14,12},{28,4}};
    for(int k=0;k<3;k++) for(int p=pos[k][0];p<pos[k][0]+pos[k][1];p+=2) {
        uint16_t c=get16(&e[p]);
        if(c==0x0000 || c==0xFFFF) continue;
        if(n<max-1) out[n++]=(c<128)?(char)c:'_';
    }
    out[n]=0;
}

// root file lookup; supports Windows long-name entries sufficiently for ASCII firmware names.
static bool find_file(const char *filename, uint8_t *root, int *entry_index, char *resolved, size_t rcap) {
    char lfn[128]={0};
    for(int i=0;i<(int)FAT_ROOT_ENTRIES;i++) {
        uint8_t *e=&root[i*32];
        if(e[0]==0x00) break;
        if(e[0]==0xE5){lfn[0]=0;continue;}
        if(e[11]==0x0F){
            char part[32]; lfn_part(e,part,sizeof(part));
            char tmp[128]; snprintf(tmp,sizeof(tmp),"%s%s",part,lfn); strncpy(lfn,tmp,sizeof(lfn)-1); lfn[sizeof(lfn)-1]=0;
            continue;
        }
        if(e[11]&0x08){lfn[0]=0;continue;}
        char shortname[32]; short_name_to_text(e,shortname,sizeof(shortname));
        if((lfn[0] && strcasecmp(lfn,filename)==0) || strcasecmp(shortname,filename)==0) {
            *entry_index=i; if(resolved) snprintf(resolved,rcap,"%s",lfn[0]?lfn:shortname); return true;
        }
        lfn[0]=0;
    }
    return false;
}

static void text_to_short83(const char *name, uint8_t out[11]) {
    memset(out,' ',11); const char *dot=strrchr(name,'.');
    size_t bl=dot?(size_t)(dot-name):strlen(name); if(bl>8) bl=8; if(bl) memcpy(out,name,bl);
    if(dot){size_t el=strlen(dot+1);if(el>3)el=3;memcpy(out+8,dot+1,el);} 
    for(int i=0;i<11;i++) if(out[i]>='a'&&out[i]<='z') out[i]-=32;
}

bool fs_init(void) { return volume_valid() || format_volume(); }

bool fs_list_files(file_info_t *files, int *count, int max_count) {
    uint8_t root[FAT_ROOT_SECTORS*FAT_SECTOR_SIZE]; if(!read_root(root)) return false; *count=0; char lfn[128]={0};
    for(int i=0;i<(int)FAT_ROOT_ENTRIES && *count<max_count;i++){
        uint8_t *e=&root[i*32]; if(e[0]==0x00)break; if(e[0]==0xE5){lfn[0]=0;continue;}
        if(e[11]==0x0F){char part[32];lfn_part(e,part,sizeof(part));char tmp[128];snprintf(tmp,sizeof(tmp),"%s%s",part,lfn);strncpy(lfn,tmp,sizeof(lfn)-1);continue;}
        if(e[11]&0x08){lfn[0]=0;continue;}
        char name[128]; short_name_to_text(e,name,sizeof(name)); if(lfn[0])strncpy(name,lfn,sizeof(name)-1);
        files[*count].valid=true; snprintf(files[*count].filename,MAX_FILENAME_LEN,"%s",name);
        files[*count].size=get32(&e[28]); files[*count].timestamp=0;
        const char *dot=strrchr(name,'.'); files[*count].type=(dot && (!strcasecmp(dot,".hex")||!strcasecmp(dot,".ihx")))?FILE_TYPE_HEX:FILE_TYPE_BIN; (*count)++; lfn[0]=0;
    }
    return true;
}

bool fs_read_file(const char *filename, uint8_t **buffer, uint32_t *size) {
    uint8_t root[FAT_ROOT_SECTORS*FAT_SECTOR_SIZE]; uint8_t fat[FAT_SECTORS*FAT_SECTOR_SIZE]; int idx;
    if(!read_root(root)||!read_fat(fat)||!find_file(filename,root,&idx,NULL,0)) return false;
    uint8_t *e=&root[idx*32]; uint32_t sz=get32(&e[28]); uint16_t cl=get16(&e[26]);
    if(sz==0){*buffer=NULL;*size=0;return true;} if(!cl || cl>=0xFF8 || sz>USB_DISK_SIZE){return false;}
    uint8_t *p=malloc(sz); if(!p)return false; uint32_t done=0;
    while(done<sz && cl>=2 && cl<0xFF8){uint32_t lba=cluster_lba(cl);for(uint32_t s=0;s<FAT_CLUSTER_SECTORS && done<sz;s++){uint8_t sec[512];raw_read_sector(lba+s,sec);uint32_t n=(sz-done>512)?512:sz-done;memcpy(p+done,sec,n);done+=n;}cl=fat_get(fat,cl);}
    if(done!=sz){free(p);return false;}*buffer=p;*size=sz;return true;
}

static int find_free_dir(uint8_t *root){for(int i=0;i<(int)FAT_ROOT_ENTRIES;i++){if(root[i*32]==0x00||root[i*32]==0xE5)return i;}return -1;}
static int find_free_cluster(uint8_t *fat){for(int c=2;c<2+(int)FAT_MAX_CLUSTERS;c++)if(fat_get(fat,c)==0)return c;return -1;}

bool fs_write_file(const char *filename,const uint8_t *data,uint32_t size,file_type_t type){
    if(size>USB_DISK_SIZE || !volume_valid()) return false;
    uint8_t root[FAT_ROOT_SECTORS*FAT_SECTOR_SIZE], fat[FAT_SECTORS*FAT_SECTOR_SIZE]; if(!read_root(root)||!read_fat(fat))return false;
    int old; if(find_file(filename,root,&old,NULL,0)){root[old*32]=0xE5;}
    int dir=find_free_dir(root); if(dir<0)return false;
    uint32_t clusters=(size+FAT_CLUSTER_SIZE-1)/FAT_CLUSTER_SIZE; uint16_t first=0,prev=0;
    for(uint32_t i=0;i<clusters;i++){int c=find_free_cluster(fat);if(c<0)return false;if(!first)first=(uint16_t)c;if(prev)fat_set(fat,prev,(uint16_t)c);prev=(uint16_t)c;fat_set(fat,(uint16_t)c,0xFFF);}
    uint32_t off=0; uint16_t cl=first;
    for(uint32_t n=0;n<clusters;n++){for(uint32_t s=0;s<FAT_CLUSTER_SECTORS;s++){uint8_t sec[512];memset(sec,0xFF,sizeof(sec));uint32_t remain=size-off;uint32_t take=remain>512?512:remain;if(take)memcpy(sec,data+off,take);if(!raw_write_sector(cluster_lba(cl)+s,sec))return false;off+=take;if(off>=size)break;}cl=fat_get(fat,cl);}
    uint8_t *e=&root[dir*32];memset(e,0,32);text_to_short83(filename,e);e[11]=0x20;put16(&e[26],first);put32(&e[28],size);
    bool ok=write_fat(fat)&&write_root(root); (void)type; return ok;
}

bool fs_delete_file(const char *filename){uint8_t root[FAT_ROOT_SECTORS*FAT_SECTOR_SIZE],fat[FAT_SECTORS*FAT_SECTOR_SIZE];int idx;if(!read_root(root)||!read_fat(fat)||!find_file(filename,root,&idx,NULL,0))return false;uint16_t cl=get16(&root[idx*32+26]);while(cl>=2&&cl<0xFF8){uint16_t n=fat_get(fat,cl);fat_set(fat,cl,0);cl=n;}root[idx*32]=0xE5;return write_fat(fat)&&write_root(root);}

// ---------------------------------------------------------------------------
// TinyUSB MSC callbacks
// ---------------------------------------------------------------------------
bool tud_msc_test_unit_ready_cb(uint8_t lun){(void)lun;return g_usb_ready && !g_msc_ejected;}
void tud_msc_capacity_cb(uint8_t lun,uint32_t *block_count,uint16_t *block_size){(void)lun;*block_count=USB_DISK_SECTOR_COUNT;*block_size=USB_DISK_SECTOR_SIZE;}
bool tud_msc_is_writable_cb(uint8_t lun){(void)lun;return true;}
void tud_msc_inquiry_cb(uint8_t lun,uint8_t vendor_id[8],uint8_t product_id[16],uint8_t product_rev[4]){(void)lun;memcpy(vendor_id,"STMPRG  ",8);memcpy(product_id,"RP2040 Firmware ",16);memcpy(product_rev,"1.0 ",4);}
void tud_msc_start_stop_cb(uint8_t lun,uint8_t power_condition,bool start,bool load_eject){(void)lun;(void)power_condition;g_msc_ejected=load_eject&&!start;}
int32_t tud_msc_read10_cb(uint8_t lun,uint32_t lba,uint32_t offset,void *buffer,uint32_t bufsize){(void)lun;if(lba>=USB_DISK_SECTOR_COUNT)return -1;memcpy(buffer,flash_ptr(lba*512+offset),bufsize);return (int32_t)bufsize;}
int32_t tud_msc_write10_cb(uint8_t lun,uint32_t lba,uint32_t offset,uint8_t *buffer,uint32_t bufsize){(void)lun;if(lba>=USB_DISK_SECTOR_COUNT)return -1;uint32_t done=0;while(done<bufsize){uint32_t take=bufsize-done;if(take>512-offset)take=512-offset;uint8_t sec[512];raw_read_sector(lba,sec);memcpy(sec+offset,buffer+done,take);if(!raw_write_sector(lba,sec))return -1;done+=take;offset+=take;if(offset>=512){offset=0;lba++;}}return (int32_t)bufsize;}
