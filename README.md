# RP2040-Zero STM8 离线烧录器

目标：STM8S003F3P6 优先，STM32/SWD 暂不启用。

## Flash布局
- 0x000000-0x0FFFFF：RP2040程序/OLED/配置
- 0x100000-0x1FFFFF：1MB FAT12 USB固件盘

USB连接电脑后，设备枚举为 Mass Storage，固件可直接复制到盘内；OLED菜单可列出、选择、删除固件。

## 硬件
- GPIO28 SDA / GPIO29 SCL：0.96寸 SSD1315/SSD1306 I2C OLED
- GPIO27/26/15/14：K1/K2/K3/K4
- GPIO2：STM8 SWIM（建议串联 220~1000Ω）
- GPIO4：STM8 NRST
- GND：共地
- 目标STM8必须有合适的VDD；RP2040 GPIO不可承受5V。

## STM8
通信按照 ST UM0470 的 SWIM entry、低速22-clock RZ bit、ROTF/WOTF框架实现。STM8S003F3P6 不使用虚构的 STM32 风格 DEV_ID 自动识别；连接成功后采用用户选择的 STM8 型号参数。

首次实机请先验证：SWIM进入、CSR读取、读取0x8000、再做擦除/写入/回读校验。未经过实机验证前，不要把目标板的唯一原始程序当作测试数据。

## 当前版本说明
- STM32/SWD：暂不启用。
- 固件盘：1MB FAT12，直接通过USB Mass Storage访问。
- 固件管理：列表、文件信息、删除、烧录。
- 备份：可选整片Flash读取并保存到固件盘。
- SWIM：使用UM0470低速协议框架；最终硬件验证仍需在实际STM8S003F3P6上用示波器/逻辑分析仪确认时序，再进行首次擦写。
