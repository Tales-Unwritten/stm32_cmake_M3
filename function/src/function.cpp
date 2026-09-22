#include "device_oled.hpp"
#include "device_serial.hpp"
#include "function.hpp"
#include "inter_i2c_bus.hpp"
#include "inter_i2c_dev.hpp"

#include "delay.h"
#include <string.h>

#ifdef ENABLE_FLASH_SELFTEST
#include "flash_selftest.hpp"
#endif

#ifdef ENABLE_DMA_SELFTEST
#include "dma_selftest.hpp"
#endif

#ifdef ENABLE_I2C_WDT_SELFTEST
#include "i2c_wdt_selftest.hpp"
#endif

// ============================================================
//  OLED 自测（SSD1306 128×64，软件 I2C）
// ============================================================
// 接线：SCL = PB3，SDA = PB4（开漏输出 + 内部上拉，建议外部 4.7k 上拉）
//   注意 PB3/PB4 复位后默认复用为 JTAG 的 JTDO/NJTRST，本工程已在
//   HAL_MspInit() 中用 __HAL_AFIO_REMAP_SWJ_NOJTAG() 释放为普通 GPIO，
//   故此处无需再调 set_af()。
// 从机地址：0x3C（7 位，SA0=0）；inter_i2c_dev 内部左移 1 位 → 0x78。
// 观察口：debug_uart(USART1 PA9/PA10, 115200) 打印每步自测结果。
// ============================================================

I2C_Bus_Info_t cfg{
    GPIOB, // SCL 端口
    GPIOB, // SDA 端口
    pin3,  // SCL = PB3
    pin4,  // SDA = PB4
};

inter_i2c_bus oled_bus(cfg, 1); // 半周期延时 1us

inter_i2c_dev oled_dev(&oled_bus, 0x3C);

Oled oled(oled_dev);

// ── 自测日志（debug_uart 阻塞发送） ────────────────────────────
static void oled_log(const char *s)
{
    debug_uart.send_data((const uint8_t *)s, (uint16_t)strlen(s));
}

void app_setup(void)
{
    DebugPort_Init();

#if defined(ENABLE_I2C_WDT_SELFTEST)
    // ── 硬件 I2C(PB6/PB7) + EEPROM(0xA0/0xA1) + IWDG/WWDG 自测固件 ──
    // 通信通道：debug_uart（USART1 PA9/PA10，与 device_serial 唯一实例共用）
    // 注意：本函数不返回（看门狗一旦启动无法关闭，末尾常驻喂狗），
    //       因此下面两个分支必须互斥，不允许掉入正常固件分支。
    i2c_wdt_selftest_main();
#elif defined(ENABLE_FLASH_SELFTEST)
    // ── 片上 flash_port 自测固件：跑完用例后进主循环空转 ──────
    flash_selftest_main();
#else
    // ── OLED 点屏自测 ──────────────────────────────────────
#ifdef ENABLE_DMA_SELFTEST
    dma_selftest_main(); // [DMA 内存搬运自测，仅 DMA_SELFTEST=ON 的构建存在]
#endif
    oled_bus.init(); // 配置 PB3/PB4 为开漏输出并释放总线

    // 1) 地址探测：先确认总线上有 ACK，再走初始化序列
    oled_log(oled_dev.ping() ? "[OLED] ping 0x3C ... ACK\r\n"
                             : "[OLED] ping 0x3C ... NACK (check SCL/SDA wiring & pull-ups)\r\n");

    // 2) 初始化 + 清屏
    oled.init();
    oled.clear();

    // 3) 静态首屏
    oled.print(0, 0, "OLED TEST", Oled::Font::F8x16);
    oled.print(0, 2, "SSD1306 128x64", Oled::Font::F8x16);
    oled.print(0, 5, "SCL=PB3 SDA=PB4", Oled::Font::F6x8);
    oled_log("[OLED] setup done\r\n");
#endif
}

void app_loop(void)
{
#if defined(ENABLE_I2C_WDT_SELFTEST) || defined(ENABLE_FLASH_SELFTEST)
    // 自测固件在 app_setup() 内自循环，不会走到这里
#else
    // 循环切换几屏，验证 ASCII 8×16 / 6×8 与整型渲染
    static uint8_t page = 0;

    oled.clear();
    switch (page)
    {
    case 0:
        oled.print(0, 0, "1) F8x16", Oled::Font::F8x16);
        oled.print(0, 2, "ABCDEFGHIJKLMNOP", Oled::Font::F8x16);
        break;
    case 1:
        oled.print(0, 1, "2) F6x8", Oled::Font::F6x8);
        oled.print(0, 3, "abcdefghijklmnopqrstu", Oled::Font::F6x8);
        break;
    default:
        oled.print(0, 0, "3) INT/UINT", Oled::Font::F8x16);
        oled.printInt(0, 2, -12345, Oled::Font::F8x16);
        oled.printUint(0, 4, 4294967295u, Oled::Font::F8x16);
        break;
    }
    page = (uint8_t)((page + 1) % 3);

    delay_ms(1500);
#endif
}
