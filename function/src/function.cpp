#include "device_serial.hpp"
#include "device_w25qxx.hpp"
#include "function.hpp"
#include "inter_flash.hpp"
#include "inter_soft_spi.hpp"
#include "inter_spi_bus.hpp"
#include "stdlib.h"
#include "string"
#include "string.h"

#include "delay.h"

#ifdef ENABLE_FLASH_SELFTEST
#include "flash_selftest.hpp"
#endif

// SCK=PA5, MOSI=PA7, MISO=PA6, CS=PA4, 低有效, MODE0, MSB, 最快
soft_spi_bus spi_bus_64({GPIOA, pin5, GPIOA, pin7, GPIOA, pin6, GPIOA, pin4, active_low, soft_mode_0, MSB, 0});

w25qxx flash64(spi_bus_64);
flash_port mimo_flash;

void function_init(void)
{
    // DebugPort_Init();
#ifdef ENABLE_FLASH_SELFTEST
    // ── 片上 flash_port 自测固件：跑完用例后进主循环空转 ──────
    // 通信通道：debug_uart（USART1 PA9/PA10，与 device_serial 唯一实例共用）
    DebugPort_Init();
    flash_selftest_main();
#else

    uint8_t momo_id = 0;
    spi_bus_64.init();
    DebugPort_Init();
    soft_485_init();
    // flash64.init();
    mimo_flash.init(0x0801FC00, 0x400);
    mimo_flash.write_byte(0x0801FC00, 0x67);
    momo_id = mimo_flash.read_byte(0x0801FC00);

    soft_485_send(&momo_id, 1);
    mimo_flash.erase(0x0801FC01);
    delay_ms(10);
    momo_id = mimo_flash.read_byte(0x0801FC00);
    soft_485_send(&momo_id, 1);
    debug_uart.send_data((uint8_t *)"aa\n", strlen("aa\n"));
#endif
}

void function_loop(void)
{
    // debug_uart.send_data((uint8_t *)"data\n", strlen("data\n"));
    // soft_485_send((uint8_t *)"soft_rs485\r\n", strlen("soft_rs485\r\n"));
    delay_ms(800);
}
