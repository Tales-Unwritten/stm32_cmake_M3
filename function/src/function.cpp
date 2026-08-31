#include "device_serial.hpp"
#include "device_w25qxx.hpp"
#include "function.hpp"
#include "inter_soft_spi.hpp"
#include "inter_spi_bus.hpp"
#include "stdlib.h"
#include "string"
#include "string.h"

#include "delay.h"

// SCK=PA5, MOSI=PA7, MISO=PA6, CS=PA4, 低有效, MODE0, MSB, 最快
soft_spi_bus spi_bus_64({GPIOA, pin5, GPIOA, pin7, GPIOA, pin6, GPIOA, pin4, active_low, soft_mode_0, MSB, 0});

w25qxx flash64(spi_bus_64);

void function_init(void)
{
    // DebugPort_Init();
    spi_bus_64.init();
    soft_485_init();
    flash64.init();
}

void function_loop(void)
{
    // debug_uart.send_data((uint8_t *)"data\n", strlen("data\n"));
    // soft_485_send((uint8_t *)"soft_rs485\r\n", strlen("soft_rs485\r\n"));
    uint32_t id[1];
    id[0] = flash64.jedec_id();
    soft_485_send((uint8_t *)id, sizeof(id)); // sizeof(id) = 4
    delay_ms(800);
}
