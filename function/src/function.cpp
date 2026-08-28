#include "device_serial.hpp"
#include "function.hpp"
#include "string.h"

#include "delay.h"

void function_init(void)
{
    // DebugPort_Init();
    soft_485_init();
}

void function_loop(void)
{
    // debug_uart.send_data((uint8_t *)"data\n", strlen("data\n"));
    soft_485_send((uint8_t *)"soft_rs485\r\n", strlen("soft_rs485\r\n"));
    delay_ms(500);
}
