#pragma once

#include "inter_soft_uart.hpp"
#include "inter_usart.hpp"

// extern usart_port<256> debug_uart;
extern usart_port<256> rs232_uart;
// extern usart_port<256> rs485_uart;
extern soft_uart_port<256> soft_rs485;

//
extern io_ctrl en_485;

#ifdef __cplusplus
extern "C"
{
#endif

    // void DebugPort_Init(void);
    void soft_485_init(void);
    void soft_485_send(uint8_t *data_t, uint16_t len_t);

#ifdef __cplusplus
}
#endif
