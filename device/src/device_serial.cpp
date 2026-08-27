#include "device_serial.hpp"

/* USART0, PA9(TX), PA10(RX), AF7, 115200, 256B RX */
usart_port debug_uart({USART0, GPIOA, pin9, GPIOA, pin10, afio_enum_t::NONE, 115200, 256});

/* USART2, PD5(TX), PD6(RX), AF7, 115200, 256B RX */
usart_port rs232_uart({USART2, GPIOD, pin5, GPIOD, pin6, afio_enum_t::NONE, 115200, 256});

/* USART1, PA2(TX), PA3(RX), AF7, 115200, 256B RX */
usart_port rs485_uart({USART1, GPIOA, pin2, GPIOA, pin3, afio_enum_t::NONE, 115200, 256});

io_ctrl en_485(GPIOG, pin13);

void DebugPort_Init(void)
{
    debug_uart.init();
}

void RS485Port_Init(void)
{
    rs485_uart.init();
}

void RS232Port_Init(void)
{
    rs232_uart.init();
}
