#include "device_serial.hpp"
#include "delay.h"

/* USART1, PA9(TX), PA10(RX), 115200, 256B RX */
// usart_port<256> debug_uart({usart1, GPIOA, pin9, GPIOA, pin10, afio_enum_t::NONE, 115200});

/* USART2, PD5(TX), PD6(RX), AF7, 115200, 256B RX */
usart_port<256> rs232_uart({usart2, GPIOD, pin5, GPIOD, pin6, afio_enum_t::NONE, 115200});

/* USART3, PA2(TX), PA3(RX), 115200, 256B RX (引脚请按实际板子核对) */
// usart_port<256> rs485_uart({usart3, GPIOA, pin2, GPIOA, pin3, afio_enum_t::NONE, 115200});

soft_uart_port<256> soft_rs485({GPIOA, pin3, GPIOA, pin2, 115200});

io_ctrl en_485(GPIOB, pin12);

// void DebugPort_Init(void)
// {
//     debug_uart.init();
// }

// void RS485Port_Init(void)
// {
// rs485_uart.init();
// }

void soft_485_init(void)
{
    en_485.init(mode_out_pp, pulldown, speed_high);
    soft_rs485.init();
}

void soft_485_send(uint8_t *data_t, uint16_t len_t)
{
    en_485.high();
    delay_us(50);   // DE 建立：等 485 收发器驱动使能 + 总线稳定（约 6 位 @115200）
    soft_rs485.send_data(data_t, len_t);
    delay_us(50);   // 保持 DE 高电平：让最后一个停止位完整走完总线再释放
    en_485.low();
}

// void RS232Port_Init(void)
// {
//     rs232_uart.init();
// }
