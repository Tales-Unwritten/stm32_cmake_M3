/**
 * @file    string_pro.cpp
 * @brief   字符串协议桥接层
 *
 * 数据流：
 *   USART ISR → uart_buffer_t.rx_flag=1
 *     → StringCmd_Task()
 *       → 遍历端口表，找到有数据的端口
 *       → command_process() 解析执行
 *       → 通过同一端口的 send() 发回回复
 *
 * 端口表：
 *   新增串口时在 string_ports[] 中添加一行即可
 *   数据从哪个串口来，回复就从哪个串口走
 */

#include "string_pro.hpp"
#include "str_cmd.hpp"
#include "device_serial.hpp"
#include "inter_soft_uart.hpp"
#include <cstring>

#if ACTIVE_PROTOCOL == PROTOCOL_STRING_CMD

/* ── 端口表（与 Modbus 相同模式） ────────────────────────────
 * 当前数据通道为软串口 suart（PA9/PA10，与 boot 硬件 USART0 同引脚），
 * 硬件 debug_uart 因同引脚冲突被禁用，故挂 suart 而非 debug_uart */

extern soft_uart_port<256> suart;   // 定义于 user/src/founction.cpp

static void send_via_debug(const uint8_t *data, uint16_t len)
{
    (void)suart.send_data(data, len);
}

static const string_port_t string_ports[] =
    {
        { suart.buffer(), send_via_debug },
        /* 新增串口在此追加：
        { rs485_uart.buffer(), send_via_rs485 },  */
    };


static const uint8_t string_port_count = sizeof(string_ports) / sizeof(string_ports[0]);

/* ── 初始化 ────────────────────────────────────────────────── */

void StringCmd_Init(void) {}

/* ── 轮询 ──────────────────────────────────────────────────── */

void StringCmd_Task(void)
{
    for (uint8_t port_index = 0; port_index < string_port_count; port_index++)
    {
        uart_buffer_t *buffer = string_ports[port_index].buffer;
        if (buffer->rx_flag != 1) continue;

        uint16_t data_length = buffer->rx_len;
        if (data_length < 2)
        {
            buffer->rx_len = 0;
            buffer->rx_flag = 0;
            continue;
        }

        /* 拷贝到本地缓冲区 */
        char line_buffer[256];
        uint16_t copy_length = (data_length < sizeof(line_buffer) - 1) ? data_length : (uint16_t)(sizeof(line_buffer) - 1);
        memcpy(line_buffer, buffer->rx_buf, copy_length);
        line_buffer[copy_length] = '\0';
        buffer->rx_len = 0;
        buffer->rx_flag = 0;

        /* 去除结尾 \r\n */
        while (copy_length > 0 && (line_buffer[copy_length - 1] == '\r' || line_buffer[copy_length - 1] == '\n'))
        {
            line_buffer[--copy_length] = '\0';
        }
        /* 解析执行（空串也送入，由 command_process 报错） */
        char reply_buffer[256] = {0};
        command_process(line_buffer, reply_buffer, sizeof(reply_buffer));

        /* 从哪个口来，从哪个口回 */
        if (reply_buffer[0])
        {
            string_ports[port_index].send((const uint8_t *)reply_buffer, (uint16_t)strlen(reply_buffer));
        }
    }
}

#endif
