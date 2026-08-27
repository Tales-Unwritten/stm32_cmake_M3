#pragma once

#include "inter_usart.hpp"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 字符串协议端口 —— 每个串口实例对应一个条目
   buffer: 指向串口的 uart_buffer_t（输入数据来源）
   send:   向该串口发送回复的函数 */
typedef struct {
    uart_buffer_t *buffer;
    void (*send)(const uint8_t *data, uint16_t len);
} string_port_t;

void StringCmd_Init(void);
void StringCmd_Task(void);

#ifdef __cplusplus
}
#endif
