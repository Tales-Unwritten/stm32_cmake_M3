#pragma once

#ifdef __cplusplus

#include "modbus_crc.hpp"
#include "modbus_map.hpp"
#include "device_serial.hpp"

#include <stdint.h>

/*
 * 协议常量
 */
#define MODBUS_SLAVE_ADDR        0x01  /* 从机地址 */
#define MODBUS_RX_BUF_SIZE       256   /* 接收缓冲区字节数 */
#define MODBUS_TX_BUF_SIZE       256   /* 发送缓冲区字节数 */
#define MODBUS_MAX_REG_QUANTITY  125   /* 单次最大寄存器数量（Modbus 规范） */
#define MODBUS_MAX_COIL_QUANTITY 2000  /* 单次最大线圈数量（Modbus 规范） */

/* Modbus 异常码（Exception Code） */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION   0x01 /* 不支持的功能码 */
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR  0x02 /* 数据地址越界 */
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE 0x03 /* 数据值非法 */

/* 发送函数指针类型 */
typedef void (*modbus_send_fn)(uint8_t *buf, uint16_t len);

/*
 * modbus_t —— 头部对齐 uart_buffer_t 布局
 *
 *   [rx_buf*] [rx_len] [rx_flag] [tx_data*] [tx_len] [tx_idx] [tx_busy]
 *   [rx_buf_arr[256]] [tx_buf_arr[256]] [send]
 *
 *   Modbus_FillData() 直接从串口 uart_buffer_t 拷贝头部字段 + 数据。
 */
typedef struct
{
    /* ── 串口兼容头部（和 uart_buffer_t 同序，方便字段对接）── */
    uint8_t        *rx_buf;
    volatile uint16_t rx_len;
    volatile uint8_t  rx_flag;
    uint8_t        *tx_data;
    volatile uint16_t tx_len;
    volatile uint16_t tx_idx;
    volatile uint8_t  tx_busy;

    /* ── 实际存储数组 ── */
    uint8_t  rx_buf_arr[MODBUS_RX_BUF_SIZE];
    uint8_t  tx_buf_arr[MODBUS_TX_BUF_SIZE];

    /* ── Modbus 专用 ── */
    modbus_send_fn send;

} modbus_t;

typedef struct
{
    uart_buffer_t  *buf;      /* 指向串口 buffer，Modbus_FillData 直接对接 */
    modbus_send_fn  send;
} modbus_port_t;

extern modbus_t Modbus;

/* 只声明，不赋值 */
extern const modbus_port_t modbus_ports[];
extern const uint8_t       modbus_port_count;

void Modbus_Init(void);

void Modbus_Task(void);

void Modbus_FillData(uart_buffer_t *buf, modbus_send_fn send_fn);

void Modbus_FrameProcess(void);

void Modbus_PortSend(uint8_t *buf, uint16_t len);

/*
 * Modbus_ExceptionResponse —— 发送异常响应帧
 *
 * 帧格式: [从机地址][功能码|0x80][异常码][CRC_L][CRC_H]
 *
 * 参数:
 *   func           —— 原始功能码（如 0x01）
 *   exception_code —— 异常码（0x01/0x02/0x03）
 *
 * 异常码定义:
 *   0x01 ILLEGAL FUNCTION   —— 不支持的功能码
 *   0x02 ILLEGAL DATA ADDR  —— 数据地址越界
 *   0x03 ILLEGAL DATA VALUE —— 数据值非法（如线圈数量为 0 或超限）
 */
void Modbus_ExceptionResponse(uint8_t func, uint8_t exception_code);

#endif
