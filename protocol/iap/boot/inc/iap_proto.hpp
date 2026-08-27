#pragma once
/**
 * @file    iap_proto.hpp
 * @brief   IAP 升级协议 —— 端口表（对齐 modbus/string 模式）+ 帧处理
 *
 * 帧格式：
 *   0xA5 0x5A | CMD(1) | LEN(2 LE) | PAYLOAD[LEN] | CRC16(2 LE, 覆盖整帧)
 *
 * 命令（见 iap_conf.hpp IAP_CMD_*）：
 *   QUERY    无负载 -> 应答 [status][state][active][pending][boot_ver(4)][act_ver(4)][oth_ver(4)]
 *   START    [version(4)][image_len(4)][crc32(4)][slot(1)] -> [status][next_seq(2)]
 *   DATA     [seq(2)][payload<=256] -> [status][next_seq(2)]
 *   END      无负载 -> [status][next_seq(2)]（status=OK 表示全量校验通过）
 *   ACTIVATE 无负载 -> [status][next_seq(2)]（随后复位）
 *   ABORT    无负载 -> [status][next_seq(2)]（退出升级模式，跳 active app）
 *   REBOOT   无负载 -> [status][next_seq(2)]（复位）
 *
 * 传输语义：逐包停等（主机发一包等应答再发下一包），包序号去重。
 */

#include "inter_usart.hpp"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 协议端口（与 modbus_port_t 同构：输入缓冲 + 发送回调） */
typedef struct {
    uart_buffer_t *buffer;
    void (*send)(uint8_t *data, uint16_t len);
} iap_port_t;

extern const iap_port_t iap_ports[];
extern const uint8_t iap_port_count;

/** @brief 协议初始化（复位内存状态） */
void Iap_Proto_Init(void);

/** @brief 轮询端口表并处理一帧（升级模式主循环中调用） */
void Iap_Proto_Task(void);

#ifdef __cplusplus
}
#endif
