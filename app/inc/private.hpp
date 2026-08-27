#pragma once
#include "hard_eeprom.h"

#ifdef __cplusplus
extern "C" {
#endif

/* EEPROM 地址分配 */
#define EEPROM_ADDR_BAUD_VAL0    0x00
#define EEPROM_ADDR_BAUD_VAL1    0x10
#define EEPROM_ADDR_BAUD_VAL2    0x20

/* 支持的波特率列表 */
extern const uint32_t BaudValue[];
extern const uint32_t BaudValue_Size;

/* 波特率持久化（纯 EEPROM 读写，不依赖旧硬件驱动） */
void     Save_BaudValue(uint32_t baud, uint8_t interface);
uint32_t Load_BaudValue(uint8_t interface);
void     config_baudrate(uint32_t baud, uint8_t interface);

#ifdef __cplusplus
}
#endif
