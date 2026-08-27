#pragma once

#ifdef __cplusplus

#include <stdint.h>
uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len);
uint8_t  CRC8_Compute(const uint8_t *buf, uint16_t len);
uint32_t CRC32_Compute(const uint8_t *buf, uint16_t len);

#endif //__cplusplus
