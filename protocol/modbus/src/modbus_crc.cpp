#include "modbus_crc.hpp"
#include <stddef.h>

/** =========================================================================
 * CRC-8 (Maxim / DS18B20 风格)
 * 多项式：0x31 (代码中使用反射值 0x8C)
 * 初始值：0x00
 * 结果异或：0x00
 * ========================================================================= */
uint8_t CRC8_Compute(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0x00;
    uint16_t pos;
    uint8_t i;
    if (buf == NULL)
    {
        return 0;
    }
    for (pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x01)
            {
                crc >>= 1;
                crc ^= 0x8C; /* 0x31 的反射多项式 */
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t pos;
    uint8_t i;

    /** 安全性检查：如果指针为空，返回 */
    if (buf == NULL)
    {
        return 0;
    }
    for (pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/** =========================================================================
 * CRC-32 (IEEE 802.3 / Ethernet 风格)
 * 多项式：0x04C11DB7 (代码中使用反射值 0xEDB88320)
 * 初始值：0xFFFFFFFF
 * 结果异或：0xFFFFFFFF
 * ========================================================================= */
uint32_t CRC32_Compute(const uint8_t *buf, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    uint16_t pos;
    uint8_t i;

    if (buf == NULL)
    {
        return 0;
    }

    for (pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];

        for (i = 0; i < 8; i++)
        {
            if (crc & 0x00000001)
            {
                crc >>= 1;
                crc ^= 0xEDB88320; /* 0x04C11DB7 的反射多项式 */
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFF; /* 标准 CRC32 最后需要取反 */
}
