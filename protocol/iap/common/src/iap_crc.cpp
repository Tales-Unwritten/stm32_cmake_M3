/**
 * @file    iap_crc.cpp
 * @brief   IAP 校验工具实现
 *
 * CRC16（0xA001 反射多项式，init 0xFFFF）：
 *   标准测试向量 crc16("123456789") = 0x4B37（已宿主验证）
 * CRC32（IEEE 802.3，反射多项式 0xEDB88320，init 0xFFFFFFFF，结果异或）：
 *   与 Python zlib.crc32 一致（打包脚本同源校验）
 */

#include "iap_crc.hpp"

uint16_t Iap_Crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x0001u)
                crc = (crc >> 1) ^ 0xA001u;
            else
                crc >>= 1;
        }
    }
    return crc;
}

uint32_t Iap_Crc32_Step(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x00000001u)
                crc = (crc >> 1) ^ 0xEDB88320u;
            else
                crc >>= 1;
        }
    }
    return crc;
}

uint32_t Iap_Crc32(const uint8_t *data, uint32_t len)
{
    return Iap_Crc32_Step(0xFFFFFFFFu, data, len) ^ 0xFFFFFFFFu;
}
