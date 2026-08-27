#pragma once
/**
 * @file    iap_crc.hpp
 * @brief   IAP 校验工具：CRC16（帧/参数记录）+ CRC32（镜像校验）
 *
 * CRC16：Modbus 多项式 0xA001，init 0xFFFF，无最终异或
 *        （与 protocol/modbus 的 Modbus_CRC16 一致）
 * CRC32：IEEE 802.3，反射多项式 0xEDB88320，init 0xFFFFFFFF，最终异或
 *        （与 tools/iap_pack.py 的 zlib.crc32 一致）
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CRC16（一次计算） */
uint16_t Iap_Crc16(const uint8_t *data, uint32_t len);

/**
 * @brief CRC32 增量计算（分块读取 flash 时使用）
 * @param crc 上一块返回的中间值；首次调用传 0xFFFFFFFF
 * @return 中间值（全部块计算完后 与 0xFFFFFFFF 异或 得最终值）
 */
uint32_t Iap_Crc32_Step(uint32_t crc, const uint8_t *data, uint32_t len);

/** @brief CRC32（一次计算，等价于 Iap_Crc32_Step(0xFFFFFFFF, ...) ^ 0xFFFFFFFF） */
uint32_t Iap_Crc32(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif
