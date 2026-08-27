#pragma once

#ifdef __cplusplus

#include "stdint.h"
/**
 * @brief SPI 总线抽象接口（软/硬 SPI 统一入口）
 *
 * 软件 SPI（soft_spi_bus）与硬件 SPI（spi_port）实现同一接口，
 * 设备驱动只需依赖本基类，即可透明切换软/硬 SPI 总线，
 * 保证「接口功能一致」。
 *
 * 公共方法（设备驱动依赖的最小集合）：
 *   init() / deinit()              生命周期
 *   transfer() / transfer_byte()   阻塞全双工传输
 *   cs_select() / cs_deselect()    片选控制
 *
 * 平台特有扩展保留在各自实现类中，不进入公共接口：
 *   spi_port::periph() / has_cs()
 *   soft_spi_bus::set_mode() / set_bit_order() / set_speed() / lock() / unlock()
 */
class spi_bus
{
public:
    virtual ~spi_bus() = default;

    // ── 生命周期 ──────────────────────────────────────────
    virtual void init()   = 0;
    virtual void deinit() = 0;

    // ── 数据传输（阻塞、全双工） ──────────────────────────
    virtual void    transfer(const uint8_t *tx, uint8_t *rx, uint16_t len) = 0;
    virtual uint8_t transfer_byte(uint8_t data) = 0;

    // ── 片选控制 ──────────────────────────────────────────
    virtual void cs_select()   = 0;
    virtual void cs_deselect() = 0;
};

#endif /* __cplusplus */
