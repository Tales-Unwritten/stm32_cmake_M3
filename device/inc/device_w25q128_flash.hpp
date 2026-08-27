#pragma once

#ifdef __cplusplus

#include "inter_spi.hpp"

/**
 * @brief W25Q128 Flash 驱动（最小验证版）
 *
 * 仅实现 JEDEC ID 读取，用于验证 spi_port 封装是否正确。
 * GPIOA: CS=PA4, CLK=PA5, MISO=PA6, MOSI=PA7, SPI1, AF5
 * （占位配置，用户按实物修改引脚）
 */
class w25q128_flash {
public:
    w25q128_flash();
    void init();
    bool is_initialized() const noexcept { return _initialized; }

    /** @brief 读取 JEDEC Manufacturer + Device ID（期望 0xEF7018） */
    uint32_t read_jedec_id();

private:
    spi_port _spi;
    bool     _initialized;
};

#endif
