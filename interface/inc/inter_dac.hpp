#pragma once
// ============================================================
// @platform GD32F4xx
// ============================================================

#ifdef __cplusplus

#include <cstdint>
#include "inter_io_ctrl.hpp"
#include "gd32f4xx.h"

enum class dac_id : uint8_t { dac0 };

struct DacPortConfig {
    dac_id   periph;           ///< DAC0
    GPIO_TypeDef* out0_port;   ///< PA4（nullptr=不用此通道）
    pin_enum_t    out0_pin;
    GPIO_TypeDef* out1_port;   ///< PA5（nullptr=不用此通道）
    pin_enum_t    out1_pin;
    uint32_t trigger;          ///< DAC_TRIGGER_SOFTWARE / EXTERNAL_TRIGGER_DISABLE
    uint32_t vref_mv;          ///< 参考电压 mV（默认 3300）
};

/**
 * @brief DAC 端口（双通道电压输出）
 *
 *   static dac_port dac({
 *       dac_id::dac0, 0, 0, GPIOA, pin5,
 *       EXTERNAL_TRIGGER_DISABLE, 3300
 *   });
 *   dac.init();
 *   dac.set_mv(1, 1650);  // PA5 输出 1.65V
 */
class dac_port {
public:
    explicit dac_port(const DacPortConfig &cfg);
    ~dac_port();

    dac_port(const dac_port &) = delete;
    dac_port &operator=(const dac_port &) = delete;

    void init();
    void deinit();
    [[nodiscard]] bool is_initialized() const noexcept { return _initialized; }

    void set_mv(uint8_t channel, uint32_t mv);
    void set_raw(uint8_t channel, uint16_t value);

    [[nodiscard]] dac_id periph() const noexcept { return _cfg.periph; }

private:
    void _enable_clock();

    DacPortConfig _cfg;
    io_ctrl       _out0;
    io_ctrl       _out1;
    bool          _has_ch[2];
    bool          _initialized;
    uint32_t      _periph;
};

#endif
