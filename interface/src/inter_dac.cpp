// ============================================================
// @platform GD32F4xx（当前平台）
// ============================================================

#include "inter_dac.hpp"

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_dac.h"

dac_port::dac_port(const DacPortConfig &cfg)
    : _cfg(cfg)
    , _out0(cfg.out0_port ? cfg.out0_port : GPIOA, cfg.out0_pin != pin_none ? cfg.out0_pin : pin0)
    , _out1(cfg.out1_port ? cfg.out1_port : GPIOA, cfg.out1_pin != pin_none ? cfg.out1_pin : pin0)
    , _has_ch{cfg.out0_port != nullptr, cfg.out1_port != nullptr}
    , _initialized(false)
    , _periph(0)
{}

dac_port::~dac_port() { deinit(); }

void dac_port::_enable_clock()
{
    rcu_periph_clock_enable(RCU_DAC);
}

void dac_port::init()
{
    if (_initialized) return;
    _periph = DAC0;  // GD32 只有一个 DAC
    _enable_clock();

    // GPIO 模拟模式
    if (_has_ch[0]) {
        _out0.init(mode_analog, nopull);
    }
    if (_has_ch[1]) {
        _out1.init(mode_analog, nopull);
    }

    // 初始化两个通道
    for (uint8_t ch = 0; ch < 2; ch++) {
        if (!_has_ch[ch]) continue;
        uint32_t dac_ch = (ch == 0) ? DAC_OUT0 : DAC_OUT1;

        dac_output_buffer_enable(_periph, dac_ch);
        dac_enable(_periph, dac_ch);
    }

    _initialized = true;
}

void dac_port::deinit()
{
    if (!_initialized) return;

    for (uint8_t ch = 0; ch < 2; ch++) {
        if (!_has_ch[ch]) continue;
        dac_disable(_periph, (ch == 0) ? DAC_OUT0 : DAC_OUT1);
    }
    if (_has_ch[0]) _out0.deinit();
    if (_has_ch[1]) _out1.deinit();

    _periph = 0;
    _initialized = false;
}

void dac_port::set_raw(uint8_t channel, uint16_t value)
{
    if (!_initialized || channel > 1 || !_has_ch[channel]) return;
    if (value > 4095) value = 4095;

    uint32_t dac_ch = (channel == 0) ? DAC_OUT0 : DAC_OUT1;
    dac_data_set(_periph, dac_ch, DAC_ALIGN_12B_R, value);
}

void dac_port::set_mv(uint8_t channel, uint32_t mv)
{
    if (mv > _cfg.vref_mv) mv = _cfg.vref_mv;
    uint16_t raw = (uint16_t)((uint32_t)mv * 4095 / _cfg.vref_mv);
    set_raw(channel, raw);
}
