// ============================================================
// @platform GD32F4xx（当前平台）
// ============================================================

#include "inter_dma.hpp"

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_dma.h"

static dma_channel_enum _to_ch(uint8_t channel)
{
    return (dma_channel_enum)((uint32_t)DMA_CH0 + channel);
}

dma_channel::dma_channel(const DmaConfig &cfg)
    : _cfg(cfg), _initialized(false), _periph(0),
      _width(DMA_PERIPH_WIDTH_8BIT),
      _dir(DMA_MEMORY_TO_MEMORY),
      _src_inc(true), _dst_inc(true),
      _circular(false), _sub_periph(0)
{}

dma_channel::~dma_channel() { deinit(); }

void dma_channel::_enable_clock()
{
    if (_cfg.controller == dma_id::dma0)
        rcu_periph_clock_enable(RCU_DMA0);
    else
        rcu_periph_clock_enable(RCU_DMA1);
}

void dma_channel::init()
{
    if (_initialized) return;
    _periph = (_cfg.controller == dma_id::dma0) ? DMA0 : DMA1;
    _enable_clock();
    _initialized = true;
}

void dma_channel::deinit()
{
    if (!_initialized) return;
    stop();
    dma_deinit(_periph, _to_ch(_cfg.channel));
    _periph = 0;
    _initialized = false;
}

void dma_channel::set_width(uint32_t width)   { _width = width; }
void dma_channel::set_direction(uint32_t dir)  { _dir = dir; }
void dma_channel::set_increment(bool s, bool d) { _src_inc = s; _dst_inc = d; }
void dma_channel::set_circular(bool enable)    { _circular = enable; }
void dma_channel::set_subperipheral(uint32_t sub_periph) { _sub_periph = sub_periph; }

void dma_channel::start(void *src, void *dst, uint32_t count)
{
    if (!_initialized || !src || !dst || count == 0) return;

    dma_deinit(_periph, _to_ch(_cfg.channel));

    dma_single_data_parameter_struct dma_cfg;
    dma_single_data_para_struct_init(&dma_cfg);

    dma_cfg.direction           = _dir;
    dma_cfg.periph_memory_width = _width;
    dma_cfg.number              = count;
    dma_cfg.priority            = _cfg.priority;
    dma_cfg.circular_mode       = _circular ? DMA_CIRCULAR_MODE_ENABLE : DMA_CIRCULAR_MODE_DISABLE;

    // 外设请求源选择（如 GD32F470: ADC0 -> DMA1 CH0 SUBPERI0，官方例程映射）
    if (_sub_periph != 0)
        dma_channel_subperipheral_select(_periph, _to_ch(_cfg.channel), (dma_subperipheral_enum)_sub_periph);

    if (_dir == DMA_MEMORY_TO_MEMORY) {
        dma_cfg.periph_addr    = (uint32_t)src;
        dma_cfg.periph_inc     = _src_inc ? DMA_PERIPH_INCREASE_ENABLE : DMA_PERIPH_INCREASE_DISABLE;
        dma_cfg.memory0_addr   = (uint32_t)dst;
        dma_cfg.memory_inc     = _dst_inc ? DMA_MEMORY_INCREASE_ENABLE : DMA_MEMORY_INCREASE_DISABLE;
    } else if (_dir == DMA_PERIPH_TO_MEMORY) {
        dma_cfg.periph_addr    = (uint32_t)src;
        dma_cfg.periph_inc     = DMA_PERIPH_INCREASE_DISABLE;
        dma_cfg.memory0_addr   = (uint32_t)dst;
        dma_cfg.memory_inc     = _dst_inc ? DMA_MEMORY_INCREASE_ENABLE : DMA_MEMORY_INCREASE_DISABLE;
    } else {
        dma_cfg.periph_addr    = (uint32_t)dst;
        dma_cfg.periph_inc     = DMA_PERIPH_INCREASE_DISABLE;
        dma_cfg.memory0_addr   = (uint32_t)src;
        dma_cfg.memory_inc     = _src_inc ? DMA_MEMORY_INCREASE_ENABLE : DMA_MEMORY_INCREASE_DISABLE;
    }

    dma_single_data_mode_init(_periph, _to_ch(_cfg.channel), &dma_cfg);

    dma_flag_clear(_periph, _to_ch(_cfg.channel), DMA_FLAG_FTF);
    dma_channel_enable(_periph, _to_ch(_cfg.channel));
}

void dma_channel::stop()
{
    if (!_initialized) return;
    dma_channel_disable(_periph, _to_ch(_cfg.channel));
}

bool dma_channel::done() const
{
    if (!_initialized) return true;
    return (dma_flag_get(_periph, _to_ch(_cfg.channel), DMA_FLAG_FTF) == SET);
}
