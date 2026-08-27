// ============================================================
// @platform GD32F4xx（当前平台）
//   参考实现：王海涛 adc_bsp.c（时钟分频显式配置、硬件序列扫描）
// ============================================================

#include "inter_adc.hpp"

#include <new>
#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_adc.h"
#include "gd32f4xx_gpio.h"

adc_port::adc_port(const AdcPortConfig &cfg)
    : _cfg(cfg), _ch_pins{}, _dma(nullptr),
      _initialized(false), _periph(0)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (i < cfg.channel_count && cfg.channels[i].port != nullptr) {
            _ch_pins[i] = new (_ch_storage[i].data)
                io_ctrl(cfg.channels[i].port, cfg.channels[i].pin);
        }
    }
    if (cfg.use_dma) {
        _dma = new (_dma_storage.data)
            dma_channel({cfg.dma_controller, cfg.dma_channel, cfg.dma_priority});
    }
}

adc_port::~adc_port() { deinit(); }

void adc_port::_enable_clock()
{
    switch (_cfg.periph) {
        case adc_id::adc0: rcu_periph_clock_enable(RCU_ADC0); break;
        case adc_id::adc1: rcu_periph_clock_enable(RCU_ADC1); break;
        case adc_id::adc2: rcu_periph_clock_enable(RCU_ADC2); break;
    }
}

void adc_port::init()
{
    if (_initialized) return;

    switch (_cfg.periph) {
        case adc_id::adc0: _periph = ADC0; break;
        case adc_id::adc1: _periph = ADC1; break;
        case adc_id::adc2: _periph = ADC2; break;
    }

    _enable_clock();

    // ── ADC 时钟分频（3 个 ADC 共享，对齐参考实现显式配置） ──
    adc_clock_config(_cfg.clock_div);

    // 温度传感器(CH16)/内部参考(CH17)使能（3 个 ADC 共享）
    if (_cfg.enable_temp_vref) {
        ADC_SYNCCTL |= ADC_TEMP_VREF_CHANNEL_SWITCH;
    }

    // GPIO: 模拟模式
    for (uint8_t i = 0; i < _cfg.channel_count && i < 8; i++) {
        if (_ch_pins[i] == nullptr && _cfg.channels[i].port != nullptr) {
            _ch_pins[i] = new (_ch_storage[i].data)
                io_ctrl(_cfg.channels[i].port, _cfg.channels[i].pin);
        }
        if (_ch_pins[i]) {
            _ch_pins[i]->init(mode_analog, nopull);
        }
    }

    // ADC 独立模式
    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
    // 独立模式下必须显式关闭同步 DMA（对齐师傅：SYNCDMA 位残留会影响独立 ADC 的 DMA 请求）
    adc_sync_dma_config(ADC_SYNC_DMA_DISABLE);
    // EOCM=1：每次转换结束即置位 EOC（GD32F4xx 默认 EOCM=0 只在序列结束时置位，
    // 与 STM32 相反；轮询路径依赖每次转换的 EOC，DMA 路径会在 dma_start 里切回 0）
    ADC_CTL1(_periph) |= ADC_CTL1_EOCM;
    adc_special_function_config(_periph, ADC_SCAN_MODE, (_cfg.scan_enable || _cfg.use_dma) ? ENABLE : DISABLE);
    adc_special_function_config(_periph, ADC_CONTINUOUS_MODE, _cfg.use_dma ? ENABLE : DISABLE);   // DMA 模式连续转换（对齐参考实现）
    adc_data_alignment_config(_periph, _cfg.data_alignment);
    adc_resolution_config(_periph, _cfg.resolution);
    adc_external_trigger_config(_periph, ADC_ROUTINE_CHANNEL,
                                EXTERNAL_TRIGGER_DISABLE);

    // 硬件扫描：配置规则序列（rank 从 0 起，GD32 库按 rank 直接移位）
    if (_cfg.scan_enable && _cfg.channel_count > 0) {
        for (uint8_t i = 0; i < _cfg.channel_count && i < 8; i++) {
            adc_routine_channel_config(_periph, i,            // ⚠️ rank 0-based
                                       _cfg.channels[i].channel,
                                       _cfg.channels[i].sample_time);
        }
        adc_channel_length_config(_periph, ADC_ROUTINE_CHANNEL, _cfg.channel_count);
    }

    // ⚠️ GD32F4xx 校准流程：先 adc_enable，再校准（RSTCLB/CLB 在 ADC 使能后才工作）
    adc_enable(_periph);
    adc_calibration_enable(_periph);

    // DMA 模式：仅初始化 DMA 通道；DMA 请求使能延后到 dma_start()
    //（若提前使能，转换会挂起等待未配置的 DMA 请求，scan/read_raw 全部超时）
    if (_cfg.use_dma) {
        _dma->init();
    }
    _initialized = true;
}

void adc_port::deinit()
{
    if (!_initialized) return;
    adc_disable(_periph);

    if (_cfg.use_dma) {
        adc_dma_mode_disable(_periph);
        if (_dma) {
            _dma->deinit();
            _dma->~dma_channel();
            _dma = nullptr;
        }
    }

    for (uint8_t i = 0; i < _cfg.channel_count && i < 8; i++) {
        if (_ch_pins[i]) {
            _ch_pins[i]->deinit();
            _ch_pins[i]->~io_ctrl();
            _ch_pins[i] = nullptr;
        }
    }
    _periph = 0;
    _initialized = false;
}

void adc_port::calibrate()
{
    if (!_initialized) return;

    // GD32F4xx：校准在 ADC 已使能状态下执行（RSTCLB→等清→CLB→等清）
    adc_calibration_enable(_periph);
}

void adc_port::_config_channel(uint8_t index)
{
    if (index >= _cfg.channel_count) return;

    adc_routine_channel_config(_periph, 0,
                               _cfg.channels[index].channel,
                               _cfg.channels[index].sample_time);
    adc_channel_length_config(_periph, ADC_ROUTINE_CHANNEL, 1);
}

uint16_t adc_port::_do_convert()
{
    // 单通道读取：序列长度=1，单次模式（CTN=0）触发恰好转换一次。
    // EOCM=1 保证 EOC 在每次转换结束置位（而非序列末）
    ADC_CTL1(_periph) |= ADC_CTL1_EOCM;
    adc_special_function_config(_periph, ADC_CONTINUOUS_MODE, DISABLE);
    // 排空在途转换：ROVF 置位会让 ADC 停滞（手册 14.4.11），残留 EOC 一并清掉
    adc_flag_clear(_periph, ADC_FLAG_ROVF);
    if (adc_flag_get(_periph, ADC_FLAG_EOC)) {
        for (;;) {
            (void)adc_routine_data_read(_periph);
            adc_flag_clear(_periph, ADC_FLAG_EOC);
            adc_flag_clear(_periph, ADC_FLAG_ROVF);
            uint32_t t = 0x800;   // ~43us，超过一次转换时间
            while (t-- && !adc_flag_get(_periph, ADC_FLAG_EOC)) {}
            if (!adc_flag_get(_periph, ADC_FLAG_EOC)) break;
        }
    }

    adc_flag_clear(_periph, ADC_FLAG_WDE | ADC_FLAG_EOC | ADC_FLAG_EOIC |
                           ADC_FLAG_STIC | ADC_FLAG_STRC | ADC_FLAG_ROVF);
    adc_software_trigger_enable(_periph, ADC_ROUTINE_CHANNEL);

    uint32_t timeout = 0xFFFF;
    while (!adc_flag_get(_periph, ADC_FLAG_EOC)) {
        if (--timeout == 0) return 0;
    }
    adc_flag_clear(_periph, ADC_FLAG_EOC);

    return (uint16_t)adc_routine_data_read(_periph);
}

uint16_t adc_port::read_raw(uint8_t ch_index)
{
    if (!_initialized || ch_index >= _cfg.channel_count) return 0;
    _config_channel(ch_index);
    return _do_convert();
}

uint32_t adc_port::read_mv(uint8_t ch_index)
{
    uint16_t raw = read_raw(ch_index);
    uint32_t max_raw;
    switch (_cfg.resolution) {
        case ADC_RESOLUTION_12B: max_raw = 4095; break;
        case ADC_RESOLUTION_10B: max_raw = 1023; break;
        case ADC_RESOLUTION_8B:  max_raw = 255;  break;
        default:                 max_raw = 63;   break;
    }
    return (uint32_t)raw * _cfg.vref_mv / max_raw;
}

void adc_port::scan_raw(uint16_t *results)
{
    if (!_initialized || !results) return;

    // 非扫描模式：兼容旧行为（逐通道软件转换）
    if (!_cfg.scan_enable) {
        for (uint8_t i = 0; i < _cfg.channel_count; i++) {
            results[i] = read_raw(i);
        }
        return;
    }

    // EOCM=1：每次转换结束即置位 EOC（GD32F4xx 默认 EOCM=0 只在序列末置位，
    // 会导致轮询只能读到序列最后一个通道的值）
    ADC_CTL1(_periph) |= ADC_CTL1_EOCM;
    adc_special_function_config(_periph, ADC_CONTINUOUS_MODE, DISABLE);

    // 排空上一次连续转换的在途状态：ROVF 置位会让 ADC 停滞（手册 14.4.11），
    // 残留 EOC 一并清掉，避免读到旧值
    adc_flag_clear(_periph, ADC_FLAG_ROVF);
    if (adc_flag_get(_periph, ADC_FLAG_EOC)) {
        for (;;) {
            (void)adc_routine_data_read(_periph);
            adc_flag_clear(_periph, ADC_FLAG_EOC);
            adc_flag_clear(_periph, ADC_FLAG_ROVF);
            uint32_t t = 0x800;
            while (t-- && !adc_flag_get(_periph, ADC_FLAG_EOC)) {}
            if (!adc_flag_get(_periph, ADC_FLAG_EOC)) break;
        }
    }

    // 恢复完整扫描序列：read_raw() 会把序列长度改成 1，
    // 必须先按配置重排 rank + 长度，否则只转换最后一个通道
    for (uint8_t i = 0; i < _cfg.channel_count && i < 8; i++) {
        adc_routine_channel_config(_periph, i,
                                   _cfg.channels[i].channel,
                                   _cfg.channels[i].sample_time);
    }
    adc_channel_length_config(_periph, ADC_ROUTINE_CHANNEL, _cfg.channel_count);

    adc_special_function_config(_periph, ADC_CONTINUOUS_MODE, ENABLE);
    adc_flag_clear(_periph, ADC_FLAG_WDE | ADC_FLAG_EOC | ADC_FLAG_EOIC |
                           ADC_FLAG_STIC | ADC_FLAG_STRC | ADC_FLAG_ROVF);

    // 硬件扫描：连续模式 + 一次触发，读满一轮序列
    adc_software_trigger_enable(_periph, ADC_ROUTINE_CHANNEL);
    for (uint8_t i = 0; i < _cfg.channel_count; i++) {
        uint32_t timeout = 0xFFFF;
        while (!adc_flag_get(_periph, ADC_FLAG_EOC)) {
            if (--timeout == 0) { results[i] = 0; return; }
        }
        adc_flag_clear(_periph, ADC_FLAG_EOC);
        results[i] = (uint16_t)adc_routine_data_read(_periph);
    }
    // 立即退出连续模式（在途转换由下次调用的排空逻辑处理）
    adc_special_function_config(_periph, ADC_CONTINUOUS_MODE, DISABLE);
}

void adc_port::scan_mv(uint32_t *results)
{
    if (!_initialized || !results) return;

    uint32_t max_raw;
    switch (_cfg.resolution) {
        case ADC_RESOLUTION_12B: max_raw = 4095; break;
        case ADC_RESOLUTION_10B: max_raw = 1023; break;
        case ADC_RESOLUTION_8B:  max_raw = 255;  break;
        default:                 max_raw = 63;   break;
    }

    uint16_t raw[8];
    scan_raw(raw);
    for (uint8_t i = 0; i < _cfg.channel_count; i++) {
        results[i] = (uint32_t)raw[i] * _cfg.vref_mv / max_raw;
    }
}

// ============================================================
//  DMA 连续采集（对齐参考实现 adc_bsp.c 的 DMA 模式）
// ============================================================

bool adc_port::dma_start(uint16_t *buf, uint16_t frames)
{
    if (!_initialized || !buf || !_dma || frames == 0) return false;
    if (_cfg.channel_count == 0) return false;

    // 退出连续模式 + 排空在途 EOC（若刚从轮询模式切换过来）
    adc_special_function_config(_periph, ADC_CONTINUOUS_MODE, DISABLE);
    // ROVF 置位会让 ADC 停滞（手册 14.4.11），先清除；残留 EOC 一并排空
    adc_flag_clear(_periph, ADC_FLAG_ROVF);
    if (adc_flag_get(_periph, ADC_FLAG_EOC)) {
        for (;;) {
            (void)adc_routine_data_read(_periph);
            adc_flag_clear(_periph, ADC_FLAG_EOC);
            adc_flag_clear(_periph, ADC_FLAG_ROVF);
            uint32_t t = 0x800;
            while (t-- && !adc_flag_get(_periph, ADC_FLAG_EOC)) {}
            if (!adc_flag_get(_periph, ADC_FLAG_EOC)) break;
        }
    }

    // 恢复完整扫描序列（read_raw 会遗留长度=1，必须重排回全部通道）
    for (uint8_t i = 0; i < _cfg.channel_count && i < 8; i++) {
        adc_routine_channel_config(_periph, i,
                                   _cfg.channels[i].channel,
                                   _cfg.channels[i].sample_time);
    }
    adc_channel_length_config(_periph, ADC_ROUTINE_CHANNEL, _cfg.channel_count);

    // DMA 通道：外设→内存，16bit，内存递增，循环模式
    _dma->set_width(DMA_PERIPH_WIDTH_16BIT);
    _dma->set_direction(DMA_PERIPH_TO_MEMORY);
    _dma->set_increment(false, true);
    _dma->set_circular(true);
    _dma->set_subperipheral(_cfg.dma_sub_periph);

    // ⚠️ ADC_RDATA 宏是解引用值，必须取地址：&ADC_RDATA(periph) = 外设地址 + 0x40
    _dma->start((void *)&ADC_RDATA(_periph), buf,
                (uint32_t)_cfg.channel_count * frames);

    // 对齐官方例程 ADC0_routine_sequence_with_DMA 与师傅 BSP 的启动顺序:
    // 清全部标志 → 开 DMA 请求 → 进连续模式 → DDM=1 → 软件触发
    // EOCM=0（官方例程默认）：DMA 请求在序列末发出，DMA 连续搬完整个序列
    ADC_CTL1(_periph) &= ~ADC_CTL1_EOCM;
    adc_flag_clear(_periph, ADC_FLAG_WDE | ADC_FLAG_EOC | ADC_FLAG_EOIC |
                           ADC_FLAG_STIC | ADC_FLAG_STRC | ADC_FLAG_ROVF);
    adc_dma_mode_enable(_periph);
    adc_special_function_config(_periph, ADC_CONTINUOUS_MODE, ENABLE);
    // DDM=1：序列最后一次转换完成才发 DMA 请求（师傅单 ADC 模式同款，
    // 避免连续模式下 EOC 保持置位导致后续请求丢失）
    adc_dma_request_after_last_enable(_periph);
    adc_software_trigger_enable(_periph, ADC_ROUTINE_CHANNEL);
    return true;
}

bool adc_port::dma_done() const
{
    if (!_dma) return false;
    return _dma->done();
}

void adc_port::dma_stop()
{
    if (!_initialized || !_dma) return;
    _dma->stop();
    adc_dma_request_after_last_disable(_periph);   // 关 DDM，恢复每次转换即 EOC
    adc_dma_mode_disable(_periph);                 // 关 DMA 请求，恢复 scan/read_raw 可用
    ADC_CTL1(_periph) |= ADC_CTL1_EOCM;            // 恢复 EOCM=1，轮询路径按每次转换取 EOC
    adc_special_function_config(_periph, ADC_CONTINUOUS_MODE, DISABLE);  // 退出连续，轮询路径确定性
    // 恢复完整扫描序列，供后续 scan_* 直接使用
    if (_cfg.scan_enable) {
        for (uint8_t i = 0; i < _cfg.channel_count && i < 8; i++) {
            adc_routine_channel_config(_periph, i,
                                       _cfg.channels[i].channel,
                                       _cfg.channels[i].sample_time);
        }
        adc_channel_length_config(_periph, ADC_ROUTINE_CHANNEL, _cfg.channel_count);
    }
}
