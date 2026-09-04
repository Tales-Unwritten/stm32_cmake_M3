// ============================================================
// @platform STM32F103xx（从 GD32F4xx 版本移植，基于 STM32F1xx HAL ADC 库）
//   参考实现：师傅 STM32F10x 工程 ADC_BSP（王海涛，标准库版）的
//   阻塞轮询/ DMA 启动语义，寄存器操作全部交由 HAL 完成。
//
// 与 GD32F4xx 版本的关键实现差异（公开 API 除字段裁剪外未变）：
//   1. 全部使用 HAL ADC API（Init/ConfigChannel/Start/PollForConversion/
//      Stop/Start_DMA/Stop_DMA/Ex_Calibration），不再直接操作寄存器。
//   2. DMA 模式走 HAL 官方中断链：HAL_ADC_Start_DMA 内部 HAL_DMA_Start_IT，
//      完成中断由 inter_dma 内置 ISR → HAL_DMA_IRQHandler → ADC_DMAConvCplt
//      置 hadc 状态位 REG_EOC；dma_done() 据此判定（首轮完成）。
//   3. DMA 通道资源来自 inter_dma 的 dma_channel（M2 HAL 托管模式），
//      与 GD32 版使用 dma_channel 的结构保持一致。
//   4. F1 无 EOCM 位：轮询路径统一"单通道序列（L=0）+ 软件触发"逐通道
//      转换（read_raw 同语义）；硬件序列扫描仅 DMA 模式使用。
//   5. ADC2 无 DMA 能力、ADC1→DMA1_Ch1、ADC3→DMA2_Ch5（硬件固定映射）。
//   6. 分辨率/对齐无配置位（固定 12bit 右对齐），read_mv 恒按 4095 满量程。
// ============================================================

#include "inter_adc.hpp"

#include <new>

static ADC_TypeDef *_adc_ptr(adc_id id)
{
    switch (id)
    {
    case adc_id::adc1:
        return ADC1;
    case adc_id::adc2:
        return ADC2;
    default:
        return ADC3;
    }
}

static uint32_t _adc_clk_bit(adc_id id)
{
    switch (id)
    {
    case adc_id::adc1:
        return RCC_APB2ENR_ADC1EN;
    case adc_id::adc2:
        return RCC_APB2ENR_ADC2EN;
    default:
        return RCC_APB2ENR_ADC3EN;
    }
}

/** @brief 返回该 DMA 映射对应的中断号（仅内置 ISR 的两个组合有效） */
static IRQn_Type _dma_irqn(dma_id ctrl, uint8_t channel)
{
    if (ctrl == dma_id::dma1 && channel == 0)
        return DMA1_Channel1_IRQn;
    if (ctrl == dma_id::dma2 && channel == 4)
        return DMA2_Channel4_5_IRQn;
    return (IRQn_Type)-1; // NVIC 无效值，调用方先经 _dma_map_ok() 过滤
}

static bool _dma_map_ok(dma_id ctrl, uint8_t channel)
{
    return (ctrl == dma_id::dma1 && channel == 0) || (ctrl == dma_id::dma2 && channel == 4);
}

adc_port::adc_port(const AdcPortConfig &cfg) : _cfg(cfg), _ch_pins{}, _dma(nullptr), _initialized(false)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (i < cfg.channel_count && cfg.channels[i].port != nullptr)
        {
            _ch_pins[i] = new (_ch_storage[i].data) io_ctrl(cfg.channels[i].port, cfg.channels[i].pin);
        }
    }
    if (cfg.use_dma)
    {
        _dma = new (_dma_storage.data) dma_channel({cfg.dma_controller, cfg.dma_channel, cfg.dma_priority});
    }
}

adc_port::~adc_port()
{
    deinit();
}

void adc_port::_enable_clock()
{
    SET_BIT(RCC->APB2ENR, _adc_clk_bit(_cfg.periph));
    // 外设时钟门控使能后短暂延时（对齐 HAL 内部 tmpreg 技巧）
    for (uint32_t d = 0; d < 8; d++)
    {
        (void)RCC->CFGR;
    }
}

/** @brief 切换规则序列长度与连续模式（L/CONT/SCAN 位经 HAL_ADC_Init 重写）。
 *         要求 ADC 处于 READY（转换路径调用前均已 Stop）。 */
void adc_port::_set_mode(uint8_t n, bool cont)
{
    const bool scan = n > 1;
    if (_hadc.Init.NbrOfConversion == n && _hadc.Init.ContinuousConvMode == (cont ? ENABLE : DISABLE) &&
        _hadc.Init.ScanConvMode == (scan ? ENABLE : DISABLE))
    {
        return; // 模式未变，避免无谓重写
    }
    _hadc.Init.NbrOfConversion = n;
    _hadc.Init.ContinuousConvMode = cont ? ENABLE : DISABLE;
    _hadc.Init.ScanConvMode = scan ? ENABLE : DISABLE;
    (void)HAL_ADC_Init(&_hadc);
}

void adc_port::init()
{
    if (_initialized)
        return;

    _hadc.Instance = _adc_ptr(_cfg.periph);
    _enable_clock();

    // ADC 时钟分频（RCC.CFGR.ADCPRE，3 个 ADC 共享；72MHz PCLK2 下必须 ≤14MHz）
    MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, _cfg.clock_div);

    // 温度传感器(CH16)/内部参考(CH17)：仅 ADC1 有效（HAL 无 TSVREFE API，置一次位）
    if (_cfg.enable_temp_vref && _cfg.periph == adc_id::adc1)
    {
        SET_BIT(_hadc.Instance->CR2, ADC_CR2_TSVREFE);
    }

    // GPIO: 模拟模式（SMPR/rank 由每次 ConfigChannel 惰性配置，无残留）
    for (uint8_t i = 0; i < _cfg.channel_count && i < 8; i++)
    {
        if (_ch_pins[i] == nullptr && _cfg.channels[i].port != nullptr)
        {
            _ch_pins[i] = new (_ch_storage[i].data) io_ctrl(_cfg.channels[i].port, _cfg.channels[i].pin);
        }
        if (_ch_pins[i])
        {
            _ch_pins[i]->init(mode_analog, nopull);
        }
    }

    // HAL ADC 基础配置（默认轮询单通道模式；DMA/多通道由 _set_mode 切换）
    _hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    _hadc.Init.ScanConvMode = DISABLE;
    _hadc.Init.ContinuousConvMode = DISABLE;
    _hadc.Init.NbrOfConversion = 1;
    _hadc.Init.DiscontinuousConvMode = DISABLE;
    _hadc.Init.NbrOfDiscConversion = 0;
    _hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    if (HAL_ADC_Init(&_hadc) != HAL_OK)
        return;

    // F1 校准流程（HAL 内部：断电→上电→延时→RSTCAL→CAL）
    if (HAL_ADCEx_Calibration_Start(&_hadc) != HAL_OK)
        return;
    // 校准后断电复位到 READY，后续轮询/DMA 由 HAL_ADC_Start 自行上电
    (void)HAL_ADC_Stop(&_hadc);

    // DMA 模式：仅初始化 DMA 通道（绑定 Instance + 注册 ISR 槽）；通道配置延后到 dma_start
    if (_cfg.use_dma)
    {
        _dma->init();
    }
    _initialized = true;
}

void adc_port::deinit()
{
    if (!_initialized)
        return;

    if (_hadc.State != HAL_ADC_STATE_READY)
    {
        if (_cfg.use_dma && _dma)
        {
            (void)HAL_ADC_Stop_DMA(&_hadc); // 停 ADC + Abort DMA
            HAL_NVIC_DisableIRQ(_dma_irqn(_cfg.dma_controller, _cfg.dma_channel));
        }
        else
        {
            (void)HAL_ADC_Stop(&_hadc);
        }
    }

    if (_cfg.use_dma && _dma)
    {
        _dma->deinit();
        _dma->~dma_channel();
        _dma = nullptr;
    }

    for (uint8_t i = 0; i < _cfg.channel_count && i < 8; i++)
    {
        if (_ch_pins[i])
        {
            _ch_pins[i]->deinit();
            _ch_pins[i]->~io_ctrl();
            _ch_pins[i] = nullptr;
        }
    }
    _initialized = false;
}

void adc_port::calibrate()
{
    if (!_initialized)
        return;
    // 前置要求：无转换进行中（DMA 运行中需先 dma_stop）
    if (_hadc.State != HAL_ADC_STATE_READY)
        return;
    (void)HAL_ADCEx_Calibration_Start(&_hadc);
    (void)HAL_ADC_Stop(&_hadc);
}

uint16_t adc_port::_convert_one(uint8_t index)
{
    // 轮询单通道：序列长度=1、非连续、非扫描；软件触发；读后断电复位。
    // 注意：序列 L=0 时硬件只转换 SQ1（rank1），此处必须固定 Rank=1，
    //       不能使用配置的 rank_t（rank_t 仅作用于 dma_start 的多通道序列排布）。
    _set_mode(1, false);

    ADC_ChannelConfTypeDef cc{};
    cc.Channel = _cfg.channels[index].channel;
    cc.Rank = ADC_REGULAR_RANK_1;
    cc.SamplingTime = _cfg.channels[index].sample_time;
    if (HAL_ADC_ConfigChannel(&_hadc, &cc) != HAL_OK)
        return 0;

    if (HAL_ADC_Start(&_hadc) != HAL_OK)
        return 0;
    if (HAL_ADC_PollForConversion(&_hadc, 50) != HAL_OK)
    { // 50ms 超时保护
        (void)HAL_ADC_Stop(&_hadc);
        return 0;
    }
    const uint16_t value = (uint16_t)HAL_ADC_GetValue(&_hadc); // 读 DR 自动清 EOC
    (void)HAL_ADC_Stop(&_hadc);
    return value;
}

uint16_t adc_port::read_raw(uint8_t ch_index)
{
    if (!_initialized || ch_index >= _cfg.channel_count)
        return 0;
    return _convert_one(ch_index);
}

uint32_t adc_port::read_mv(uint8_t ch_index)
{
    // F1 固定 12bit 右对齐，满量程恒为 4095
    return (uint32_t)read_raw(ch_index) * _cfg.vref_mv / 4095U;
}

void adc_port::scan_raw(uint16_t *results)
{
    if (!_initialized || !results)
        return;
    // F1 无 EOCM 位，硬件扫描无法逐通道轮询 → 逐通道软件转换（结果一致）
    for (uint8_t i = 0; i < _cfg.channel_count; i++)
    {
        results[i] = _convert_one(i);
    }
}

void adc_port::scan_mv(uint32_t *results)
{
    if (!_initialized || !results)
        return;

    uint16_t raw[8];
    scan_raw(raw);
    for (uint8_t i = 0; i < _cfg.channel_count; i++)
    {
        results[i] = (uint32_t)raw[i] * _cfg.vref_mv / 4095U;
    }
}

// ============================================================
//  DMA 连续采集（HAL 官方链：Start_DMA + 中断 → REG_EOC 判定）
//   通道资源 = inter_dma::dma_channel（M2 HAL 托管）
// ============================================================

bool adc_port::dma_start(uint16_t *buf, uint16_t frames)
{
    if (!_initialized || !buf || !_dma || frames == 0)
        return false;
    if (_cfg.channel_count == 0)
        return false;
    if (_cfg.periph == adc_id::adc2)
        return false; // F1：ADC2 无 DMA 能力
    if (!_dma_map_ok(_cfg.dma_controller, _cfg.dma_channel))
        return false; // 无内置 ISR 的映射

    // 可重入：若上次转换/DMA 仍在进行，先复位到 READY
    if (_hadc.State != HAL_ADC_STATE_READY)
    {
        (void)HAL_ADC_Stop(&_hadc);
    }

    // 多通道硬件扫描序列 + 连续模式（L=n-1 / SCAN / CONT，经 HAL_ADC_Init 写入）
    _set_mode(_cfg.channel_count, true);
    for (uint8_t i = 0; i < _cfg.channel_count && i < 8; i++)
    {
        ADC_ChannelConfTypeDef cc{};
        cc.Channel = _cfg.channels[i].channel;
        cc.Rank = _cfg.channels[i].rank_t;
        cc.SamplingTime = _cfg.channels[i].sample_time;
        if (HAL_ADC_ConfigChannel(&_hadc, &cc) != HAL_OK)
            return false;
    }

    // DMA 通道 M2 托管：缓存配置 → HAL_DMA_Init（Start_DMA 内部 Start_IT 的 CCR 前提）
    _dma->set_width(DMA_PDATAALIGN_HALFWORD);
    _dma->set_direction(DMA_PERIPH_TO_MEMORY);
    _dma->set_increment(false, true);
    _dma->set_circular(true);
    if (_dma->hal_configure() != HAL_OK)
        return false;
    __HAL_LINKDMA(&_hadc, DMA_Handle, *_dma->hal_handle()); // 互链 hadc ↔ dma handle

    // 开启 DMA 中断（ISR 由 inter_dma 内置，路由回本通道）
    const IRQn_Type irqn = _dma_irqn(_cfg.dma_controller, _cfg.dma_channel);
    HAL_NVIC_EnableIRQ(irqn);

    // 启动：内部完成 清标志 → CR2.DMA=1 → HAL_DMA_Start_IT → SWSTART|EXTTRIG 触发
    if (HAL_ADC_Start_DMA(&_hadc, (uint32_t *)buf, (uint32_t)_cfg.channel_count * frames) != HAL_OK)
    {
        HAL_NVIC_DisableIRQ(irqn);
        return false;
    }
    return true;
}

bool adc_port::dma_done() const
{
    if (!_dma)
        return false;
    // Start_DMA 起始清除 REG_EOC；首轮 DMA 完成中断（ADC_DMAConvCplt）置位后保持
    return (_hadc.State & HAL_ADC_STATE_REG_EOC) != 0u;
}

void adc_port::dma_stop()
{
    if (!_initialized || !_dma)
        return;
    if (_hadc.State != HAL_ADC_STATE_READY)
    {
        (void)HAL_ADC_Stop_DMA(&_hadc); // 停 ADC + DMA Abort + 状态复位 READY
    }
    HAL_NVIC_DisableIRQ(_dma_irqn(_cfg.dma_controller, _cfg.dma_channel));

    // 复位为轮询单通道模式（后续 read_raw/scan_raw 直接可用）
    _set_mode(1, false);
}
