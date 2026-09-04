// ============================================================
// @platform STM32F103xx（从 GD32F4xx 版本移植，使用 HAL 库）
//
// 与 GD32F4xx 版本的关键实现差异（公开 API 除裁剪项外未变）：
//   1. 通道寻址：GD32 用 "DMA_CH0 + n" 连续枚举；F1 无该宏，改为显式
//      映射表（DMA1_Channel1..7 / DMA2_Channel1..5，通道号 0-based）。
//   2. 参数模型：GD32 在 start() 组装 dma_single_data_parameter_struct 后
//      一次性 init；F1 HAL 把配置挂在 DMA_HandleTypeDef.Init 上，由
//      HAL_DMA_Init() 写入 CCR，HAL_DMA_Start() 只负责地址/长度与使能。
//   3. 数据宽度：GD32 单一 periph_memory_width 字段；F1 分为外设/内存两个
//      字段，set_width() 语义保持"两侧同宽"。
//   4. 请求源选择：GD32F4xx 用 dma_channel_subperipheral_select()；F1 无此
//      概念（请求源由通道号硬件固定），set_subperipheral() 已从 API 裁剪。
//   5. 完成标志：GD32 查 FTF；F1 读 TC 标志（HAL_DMA_PollForTransfer 不支持
//      循环模式，故此处自行查标志而非调用它）。
// ============================================================

#include "inter_dma.hpp"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma_ex.h" // __HAL_DMA_GET_FLAG / __HAL_DMA_GET_TC_FLAG_INDEX（高密度分支含 DMA2）

// F1 高密度（STM32F103xE）：DMA1 共 7 通道、DMA2 共 5 通道
static DMA_Channel_TypeDef *_channel_of(dma_id ctrl, uint8_t channel)
{
    if (ctrl == dma_id::dma1)
    {
        static DMA_Channel_TypeDef *const kMap[] = {
            DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6, DMA1_Channel7,
        };
        if (channel >= sizeof(kMap) / sizeof(kMap[0]))
            return nullptr;
        return kMap[channel];
    }
    static DMA_Channel_TypeDef *const kMap[] = {
        DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4, DMA2_Channel5,
    };
    if (channel >= sizeof(kMap) / sizeof(kMap[0]))
        return nullptr;
    return kMap[channel];
}

// F1 的 PSIZE/MSIZE 位域位置不同（PSIZE@bit9:8、MSIZE@bit11:10），两组 HAL 宏值
// 不能混用；set_width() 传入 PDATAALIGN 编码，内存侧需换算为对应的 MDATAALIGN 编码
static uint32_t _mem_align(uint32_t periph_align)
{
    if (periph_align == DMA_PDATAALIGN_HALFWORD)
        return DMA_MDATAALIGN_HALFWORD;
    if (periph_align == DMA_PDATAALIGN_WORD)
        return DMA_MDATAALIGN_WORD;
    return DMA_MDATAALIGN_BYTE;
}

dma_channel::dma_channel(const DmaConfig &cfg)
    : _cfg(cfg), _initialized(false), _width(DMA_PDATAALIGN_BYTE), _dir(DMA_MEMORY_TO_MEMORY), _src_inc(true),
      _dst_inc(true), _circular(false)
{
}

dma_channel::~dma_channel()
{
    deinit();
}

void dma_channel::_enable_clock()
{
    if (_cfg.controller == dma_id::dma1)
        __HAL_RCC_DMA1_CLK_ENABLE();
    else
        __HAL_RCC_DMA2_CLK_ENABLE();
}

// ── M2（HAL 托管）中断路由表 ──────────────────────────────
// 仅 ADC 相关通道提供强符号 ISR（覆盖 startup weak 向量）：
//   [0] DMA1_Ch1（ADC1）、[1] DMA2_Ch5（ADC3，与 Ch4 共用中断向量）
// 同一通道同时只允许一个 dma_channel 实例处于 M2 模式（硬件约束）。
static dma_channel *s_isr_owner[2] = {nullptr, nullptr};

extern "C" void DMA1_Channel1_IRQHandler(void)
{
    if (s_isr_owner[0] != nullptr)
        HAL_DMA_IRQHandler(s_isr_owner[0]->hal_handle());
}

extern "C" void DMA2_Channel4_5_IRQHandler(void)
{
    if (s_isr_owner[1] != nullptr)
        HAL_DMA_IRQHandler(s_isr_owner[1]->hal_handle());
}

void dma_channel::init()
{
    if (_initialized)
        return;
    DMA_Channel_TypeDef *ch = _channel_of(_cfg.controller, _cfg.channel);
    if (ch == nullptr)
        return; // 通道号越界（DMA1 > 6 / DMA2 > 4）：保持未初始化，后续操作安全跳过
    _handle.Instance = ch;
    _enable_clock();
    _initialized = true;

    // 注册 ISR 槽（仅 M2 相关通道；M1 poll 用法不依赖中断，注册亦无副作用）
    if (_cfg.controller == dma_id::dma1 && _cfg.channel == 0)
        s_isr_owner[0] = this;
    else if (_cfg.controller == dma_id::dma2 && _cfg.channel == 4)
        s_isr_owner[1] = this;
}

void dma_channel::deinit()
{
    if (!_initialized)
        return;
    (void)HAL_DMA_DeInit(&_handle); // 停通道 + 复位 CCR/CNDTR/CPAR/CMAR + 清全部标志
    _initialized = false;
    if (s_isr_owner[0] == this)
        s_isr_owner[0] = nullptr;
    else if (s_isr_owner[1] == this)
        s_isr_owner[1] = nullptr;
}

void dma_channel::set_width(uint32_t width)
{
    _width = width;
}
void dma_channel::set_direction(uint32_t dir)
{
    _dir = dir;
}
void dma_channel::set_increment(bool s, bool d)
{
    _src_inc = s;
    _dst_inc = d;
}
void dma_channel::set_circular(bool enable)
{
    _circular = enable;
}

HAL_StatusTypeDef dma_channel::_apply_config()
{
    // 缓存配置 → HAL Init 字段（CCR 在 HAL_DMA_Init 中写入）
    _handle.Init.Direction = _dir;
    _handle.Init.PeriphDataAlignment = _width;
    _handle.Init.MemDataAlignment = _mem_align(_width); // PSIZE/MSIZE 位域位置不同，需换算
    _handle.Init.Priority = _cfg.priority;
    _handle.Init.Mode = _circular ? DMA_CIRCULAR : DMA_NORMAL;

    // 与 GD32 版语义一致：仅 M2M 时外设侧按 _src_inc 递增（外设充当源）；
    // P2M / M2P 中外设地址固定不递增，_src_inc 只作用于内存侧
    if (_dir == DMA_MEMORY_TO_MEMORY)
    {
        _handle.Init.PeriphInc = _src_inc ? DMA_PINC_ENABLE : DMA_PINC_DISABLE;
        _handle.Init.MemInc = _dst_inc ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
    }
    else if (_dir == DMA_PERIPH_TO_MEMORY)
    {
        _handle.Init.PeriphInc = DMA_PINC_DISABLE;
        _handle.Init.MemInc = _dst_inc ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
    }
    else
    { // DMA_MEMORY_TO_PERIPH
        _handle.Init.PeriphInc = DMA_PINC_DISABLE;
        _handle.Init.MemInc = _src_inc ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
    }

    return HAL_DMA_Init(&_handle); // 写 CCR
}

void dma_channel::start(void *src, void *dst, uint32_t count)
{
    if (!_initialized || !src || !dst || count == 0)
        return;

    // 先停旧传输并把句柄复位到 READY（F1 HAL 要求 Start 前 State==READY；
    // Abort 在无传输时返回 HAL_ERROR，此处忽略，仅取其"停通道 + 清标志"作用）
    (void)HAL_DMA_Abort(&_handle);

    (void)_apply_config();
    (void)HAL_DMA_Start(&_handle, (uint32_t)src, (uint32_t)dst, count); // 设地址/长度、清标志、使能
}

HAL_StatusTypeDef dma_channel::hal_configure()
{
    if (!_initialized)
        return HAL_ERROR;
    // 停旧传输并复位到 READY（HAL_ADC_Start_DMA 内部 Start_IT 要求 State==READY）
    (void)HAL_DMA_Abort(&_handle);
    return _apply_config();
}

void dma_channel::stop()
{
    if (!_initialized)
        return;
    // 停通道 + 清标志 + State→READY（比 GD32 版多清标志一步：stop() 后 done()
    // 返回 false，语义更直观；下次 start() 前无需额外复位）
    (void)HAL_DMA_Abort(&_handle);
}

bool dma_channel::done() const
{
    if (!_initialized)
        return true;
    // normal 模式：传输完成时硬件自动清 EN，TC 标志保持置位（下次 start 清标志）；
    // 循环模式：每完成一轮 TC 置位 —— 语义与 GD32 版读 FTF 一致
    return __HAL_DMA_GET_FLAG(&_handle, __HAL_DMA_GET_TC_FLAG_INDEX(&_handle)) != RESET;
}
