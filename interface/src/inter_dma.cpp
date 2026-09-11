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
//   6. CCR 残留：F1 的 HAL_DMA_Init() 清位掩码不含 DMA_CCR_MEM2MEM
//      （见 stm32f1xx_hal_dma.c），故 _apply_config() 在 Init 前精准清该位。
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

// M2M 搬运的数据宽度：取源/目的地址的公共对齐（宽度 > 实际对齐会搬出错位数据）。
// unit 带回该宽度对应的字节数，供调用方算数据单元个数
static uint32_t _width_for(uintptr_t src, uintptr_t dst, uint32_t &unit)
{
    if (((src | dst) & 0x3U) == 0U)
    {
        unit = 4U;
        return DMA_PDATAALIGN_WORD;
    }
    if (((src | dst) & 0x1U) == 0U)
    {
        unit = 2U;
        return DMA_PDATAALIGN_HALFWORD;
    }
    unit = 1U;
    return DMA_PDATAALIGN_BYTE;
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
// F1 高密度：DMA1 七个通道、DMA2 五个通道，每个硬件通道一个槽位。
// init() 时登记占用者，ISR 把中断转交给 HAL_DMA_IRQHandler（它内部判空回调，
// 无回调时只清标志），由 HAL 再调外设层注册的回调（如 ADC_DMAConvCplt）。
static dma_channel *s_isr_owner[2][7] = {}; // [0]=DMA1(7 通道)，[1]=DMA2(仅前 5 个有效)

static uint8_t _ctrl_index(dma_id ctrl)
{
    return (ctrl == dma_id::dma1) ? 0U : 1U;
}

static void _dma_irq_dispatch(dma_id ctrl, uint8_t channel)
{
    dma_channel *owner = s_isr_owner[_ctrl_index(ctrl)][channel];
    if (owner != nullptr)
    {
        HAL_DMA_IRQHandler(owner->hal_handle());
        return;
    }

    // 该通道中断被打开但没人登记（例如绕过 inter_dma、直接用 HAL 配的通道）：
    // 没有句柄可交，标志不清则 NVIC 会反复进中断形成风暴。这里清掉该通道标志
    // 兜底：DMA 传输本身不受影响，只是不再重复进中断。
    DMA_TypeDef *base = (ctrl == dma_id::dma1) ? DMA1 : DMA2;
    base->IFCR = (DMA_ISR_GIF1 << (4U * channel));
}

// 12 个强符号 ISR（覆盖 startup 的 weak 矢量）；DMA2_Ch4/Ch5 共用一个矢量，
// 两个槽位都分发一次（HAL_DMA_IRQHandler 只处理自己通道的标志，互不干扰）。
extern "C" void DMA1_Channel1_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma1, 0);
}
extern "C" void DMA1_Channel2_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma1, 1);
}
extern "C" void DMA1_Channel3_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma1, 2);
}
extern "C" void DMA1_Channel4_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma1, 3);
}
extern "C" void DMA1_Channel5_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma1, 4);
}
extern "C" void DMA1_Channel6_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma1, 5);
}
extern "C" void DMA1_Channel7_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma1, 6);
}

extern "C" void DMA2_Channel1_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma2, 0);
}
extern "C" void DMA2_Channel2_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma2, 1);
}
extern "C" void DMA2_Channel3_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma2, 2);
}
extern "C" void DMA2_Channel4_5_IRQHandler(void)
{
    _dma_irq_dispatch(dma_id::dma2, 3); // DMA2_Channel4
    _dma_irq_dispatch(dma_id::dma2, 4); // DMA2_Channel5（与 CH4 共用矢量）
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

    // 登记中断路由占用者（M2 用；M1 轮询/搬运不依赖中断，登记亦无副作用）
    s_isr_owner[_ctrl_index(_cfg.controller)][_cfg.channel] = this;
}

void dma_channel::deinit()
{
    if (!_initialized)
        return;
    (void)HAL_DMA_DeInit(&_handle); // 停通道 + 复位 CCR/CNDTR/CPAR/CMAR + 清全部标志
    _initialized = false;

    dma_channel **slot = &s_isr_owner[_ctrl_index(_cfg.controller)][_cfg.channel];
    if (*slot == this)
        *slot = nullptr;
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
    // F1 的 HAL_DMA_Init() 清位掩码只含 DIR/CIRC/PINC/MINC/PSIZE/MSIZE/PL，**不含**
    // MEM2MEM（见 stm32f1xx_hal_dma.c）：若本通道上一轮跑过 M2M，该位会残留，之后
    // 配成"外设→内存/内存→外设"时 DMA 会脱离外设请求自由狂奔（缓冲区瞬间填满）。
    // 这里只精准清这一位——不用 HAL_DMA_DeInit，因为它会连带清掉调用方注册的回调
    // （XferCpltCallback 等）和中断使能位，M1 自挂回调/自开中断的用法依赖它们。
    // 调用前通道已停（start()/hal_configure() 都先 Abort），故此处写 CCR 安全。
    _handle.Instance->CCR &= ~DMA_CCR_MEM2MEM;

    // 缓存配置 → HAL Init 字段（其余 CCR 位由 HAL_DMA_Init 写）
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

HAL_StatusTypeDef dma_channel::start(void *src, void *dst, uint32_t count)
{
    if (!_initialized || !src || !dst || count == 0)
        return HAL_ERROR;

    // 先停旧传输并把句柄复位到 READY（F1 HAL 要求 Start 前 State==READY；
    // Abort 在无传输时返回 HAL_ERROR，此处忽略，仅取其"停通道 + 清标志"作用）
    (void)HAL_DMA_Abort(&_handle);

    const HAL_StatusTypeDef st = _apply_config();
    if (st != HAL_OK)
        return st;

    return HAL_DMA_Start(&_handle, (uint32_t)src, (uint32_t)dst, count); // 装载地址/长度、清标志、使能
}

HAL_StatusTypeDef dma_channel::hal_configure()
{
    if (!_initialized)
        return HAL_ERROR;
    // 停旧传输并复位到 READY（HAL_ADC_Start_DMA 内部 Start_IT 要求 State==READY）
    (void)HAL_DMA_Abort(&_handle);
    return _apply_config();
}

HAL_StatusTypeDef dma_channel::copy_memory(void *dst, const void *src, uint32_t bytes, uint32_t timeout_ms)
{
    if (!_initialized || (dst == nullptr) || (src == nullptr) || (bytes == 0U))
        return HAL_ERROR;

    uint32_t unit = 1U;
    const uint32_t width = _width_for((uintptr_t)src, (uintptr_t)dst, unit);

    // 每次都重设配置：本通道上一次可能是别的用途（外设模式/别的宽度/循环）
    set_width(width);
    set_direction(DMA_MEMORY_TO_MEMORY);
    set_increment(true, true);
    set_circular(false);

    uint8_t *d = static_cast<uint8_t *>(dst);
    const uint8_t *s = static_cast<const uint8_t *>(src);

    // CNDTR 是 16 位：单片最多 65535 个数据单元，超出分片续传
    const uint32_t whole = bytes / unit * unit; // 能凑满整数个数据单元的字节数
    uint32_t moved = 0U;
    while (moved < whole)
    {
        const uint32_t remain_units = (whole - moved) / unit;
        const uint32_t chunk_units = (remain_units > 0xFFFFU) ? 0xFFFFU : remain_units;

        // start() 内部：Abort（复位到 READY）→ 重写 CCR → 装载地址/长度并使能。
        // 注意 HAL_DMA_Start 不开任何中断（只有 HAL_DMA_Start_IT 才开 TC/TE），
        // 所以本路径是纯轮询：TC 由硬件置位，done() 读它即可。
        if (start(const_cast<uint8_t *>(s) + moved, d + moved, chunk_units) != HAL_OK)
            return HAL_ERROR;

        const uint32_t t0 = HAL_GetTick();
        while (!done())
        {
            if ((uint32_t)(HAL_GetTick() - t0) > timeout_ms)
            {
                stop(); // 停通道 + 清标志（下一次调用可正常重启）
                return HAL_TIMEOUT;
            }
        }
        moved += chunk_units * unit;
    }

    // 尾部不足一个数据单元的字节（0 ~ unit-1 个）由 CPU 补齐
    for (uint32_t i = moved; i < bytes; i++)
        d[i] = s[i];

    return HAL_OK;
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

uint32_t dma_channel::remaining() const
{
    if (!_initialized)
        return 0U;
    // CNDTR 是"还剩多少个数据单元"（宽度无关）；循环模式下每圈从 count 递减回绕
    return (uint32_t)_handle.Instance->CNDTR;
}

IRQn_Type dma_channel::irq_of(dma_id ctrl, uint8_t channel) noexcept
{
    if (ctrl == dma_id::dma1)
    {
        static const IRQn_Type kMap[] = {DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, DMA1_Channel3_IRQn, DMA1_Channel4_IRQn,
                                         DMA1_Channel5_IRQn, DMA1_Channel6_IRQn, DMA1_Channel7_IRQn};
        return (channel < (sizeof(kMap) / sizeof(kMap[0]))) ? kMap[channel] : IRQ_NONE;
    }

    // DMA2_Ch4/Ch5 共用同一个向量
    static const IRQn_Type kMap2[] = {DMA2_Channel1_IRQn, DMA2_Channel2_IRQn, DMA2_Channel3_IRQn, DMA2_Channel4_5_IRQn,
                                      DMA2_Channel4_5_IRQn};
    return (channel < (sizeof(kMap2) / sizeof(kMap2[0]))) ? kMap2[channel] : IRQ_NONE;
}
