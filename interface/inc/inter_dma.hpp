#pragma once
// ============================================================
// @platform STM32F103xx（STM32F10xxx / Cortex-M3，基于 STM32F1xx HAL 库）
// ============================================================

#ifdef __cplusplus

#include "stm32f1xx_hal.h" // 必须从 hal.h 顶层进入（直接 include 具体外设头会触发循环包含，typedef 未就绪）
#include <cstdint>

// 控制器编号：dma1 ↔ STM32 DMA1（7 通道），dma2 ↔ STM32 DMA2（5 通道）
enum class dma_id : uint8_t
{
    dma1,
    dma2
};
enum dma_priority_t : uint32_t
{
    _low = DMA_PRIORITY_LOW,
    _medium = DMA_PRIORITY_MEDIUM,
    _high = DMA_PRIORITY_HIGH,
    _very_high = DMA_PRIORITY_VERY_HIGH
};

struct DmaConfig
{
    dma_id controller; ///< dma1 = DMA1 / dma2 = DMA2
    uint8_t channel;   ///< 通道号（0-based）：DMA1 0~6（7 通道）/ DMA2 0~4（5 通道）
    uint32_t priority; ///< DMA_PRIORITY_LOW / MEDIUM / HIGH / VERY_HIGH
};

/**
 * @brief DMA 通道（阻塞 M2M + 外设 DMA 基础配置）
 *
 * 两种运行模式（互斥，按需选择其一）：
 *
 *  M1 自管理轮询（本类直接控制传输）：
 *   static dma_channel dma({dma_id::dma1, 0, DMA_PRIORITY_MEDIUM});
 *   dma.init();
 *   dma.set_width(DMA_PDATAALIGN_BYTE);
 *   dma.set_direction(DMA_MEMORY_TO_MEMORY);
 *   dma.set_increment(true, true);
 *   dma.start(src, dst, 64);
 *   while (!dma.done());
 *
 *  M2 HAL 托管（把通道借给 HAL 栈，如 HAL_ADC_Start_DMA 的官方中断链）：
 *   dma.init();
 *   dma.set_width(DMA_PDATAALIGN_HALFWORD);
 *   dma.set_direction(DMA_PERIPH_TO_MEMORY);
 *   dma.set_increment(false, true);
 *   dma.set_circular(true);
 *   dma.hal_configure();                          // 缓存配置 → HAL_DMA_Init
 *   __HAL_LINKDMA(&hadc, DMA_Handle, dma.hal_handle());
 *   HAL_NVIC_EnableIRQ(...);
 *   HAL_ADC_Start_DMA(&hadc, buf, len);           // HAL 内部 HAL_DMA_Start_IT
 *   ...                                            // 中断由 inter_dma 内置 ISR 路由
 *   HAL_ADC_Stop_DMA(&hadc);                      // HAL 内部 Abort，通道回 READY
 *   M2 期间不得调用 start()/stop()/deinit()（通道控制权在 HAL）；
 *   结束后可回到 M1 或再次 M2。
 *
 * @note 与 GD32 版差异：F1 的 DMA 请求源由通道号硬件固定（如 ADC1→DMA1_Ch1），
 *       选择通道即选择请求源，无软件选择寄存器，故 set_subperipheral() 已裁剪。
 */
class dma_channel
{
  public:
    explicit dma_channel(const DmaConfig &cfg);
    ~dma_channel();

    dma_channel(const dma_channel &) = delete;
    dma_channel &operator=(const dma_channel &) = delete;

    void init();
    void deinit();
    [[nodiscard]] bool is_initialized() const noexcept
    {
        return _initialized;
    }

    // ── 传输配置（在 start / hal_configure 之前调用） ──────
    void set_width(uint32_t width);   // DMA_PDATAALIGN_BYTE / HALFWORD / WORD（外设与内存同宽）
    void set_direction(uint32_t dir); // DMA_MEMORY_TO_MEMORY / PERIPH_TO_MEMORY / MEMORY_TO_PERIPH
    void set_increment(bool src_inc, bool dst_inc);
    void set_circular(bool enable); // 循环模式（ADC 连续采集等）

    // ── M1：自管理传输控制 ───────────────────────────────
    void start(void *src, void *dst, uint32_t count);
    void stop();
    [[nodiscard]] bool done() const;

    // ── M2：HAL 托管模式（把通道交给 HAL 栈，如 ADC DMA） ─
    /** @brief 缓存配置写入 handle.Init 并执行 HAL_DMA_Init（Start_IT 前置） */
    HAL_StatusTypeDef hal_configure();
    /** @brief 暴露内部 HAL 句柄（配合 __HAL_LINKDMA 挂到外设 handle） */
    DMA_HandleTypeDef *hal_handle() noexcept
    {
        return &_handle;
    }

  private:
    void _enable_clock();
    HAL_StatusTypeDef _apply_config(); // 组装 Init + HAL_DMA_Init（M1/M2 共用）

    DmaConfig _cfg;
    bool _initialized;
    DMA_HandleTypeDef _handle{}; // F1 HAL 句柄（Instance 在 init() 时按通道号绑定）

    // 缓存的配置
    uint32_t _width;
    uint32_t _dir;
    bool _src_inc;
    bool _dst_inc;
    bool _circular;
};

#endif
