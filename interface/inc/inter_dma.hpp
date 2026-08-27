#pragma once
// ============================================================
// @platform GD32F4xx
// ============================================================

#ifdef __cplusplus

#include <cstdint>
#include "gd32f4xx.h"

enum class dma_id : uint8_t { dma0, dma1 };

struct DmaConfig {
    dma_id   controller;       ///< DMA0 / DMA1
    uint8_t  channel;          ///< 通道号 0~7
    uint32_t priority;         ///< DMA_PRIORITY_LOW / MEDIUM / HIGH / ULTRA_HIGH
};

/**
 * @brief DMA 通道（阻塞 M2M + 外设 DMA 基础配置）
 *
 *   static dma_channel dma({dma_id::dma0, 0, DMA_PRIORITY_MEDIUM});
 *   dma.init();
 *   dma.set_width(DMA_MEMORY_WIDTH_8BIT);
 *   dma.set_direction(DMA_MEMORY_TO_MEMORY);
 *   dma.set_increment(true, true);
 *   dma.start(src, dst, 64);
 *   while (!dma.done());
 */
class dma_channel {
public:
    explicit dma_channel(const DmaConfig &cfg);
    ~dma_channel();

    dma_channel(const dma_channel &) = delete;
    dma_channel &operator=(const dma_channel &) = delete;

    void init();
    void deinit();
    [[nodiscard]] bool is_initialized() const noexcept { return _initialized; }

    // ── 传输配置（在 start 之前调用） ──────────────────
    void set_width(uint32_t width);     // DMA_PERIPH_WIDTH_8/16/32BIT
    void set_direction(uint32_t dir);   // DMA_MEMORY_TO_MEMORY / PERIPH_TO_MEMORY / MEMORY_TO_PERIPH
    void set_increment(bool src_inc, bool dst_inc);
    void set_circular(bool enable);     // 循环模式（ADC 连续采集等）
    void set_subperipheral(uint32_t sub_periph);  // 外设请求源选择（dma_subperipheral_enum，如 ADC0=DMA0_CH0 SUBPERI4）

    // ── 传输控制 ──────────────────────────────────────
    void start(void *src, void *dst, uint32_t count);
    void stop();
    [[nodiscard]] bool done() const;

private:
    void _enable_clock();

    DmaConfig  _cfg;
    bool       _initialized;
    uint32_t   _periph;

    // 缓存的配置
    uint32_t   _width;
    uint32_t   _dir;
    bool       _src_inc;
    bool       _dst_inc;
    bool       _circular;
    uint32_t   _sub_periph;
};

#endif
