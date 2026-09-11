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
 *   或一步到位（自动选宽度/分片/补尾字节，阻塞到完成）：
 *   dma.copy_memory(dst, src, 512);
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
 *   M2 期间不得调用 start()/stop()/copy_memory()/deinit()（通道控制权在 HAL）；
 *   结束后可回到 M1 或再次 M2。
 *
 * @note 中断路由：inter_dma 已为 DMA1_Ch1~7、DMA2_Ch1~5 全部 12 个通道提供强符号 ISR
 *       （DMA2_Ch4/Ch5 共用一个向量），统一转发给 HAL_DMA_IRQHandler。
 *       外设侧只需自己 HAL_NVIC_EnableIRQ(对应通道中断) 即可，不用再写 IRQHandler。
 *       一个硬件通道只能有一个 dma_channel 实例（后 init 者覆盖路由槽位）。
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
    /** @brief 启动一次传输（写入缓存的配置 + HAL_DMA_Start 装载地址/长度并使能）
     *  @return HAL_OK 已启动；HAL_ERROR 未初始化或参数非法；HAL_BUSY HAL 状态非 READY */
    HAL_StatusTypeDef start(void *src, void *dst, uint32_t count);
    void stop();
    [[nodiscard]] bool done() const;

    /**
     * @brief 当前传输剩余的数据单元个数（读 CNDTR，宽度无关，不是字节数）
     *
     * 定长接收/发送过程中随时可读；循环模式下每圈从 count 递减到 0 再回绕。
     * 典型用法（串口不定长接收：循环模式 + IDLE 中断）：
     *   dma.set_circular(true);
     *   dma.start((void *)&USART1->DR, rx_buf, sizeof(rx_buf));
     *   USART1->CR3 |= USART_CR3_DMAR;
     *   // IDLE 中断里：本次已收字节数 =
     *   uint16_t got = sizeof(rx_buf) - dma.remaining();
     */
    [[nodiscard]] uint32_t remaining() const;

    /**
     * @brief 内存→内存搬运（M2M，阻塞到完成）
     *
     * 相比手写 start()/done()，本接口代劳了三件容易出错的事：
     *   1. 宽度按源/目的地址的公共对齐自动选择（WORD/HALFWORD/BYTE）——F1 的 DMA
     *      不做非对齐访问，宽度选大于实际对齐会搬出错位数据；
     *   2. 单个数据单元数超过 65535（CNDTR 为 16 位）时自动分片；
     *   3. 尾部不足一个数据单元的字节由 CPU 补齐。
     *
     * @param dst 目的地址  @param src 源地址  @param bytes 字节数（非 0）
     * @param timeout_ms 单片超时（毫秒），默认 100
     * @return HAL_OK 成功 / HAL_ERROR 未初始化或参数非法 / HAL_TIMEOUT 超时
     * @note 搬运全程轮询 TC 标志（不需要中断）；调用后通道停在 IDLE（EN=0），
     *       可继续 start()/copy_memory()，或切到 M2 交回 HAL。
     */
    HAL_StatusTypeDef copy_memory(void *dst, const void *src, uint32_t bytes, uint32_t timeout_ms = 100);

    // ── M2：HAL 托管模式（把通道交给 HAL 栈，如 ADC DMA） ─
    /** @brief 缓存配置写入 handle.Init 并执行 HAL_DMA_Init（Start_IT 前置） */
    HAL_StatusTypeDef hal_configure();
    /** @brief 暴露内部 HAL 句柄（配合 __HAL_LINKDMA 挂到外设 handle） */
    DMA_HandleTypeDef *hal_handle() noexcept
    {
        return &_handle;
    }

    // ── 平台映射查询（供外设层配 NVIC，避免各自再抄一张表） ──

    // IRQn_Type 取值域为 -64..63（F103xE 真实最大向量 DMA2_Channel4_5 = 59），
    // 哨兵取域内且不与任何真实向量冲突的 63。注意勿用 0x7F：它越出枚举值域，
    // GCC 能容忍但 clang 在常量求值时报错（constexpr 初始值非常量表达式）。
    static constexpr IRQn_Type IRQ_NONE = static_cast<IRQn_Type>(63); ///< 无效中断号哨兵

    /** @brief 通道对应的 NVIC 中断号（DMA2_Ch4/Ch5 返回同一向量）；越界返回 IRQ_NONE */
    static IRQn_Type irq_of(dma_id ctrl, uint8_t channel) noexcept;

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
