#pragma once
// ============================================================
// @platform STM32F103xx（STM32F10xxx / Cortex-M3，基于 STM32F1xx HAL 库）
//   实现采用 HAL ADC 官方 API（配置/校准/轮询/Start_DMA 中断链），
//   DMA 通道资源由 inter_dma 的 dma_channel 以 M2（HAL 托管）模式提供。
// ============================================================

#ifdef __cplusplus

#include "inter_dma.hpp"
#include "inter_io_ctrl.hpp"   // 引入 stm32f1xx_hal.h（必须先完整展开，hal_adc.h 依赖其 typedef）
#include "stm32f1xx_hal_adc.h" // ADC_CHANNEL_x / ADC_SAMPLETIME_x 参数宏与 HAL ADC 句柄（hal.h 展开后安全）
#include <cstdint>

// 外设编号：adc1 ↔ ADC1，adc2 ↔ ADC2，adc3 ↔ ADC3
enum class adc_id : uint8_t
{
    adc1,
    adc2,
    adc3
};

enum adc_rank_t : uint32_t
{
    rank1 = ADC_REGULAR_RANK_1,
    rank2 = ADC_REGULAR_RANK_2,
    rank3 = ADC_REGULAR_RANK_3,
    rank4 = ADC_REGULAR_RANK_4,
    rank5 = ADC_REGULAR_RANK_5,
    rank6 = ADC_REGULAR_RANK_6,
    rank7 = ADC_REGULAR_RANK_7,
    rank8 = ADC_REGULAR_RANK_8,
    rank9 = ADC_REGULAR_RANK_9,
    rank10 = ADC_REGULAR_RANK_10,
    rank11 = ADC_REGULAR_RANK_11,
    rank12 = ADC_REGULAR_RANK_12,
    rank13 = ADC_REGULAR_RANK_13,
    rank14 = ADC_REGULAR_RANK_14,
    rank15 = ADC_REGULAR_RANK_15,
    rank16 = ADC_REGULAR_RANK_16,
};

// ============================================================
//  通道配置
// ============================================================

struct AdcChannelConfig
{
    GPIO_TypeDef *port = nullptr; ///< GPIO 端口（nullptr = 不配置引脚，仅选通道号）
    pin_enum_t pin = pin_none;    ///< 引脚掩码
    uint32_t channel = 0;         ///< ADC 通道号: ADC_CHANNEL_0 ~ ADC_CHANNEL_17（F1 无 CH18；
                                  ///< CH16/CH17 = 温度/内部参考，仅 ADC1 可达，需 enable_temp_vref）
    adc_rank_t rank_t = rank1;
    uint32_t sample_time = ADC_SAMPLETIME_55CYCLES_5; ///< 采样时间: ADC_SAMPLETIME_1CYCLE_5 ~ _239CYCLES_5
};

// ============================================================
//  端口配置
// ============================================================

struct AdcPortConfig
{
    adc_id periph = adc_id::adc1; ///< 外设 ID（adc1/2/3 ↔ ADC1/2/3）
    uint32_t resolution = 0;      ///< [F1 忽略] 固定 12bit，无分辨率配置位（字段仅为兼容 GD32 版 API 形状保留）
    uint32_t data_alignment = 0;  ///< [F1 忽略] 固定右对齐（字段仅为兼容 GD32 版 API 形状保留）
    uint32_t trigger_mode = 0;    ///< [F1 忽略] 固定软件触发（字段仅为兼容保留）
    uint32_t vref_mv = 3300;      ///< 参考电压 mV
    uint32_t clock_div = RCC_CFGR_ADCPRE_DIV6; ///< ADC 时钟分频（3 个 ADC 共享，PCLK2=72MHz 时 DIV6=12MHz ≤14MHz 上限）
    bool scan_enable = false;                  ///< 硬件扫描模式（多通道序列；DMA 模式强制多通道序列）
    bool enable_temp_vref = false;             ///< 使能温度传感器(CH16)/内部参考(CH17)（仅 adc1/ADC1 有效）
    AdcChannelConfig channels[8] = {};         ///< 通道列表（port=0 结束）
    uint8_t channel_count = 0;                 ///< 1~8

    // ── DMA 模式（HAL_ADC_Start_DMA 官方中断链 + inter_dma M2 托管） ──
    //   F1 高密度固定映射（请求源由通道号硬件决定，无 SUBPERI 选择）：
    //     ADC1 -> DMA1_Channel1（dma1 + ch0）  中断向量 DMA1_Channel1_IRQn
    //     ADC3 -> DMA2_Channel5（dma2 + ch4）  中断向量 DMA2_Channel4_5_IRQn
    //     ADC2 无 DMA 能力（use_dma 时 dma_start() 返回 false）
    //   其余 dma_controller/dma_channel 组合无内置 ISR，dma_start() 返回 false
    bool use_dma = false;                        ///< 启用 DMA 连续采集
    dma_id dma_controller = dma_id::dma1;        ///< DMA 控制器（ADC1→dma1，ADC3→dma2）
    uint8_t dma_channel = 0;                     ///< DMA 通道号 0-based（ADC1→0，ADC3→4）
    uint32_t dma_priority = DMA_PRIORITY_MEDIUM; ///< DMA 优先级
};

// ============================================================
//  ADC 端口
// ============================================================

/**
 * @brief ADC 端口（HAL API：阻塞单通道轮询 + DMA 连续采集）
 *
 * 验证用法：PC1 接 3.3V（ADC1_IN11）
 *
 *   static adc_port adc(AdcPortConfig{
 *       .periph = adc_id::adc1,          // ADC1
 *       .channels = {{GPIOC, pin1, ADC_CHANNEL_11, ADC_SAMPLETIME_55CYCLES_5}},
 *       .channel_count = 1,
 *       .use_dma = true,                 // ADC1 → DMA1_Channel1
 *       .dma_controller = dma_id::dma1,
 *       .dma_channel = 0,
 *   });
 *   adc.init();                          // 内部已自动校准
 *   uint32_t mv = adc.read_mv(0);        // 轮询单通道（软件触发）
 *   adc.dma_start(buf, 64);              // DMA 循环采集（HAL_ADC_Start_DMA）
 *   while (!adc.dma_done());
 *   adc.dma_stop();
 *
 * @note 轮询与 DMA 路径互斥：DMA 运行中调用 read_raw/scan_raw 前必须先 dma_stop()
 */
class adc_port
{
  public:
    explicit adc_port(const AdcPortConfig &cfg);
    ~adc_port();

    adc_port(const adc_port &) = delete;
    adc_port &operator=(const adc_port &) = delete;

    // ── 生命周期 ──────────────────────────────────────────
    void init();
    void deinit();
    [[nodiscard]] bool is_initialized() const noexcept
    {
        return _initialized;
    }

    // ── 自校准 ────────────────────────────────────────────

    /**
     * @brief ADC 校准（init() 已自动执行，一般无需手动调用）
     * @note  HAL_ADCEx_Calibration_Start 内部管理上电/断电；
     *        若 DMA 采集运行中调用需先 dma_stop()
     */
    void calibrate();

    // ── 单通道阻塞转换（软件触发轮询） ────────────────────

    uint16_t read_raw(uint8_t ch_index);
    uint32_t read_mv(uint8_t ch_index);

    // ── 扫描（所有通道依次转换） ──────────────────────────
    // 注：F1 无 GD32 的 EOCM 位，硬件扫描无法逐通道轮询，此路径
    //     退化为逐通道软件转换（结果一致）；硬件序列扫描请用 DMA 模式

    void scan_raw(uint16_t *results);
    void scan_mv(uint32_t *results);

    // ── DMA 连续采集（HAL 官方链：连续转换 + DMA 循环搬运） ─

    /**
     * @brief 启动 DMA 循环采集
     * @param buf    结果缓冲区（uint16_t，长度 ≥ channel_count × frames）
     * @param frames 采集帧数（每帧 = 所有通道转换一轮）
     * @return true = 启动成功
     * @note  启动后缓冲区被持续刷新（循环模式）；dma_done() 判断首轮完成
     */
    bool dma_start(uint16_t *buf, uint16_t frames);

    /** @brief DMA 首轮采集是否完成（DMA 中断链置 HAL REG_EOC 状态位） */
    [[nodiscard]] bool dma_done() const;

    /** @brief 停止 DMA 采集并复位为轮询模式 */
    void dma_stop();

    // ── 查询 ──────────────────────────────────────────────
    [[nodiscard]] adc_id periph() const noexcept
    {
        return _cfg.periph;
    }
    [[nodiscard]] uint8_t channel_count() const noexcept
    {
        return _cfg.channel_count;
    }

  private:
    void _enable_clock();
    void _set_mode(uint8_t n, bool cont); // 序列长度/连续模式切换（经 HAL_ADC_Init 重写 L/CONT/SCAN）
    uint16_t _convert_one(uint8_t index); // 单通道轮询转换（Start→Poll→读→Stop）

    AdcPortConfig _cfg;

    // GPIO aligned storage + placement new
    struct
    {
        alignas(io_ctrl) uint8_t data[sizeof(io_ctrl)];
    } _ch_storage[8];
    io_ctrl *_ch_pins[8];

    // DMA 通道（use_dma 时有效；M2 托管模式借给 HAL_ADC_Start_DMA）
    struct
    {
        alignas(dma_channel) uint8_t data[sizeof(dma_channel)];
    } _dma_storage;
    dma_channel *_dma;

    bool _initialized;
    ADC_HandleTypeDef _hadc{}; // F1 HAL ADC 句柄（Instance 在 init() 绑定）
};

#endif
