#pragma once
// ============================================================
// @platform GD32F4xx
//   移植到新 MCU 时替换：
//     - [PORT] #include "gd32f4xx.h" → 目标 SDK 头文件
//     - [PORT] enum class adc_id（按目标芯片调整枚举项）
// ============================================================

#ifdef __cplusplus

#include <cstdint>
#include "inter_io_ctrl.hpp"
#include "inter_dma.hpp"
#include "gd32f4xx.h"

enum class adc_id : uint8_t { adc0, adc1, adc2 };

// ============================================================
//  通道配置
// ============================================================

struct AdcChannelConfig {
    GPIO_TypeDef* port   = nullptr;  ///< GPIO 端口（nullptr = 不配置引脚，仅选通道号）
    pin_enum_t    pin    = pin_none;  ///< 引脚掩码
    uint32_t channel     = 0;   ///< ADC 通道号: ADC_CHANNEL_0 ~ ADC_CHANNEL_18
    uint32_t sample_time = ADC_SAMPLETIME_144;  ///< 本通道采样时间: ADC_SAMPLETIME_3 ~ _480
};

// ============================================================
//  端口配置
// ============================================================

struct AdcPortConfig {
    adc_id            periph           = adc_id::adc0;   ///< 外设 ID
    uint32_t          resolution       = ADC_RESOLUTION_12B;   ///< ADC_RESOLUTION_12B / 10B / 8B / 6B
    uint32_t          data_alignment   = ADC_DATAALIGN_RIGHT;  ///< ADC_DATAALIGN_RIGHT / LEFT
    uint32_t          trigger_mode     = EXTERNAL_TRIGGER_DISABLE; ///< 触发模式（固定软件触发）
    uint32_t          vref_mv          = 3300;   ///< 参考电压 mV
    uint32_t          clock_div        = ADC_ADCCK_PCLK2_DIV4; ///< ADC 时钟分频（3 个 ADC 共享，默认 120M/4=30MHz）
    bool              scan_enable      = false;  ///< 硬件扫描模式（多通道一次序列转换，对齐参考实现）
    bool              enable_temp_vref = false;  ///< 使能温度传感器(CH16)/内部参考(CH17)通道
    AdcChannelConfig  channels[8]      = {};     ///< 通道列表（port=0 结束）
    uint8_t           channel_count    = 0;      ///< 1~8

    // ── DMA 模式（对齐官方例程 ADC0_routine_sequence_with_DMA：连续转换 + DMA 循环搬运） ──
    // 注意: GD32F450/470 的 ADC DMA 请求映射与 GD32F407 不同（官方例程 + 师傅 DMA_BSP 双重印证）:
    //    ADC0 -> DMA1_CH0 SUBPERI0, ADC1 -> DMA1_CH2 SUBPERI1, ADC2 -> DMA1_CH1 SUBPERI2
    bool              use_dma          = false;  ///< 启用 DMA 连续采集（需配合 scan_enable）
    dma_id            dma_controller   = dma_id::dma1;  ///< DMA 控制器（ADC 全在 DMA1）
    uint8_t           dma_channel      = 0;      ///< DMA 通道号（ADC0->CH0, ADC1->CH2, ADC2->CH1）
    uint32_t          dma_priority     = DMA_PRIORITY_MEDIUM;  ///< DMA 优先级
    uint32_t          dma_sub_periph   = DMA_SUBPERI0;        ///< 外设请求源（ADC0->SUBPERI0）
};

// ============================================================
//  ADC 端口
// ============================================================

/**
 * @brief ADC 端口（阻塞单通道 + 多通道扫描）
 *
 * 测试用：PA0 接 3.3V, PB3 接 3.3V
 *
 *   static adc_port adc({
 *       .periph = adc_id::adc0,
 *       .channels = {{{GPIOA, pin0, ADC_CHANNEL_0, ADC_SAMPLETIME_144},
 *                     {GPIOB, pin3, ADC_CHANNEL_3, ADC_SAMPLETIME_144}}},
 *       .channel_count = 2,
 *       .scan_enable = true,          // 硬件扫描模式
 *   });
 *   adc.init();                       // 内部已自动校准（校准必须先于使能）
 *   uint32_t mv = adc.read_mv(0);     // 读 PA0
 *   uint32_t t[2]; adc.scan_mv(t);    // 一次序列转换取全部
 */
class adc_port {
public:
    explicit adc_port(const AdcPortConfig &cfg);
    ~adc_port();

    adc_port(const adc_port &) = delete;
    adc_port &operator=(const adc_port &) = delete;

    // ── 生命周期 ──────────────────────────────────────────
    void init();
    void deinit();
    [[nodiscard]] bool is_initialized() const noexcept { return _initialized; }

    // ── 自校准 ────────────────────────────────────────────

    /**
     * @brief ADC 校准（init() 已自动执行，一般无需手动调用）
     * @note  GD32 要求校准必须在 ADC 使能之前；若已使能本方法会
     *        临时关闭→校准→重新使能（安全但多一次开关）
     */
    void calibrate();

    // ── 单通道阻塞转换 ────────────────────────────────────

    uint16_t read_raw(uint8_t ch_index);
    uint32_t read_mv(uint8_t ch_index);

    // ── 扫描（所有通道顺序转换） ──────────────────────────

    void scan_raw(uint16_t *results);
    void scan_mv(uint32_t *results);

    // ── DMA 连续采集（对齐参考实现，CPU 零参与） ───────────

    /**
     * @brief 启动 DMA 循环采集（ADC 连续转换 + DMA 循环搬运）
     * @param buf    结果缓冲区（uint16_t，长度 ≥ channel_count × frames）
     * @param frames 采集帧数（每帧 = 所有通道转换一轮）
     * @return true = 启动成功
     * @note  启动后缓冲区被持续刷新（循环模式）；dma_done() 判断首轮完成
     */
    bool dma_start(uint16_t *buf, uint16_t frames);

    /** @brief DMA 首轮采集是否完成 */
    [[nodiscard]] bool dma_done() const;

    /** @brief 停止 DMA 采集 */
    void dma_stop();

    // ── 查询 ──────────────────────────────────────────────
    [[nodiscard]] adc_id periph() const noexcept { return _cfg.periph; }
    [[nodiscard]] uint8_t channel_count() const noexcept { return _cfg.channel_count; }

private:
    void _enable_clock();
    void _config_channel(uint8_t index);
    uint16_t _do_convert();

    AdcPortConfig _cfg;

    // PWM 模式同款 aligned storage + placement new
    struct { alignas(io_ctrl) uint8_t data[sizeof(io_ctrl)]; } _ch_storage[8];
    io_ctrl *_ch_pins[8];

    // DMA 通道（use_dma 时有效）
    struct { alignas(dma_channel) uint8_t data[sizeof(dma_channel)]; } _dma_storage;
    dma_channel *_dma;

    bool     _initialized;
    uint32_t _periph;
};

#endif
