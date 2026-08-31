#pragma once
#define STM32F1xx // 适配F1xx系列的重映射接口,确保第一时间生效正确识别芯片

#include "stm32f1xx_hal.h"

#include "assert.h"
#include "stdint.h"

#ifdef __cplusplus

enum polarity : uint8_t
{
    Low = RESET,
    Hig = !RESET,
    active_low = Low,   // 语义别名：低有效（如 CS 选通信号）
    active_high = Hig,  // 语义别名：高有效
};

enum pin_enum_t : uint16_t
{
    pin_none = 0,
    pin0 = GPIO_PIN_0,
    pin1 = GPIO_PIN_1,
    pin2 = GPIO_PIN_2,
    pin3 = GPIO_PIN_3,
    pin4 = GPIO_PIN_4,
    pin5 = GPIO_PIN_5,
    pin6 = GPIO_PIN_6,
    pin7 = GPIO_PIN_7,
    pin8 = GPIO_PIN_8,
    pin9 = GPIO_PIN_9,
    pin10 = GPIO_PIN_10,
    pin11 = GPIO_PIN_11,
    pin12 = GPIO_PIN_12,
    pin13 = GPIO_PIN_13,
    pin14 = GPIO_PIN_14,
    pin15 = GPIO_PIN_15,
    pinall = GPIO_PIN_All,
};

enum mode_enum_t : uint32_t
{
    mode_input = GPIO_MODE_INPUT,
    mode_out_pp = GPIO_MODE_OUTPUT_PP,
    mode_out_od = GPIO_MODE_OUTPUT_OD,
    mode_af_pp = GPIO_MODE_AF_PP,
    mode_af_od = GPIO_MODE_AF_OD,
    mode_af_input = mode_input,

    mode_analog = GPIO_MODE_ANALOG,
    mode_it_rising = GPIO_MODE_IT_RISING,
    mode_it_falling = GPIO_MODE_IT_FALLING,
    mode_it_rising_falling = GPIO_MODE_EVT_RISING_FALLING,

    mode_evt_rising = GPIO_MODE_EVT_RISING,
    mdoe_evt_falling = GPIO_MODE_EVT_FALLING,
    mode_evt_rising_falling = GPIO_MODE_EVT_RISING_FALLING,
};

enum speed_enum_t : uint32_t
{
    speed_low = GPIO_SPEED_FREQ_LOW,
    speed_medium = GPIO_SPEED_FREQ_MEDIUM,
    speed_high = GPIO_SPEED_FREQ_HIGH,
};

enum pull_enum_t : uint32_t
{
    nopull = GPIO_NOPULL,
    pullup = GPIO_PULLUP,
    pulldown = GPIO_PULLDOWN,
};

enum class afio_enum_t : uint32_t
{
    NONE = 0,

    // ==================== SPI ====================
    // SPI1
    RM_SPI1_ENABLE,  // ENABLE:  Remap     (NSS/PA15, SCK/PB3, MISO/PB4, MOSI/PB5)
    RM_SPI1_DISABLE, // DISABLE: No remap  (NSS/PA4,  SCK/PA5, MISO/PA6, MOSI/PA7)

#if defined(AFIO_MAPR_SPI3_REMAP)
    // SPI3 (connectivity line devices only)
    RM_SPI3_ENABLE,  // ENABLE:  Remap     (SPI3_NSS-I2S3_WS/PA4, SPI3_SCK-I2S3_CK/PC10, SPI3_MISO/PC11,
                     // SPI3_MOSI-I2S3_SD/PC12)
    RM_SPI3_DISABLE, // DISABLE: No remap  (SPI3_NSS-I2S3_WS/PA15, SPI3_SCK-I2S3_CK/PB3, SPI3_MISO/PB4,
                     // SPI3_MOSI-I2S3_SD/PB5)
#endif

    // ==================== I2C ====================
    // I2C1
    RM_I2C1_ENABLE,  // ENABLE:  Remap     (SCL/PB8, SDA/PB9)
    RM_I2C1_DISABLE, // DISABLE: No remap  (SCL/PB6, SDA/PB7)

    // ==================== USART ====================
    // USART1
    RM_USART1_ENABLE,  // ENABLE:  Remap     (TX/PB6,  RX/PB7)
    RM_USART1_DISABLE, // DISABLE: No remap  (TX/PA9,  RX/PA10)

    // USART2
    RM_USART2_ENABLE,  // ENABLE:  Remap     (CTS/PD3, RTS/PD4, TX/PD5, RX/PD6, CK/PD7)
    RM_USART2_DISABLE, // DISABLE: No remap  (CTS/PA0, RTS/PA1, TX/PA2, RX/PA3, CK/PA4)

    // USART3
    RM_USART3_ENABLE,  // ENABLE:  Full remap    (TX/PD8,  RX/PD9,  CK/PD10, CTS/PD11, RTS/PD12)
    RM_USART3_PARTIAL, // PARTIAL: Partial remap  (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14)
    RM_USART3_DISABLE, // DISABLE: No remap       (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14)

    // ==================== TIM1 ====================
    RM_TIM1_ENABLE,  // ENABLE:  Full remap    (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8,
                     // CH2N/PE10, CH3N/PE12)
    RM_TIM1_PARTIAL, // PARTIAL: Partial remap  (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7,
                     // CH2N/PB0, CH3N/PB1)
    RM_TIM1_DISABLE, // DISABLE: No remap       (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12,
                     // CH1N/PB13, CH2N/PB14, CH3N/PB15)

    // ==================== TIM2 ====================
    RM_TIM2_ENABLE,    // ENABLE:    Full remap       (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11)
    RM_TIM2_PARTIAL_2, // PARTIAL_2: Partial remap    (CH1/ETR/PA0,  CH2/PA1, CH3/PB10, CH4/PB11)
    RM_TIM2_PARTIAL_1, // PARTIAL_1: Partial remap    (CH1/ETR/PA15, CH2/PB3, CH3/PA2,  CH4/PA3)
    RM_TIM2_DISABLE,   // DISABLE:   No remap         (CH1/ETR/PA0,  CH2/PA1, CH3/PA2,  CH4/PA3)

    // ==================== TIM3 ====================
    RM_TIM3_ENABLE,  // ENABLE:  Full remap    (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9). TIM3_ETR on PE0 not remapped.
    RM_TIM3_PARTIAL, // PARTIAL: Partial remap  (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1). TIM3_ETR on PE0 not remapped.
    RM_TIM3_DISABLE, // DISABLE: No remap       (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1). TIM3_ETR on PE0 not remapped.

    // ==================== TIM4 ====================
    RM_TIM4_ENABLE,  // ENABLE:  Full remap (CH1/PD12, CH2/PD13, CH3/PD14, CH4/PD15). TIM4_ETR on PE0 not remapped.
    RM_TIM4_DISABLE, // DISABLE: No remap  (CH1/PB6,  CH2/PB7,  CH3/PB8,  CH4/PB9).  TIM4_ETR on PE0 not remapped.

#if defined(AFIO_MAPR2_TIM9_REMAP)
    // ==================== TIM9 (MAPR2) ====================
    RM_TIM9_ENABLE,  // ENABLE:  Remap     (TIM9_CH1/PE5, TIM9_CH2/PE6)
    RM_TIM9_DISABLE, // DISABLE: No remap  (TIM9_CH1/PA2, TIM9_CH2/PA3)
#endif

#if defined(AFIO_MAPR2_TIM10_REMAP)
    // ==================== TIM10 (MAPR2) ====================
    RM_TIM10_ENABLE,  // ENABLE:  Remap     (TIM10_CH1/PF6)
    RM_TIM10_DISABLE, // DISABLE: No remap  (TIM10_CH1/PB8)
#endif

#if defined(AFIO_MAPR2_TIM11_REMAP)
    // ==================== TIM11 (MAPR2) ====================
    RM_TIM11_ENABLE,  // ENABLE:  Remap     (TIM11_CH1/PF7)
    RM_TIM11_DISABLE, // DISABLE: No remap  (TIM11_CH1/PB9)
#endif

#if defined(AFIO_MAPR2_TIM12_REMAP)
    // ==================== TIM12 (MAPR2, high density value line) ====================
    RM_TIM12_ENABLE,  // ENABLE:  Remap     (TIM12_CH1/PB12, TIM12_CH2/PB13)
    RM_TIM12_DISABLE, // DISABLE: No remap  (TIM12_CH1/PC4,  TIM12_CH2/PC5)
#endif

#if defined(AFIO_MAPR2_TIM13_REMAP)
    // ==================== TIM13 (MAPR2) ====================
    RM_TIM13_ENABLE,  // ENABLE:  Remap     STM32F100:(TIM13_CH1/PF8), Others:(TIM13_CH1/PB0)
    RM_TIM13_DISABLE, // DISABLE: No remap  STM32F100:(TIM13_CH1/PA6), Others:(TIM13_CH1/PC8)
#endif

#if defined(AFIO_MAPR2_TIM14_REMAP)
    // ==================== TIM14 (MAPR2) ====================
    RM_TIM14_ENABLE,  // ENABLE:  Remap     STM32F100:(TIM14_CH1/PB1), Others:(TIM14_CH1/PF9)
    RM_TIM14_DISABLE, // DISABLE: No remap  STM32F100:(TIM14_CH1/PC9), Others:(TIM14_CH1/PA7)
#endif

#if defined(AFIO_MAPR2_TIM15_REMAP)
    // ==================== TIM15 (MAPR2) ====================
    RM_TIM15_ENABLE,  // ENABLE:  Remap     (TIM15_CH1/PB14, TIM15_CH2/PB15)
    RM_TIM15_DISABLE, // DISABLE: No remap  (TIM15_CH1/PA2,  TIM15_CH2/PA3)
#endif

#if defined(AFIO_MAPR2_TIM16_REMAP)
    // ==================== TIM16 (MAPR2) ====================
    RM_TIM16_ENABLE,  // ENABLE:  Remap     (TIM16_CH1/PA6)
    RM_TIM16_DISABLE, // DISABLE: No remap  (TIM16_CH1/PB8)
#endif

#if defined(AFIO_MAPR2_TIM17_REMAP)
    // ==================== TIM17 (MAPR2) ====================
    RM_TIM17_ENABLE,  // ENABLE:  Remap     (TIM17_CH1/PA7)
    RM_TIM17_DISABLE, // DISABLE: No remap  (TIM17_CH1/PB9)
#endif

#if defined(AFIO_MAPR_TIM5CH4_IREMAP)
    // ==================== TIM5CH4 ====================
    RM_TIM5CH4_ENABLE,  // ENABLE:  LSI internal clock connected to TIM5_CH4 for calibration (high density value line
                        // only)
    RM_TIM5CH4_DISABLE, // DISABLE: TIM5_CH4 connected to PA3
#endif

#if defined(AFIO_MAPR2_TIM1_DMA_REMAP)
    // ==================== TIM DMA Remap (MAPR2) ====================
    // TIM1 DMA
    RM_TIM1DMA_ENABLE,  // ENABLE:  Remap     (TIM1_CH1/DMA1_Ch6, TIM1_CH2/DMA1_Ch6)
    RM_TIM1DMA_DISABLE, // DISABLE: No remap  (TIM1_CH1/DMA1_Ch2, TIM1_CH2/DMA1_Ch3)
#endif

#if defined(AFIO_MAPR2_TIM67_DAC_DMA_REMAP)
    // TIM6/TIM7 DAC DMA
    RM_TIM67DACDMA_ENABLE,  // ENABLE:  Remap     (TIM6_DAC1/DMA1_Ch3, TIM7_DAC2/DMA1_Ch4)
    RM_TIM67DACDMA_DISABLE, // DISABLE: No remap  (TIM6_DAC1/DMA2_Ch3, TIM7_DAC2/DMA2_Ch4)
#endif

#if defined(AFIO_MAPR_CAN_REMAP_REMAP1)
    // ==================== CAN ====================
    // CAN1 (single CAN interface devices)
    RM_CAN1_1, // CASE 1: CAN_RX/PA11, CAN_TX/PA12
    RM_CAN1_2, // CASE 2: CAN_RX/PB8,  CAN_TX/PB9  (not available on 36-pin package)
    RM_CAN1_3, // CASE 3: CAN_RX/PD0,  CAN_TX/PD1
#endif

#if defined(AFIO_MAPR_CAN2_REMAP)
    // CAN2 (connectivity line devices only)
    RM_CAN2_ENABLE,  // ENABLE:  Remap     (CAN2_RX/PB5,  CAN2_TX/PB6)
    RM_CAN2_DISABLE, // DISABLE: No remap  (CAN2_RX/PB12, CAN2_TX/PB13)
#endif

    // ==================== PD0/PD1 ====================
    RM_PD01_ENABLE,  // ENABLE:  PD0 remapped on OSC_IN, PD1 remapped on OSC_OUT (36/48/64-pin packages, HSE not
                     // used)
    RM_PD01_DISABLE, // DISABLE: No remapping of PD0 and PD1

#if defined(AFIO_MAPR_ETH_REMAP)
    // ==================== Ethernet (connectivity line devices only) ====================
    // ETH MAC remap
    RM_ETH_ENABLE,  // ENABLE:  Remap     (RX_DV-CRS_DV/PD8, RXD0/PD9, RXD1/PD10, RXD2/PD11, RXD3/PD12)
    RM_ETH_DISABLE, // DISABLE: No remap  (RX_DV-CRS_DV/PA7, RXD0/PC4, RXD1/PC5,  RXD2/PB0,  RXD3/PB1)
#endif

#if defined(AFIO_MAPR_MII_RMII_SEL)
    // ETH MII/RMII selection
    RM_ETH_RMII, // Configure Ethernet MAC for connection with an RMII PHY
    RM_ETH_MII,  // Configure Ethernet MAC for connection with an MII PHY
#endif

#if defined(AFIO_MAPR_PTP_PPS_REMAP)
    // ETH PTP PPS
    RM_ETH_PTP_PPS_ENABLE,  // ENABLE:  PTP_PPS output on PB5 pin
    RM_ETH_PTP_PPS_DISABLE, // DISABLE: PTP_PPS not output on PB5 pin
#endif

    // ==================== ADC ====================
    // ADC1 external trigger injected
    RM_ADC1_ETRGINJ_ENABLE,  // ENABLE:  ADC1 Ext injected trigger connected to TIM8 Channel4
    RM_ADC1_ETRGINJ_DISABLE, // DISABLE: ADC1 Ext injected trigger connected to EXTI15

    // ADC1 external trigger regular
    RM_ADC1_ETRGREG_ENABLE,  // ENABLE:  ADC1 Ext regular trigger connected to TIM8 TRG0
    RM_ADC1_ETRGREG_DISABLE, // DISABLE: ADC1 Ext regular trigger connected to EXTI11

#if defined(AFIO_MAPR_ADC2_ETRGINJ_REMAP)
    // ADC2 external trigger injected
    RM_ADC2_ETRGINJ_ENABLE,  // ENABLE:  ADC2 Ext injected trigger connected to TIM8 Channel4
    RM_ADC2_ETRGINJ_DISABLE, // DISABLE: ADC2 Ext injected trigger connected to EXTI15
#endif

#if defined(AFIO_MAPR_ADC2_ETRGREG_REMAP)
    // ADC2 external trigger regular
    RM_ADC2_ETRGREG_ENABLE,  // ENABLE:  ADC2 Ext regular trigger connected to TIM8 TRG0
    RM_ADC2_ETRGREG_DISABLE, // DISABLE: ADC2 Ext regular trigger connected to EXTI11
#endif

    // ==================== SWJ (JTAG/SWD) ====================
    RM_SWJ_ENABLE,   // Full SWJ (JTAG-DP + SW-DP): Reset State
    RM_SWJ_NONJTRST, // Full SWJ (JTAG-DP + SW-DP) but without NJTRST
    RM_SWJ_NOJTAG,   // JTAG-DP Disabled and SW-DP Enabled
    RM_SWJ_DISABLE,  // JTAG-DP Disabled and SW-DP Disabled

#if defined(AFIO_MAPR_TIM2ITR1_IREMAP)
    // ==================== TIM2 ITR1 (connectivity line devices only) ====================
    RM_TIM2ITR1_TO_USB, // Connect USB OTG SOF output to TIM2_ITR1 for calibration
    RM_TIM2ITR1_TO_ETH, // Connect TIM2_ITR1 to Ethernet PTP output for calibration
#endif

#if defined(AFIO_MAPR2_FSMC_NADV_REMAP)
    // ==================== FSMC NADV (MAPR2) ====================
    RM_FSMCNADV_DISCONNECTED, // NADV signal not connected, I/O pin available for other peripherals
    RM_FSMCNADV_CONNECTED,    // NADV signal connected to output (default)
#endif

#if defined(AFIO_MAPR2_MISC_REMAP)
    // ==================== MISC Remap (MAPR2, high density value line) ====================
    RM_MISC_ENABLE,  // ENABLE:  DMA2_Ch5 interrupt mapped at position 60, TIM15 TRGO as DAC Trigger3, TIM15 triggers
                     // TIM1/3
    RM_MISC_DISABLE, // DISABLE: DMA2_Ch5 interrupt mapped with DMA2_Ch4 at position 59, TIM5 TRGO as DAC Trigger3,
                     // TIM5 triggers TIM1/3
#endif
};
#endif

/**
 * @brief  GPIO 单引脚管理类（非模板、零堆分配）
 *
 * - 构造时绑定端口+引脚，init() 后可用
 * - 禁止拷贝，允许移动（移动后源对象失效）
 * - 所有位操作通过 BOP/BC/TG 寄存器，硬件级原子
 * - 断言仅在 Debug 生效；Release 下通过返回值/早期返回保证安全
 */
class io_ctrl
{
public:
#ifdef STM32F1xx
public:
public:
    explicit io_ctrl(GPIO_TypeDef *gpio_periph, pin_enum_t pin);
    ~io_ctrl();

    io_ctrl(const io_ctrl &) = delete;
    io_ctrl &operator=(const io_ctrl &) = delete;

    io_ctrl(io_ctrl &&other) noexcept;
    io_ctrl &operator=(io_ctrl &&other) noexcept;

    // ── 生命周期 ──────────────────────────────────────────

    /**
     * @brief 初始化 GPIO 引脚（调用前必须未初始化）
     * @param mode   mode_input / mode_out_pp / mode_out_od / mode_af_pp / mode_af_od / mode_analog
     * @param pull   nopull / pullup / pulldown
     * @param speed  speed_low / speed_medium / speed_high（仅输出/AF 有效）
     */
    void init(mode_enum_t mode, pull_enum_t pull = nopull, speed_enum_t speed = speed_high);

    /** @brief 重新配置已初始化引脚的模式（不必先 deinit） */
    void reinit(mode_enum_t mode, pull_enum_t pull = nopull, speed_enum_t speed = speed_high);

    /** @brief 反初始化，恢复为模拟输入（最低功耗），可再次 init() */
    void deinit();

    // ── 独立属性修改（仅已初始化时有效） ──────────────────

    /** @brief 修改上/下拉（输入/输出/AF 均有效） */
    void set_pull(pull_enum_t pull);

    /** @brief 修改输出速度（仅输出/AF 有效） */
    void set_speed(speed_enum_t speed);

    // ── 复用功能 ──────────────────────────────────────────

    /**
     * @brief 设置引脚复用功能的重映射选项（STM32F1 系列 AFIO remap）
     * @note  仅在引脚已初始化为复用功能模式（mode_af_pp 或 mode_af_od）时调用有效。
     *        若当前为非 AF 模式，调用将被忽略。
     *        应在 init() 之后、需要使用该外设前调用。
     */
    void set_af(afio_enum_t alt_func_num);

    // ── 锁定 ──────────────────────────────────────────────

    /** @brief 锁定引脚配置（仅系统复位解除） */
    void lock();

    // ── 输出（BOP/BC 寄存器，硬件级原子） ─────────────────

    void high();
    void low();
    void toggle();
    void set(bool value);           // true=high, false=low面向逻辑接口的便捷操作
    void write(polarity bit_value); // 面向硬件状态的接口

    // ── 输入 ──────────────────────────────────────────────

    [[nodiscard]] polarity read();        // ISTAT，任意模式有效
    [[nodiscard]] polarity read_output(); // OCTL，输出/AF 模式有效

    // ── 查询 ──────────────────────────────────────────────

    [[nodiscard]] bool is_initialized() const noexcept
    {
        return _initialized;
    }
    [[nodiscard]] GPIO_TypeDef *port() const noexcept
    {
        return _gpio_periph;
    }
    [[nodiscard]] pin_enum_t pin() const noexcept
    {
        return _pin;
    }
    [[nodiscard]] mode_enum_t mode() const noexcept
    {
        return _mode;
    }

    /** @brief 快速判初始化，支持 if(pin) 写法 */
    [[nodiscard]] explicit operator bool() const noexcept
    {
        return _initialized;
    }

private:
    GPIO_TypeDef *_gpio_periph;
    pin_enum_t _pin;
    mode_enum_t _mode;
    pull_enum_t _pull;
    speed_enum_t _speed; // 缓存的输出速度，供 set_otype 保持
    bool _initialized;

    static constexpr mode_enum_t MODE_UNSET = mode_input;
    static constexpr pull_enum_t PULL_DEF = nopull;
    static constexpr speed_enum_t SPEED_DEF = speed_high;
    static constexpr afio_enum_t AF_DEF = afio_enum_t::NONE;

    void _enable_clock();
    void _apply(mode_enum_t mode, pull_enum_t pull, speed_enum_t speed);

    /** @brief Release 安全的状态检查（断言之外的第二道防线） */
    [[nodiscard]] bool _check_init() const noexcept
    {
        return _initialized;
    }
    [[nodiscard]] bool _check_out() const noexcept;
};

#endif /* __cplusplus */
