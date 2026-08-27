#include "inter_io_ctrl.hpp"

// ============================================================
//  构造 / 析构 / 移动
// ============================================================

io_ctrl::io_ctrl(GPIO_TypeDef *gpio_periph, pin_enum_t pin)
    : _gpio_periph(gpio_periph), _pin(pin), _mode(MODE_UNSET), _pull(PULL_DEF), _speed(SPEED_DEF), _initialized(false)
{
}

io_ctrl::~io_ctrl()
{
    deinit();
}

io_ctrl::io_ctrl(io_ctrl &&other) noexcept
    : _gpio_periph(other._gpio_periph), _pin(other._pin), _mode(other._mode), _pull(other._pull), _speed(other._speed),
      _initialized(other._initialized)
{
    other._initialized = false;
    other._mode = MODE_UNSET;
}

io_ctrl &io_ctrl::operator=(io_ctrl &&other) noexcept
{
    if (this != &other)
    {
        deinit();

        _gpio_periph = other._gpio_periph;
        _pin = other._pin;
        _mode = other._mode;
        _pull = other._pull;
        _speed = other._speed;
        _initialized = other._initialized;

        other._initialized = false;
        other._mode = MODE_UNSET;
    }
    return *this;
}

// ============================================================
//  时钟使能
// ============================================================

void io_ctrl::_enable_clock()
{
    // rcu_periph_clock_enable 内部幂等
    if (_gpio_periph == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (_gpio_periph == GPIOB)  
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (_gpio_periph == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (_gpio_periph == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (_gpio_periph == GPIOE)
        __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (_gpio_periph == GPIOF)
        __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (_gpio_periph == GPIOG)
        __HAL_RCC_GPIOG_CLK_ENABLE();
#ifdef GPIOH
    else if (_gpio_periph == GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
    else
    {
        assert(0);
    } // 非法端口
}

// ============================================================
//  配置应用
// ============================================================

void io_ctrl::_apply(mode_enum_t mode, pull_enum_t pull, speed_enum_t speed)
{
    GPIO_InitTypeDef GPIO_StructInit = {.Pin = _pin, .Mode = mode, .Pull = pull, .Speed = speed};
    HAL_GPIO_Init(_gpio_periph, &GPIO_StructInit);
    _mode = mode;
    _speed = speed;
    _pull = pull;
}

// ============================================================
//  init / reinit / deinit
// ============================================================

void io_ctrl::init(mode_enum_t mode, pull_enum_t pull, speed_enum_t speed)
{
    if (_initialized)
        return; // Release 安全

    _enable_clock();
    _apply(mode, pull, speed);
    _initialized = true;
}

void io_ctrl::reinit(mode_enum_t mode, pull_enum_t pull, speed_enum_t speed)
{
    if (!_initialized)
        return; // Release 安全

    _apply(mode, pull, speed);
}

void io_ctrl::deinit()
{
    if (_initialized)
    {
        HAL_GPIO_DeInit(_gpio_periph, _pin);
        _initialized = false;
        _mode = MODE_UNSET;
    }
}

// ============================================================
//  独立属性修改
// ============================================================

void io_ctrl::set_pull(pull_enum_t pull)
{
    if (!_check_init())
        return;
    GPIO_InitTypeDef GPIO_StructInit = {
        .Pin = _pin,
        .Mode = _mode,
        .Pull = pull,
        .Speed = _speed,
    };
    HAL_GPIO_Init(_gpio_periph, &GPIO_StructInit);
    _pull = pull;
}

void io_ctrl::set_speed(speed_enum_t speed)
{
    if (!_check_out())
        return;
    // gpio_output_options_set(_gpio_periph, _otype, speed, _pin);

    GPIO_InitTypeDef GPIO_StructInit = {
        .Pin = _pin,
        .Mode = _mode,
        .Pull = _pull,
        .Speed = speed,
    };
    HAL_GPIO_Init(_gpio_periph, &GPIO_StructInit);
    _speed = speed;
}

// ============================================================
//  复用功能（放宽了 AF 模式限制） ─────────────────────────
// ============================================================

void io_ctrl::set_af(afio_enum_t alt_func_num)
{
    if (!_initialized)
        return;
    if (_mode == GPIO_MODE_AF_OD || _mode == GPIO_MODE_AF_PP) // fixed: added _mode ==
    {
        // GPIO_InitTypeDef GPIO_InitStruct = {
        //     .Pin = _pin,
        //     .Mode = _mode,
        //     .Pull = _pull,
        //     .Speed = _speed,
        // };
        // HAL_GPIO_Init(_gpio_periph, &GPIO_InitStruct);

#ifdef STM32F1xx

        switch (alt_func_num)
        {

        case afio_enum_t::NONE:
            break;

        // ==================== SPI ====================
        case afio_enum_t::RM_SPI1_ENABLE:
            __HAL_AFIO_REMAP_SPI1_ENABLE();
            break;
        case afio_enum_t::RM_SPI1_DISABLE:
            __HAL_AFIO_REMAP_SPI1_DISABLE();
            break;

#if defined(AFIO_MAPR_SPI3_REMAP)
        case afio_enum_t::RM_SPI3_ENABLE:
            __HAL_AFIO_REMAP_SPI3_ENABLE();
            break;
        case afio_enum_t::RM_SPI3_DISABLE:
            __HAL_AFIO_REMAP_SPI3_DISABLE();
            break;
#endif

        // ==================== I2C ====================
        case afio_enum_t::RM_I2C1_ENABLE:
            __HAL_AFIO_REMAP_I2C1_ENABLE();
            break;
        case afio_enum_t::RM_I2C1_DISABLE:
            __HAL_AFIO_REMAP_I2C1_DISABLE();
            break;

        // ==================== USART ====================
        case afio_enum_t::RM_USART1_ENABLE:
            __HAL_AFIO_REMAP_USART1_ENABLE();
            break;
        case afio_enum_t::RM_USART1_DISABLE:
            __HAL_AFIO_REMAP_USART1_DISABLE();
            break;

        case afio_enum_t::RM_USART2_ENABLE:
            __HAL_AFIO_REMAP_USART2_ENABLE();
            break;
        case afio_enum_t::RM_USART2_DISABLE:
            __HAL_AFIO_REMAP_USART2_DISABLE();
            break;

        case afio_enum_t::RM_USART3_ENABLE:
            __HAL_AFIO_REMAP_USART3_ENABLE();
            break;
        case afio_enum_t::RM_USART3_PARTIAL:
            __HAL_AFIO_REMAP_USART3_PARTIAL();
            break;
        case afio_enum_t::RM_USART3_DISABLE:
            __HAL_AFIO_REMAP_USART3_DISABLE();
            break;

        // ==================== TIM1 ====================
        case afio_enum_t::RM_TIM1_ENABLE:
            __HAL_AFIO_REMAP_TIM1_ENABLE();
            break;
        case afio_enum_t::RM_TIM1_PARTIAL:
            __HAL_AFIO_REMAP_TIM1_PARTIAL();
            break;
        case afio_enum_t::RM_TIM1_DISABLE:
            __HAL_AFIO_REMAP_TIM1_DISABLE();
            break;

        // ==================== TIM2 ====================
        case afio_enum_t::RM_TIM2_ENABLE:
            __HAL_AFIO_REMAP_TIM2_ENABLE();
            break;
        case afio_enum_t::RM_TIM2_PARTIAL_2:
            __HAL_AFIO_REMAP_TIM2_PARTIAL_2();
            break;
        case afio_enum_t::RM_TIM2_PARTIAL_1:
            __HAL_AFIO_REMAP_TIM2_PARTIAL_1();
            break;
        case afio_enum_t::RM_TIM2_DISABLE:
            __HAL_AFIO_REMAP_TIM2_DISABLE();
            break;

        // ==================== TIM3 ====================
        case afio_enum_t::RM_TIM3_ENABLE:
            __HAL_AFIO_REMAP_TIM3_ENABLE();
            break;
        case afio_enum_t::RM_TIM3_PARTIAL:
            __HAL_AFIO_REMAP_TIM3_PARTIAL();
            break;
        case afio_enum_t::RM_TIM3_DISABLE:
            __HAL_AFIO_REMAP_TIM3_DISABLE();
            break;

        // ==================== TIM4 ====================
        case afio_enum_t::RM_TIM4_ENABLE:
            __HAL_AFIO_REMAP_TIM4_ENABLE();
            break;
        case afio_enum_t::RM_TIM4_DISABLE:
            __HAL_AFIO_REMAP_TIM4_DISABLE();
            break;

            // ==================== TIM9 (MAPR2) ====================
#if defined(AFIO_MAPR2_TIM9_REMAP)
        case afio_enum_t::RM_TIM9_ENABLE:
            __HAL_AFIO_REMAP_TIM9_ENABLE();
            break;
        case afio_enum_t::RM_TIM9_DISABLE:
            __HAL_AFIO_REMAP_TIM9_DISABLE();
            break;
#endif

            // ==================== TIM10 (MAPR2) ====================
#if defined(AFIO_MAPR2_TIM10_REMAP)
        case afio_enum_t::RM_TIM10_ENABLE:
            __HAL_AFIO_REMAP_TIM10_ENABLE();
            break;
        case afio_enum_t::RM_TIM10_DISABLE:
            __HAL_AFIO_REMAP_TIM10_DISABLE();
            break;
#endif

            // ==================== TIM11 (MAPR2) ====================
#if defined(AFIO_MAPR2_TIM11_REMAP)
        case afio_enum_t::RM_TIM11_ENABLE:
            __HAL_AFIO_REMAP_TIM11_ENABLE();
            break;
        case afio_enum_t::RM_TIM11_DISABLE:
            __HAL_AFIO_REMAP_TIM11_DISABLE();
            break;
#endif

            // ==================== TIM12 (MAPR2) ====================
#if defined(AFIO_MAPR2_TIM12_REMAP)
        case afio_enum_t::RM_TIM12_ENABLE:
            __HAL_AFIO_REMAP_TIM12_ENABLE();
            break;
        case afio_enum_t::RM_TIM12_DISABLE:
            __HAL_AFIO_REMAP_TIM12_DISABLE();
            break;
#endif

            // ==================== TIM13 (MAPR2) ====================
#if defined(AFIO_MAPR2_TIM13_REMAP)
        case afio_enum_t::RM_TIM13_ENABLE:
            __HAL_AFIO_REMAP_TIM13_ENABLE();
            break;
        case afio_enum_t::RM_TIM13_DISABLE:
            __HAL_AFIO_REMAP_TIM13_DISABLE();
            break;
#endif

            // ==================== TIM14 (MAPR2) ====================
#if defined(AFIO_MAPR2_TIM14_REMAP)
        case afio_enum_t::RM_TIM14_ENABLE:
            __HAL_AFIO_REMAP_TIM14_ENABLE();
            break;
        case afio_enum_t::RM_TIM14_DISABLE:
            __HAL_AFIO_REMAP_TIM14_DISABLE();
            break;
#endif

            // ==================== TIM15 (MAPR2) ====================
#if defined(AFIO_MAPR2_TIM15_REMAP)
        case afio_enum_t::RM_TIM15_ENABLE:
            __HAL_AFIO_REMAP_TIM15_ENABLE();
            break;
        case afio_enum_t::RM_TIM15_DISABLE:
            __HAL_AFIO_REMAP_TIM15_DISABLE();
            break;
#endif

            // ==================== TIM16 (MAPR2) ====================
#if defined(AFIO_MAPR2_TIM16_REMAP)
        case afio_enum_t::RM_TIM16_ENABLE:
            __HAL_AFIO_REMAP_TIM16_ENABLE();
            break;
        case afio_enum_t::RM_TIM16_DISABLE:
            __HAL_AFIO_REMAP_TIM16_DISABLE();
            break;
#endif

            // ==================== TIM17 (MAPR2) ====================
#if defined(AFIO_MAPR2_TIM17_REMAP)
        case afio_enum_t::RM_TIM17_ENABLE:
            __HAL_AFIO_REMAP_TIM17_ENABLE();
            break;
        case afio_enum_t::RM_TIM17_DISABLE:
            __HAL_AFIO_REMAP_TIM17_DISABLE();
            break;
#endif

            // ==================== TIM5CH4 ====================
#if defined(AFIO_MAPR_TIM5CH4_IREMAP)
        case afio_enum_t::RM_TIM5CH4_ENABLE:
            __HAL_AFIO_REMAP_TIM5CH4_ENABLE();
            break;
        case afio_enum_t::RM_TIM5CH4_DISABLE:
            __HAL_AFIO_REMAP_TIM5CH4_DISABLE();
            break;
#endif

            // ==================== TIM DMA Remap (MAPR2) ====================
#if defined(AFIO_MAPR2_TIM1_DMA_REMAP)
        case afio_enum_t::RM_TIM1DMA_ENABLE:
            __HAL_AFIO_REMAP_TIM1DMA_ENABLE();
            break;
        case afio_enum_t::RM_TIM1DMA_DISABLE:
            __HAL_AFIO_REMAP_TIM1DMA_DISABLE();
            break;
#endif

#if defined(AFIO_MAPR2_TIM67_DAC_DMA_REMAP)
        case afio_enum_t::RM_TIM67DACDMA_ENABLE:
            __HAL_AFIO_REMAP_TIM67DACDMA_ENABLE();
            break;
        case afio_enum_t::RM_TIM67DACDMA_DISABLE:
            __HAL_AFIO_REMAP_TIM67DACDMA_DISABLE();
            break;
#endif

            // ==================== CAN ====================
#if defined(AFIO_MAPR_CAN_REMAP_REMAP1)
        case afio_enum_t::RM_CAN1_1:
            __HAL_AFIO_REMAP_CAN1_1();
            break;
        case afio_enum_t::RM_CAN1_2:
            __HAL_AFIO_REMAP_CAN1_2();
            break;
        case afio_enum_t::RM_CAN1_3:
            __HAL_AFIO_REMAP_CAN1_3();
            break;
#endif

#if defined(AFIO_MAPR_CAN2_REMAP)
        case afio_enum_t::RM_CAN2_ENABLE:
            __HAL_AFIO_REMAP_CAN2_ENABLE();
            break;
        case afio_enum_t::RM_CAN2_DISABLE:
            __HAL_AFIO_REMAP_CAN2_DISABLE();
            break;
#endif

        // ==================== PD0/PD1 ====================
        case afio_enum_t::RM_PD01_ENABLE:
            __HAL_AFIO_REMAP_PD01_ENABLE();
            break;
        case afio_enum_t::RM_PD01_DISABLE:
            __HAL_AFIO_REMAP_PD01_DISABLE();
            break;

            // ==================== Ethernet ====================
#if defined(AFIO_MAPR_ETH_REMAP)
        case afio_enum_t::RM_ETH_ENABLE:
            __HAL_AFIO_REMAP_ETH_ENABLE();
            break;
        case afio_enum_t::RM_ETH_DISABLE:
            __HAL_AFIO_REMAP_ETH_DISABLE();
            break;
#endif

#if defined(AFIO_MAPR_MII_RMII_SEL)
        case afio_enum_t::RM_ETH_RMII:
            __HAL_AFIO_ETH_RMII();
            break;
        case afio_enum_t::RM_ETH_MII:
            __HAL_AFIO_ETH_MII();
            break;
#endif

#if defined(AFIO_MAPR_PTP_PPS_REMAP)
        case afio_enum_t::RM_ETH_PTP_PPS_ENABLE:
            __HAL_AFIO_ETH_PTP_PPS_ENABLE();
            break;
        case afio_enum_t::RM_ETH_PTP_PPS_DISABLE:
            __HAL_AFIO_ETH_PTP_PPS_DISABLE();
            break;
#endif

        // ==================== ADC ====================
        case afio_enum_t::RM_ADC1_ETRGINJ_ENABLE:
            __HAL_AFIO_REMAP_ADC1_ETRGINJ_ENABLE();
            break;
        case afio_enum_t::RM_ADC1_ETRGINJ_DISABLE:
            __HAL_AFIO_REMAP_ADC1_ETRGINJ_DISABLE();
            break;

        case afio_enum_t::RM_ADC1_ETRGREG_ENABLE:
            __HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();
            break;
        case afio_enum_t::RM_ADC1_ETRGREG_DISABLE:
            __HAL_AFIO_REMAP_ADC1_ETRGREG_DISABLE();
            break;

#if defined(AFIO_MAPR_ADC2_ETRGINJ_REMAP)
        case afio_enum_t::RM_ADC2_ETRGINJ_ENABLE:
            __HAL_AFIO_REMAP_ADC2_ETRGINJ_ENABLE();
            break;
        case afio_enum_t::RM_ADC2_ETRGINJ_DISABLE:
            __HAL_AFIO_REMAP_ADC2_ETRGINJ_DISABLE();
            break;
#endif

#if defined(AFIO_MAPR_ADC2_ETRGREG_REMAP)
        case afio_enum_t::RM_ADC2_ETRGREG_ENABLE:
            __HAL_AFIO_REMAP_ADC2_ETRGREG_ENABLE();
            break;
        case afio_enum_t::RM_ADC2_ETRGREG_DISABLE:
            __HAL_AFIO_REMAP_ADC2_ETRGREG_DISABLE();
            break;
#endif

        // ==================== SWJ (JTAG/SWD) ====================
        case afio_enum_t::RM_SWJ_ENABLE:
            __HAL_AFIO_REMAP_SWJ_ENABLE();
            break;
        case afio_enum_t::RM_SWJ_NONJTRST:
            __HAL_AFIO_REMAP_SWJ_NONJTRST();
            break;
        case afio_enum_t::RM_SWJ_NOJTAG:
            __HAL_AFIO_REMAP_SWJ_NOJTAG();
            break;
        case afio_enum_t::RM_SWJ_DISABLE:
            __HAL_AFIO_REMAP_SWJ_DISABLE();
            break;

            // ==================== TIM2 ITR1 ====================
#if defined(AFIO_MAPR_TIM2ITR1_IREMAP)
        case afio_enum_t::RM_TIM2ITR1_TO_USB:
            __HAL_AFIO_TIM2ITR1_TO_USB();
            break;
        case afio_enum_t::RM_TIM2ITR1_TO_ETH:
            __HAL_AFIO_TIM2ITR1_TO_ETH();
            break;
#endif

            // ==================== FSMC NADV (MAPR2) ====================
#if defined(AFIO_MAPR2_FSMC_NADV_REMAP)
        case afio_enum_t::RM_FSMCNADV_DISCONNECTED:
            __HAL_AFIO_FSMCNADV_DISCONNECTED();
            break;
        case afio_enum_t::RM_FSMCNADV_CONNECTED:
            __HAL_AFIO_FSMCNADV_CONNECTED();
            break;
#endif

            // ==================== MISC Remap (MAPR2) ====================
#if defined(AFIO_MAPR2_MISC_REMAP)
        case afio_enum_t::RM_MISC_ENABLE:
            __HAL_AFIO_REMAP_MISC_ENABLE();
            break;
        case afio_enum_t::RM_MISC_DISABLE:
            __HAL_AFIO_REMAP_MISC_DISABLE();
            break;
#endif

        default:
            break;
        }

#endif /* STM32F1xx */
    }
}

// ============================================================
//  锁定
// ============================================================

void io_ctrl::lock()
{
    if (!_initialized)
        return;
    HAL_GPIO_LockPin(_gpio_periph, _pin);
}

// ============================================================
//  输出 — BOP/BC/TG 寄存器均为单次写入，硬件原子
// ============================================================

void io_ctrl::high()
{
    // high low不对_initialized进行检查用于提高速度
    //  gpio_bit_set(_gpio_periph, _pin);
    _gpio_periph->BSRR = _pin;
}

void io_ctrl::low()
{
    // gpio_bit_reset(_gpio_periph, _pin);
    _gpio_periph->BRR = _pin;
}

void io_ctrl::toggle()
{
    // gpio_bit_toggle(_gpio_periph, _pin);
    HAL_GPIO_TogglePin(_gpio_periph, _pin);
}

void io_ctrl::set(bool value)
{
    value ? high() : low();
}

void io_ctrl::write(polarity bit_value)
{
    // gpio_bit_write(_gpio_periph, _pin, bit_value);
    // HAL_GPIO_WritePin(_gpio_periph, _pin, bit_value);
    if (bit_value == Hig)
    {
        high();
    }
    else
    {
        low();
    }
}

// ============================================================
//  输入
// ============================================================

polarity io_ctrl::read()
{
    if (!_check_init())
        return Low;
    // return gpio_input_bit_get(_gpio_periph, _pin);
    // if ((HAL_GPIO_ReadPin(_gpio_periph, _pin)))
    if (_gpio_periph->IDR & _pin)
    {
        return Hig;
    }
    else
    {
        return Low;
    }
}

polarity io_ctrl::read_output()
{
    if (!_check_out())
        return Low;
    // return gpio_output_bit_get(_gpio_periph, _pin);
    return (_gpio_periph->ODR & _pin) ? Hig : Low;
}
bool io_ctrl::_check_out() const noexcept
{
    return _initialized && (_mode == mode_out_pp || _mode == mode_out_od || _mode == mode_af_pp || _mode == mode_af_od);
}
