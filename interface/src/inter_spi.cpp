// ============================================================
// @platform STM32F1xx
//   基于 STM32 HAL 的硬件 SPI 主机端口
//   - 使用 io_ctrl 管理 GPIO
//   - 使用 SPI_HandleTypeDef + HAL_SPI_Init/TransmitReceive
// ============================================================

#include "inter_spi.hpp"
#include "stm32f1xx_hal.h"

// ============================================================
//  构造 / 析构
// ============================================================

spi_port::spi_port(const SpiPortConfig &cfg)
    : _cfg(cfg)
    , _sck(cfg.sck_port, cfg.sck_pin)
    , _mosi(cfg.mosi_port, cfg.mosi_pin)
    , _miso(cfg.miso_port, cfg.miso_pin)
    , _cs(cfg.cs_port ? cfg.cs_port : GPIOA, cfg.cs_pin != pin_none ? cfg.cs_pin : pin0)
    , _has_cs(cfg.cs_port != nullptr && cfg.cs_pin != pin_none)
    , _initialized(false)
{
}

spi_port::~spi_port()
{
    deinit();
}

// ============================================================
//  时钟使能
// ============================================================

void spi_port::_enable_clock()
{
    switch (_cfg.periph)
    {
    case spi_id::spi1:
        __HAL_RCC_SPI1_CLK_ENABLE();
        break;
    case spi_id::spi2:
        __HAL_RCC_SPI2_CLK_ENABLE();
        break;
    case spi_id::spi3:
        __HAL_RCC_SPI3_CLK_ENABLE();
        break;
    }
}

void spi_port::_disable_clock()
{
    switch (_cfg.periph)
    {
    case spi_id::spi1:
        __HAL_RCC_SPI1_CLK_DISABLE();
        break;
    case spi_id::spi2:
        __HAL_RCC_SPI2_CLK_DISABLE();
        break;
    case spi_id::spi3:
        __HAL_RCC_SPI3_CLK_DISABLE();
        break;
    }
}

// ============================================================
//  init
// ============================================================

void spi_port::init()
{
    if (_initialized)
        return;

    _enable_clock();

    // SCK / MOSI：复用推挽输出
    _sck.init(mode_af_pp, nopull, speed_high);
    _sck.set_af(_cfg.af);

    _mosi.init(mode_af_pp, nopull, speed_high);
    _mosi.set_af(_cfg.af);

    // MISO：输入（浮空），由从机驱动
    _miso.init(mode_input, nopull);

    // CS：普通推挽输出，初始释放
    if (_has_cs)
    {
        _cs.init(mode_out_pp, nopull, speed_high);
        cs_deselect();
    }

    // 配置 STM32 HAL SPI 句柄
    switch (_cfg.periph)
    {
    case spi_id::spi1:
        _hspi.Instance = SPI1;
        break;
    case spi_id::spi2:
        _hspi.Instance = SPI2;
        break;
    case spi_id::spi3:
        _hspi.Instance = SPI3;
        break;
    }

    _hspi.Init.Mode              = SPI_MODE_MASTER;
    _hspi.Init.Direction         = SPI_DIRECTION_2LINES;
    _hspi.Init.DataSize          = _cfg.data_size;
    _hspi.Init.CLKPolarity       = _cfg.clock_polarity;
    _hspi.Init.CLKPhase          = _cfg.clock_phase;
    _hspi.Init.NSS               = SPI_NSS_SOFT;
    _hspi.Init.BaudRatePrescaler = _cfg.prescaler;
    _hspi.Init.FirstBit          = _cfg.first_bit;
    _hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
    _hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    _hspi.Init.CRCPolynomial     = 10;

    if (HAL_SPI_Init(&_hspi) != HAL_OK)
    {
        // 初始化失败时不置 _initialized，并释放已初始化的 GPIO
        _sck.deinit();
        _mosi.deinit();
        _miso.deinit();
        if (_has_cs)
            _cs.deinit();
        _disable_clock();
        return;
    }

    _initialized = true;
}

// ============================================================
//  deinit
// ============================================================

void spi_port::deinit()
{
    if (!_initialized)
        return;

    HAL_SPI_DeInit(&_hspi);

    _sck.deinit();
    _mosi.deinit();
    _miso.deinit();

    if (_has_cs)
        _cs.deinit();

    _disable_clock();
    _initialized = false;
}

// ============================================================
//  全双工传输
// ============================================================

void spi_port::transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (!_initialized || len == 0)
        return;

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t tx_byte = tx ? tx[i] : 0xFF;
        uint8_t rx_byte = 0;

        if (HAL_SPI_TransmitReceive(&_hspi, &tx_byte, &rx_byte, 1, HAL_MAX_DELAY) != HAL_OK)
        {
            if (rx)
                rx[i] = 0;
            continue;
        }

        if (rx)
            rx[i] = rx_byte;
    }
}

// ============================================================
//  单字节传输
// ============================================================

uint8_t spi_port::transfer_byte(uint8_t data)
{
    if (!_initialized)
        return 0;

    uint8_t rx_byte = 0;
    if (HAL_SPI_TransmitReceive(&_hspi, &data, &rx_byte, 1, HAL_MAX_DELAY) != HAL_OK)
        return 0;

    return rx_byte;
}

// ============================================================
//  片选控制
// ============================================================

void spi_port::cs_select()
{
    if (!_has_cs)
        return;

    if (_cfg.cs_active_level == (uint32_t)SET)
        _cs.high();
    else
        _cs.low();
}

void spi_port::cs_deselect()
{
    if (!_has_cs)
        return;

    if (_cfg.cs_active_level == (uint32_t)SET)
        _cs.low();
    else
        _cs.high();
}
