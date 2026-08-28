// ============================================================
// @platform STM32F1xx
//   基于 STM32 HAL 的硬件 SPI 主机端口
//   - 使用 io_ctrl 管理 GPIO
//   - 使用 SPI_HandleTypeDef + HAL_SPI_Init/TransmitReceive
// ============================================================

#include "inter_spi.hpp"

// ============================================================
//  构造 / 析构
// ============================================================

spi_port::spi_port(const SpiPortConfig &cfg)
    : _cfg(cfg), _sck(cfg.sck_port, cfg.sck_pin), _mosi(cfg.mosi_port, cfg.mosi_pin),
      _miso(cfg.miso_port, cfg.miso_pin),
      _cs(cfg.cs_port ? cfg.cs_port : GPIOA, cfg.cs_pin != pin_none ? cfg.cs_pin : pin0),
      _has_cs(cfg.cs_port != nullptr && cfg.cs_pin != pin_none), _initialized(false)
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

    _hspi.Init.Mode = SPI_MODE_MASTER;
    _hspi.Init.Direction = SPI_DIRECTION_2LINES;
    _hspi.Init.DataSize = _cfg.data_size;
    _hspi.Init.CLKPolarity = _cfg.clock_polarity;
    _hspi.Init.CLKPhase = _cfg.clock_phase;
    _hspi.Init.NSS = SPI_NSS_SOFT;
    _hspi.Init.BaudRatePrescaler = _cfg.prescaler;
    _hspi.Init.FirstBit = _cfg.first_bit;
    _hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    _hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    _hspi.Init.CRCPolynomial = 10;

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

/**
 * @brief  SPI 全双工传输函数（逐字节收发）
 * @param  tx   发送数据缓冲区指针（可为 nullptr，此时发送 0xFF）
 * @param  rx   接收数据缓冲区指针（可为 nullptr，此时丢弃接收数据）
 * @param  len  传输字节数
 * @note   该函数以阻塞方式逐字节进行 SPI 全双工通信，
 *         每次调用 HAL_SPI_TransmitReceive 传输 1 字节。
 *         适用于数据量较小、对实时性要求不高的场景。
 */
void spi_port::transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    // 初始化检查：若 SPI 未初始化或传输长度为 0，直接返回
    if (!_initialized || len == 0)
        return;

    // 逐字节循环传输
    for (uint16_t i = 0; i < len; i++)
    {
        // 若发送缓冲区有效，取当前字节；否则发送 0xFF（SPI 空闲电平）
        uint8_t tx_byte = tx ? tx[i] : 0xFF;

        // 接收字节临时变量，由 HAL 函数填充
        uint8_t rx_byte = 0;

        // 调用 HAL 库进行阻塞式 SPI 全双工收发（传输 1 字节）
        // 参数：SPI 句柄、发送指针、接收指针、长度、超时时间（永久等待）
        if (HAL_SPI_TransmitReceive(&_hspi, &tx_byte, &rx_byte, 1, HAL_MAX_DELAY) != HAL_OK)
        {
            // 传输失败：若接收缓冲区有效，将当前字节置 0 表示错误
            if (rx)
                rx[i] = 0;

            // 跳过本次，继续下一字节（不中断整个传输过程）
            continue;
        }

        // 传输成功：若接收缓冲区有效，存入接收到的数据
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
