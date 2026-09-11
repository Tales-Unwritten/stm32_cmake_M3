// ============================================================
// @platform STM32F1xx
//   基于 STM32 HAL 的硬件 SPI 主机端口
//   - 使用 io_ctrl 管理 GPIO
//   - 使用 SPI_HandleTypeDef + HAL_SPI_Init/TransmitReceive
// ============================================================

#include "inter_spi.hpp"

#include "inter_nvic.hpp"

#include <new> // placement new（DMA 通道对象就地构造，零堆分配）

// ── SPI → DMA 请求映射（F1 硬件固定，无软件选择寄存器） ──────
// 通道号为 0-based（即 DMA1_Channel2 → 1）。
namespace
{
struct spi_dma_req_t
{
    dma_id ctrl;
    int8_t rx_ch; // -1 = 该 SPI 无 DMA 映射
    int8_t tx_ch;
};

spi_dma_req_t _spi_dma_req(spi_id id)
{
    switch (id)
    {
    case spi_id::spi1:
        return {dma_id::dma1, 1, 2}; // RX=DMA1_CH2  TX=DMA1_CH3
    case spi_id::spi2:
        return {dma_id::dma1, 3, 4}; // RX=DMA1_CH4  TX=DMA1_CH5
    default:
        return {dma_id::dma1, -1, -1}; // spi3：F103 无此 SPI
    }
}
} // namespace

// ============================================================
//  构造 / 析构
// ============================================================

spi_port::spi_port(const SpiPortConfig &cfg)
    : _cfg(cfg), _sck(cfg.sck_port, cfg.sck_pin), _mosi(cfg.mosi_port, cfg.mosi_pin),
      _miso(cfg.miso_port, cfg.miso_pin),
      _cs(cfg.cs_port ? cfg.cs_port : GPIOA, cfg.cs_pin != pin_none ? cfg.cs_pin : pin0),
      _has_cs(cfg.cs_port != nullptr && cfg.cs_pin != pin_none), _initialized(false), _dma_rx(nullptr),
      _dma_tx(nullptr)
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
    _release_dma(); // 先停 SPI 再归还 DMA 通道

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

    _cs.write(_cfg.cs_active_level); // 选中 = 输出有效电平
}

void spi_port::cs_deselect()
{
    if (!_has_cs)
        return;

    _cs.write(_cfg.cs_active_level == active_low ? Hig : Low); // 释放 = 输出无效电平
}

// ============================================================
//  DMA 块传输（可选；通道由硬件请求映射决定，见 _spi_dma_req）
// ============================================================

void spi_port::_release_dma()
{
    if (_dma_rx != nullptr)
    {
        _dma_rx->~dma_channel();
        _dma_rx = nullptr;
    }
    if (_dma_tx != nullptr)
    {
        _dma_tx->~dma_channel();
        _dma_tx = nullptr;
    }
}

bool spi_port::enable_dma()
{
    if (dma_enabled())
        return true; // 幂等

    const spi_dma_req_t req = _spi_dma_req(_cfg.periph);
    if (req.rx_ch < 0)
        return false; // 该 SPI 无 DMA 映射（spi3）

    const IRQn_Type irq_rx = dma_channel::irq_of(req.ctrl, (uint8_t)req.rx_ch);
    const IRQn_Type irq_tx = dma_channel::irq_of(req.ctrl, (uint8_t)req.tx_ch);
    if ((irq_rx == dma_channel::IRQ_NONE) || (irq_tx == dma_channel::IRQ_NONE))
        return false;

    _dma_rx = new (_dma_storage.rx) dma_channel({req.ctrl, (uint8_t)req.rx_ch, DMA_PRIORITY_MEDIUM});
    _dma_tx = new (_dma_storage.tx) dma_channel({req.ctrl, (uint8_t)req.tx_ch, DMA_PRIORITY_MEDIUM});
    _dma_rx->init();
    _dma_tx->init();
    if (!_dma_rx->is_initialized() || !_dma_tx->is_initialized())
    {
        _release_dma();
        return false;
    }

    // 先把两个通道配好并挂到 SPI 句柄上（HAL 的 TransmitReceive_DMA 要求 hdmarx/hdmatx 已就位）
    _dma_rx->set_width(DMA_PDATAALIGN_BYTE);
    _dma_rx->set_direction(DMA_PERIPH_TO_MEMORY); // 外设 → 内存
    _dma_rx->set_increment(false, true);          // SPI->DR 地址固定，内存自增
    _dma_rx->set_circular(false);
    if (_dma_rx->hal_configure() != HAL_OK)
    {
        _release_dma();
        return false;
    }

    _dma_tx->set_width(DMA_PDATAALIGN_BYTE);
    _dma_tx->set_direction(DMA_MEMORY_TO_PERIPH); // 内存 → 外设
    _dma_tx->set_increment(true, false);          // 内存自增，SPI->DR 固定
    _dma_tx->set_circular(false);
    if (_dma_tx->hal_configure() != HAL_OK)
    {
        _release_dma();
        return false;
    }

    __HAL_LINKDMA(&_hspi, hdmarx, *_dma_rx->hal_handle());
    __HAL_LINKDMA(&_hspi, hdmatx, *_dma_tx->hal_handle());

    // 传输完成靠 DMA 中断驱动 HAL 回调 → 必须开 NVIC（ISR 由 inter_dma 统一路由）
    nvic().set_priority(irq_rx, 6, 0);
    nvic().enable(irq_rx);
    nvic().set_priority(irq_tx, 6, 0);
    nvic().enable(irq_tx);
    return true;
}

bool spi_port::transfer_dma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
    if (!_initialized || !dma_enabled() || (tx == nullptr) || (rx == nullptr) || (len == 0U))
        return false;

    // 上次传输未收尾（超时被中断等）：复位到 READY
    if (HAL_SPI_GetState(&_hspi) != HAL_SPI_STATE_READY)
        (void)HAL_SPI_Abort(&_hspi);

    if (HAL_SPI_TransmitReceive_DMA(&_hspi, const_cast<uint8_t *>(tx), rx, len) != HAL_OK)
        return false;

    // 完成由 SPI 的 DMA 回调（SPI_DMATransmitReceiveCplt）把状态置回 READY
    const uint32_t t0 = HAL_GetTick();
    while (HAL_SPI_GetState(&_hspi) != HAL_SPI_STATE_READY)
    {
        if ((uint32_t)(HAL_GetTick() - t0) > timeout_ms)
        {
            (void)HAL_SPI_Abort(&_hspi);
            return false;
        }
    }
    return true;
}
