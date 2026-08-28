#include "inter_usart.hpp"
#include "string.h"
#include <cstring>

// ============================================================
//  文件内全局：端口映射表（ISR 通过它找到对应实例）
// ============================================================

namespace
{

constexpr uint8_t UART_MAX = 5;

// 当前默认实例化为 usart_port<256>，如需其他 RX 缓冲区大小，
// 请在文件底部增加对应显式实例化，并同步调整 g_port_map 类型。
usart_port_base *g_port_map[UART_MAX] = {};

uint8_t _usart_index(usart_enum_t id)
{
    switch (id)
    {
    case usart1:
        return 0;
    case usart2:
        return 1;
    case usart3:
        return 2;
    case uart4:
        return 3;
    case uart5:
        return 4;
    default:
        return UART_MAX;
    }
}

void _usart_clock_enable(usart_enum_t id)
{
    switch (id)
    {
    case usart1:
        __HAL_RCC_USART1_CLK_ENABLE();
        break;
    case usart2:
        __HAL_RCC_USART2_CLK_ENABLE();
        break;
    case usart3:
        __HAL_RCC_USART3_CLK_ENABLE();
        break;
    case uart4:
        __HAL_RCC_UART4_CLK_ENABLE();
        break;
    case uart5:
        __HAL_RCC_UART5_CLK_ENABLE();
        break;
    }
}

void _usart_clock_disable(usart_enum_t id)
{
    switch (id)
    {
    case usart1:
        __HAL_RCC_USART1_CLK_DISABLE();
        break;
    case usart2:
        __HAL_RCC_USART2_CLK_DISABLE();
        break;
    case usart3:
        __HAL_RCC_USART3_CLK_DISABLE();
        break;
    case uart4:
        __HAL_RCC_UART4_CLK_DISABLE();
        break;
    case uart5:
        __HAL_RCC_UART5_CLK_DISABLE();
        break;
    }
}

IRQn_Type _usart_irq(usart_enum_t id)
{
    switch (id)
    {
    case usart1:
        return USART1_IRQn;
    case usart2:
        return USART2_IRQn;
    case usart3:
        return USART3_IRQn;
    case uart4:
        return UART4_IRQn;
    case uart5:
        return UART5_IRQn;
    default:
        return USART1_IRQn;
    }
}

} // anonymous namespace

// ============================================================
//  平台映射：usart_enum_t -> USART_TypeDef*
// ============================================================

USART_TypeDef *usart_periph_ptr(usart_enum_t id)
{
    switch (id)
    {
    case usart1:
        return USART1;
    case usart2:
        return USART2;
    case usart3:
        return USART3;
    case uart4:
        return UART4;
    case uart5:
        return UART5;
    default:
        return USART1;
    }
}

// ============================================================
//  注册 / 注销
// ============================================================

template <uint16_t RX_BUF_SIZE> void usart_port<RX_BUF_SIZE>::_register_port()
{
    uint8_t idx = _usart_index(_cfg.usart_periph);
    if (idx < UART_MAX)
        g_port_map[idx] = this;
}

template <uint16_t RX_BUF_SIZE> void usart_port<RX_BUF_SIZE>::_unregister_port()
{
    uint8_t idx = _usart_index(_cfg.usart_periph);
    if (idx < UART_MAX)
        g_port_map[idx] = nullptr;
}

// ============================================================
//  构造 / 析构
// ============================================================

template <uint16_t RX_BUF_SIZE>
usart_port<RX_BUF_SIZE>::usart_port(const UsartPortConfig &cfg)
    : _cfg(cfg), _tx(cfg.tx_port, cfg.tx_pin), _rx(cfg.rx_port, cfg.rx_pin), _initialized(false), _buf{}
{
    _buf.rx_buf = _rx_buf;
}

template <uint16_t RX_BUF_SIZE> usart_port<RX_BUF_SIZE>::~usart_port()
{
    deinit();
}

// ============================================================
//  外设映射
// ============================================================

template <uint16_t RX_BUF_SIZE> void usart_port<RX_BUF_SIZE>::_enable_clock()
{
    _usart_clock_enable(_cfg.usart_periph);
}

template <uint16_t RX_BUF_SIZE> IRQn_Type usart_port<RX_BUF_SIZE>::_get_irq() const
{
    return _usart_irq(_cfg.usart_periph);
}

// ============================================================
//  init / deinit
// ============================================================

template <uint16_t RX_BUF_SIZE> void usart_port<RX_BUF_SIZE>::init()
{
    if (_initialized)
        return;

    // 绑定 buffer 视图（零堆分配）
    _buf.rx_buf = _rx_buf;
    _buf.rx_len = 0;
    _buf.rx_flag = 0;
    _buf.tx_busy = 0;

    // 时钟
    _enable_clock();

    // GPIO
    _tx.init(mode_af_pp, pullup, speed_high);
    _tx.set_af(_cfg.af);
    _rx.init(mode_af_pp, pullup, speed_high);
    _rx.set_af(_cfg.af);

    USART_TypeDef *usart = usart_periph_ptr(_cfg.usart_periph);

    // 先复位 USART 寄存器
    usart->CR1 = 0;
    usart->CR2 = 0;
    usart->CR3 = 0;

    // ── 波特率计算（STM32 USART BRR）────────────────────────────
    // 原理：
    //   1. USARTDIV = PCLK / (16 × BaudRate)
    //      这是 USART 实际分频系数，可能是小数。
    //   2. BRR 寄存器用 4bit 小数位表示 USARTDIV：
    //      BRR = USARTDIV × 16 = PCLK / BaudRate
    //   3. 为了减小误差，对 PCLK / BaudRate 做四舍五入：
    //      BRR = (PCLK + BaudRate/2) / BaudRate
    //
    // 例如：
    //   PCLK = 72MHz, BaudRate = 115200
    //   PCLK / BaudRate = 625.0       → BRR = 625
    //   PCLK = 16MHz,  BaudRate = 9600
    //   PCLK / BaudRate = 1666.666... → BRR = 1667（四舍五入）
    //
    // USART1 挂载在 APB2（PCLK2），USART2/3、UART4/5 挂载在 APB1（PCLK1）。
    uint32_t pclk = (usart == USART1) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    usart->BRR = (pclk + _cfg.baudrate / 2U) / _cfg.baudrate;

    // 8N1，TX/RX 使能，无硬件流控
    usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    usart->CR2 = 0;             // 1 Stop bit
    usart->CR3 = USART_CR3_EIE; // 错误中断使能

    // 接收/空闲/错误中断
    usart->CR1 |= USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_PEIE;

    HAL_NVIC_SetPriority(_get_irq(), 0, 0);
    HAL_NVIC_EnableIRQ(_get_irq());

    _register_port();
    _initialized = true;
}

template <uint16_t RX_BUF_SIZE> void usart_port<RX_BUF_SIZE>::deinit()
{
    if (!_initialized)
        return;

    _unregister_port();

    HAL_NVIC_DisableIRQ(_get_irq());

    USART_TypeDef *usart = usart_periph_ptr(_cfg.usart_periph);
    usart->CR1 = 0;
    usart->CR3 = 0;

    _tx.deinit();
    _rx.deinit();

    _buf.rx_buf = nullptr;
    _buf.rx_len = 0;
    _buf.rx_flag = 0;
    _buf.tx_busy = 0;

    _usart_clock_disable(_cfg.usart_periph);
    _initialized = false;
}

// ============================================================
//  阻塞发送（超时保护）
// ============================================================

template <uint16_t RX_BUF_SIZE> bool usart_port<RX_BUF_SIZE>::send_data(const uint8_t *data, uint16_t len)
{
    if (!_initialized || !data || len == 0)
        return false;

    USART_TypeDef *usart = usart_periph_ptr(_cfg.usart_periph);

    // 关闭发送中断，避免和阻塞发送冲突
    usart->CR1 &= ~USART_CR1_TXEIE;
    usart->CR1 &= ~USART_CR1_TCIE;

    for (uint16_t i = 0; i < len; ++i)
    {
        uint32_t timeout = 0xFFFF;
        while (!(usart->SR & USART_SR_TXE))
        {
            if (--timeout == 0)
                return false;
        }
        usart->DR = data[i];
    }

    {
        uint32_t timeout = 0xFFFF;
        while (!(usart->SR & USART_SR_TC))
        {
            if (--timeout == 0)
                return false;
        }
    }
    return true;
}

// ============================================================
//  中断发送（内部拷贝，安全）
// ============================================================

template <uint16_t RX_BUF_SIZE> bool usart_port<RX_BUF_SIZE>::send_data_it(const uint8_t *data, uint16_t len)
{
    if (!_initialized || !data || len == 0)
        return false;
    if (_buf.tx_busy)
        return false;
    if (len > TX_BUF_SIZE)
        return false;

    memcpy(_tx_buf, data, len);
    _buf.tx_data = _tx_buf;
    _buf.tx_len = len;
    _buf.tx_idx = 0;
    _buf.tx_busy = 1;

    USART_TypeDef *usart = usart_periph_ptr(_cfg.usart_periph);
    usart->CR1 |= USART_CR1_TXEIE;
    usart->CR1 |= USART_CR1_TCIE;
    return true;
}

// ============================================================
//  通用 ISR 处理函数（由各 USARTx_IRQHandler 调用）
// ============================================================

static void _uart_isr(usart_port_base *port)
{
    if (!port || !port->is_initialized())
        return;

    USART_TypeDef *usart = (USART_TypeDef *)port->periph();
    uart_buffer_t *buf = port->buffer();
    uint16_t cap = port->rx_buf_capacity();
    uint32_t sr = usart->SR;

    /* ---- 接收：RXNE ---- */
    if (sr & USART_SR_RXNE)
    {
        uint8_t data = (uint8_t)usart->DR;
        if (buf->rx_len < cap)
        {
            uint16_t idx = buf->rx_len;
            buf->rx_buf[idx] = data;
            buf->rx_len = idx + 1;
        }
    }

    /* ---- 空闲帧：IDLE ---- */
    if (sr & USART_SR_IDLE)
    {
        (void)usart->SR;
        (void)usart->DR;
        buf->rx_flag = 1;
    }

    /* ---- 错误处理 ---- */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        (void)usart->SR;
        (void)usart->DR;
        buf->rx_len = 0;
        buf->rx_flag = 0;
    }

    /* ---- 发送：TXE ---- */
    if (sr & USART_SR_TXE)
    {
        if (buf->tx_idx < buf->tx_len)
        {
            uint16_t idx = buf->tx_idx;
            usart->DR = buf->tx_data[idx];
            buf->tx_idx = idx + 1;
        }
        else
        {
            usart->CR1 &= ~USART_CR1_TXEIE;
            usart->CR1 |= USART_CR1_TCIE;
        }
    }

    /* ---- 发送完成：TC ---- */
    if (sr & USART_SR_TC)
    {
        usart->SR &= ~USART_SR_TC;
        usart->CR1 &= ~USART_CR1_TCIE;
        buf->tx_busy = 0;
    }
}

// ============================================================
//  各 USART 中断入口（全部内聚在此，用户无需外部编写）
// ============================================================

extern "C" void USART1_IRQHandler(void)
{
    _uart_isr(g_port_map[_usart_index(usart1)]);
}

extern "C" void USART2_IRQHandler(void)
{
    _uart_isr(g_port_map[_usart_index(usart2)]);
}

extern "C" void USART3_IRQHandler(void)
{
    _uart_isr(g_port_map[_usart_index(usart3)]);
}

extern "C" void UART4_IRQHandler(void)
{
    _uart_isr(g_port_map[_usart_index(uart4)]);
}

extern "C" void UART5_IRQHandler(void)
{
    _uart_isr(g_port_map[_usart_index(uart5)]);
}

// ============================================================
//  显式实例化：默认 256 字节接收缓冲区
//  如需要使用其他大小，请在此追加，例如：
//    template class usart_port<128>;
//    template class usart_port<512>;
// ============================================================

template class usart_port<256>;
// template class usart_port<512>;
