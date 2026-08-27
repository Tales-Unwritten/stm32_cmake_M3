#include "inter_usart.hpp"

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_usart.h"
#include "gd32f4xx_misc.h"

#include <new>
#include <cstring>

// ============================================================
//  文件内全局：端口映射表（ISR 通过它找到对应实例）
// ============================================================

namespace {

constexpr uint8_t UART_MAX = 8;

usart_port *g_port_map[UART_MAX] = {};

uint8_t _port_index(uint32_t periph)
{
    switch (periph)
    {
        case USART0: return 0;
        case USART1: return 1;
        case USART2: return 2;
        case UART3:  return 3;
        case UART4:  return 4;
        case USART5: return 5;
        case UART6:  return 6;
        case UART7:  return 7;
        default:     return UART_MAX;
    }
}

} // anonymous namespace

// ============================================================
//  注册 / 注销
// ============================================================

void usart_port::_register_port()
{
    uint8_t idx = _port_index(_cfg.usart_periph);
    if (idx < UART_MAX) g_port_map[idx] = this;
}

void usart_port::_unregister_port()
{
    uint8_t idx = _port_index(_cfg.usart_periph);
    if (idx < UART_MAX) g_port_map[idx] = nullptr;
}

// ============================================================
//  构造 / 析构
// ============================================================

usart_port::usart_port(const UsartPortConfig &cfg)
    : _cfg(cfg),
      _tx(cfg.tx_port, cfg.tx_pin),
      _rx(cfg.rx_port, cfg.rx_pin),
      _initialized(false),
      _rx_heap_buf(nullptr),
      _rx_buf_size(0),
      _buf{}
{
}

usart_port::~usart_port()
{
    deinit();
}

// ============================================================
//  外设映射
// ============================================================

void usart_port::_enable_clock()
{
    const auto p = _cfg.usart_periph;
    if      (p == USART0) rcu_periph_clock_enable(RCU_USART0);
    else if (p == USART1) rcu_periph_clock_enable(RCU_USART1);
    else if (p == USART2) rcu_periph_clock_enable(RCU_USART2);
    else if (p == UART3)  rcu_periph_clock_enable(RCU_UART3);
    else if (p == UART4)  rcu_periph_clock_enable(RCU_UART4);
    else if (p == USART5) rcu_periph_clock_enable(RCU_USART5);
    else if (p == UART6)  rcu_periph_clock_enable(RCU_UART6);
    else if (p == UART7)  rcu_periph_clock_enable(RCU_UART7);
}

IRQn_Type usart_port::_get_irq() const
{
    const auto p = _cfg.usart_periph;
    if      (p == USART0) return USART0_IRQn;
    else if (p == USART1) return USART1_IRQn;
    else if (p == USART2) return USART2_IRQn;
    else if (p == UART3)  return UART3_IRQn;
    else if (p == UART4)  return UART4_IRQn;
    else if (p == USART5) return USART5_IRQn;
    else if (p == UART6)  return UART6_IRQn;
    else if (p == UART7)  return UART7_IRQn;
    return USART0_IRQn;
}

// ============================================================
//  init / deinit
// ============================================================

void usart_port::init()
{
    if (_initialized) return;

    // [LEGACY] 堆分配缓冲区 - 历史遗留代码
    // 新外设应使用 usart_async_port 模板类（零堆分配）
    _rx_buf_size = _cfg.rx_buf_size;
    _rx_heap_buf = new (std::nothrow) uint8_t[_rx_buf_size];
    if (!_rx_heap_buf) return;

    // 绑定 buffer 视图
    _buf.rx_buf  = _rx_heap_buf;
    _buf.rx_len  = 0;
    _buf.rx_flag = 0;
    _buf.tx_busy = 0;

    // 时钟
    _enable_clock();

    // GPIO
    _tx.init(mode_af_pp, pullup, speed_high);
    _tx.set_af(_cfg.af);
    _rx.init(mode_af_pp, pullup, speed_high);
    _rx.set_af(_cfg.af);

    // USART 8N1
    usart_deinit(_cfg.usart_periph);
    usart_baudrate_set(_cfg.usart_periph, _cfg.baudrate);
    usart_parity_config(_cfg.usart_periph, USART_PM_NONE);
    usart_word_length_set(_cfg.usart_periph, USART_WL_8BIT);
    usart_stop_bit_set(_cfg.usart_periph, USART_STB_1BIT);

    // 中断：RXNE + IDLE + ERR
    nvic_irq_enable(_get_irq(), 0, 0);
    usart_interrupt_enable(_cfg.usart_periph, USART_INT_RBNE);
    usart_interrupt_enable(_cfg.usart_periph, USART_INT_IDLE);
    usart_interrupt_enable(_cfg.usart_periph, USART_INT_ERR);

    // 使能 USART
    usart_enable(_cfg.usart_periph);
    usart_transmit_config(_cfg.usart_periph, USART_TRANSMIT_ENABLE);
    usart_receive_config(_cfg.usart_periph, USART_RECEIVE_ENABLE);

    _register_port();
    _initialized = true;
}

void usart_port::deinit()
{
    if (!_initialized) return;

    _unregister_port();

    nvic_irq_disable(_get_irq());
    usart_interrupt_disable(_cfg.usart_periph, USART_INT_RBNE);
    usart_interrupt_disable(_cfg.usart_periph, USART_INT_IDLE);
    usart_interrupt_disable(_cfg.usart_periph, USART_INT_ERR);
    usart_interrupt_disable(_cfg.usart_periph, USART_INT_TBE);
    usart_interrupt_disable(_cfg.usart_periph, USART_INT_TC);

    usart_disable(_cfg.usart_periph);
    usart_deinit(_cfg.usart_periph);

    _tx.deinit();
    _rx.deinit();

    delete[] _rx_heap_buf;
    _rx_heap_buf = nullptr;
    _buf.rx_buf  = nullptr;
    _buf.rx_len  = 0;
    _buf.rx_flag = 0;
    _buf.tx_busy = 0;

    _initialized = false;
}

// ============================================================
//  阻塞发送（超时保护）
// ============================================================

bool usart_port::send_data(const uint8_t *data, uint16_t len)
{
    if (!_initialized || !data || len == 0) return false;

    usart_interrupt_disable(_cfg.usart_periph, USART_INT_TBE);
    usart_interrupt_disable(_cfg.usart_periph, USART_INT_TC);

    for (uint16_t i = 0; i < len; ++i)
    {
        uint32_t timeout = 0xFFFF;
        while (RESET == usart_flag_get(_cfg.usart_periph, USART_FLAG_TBE))
        {
            if (--timeout == 0) return false;   // 超时，防止硬件异常挂死
        }
        usart_data_transmit(_cfg.usart_periph, data[i]);
    }
    {
        uint32_t timeout = 0xFFFF;
        while (RESET == usart_flag_get(_cfg.usart_periph, USART_FLAG_TC))
        {
            if (--timeout == 0) return false;   // 超时，防止硬件异常挂死
        }
    }
    return true;
}

// ============================================================
//  中断发送（内部拷贝，安全）
// ============================================================

bool usart_port::send_data_it(const uint8_t *data, uint16_t len)
{
    if (!_initialized || !data || len == 0) return false;
    if (_buf.tx_busy) return false;            // 正在发送中
    if (len > TX_BUF_SIZE) return false;        // 内部缓冲区不足

    // 内部拷贝：防止用户释放缓冲区后 ISR 访问野指针
    memcpy(_tx_buf, data, len);
    _buf.tx_data = _tx_buf;                    // 指向内部缓冲区
    _buf.tx_len  = len;
    _buf.tx_idx  = 0;
    _buf.tx_busy = 1;

    usart_interrupt_enable(_cfg.usart_periph, USART_INT_TBE);
    usart_interrupt_enable(_cfg.usart_periph, USART_INT_TC);
    return true;
}

// ============================================================
//  通用 ISR 处理函数（由各 USARTx_IRQHandler 调用）
// ============================================================

static void _uart_isr(usart_port *port)
{
    if (!port || !port->is_initialized()) return;

    uint32_t         periph = port->periph();
    uart_buffer_t   *buf    = port->buffer();
    uint16_t         cap    = port->rx_buf_capacity();

    /* ---- 接收：RBNE ---- */
    if (usart_interrupt_flag_get(periph, USART_INT_FLAG_RBNE) == SET)
    {
        uint8_t data = (uint8_t)usart_data_receive(periph);

        if (buf->rx_len < cap)
        {
            buf->rx_buf[buf->rx_len++] = data;
        }
    }

    /* ---- 空闲帧：IDLE ---- */
    if (usart_interrupt_flag_get(periph, USART_INT_FLAG_IDLE) == SET)
    {
        usart_data_receive(periph);     // 读 DR 清除 IDLE 标志
        buf->rx_flag = 1;
    }

    /* ---- 错误处理 ---- */
    if (usart_interrupt_flag_get(periph, USART_INT_FLAG_ERR_ORERR) ||
        usart_interrupt_flag_get(periph, USART_INT_FLAG_ERR_FERR) ||
        usart_interrupt_flag_get(periph, USART_INT_FLAG_ERR_NERR) ||
        usart_interrupt_flag_get(periph, USART_INT_FLAG_PERR))
    {
        usart_data_receive(periph);
        usart_flag_clear(periph, USART_FLAG_ORERR);
        usart_flag_clear(periph, USART_FLAG_FERR);
        usart_flag_clear(periph, USART_FLAG_NERR);
        usart_flag_clear(periph, USART_FLAG_PERR);

        buf->rx_len = 0;  buf->rx_flag = 0;  // 丢弃残缺帧
    }

    /* ---- 发送：TBE ---- */
    if (usart_interrupt_flag_get(periph, USART_INT_FLAG_TBE) == SET)
    {
        if (buf->tx_idx < buf->tx_len)
        {
            usart_data_transmit(periph, buf->tx_data[buf->tx_idx++]);
        }
        else
        {
            usart_interrupt_disable(periph, USART_INT_TBE);
            usart_interrupt_enable(periph, USART_INT_TC);
        }
    }

    /* ---- 发送完成：TC ---- */
    if (usart_interrupt_flag_get(periph, USART_INT_FLAG_TC) == SET)
    {
        usart_flag_clear(periph, USART_FLAG_TC);
        usart_interrupt_disable(periph, USART_INT_TC);
        buf->tx_busy = 0;
    }
}

// ============================================================
//  各 USART 中断入口（全部内聚在此，用户无需外部编写）
// ============================================================

extern "C" void USART0_IRQHandler(void)
{
    _uart_isr(g_port_map[_port_index(USART0)]);
}

extern "C" void USART1_IRQHandler(void)
{
    _uart_isr(g_port_map[_port_index(USART1)]);
}

extern "C" void USART2_IRQHandler(void)
{
    _uart_isr(g_port_map[_port_index(USART2)]);
}

extern "C" void UART3_IRQHandler(void)
{
    _uart_isr(g_port_map[_port_index(UART3)]);
}

extern "C" void UART4_IRQHandler(void)
{
    _uart_isr(g_port_map[_port_index(UART4)]);
}

extern "C" void USART5_IRQHandler(void)
{
    _uart_isr(g_port_map[_port_index(USART5)]);
}

extern "C" void UART6_IRQHandler(void)
{
    _uart_isr(g_port_map[_port_index(UART6)]);
}

extern "C" void UART7_IRQHandler(void)
{
    _uart_isr(g_port_map[_port_index(UART7)]);
}
