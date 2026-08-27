#pragma once

#ifdef __cplusplus

#include <cstdint>
#include "inter_io_ctrl.hpp"

/**
 * @brief  USART 端口引脚配置
 */
struct UsartPortConfig
{
    uint32_t usart_periph;  ///< USART0 / USART1 / USART2 / UART3 / UART4 / USART5 / UART6 / UART7
    GPIO_TypeDef* tx_port;  ///< TX GPIO 端口
    pin_enum_t    tx_pin;   ///< TX 引脚掩码
    GPIO_TypeDef* rx_port;  ///< RX GPIO 端口
    pin_enum_t    rx_pin;   ///< RX 引脚掩码
    afio_enum_t   af;       ///< STM32F1 AFIO 重映射选项
    uint32_t baudrate;      ///< 波特率
    uint16_t rx_buf_size;   ///< 接收缓冲区字节数
};

/**
 * @brief  串口收发缓冲区（用户通过 buffer() 获取引用）
 *
 * ISR 在接收空闲帧（IDLE）后将 rx_flag 置 1，用户在外部轮询此标志。
 * 处理完毕后用户自行将 rx_flag 清 0、rx_len 清 0。
 */
struct uart_buffer_t
{
    /* ── 接收（ISR → 用户） ── */
    uint8_t        *rx_buf;      ///< 接收缓冲区指针
    volatile uint16_t rx_len;    ///< 当前帧数据长度
    volatile uint8_t  rx_flag;   ///< 0=空闲  1=帧就绪（IDLE 触发）

    /* ── 发送（ISR 内部使用，用户只读 tx_busy） ── */
    const uint8_t   *tx_data;    ///< 用户待发送数据指针
    volatile uint16_t tx_len;   ///< 待发送总字节数
    volatile uint16_t tx_idx;   ///< 当前发送位置
    volatile uint8_t  tx_busy;   ///< 0=空闲  1=中断发送进行中
};

// ============================================================

class usart_port
{
public:
    static constexpr uint16_t TX_BUF_SIZE = 256;   ///< 内部发送缓冲区大小

    explicit usart_port(const UsartPortConfig &cfg);
    ~usart_port();

    usart_port(const usart_port &) = delete;
    usart_port &operator=(const usart_port &) = delete;

    // ── 生命周期 ──────────────────────────────────────────

    void init();
    void deinit();
    [[nodiscard]] bool is_initialized() const noexcept { return _initialized; }

    // ── 发送 ──────────────────────────────────────────────

    /** @brief 阻塞发送（超时保护）
     *  @return true=发送成功  false=参数错误或超时 */
    bool send_data(const uint8_t *data, uint16_t len);

    /** @brief 中断发送（非阻塞），通过 buf->tx_busy 判断完成
     *  @return true=已启动  false=参数错误或正在发送中 */
    bool send_data_it(const uint8_t *data, uint16_t len);

    // ── 缓冲区访问 ────────────────────────────────────────

    /** @brief 获取收发缓冲区引用（轮询 buf.rx_flag 和 buf.tx_busy） */
    [[nodiscard]] uart_buffer_t *buffer() noexcept { return &_buf; }

    /** @brief 外设基址（ISR 内部查询用，外部无需关心） */
    [[nodiscard]] uint32_t periph() const noexcept { return _cfg.usart_periph; }

    /** @brief 接收缓冲区容量（ISR 内部查询用，外部无需关心） */
    [[nodiscard]] uint16_t rx_buf_capacity() const noexcept { return _rx_buf_size; }

private:
    void      _enable_clock();
    IRQn_Type _get_irq() const;

    void _register_port();
    void _unregister_port();

    // ── 成员 ──────────────────────────────────────────────

    UsartPortConfig _cfg;
    io_ctrl         _tx;
    io_ctrl         _rx;
    bool            _initialized;

    // 动态分配的接收缓冲区（内部拥有）
    uint8_t        *_rx_heap_buf;
    uint16_t        _rx_buf_size;

    // 暴露给用户和 ISR 的缓冲区视图
    uart_buffer_t   _buf;

    // 内部发送缓冲区（零堆分配，防止用户指针生命周期悬空）
    uint8_t         _tx_buf[TX_BUF_SIZE];
};

#endif /* __cplusplus */
