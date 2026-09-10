#pragma once
// ============================================================
// @platform STM32F103xx（STM32F10xxx / Cortex-M3，基于 STM32F1xx HAL 库）
//   USART 寄存器级收发驱动：配置直接写寄存器（不经 HAL UART 句柄）。
//   接收 = RXNE 收字节 + IDLE 判帧；发送 = 阻塞 / 中断两种模式。
//   实现见 inter_usart.cpp。
// ============================================================

#include "inter_io_ctrl.hpp" // 引入 stm32f1xx_hal.h 与 GPIO/pin/pull/afio 类型
#include <cstdint>

// ════════════════════════════════════════════════════════════
//  典型用法（完整示例见 device_serial.cpp / 各 device 层驱动）
// ════════════════════════════════════════════════════════════
//
//     // 全局对象（注意：构造不初始化外设，须先调用 init()）
//     usart_port<256> debug_uart({usart1, GPIOA, pin9, GPIOA, pin10,
//                                 afio_enum_t::NONE, 115200});
//
//     debug_uart.init();
//
//     // 发送：阻塞式 / 中断式（后者用 buffer()->tx_busy 判断完成）
//     debug_uart.send_data("AT\r\n", 5);
//     debug_uart.send_data_it(tx_buf, len);
//
//     // 接收：轮询 rx_flag（ISR 在 IDLE 空闲帧时置 1），
//     // 处理完自行清 rx_len 与 rx_flag
//     uart_buffer_t *b = debug_uart.buffer();
//     if (b->rx_flag) {
//         uint16_t n = b->rx_len;          /* 帧数据在 b->rx_buf[0..n) */
//         b->rx_len = 0;
//         b->rx_flag = 0;
//     }
// ════════════════════════════════════════════════════════════

#ifdef __cplusplus

/**
 * @brief USART 外设编号
 * @note  枚举值 0~4 即平台映射表下标（见 inter_usart.cpp k_usart_map），
 *        勿调整顺序
 */
enum usart_enum_t : uint8_t
{
    usart1,
    usart2,
    usart3,
    uart4,
    uart5,
};

/**
 * @brief 获取 USART 外设指针（由平台实现）
 * @note  非法编号回退 USART1（与实现内部 default 分支一致）
 */
USART_TypeDef *usart_periph_ptr(usart_enum_t id);

/**
 * @brief USART 端口引脚配置
 * @note  字段顺序即聚合初始化顺序：调用方（device_serial.cpp / iap_boot.cpp）
 *        按位置初始化，勿重排、勿增删字段、勿加默认值
 */
struct UsartPortConfig
{
    usart_enum_t usart_periph;  ///< usart1 / usart2 / usart3 / uart4 / uart5
    GPIO_TypeDef *tx_port;      ///< TX GPIO 端口
    pin_enum_t   tx_pin;        ///< TX 引脚掩码
    GPIO_TypeDef *rx_port;      ///< RX GPIO 端口
    pin_enum_t   rx_pin;        ///< RX 引脚掩码
    afio_enum_t  af;            ///< STM32F1 AFIO 重映射选项
    uint32_t     baudrate;      ///< 波特率
    uint32_t     preempt_priority; ///< NVIC 抢占优先级
    uint32_t     sub_priority;     ///< NVIC 子优先级
};

/**
 * @brief 串口收发缓冲区（用户通过 buffer() 获取引用）
 *
 * 接收：ISR 在空闲帧（IDLE）后将 rx_flag 置 1，用户在外部轮询此标志，
 * 处理完毕后自行将 rx_flag 清 0、rx_len 清 0。
 *
 * 发送：send_data_it() 启动后 ISR 用 tx_data/tx_len/tx_idx 内部续发，
 * 用户只读 tx_busy（1=发送中，0=空闲可再发）。
 *
 * @note 字段顺序即内存布局契约：protocol/modbus 的 modbus_t 头部按
 *       同序对接复用，勿重排字段
 */
struct uart_buffer_t
{
    /* ── 接收（ISR → 用户） ── */
    uint8_t        *rx_buf;   ///< 接收缓冲区指针
    volatile uint16_t rx_len; ///< 当前帧数据长度
    volatile uint8_t  rx_flag; ///< 0=空闲  1=帧就绪（IDLE 触发）

    /* ── 发送（ISR 内部使用，用户只读 tx_busy） ── */
    const uint8_t  *tx_data;   ///< 待发送数据指针（指向内部 _tx_buf）
    volatile uint16_t tx_len;  ///< 待发送总字节数
    volatile uint16_t tx_idx;  ///< 当前发送位置
    volatile uint8_t  tx_busy; ///< 0=空闲  1=中断发送进行中
};

// ════════════════════════════════════════════════════════════
//  串口基类：让 ISR 能统一持有不同 RX 缓冲区大小的实例
//  （g_port_map 存基类指针，ISR 经虚函数取外设/缓冲/容量）
// ════════════════════════════════════════════════════════════

class usart_port_base
{
public:
    virtual ~usart_port_base() = default;

    // ── 生命周期 ──────────────────────────────────────────
    virtual void init() = 0;
    virtual void deinit() = 0;
    [[nodiscard]] virtual bool is_initialized() const noexcept = 0;

    // ── 发送 ──────────────────────────────────────────────
    virtual bool send_data(const uint8_t *data, uint16_t len) = 0;
    virtual bool send_data_it(const uint8_t *data, uint16_t len) = 0;

    // ── ISR 内部查询（外部无需关心） ──────────────────────
    virtual uart_buffer_t *buffer() noexcept = 0;
    virtual uint32_t periph() const noexcept = 0;
    virtual uint16_t rx_buf_capacity() const noexcept = 0;
};

// ════════════════════════════════════════════════════════════
//  串口端口：模板参数 = 编译期接收缓冲区大小（零堆分配）
//  默认 256 字节；其它尺寸需在 inter_usart.cpp 底部显式实例化
// ════════════════════════════════════════════════════════════

template <uint16_t RX_BUF_SIZE = 256>
class usart_port : public usart_port_base
{
public:
    static constexpr uint16_t TX_BUF_SIZE = 256; ///< 内部发送缓冲区大小（固定）

    explicit usart_port(const UsartPortConfig &cfg);
    ~usart_port();

    usart_port(const usart_port &) = delete;
    usart_port &operator=(const usart_port &) = delete;

    // ── 生命周期 ──────────────────────────────────────────

    void init() override;
    void deinit() override;
    [[nodiscard]] bool is_initialized() const noexcept override
    {
        return _initialized;
    }

    // ── 发送 ──────────────────────────────────────────────

    /** @brief 阻塞发送（超时保护）
     *  @return true=发送成功  false=参数错误或超时 */
    bool send_data(const uint8_t *data, uint16_t len) override;

    /** @brief 中断发送（非阻塞），通过 buffer()->tx_busy 判断完成
     *  @return true=已启动  false=参数错误或正在发送中 */
    bool send_data_it(const uint8_t *data, uint16_t len) override;

    // ── 缓冲区访问 ────────────────────────────────────────

    /** @brief 获取收发缓冲区引用（轮询 buf.rx_flag 和 buf.tx_busy） */
    [[nodiscard]] uart_buffer_t *buffer() noexcept override
    {
        return &_buf;
    }

    /** @brief 外设基址（ISR 内部查询用，外部无需关心） */
    [[nodiscard]] uint32_t periph() const noexcept override
    {
        return (uint32_t)usart_periph_ptr(_cfg.usart_periph);
    }

    /** @brief 接收缓冲区容量（编译期模板大小） */
    [[nodiscard]] uint16_t rx_buf_capacity() const noexcept override
    {
        return RX_BUF_SIZE;
    }

private:
    // ── 内部助手 ──────────────────────────────────────────

    void _enable_clock();
    IRQn_Type _get_irq() const;
    void _register_port();
    void _unregister_port();

    // ── 成员 ──────────────────────────────────────────────

    UsartPortConfig _cfg;
    io_ctrl _tx;
    io_ctrl _rx;
    bool _initialized;

    // 编译期接收缓冲区（零堆分配）
    uint8_t _rx_buf[RX_BUF_SIZE];

    // 暴露给用户和 ISR 的缓冲区视图
    uart_buffer_t _buf;

    // 内部发送缓冲区（零堆分配，防止用户指针生命周期悬空）
    uint8_t _tx_buf[TX_BUF_SIZE];
};

#endif /* __cplusplus */
