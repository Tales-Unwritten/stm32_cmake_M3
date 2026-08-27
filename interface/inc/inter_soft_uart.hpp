#pragma once

#ifdef __cplusplus

#include <cstdint>
#include "inter_io_ctrl.hpp"
#include "inter_usart.hpp"   // uart_buffer_t（与硬件串口同构的缓冲类型，协议层零适配）

/**
 * @brief 软件串口（GPIO 位反转，DWT 周期计数精确时序）
 *
 * 架构对齐 interface 层规范：
 *  - TX/RX 引脚由 io_ctrl 管理（铁律 2.5：GPIO 唯一抽象）
 *  - DWT 周期计数器提供精确位时序（Cortex-M3/4/7 标准调试模块）
 *  - 零堆分配、禁拷贝、init/deinit 生命周期管理
 *  - 主频自动跟随 SystemCoreClock（改时钟配置无需改本驱动）
 *
 * 原实现来源：soft_uart（纯 C 版），改写为 C++ inter 风格。
 *
 * 波特率精度 @240MHz（SystemCoreClock 自动适配）：
 *   9600: 25000 周期/bit（误差 0.00%）  19200: 12500（0.00%）
 *   38400: 6250（0.00%）  57600: 4167（0.02%）  115200: 2083（0.02%）
 */
struct SoftUartConfig
{
    GPIO_TypeDef* tx_port;         // TX 端口，如 GPIOB
    pin_enum_t    tx_pin;          // TX 引脚掩码，如 pin6
    GPIO_TypeDef* rx_port;         // RX 端口
    pin_enum_t    rx_pin;          // RX 引脚掩码
    uint32_t baud = 115200;        // 波特率（见 BAUD_xxx 常量）
};

/**
 * @brief 串口收发缓冲区（与 usart_port 的 uart_buffer_t 同构，协议层零适配）
 *
 * 软串口无 ISR：由 poll() 在主循环中收字节并置 rx_flag（3.5 字符空闲判帧）。
 * 用户处理完毕后自行将 rx_flag 清 0、rx_len 清 0。
 *
 * 直接复用 uart_buffer_t 类型（与 usart_port 完全一致），
 * Modbus 等协议层的 buffer() 参数无需任何类型转换。
 */

/**
 * @brief 软件串口端口（阻塞发送 + 轮询接收）
 *
 * 使用示例：
 *   static soft_uart_port suart({
 *       GPIOB, pin6, GPIOB, pin7,
 *       soft_uart_port::BAUD_115200
 *   });
 *   suart.init();
 *   suart.puts("hello\r\n");
 *
 *   uint8_t b;
 *   if (suart.try_getc(&b)) { ... }        // 非阻塞轮询
 *   if (suart.getc(&b, 100)) { ... }       // 阻塞 100ms 超时
 *
 * 注意：
 *  - try_getc() 无起始位时立即返回；检测到起始位后阻塞约 10 位时间
 *    接收完整字节。应在主循环中尽可能频繁调用（两字节间隔期间
 *    不监控 RX 线）。
 *  - 发送期间关闭中断可避免 ISR 抢占导致位时序拉长（如需）。
 */
class soft_uart_port
{
public:
    // ── 波特率常量 ──────────────────────────────────────────
    static constexpr uint32_t BAUD_9600   = 9600U;
    static constexpr uint32_t BAUD_19200  = 19200U;
    static constexpr uint32_t BAUD_38400  = 38400U;
    static constexpr uint32_t BAUD_57600  = 57600U;
    static constexpr uint32_t BAUD_115200 = 115200U;

    explicit soft_uart_port(const SoftUartConfig &cfg);
    ~soft_uart_port();

    soft_uart_port(const soft_uart_port &) = delete;
    soft_uart_port &operator=(const soft_uart_port &) = delete;

    // ── 生命周期 ──────────────────────────────────────────

    void init();                   // 使能 DWT + 配置引脚（幂等）
    void deinit();                 // 引脚恢复模拟输入，可再次 init()
    [[nodiscard]] bool is_initialized() const noexcept { return _initialized; }

    // ── 发送（阻塞，约 10 位周期/字节；@115200 ≈ 87µs） ────

    void putc(uint8_t byte);       // 单字节（起始位 + 8 数据位 + 停止位）
    void puts(const char *str);    // 字符串
    void write(const uint8_t *data, uint16_t len);   // 原始缓冲区

    // ── 接收 ──────────────────────────────────────────────

    /**
     * @brief 非阻塞轮询接收（下降沿检测 + 毛刺过滤 + 中点采样）
     * @param byte [输出] 接收字节
     * @return true = 收到完整字节；false = 无起始位
     * @note  无数据时 < 1µs 返回；有数据时阻塞约 9.5 位周期
     */
    bool try_getc(uint8_t *byte);

    /**
     * @brief 阻塞接收，带超时
     * @param byte       [输出] 接收字节
     * @param timeout_ms 超时毫秒（0 = 无限等待）
     * @return true = 收到；false = 超时
     */
    bool getc(uint8_t *byte, uint32_t timeout_ms = 0);

    // ── 运行时配置 ────────────────────────────────────────

    void set_baud(uint32_t baud);  // 运行时改波特率（重新计算位周期）

    // ════════════════════════════════════════════════════════
    //  以下接口与 usart_port（硬件串口）签名对齐，
    //  协议层（Modbus 等）无需区分硬/软串口即可桥接。
    // ════════════════════════════════════════════════════════

    // ── 发送（对齐 usart_port） ───────────────────────────

    /** @brief 阻塞发送（参数校验失败返回 false），等价 write() */
    bool send_data(const uint8_t *data, uint16_t len);

    /**
     * @brief 发送（对齐 usart_port 签名）
     * @note  软串口无硬件中断发送能力：本方法退化为阻塞发送，
     *        返回时数据已全部发出（接口兼容，语义同步）
     */
    bool send_data_it(const uint8_t *data, uint16_t len);

    // ── 缓冲区访问（对齐 usart_port） ─────────────────────

    /** @brief 获取收发缓冲区引用（类型与 usart_port 完全相同，协议层零适配） */
    [[nodiscard]] uart_buffer_t *buffer() noexcept { return &_buf; }

    /** @brief 无硬件外设，恒返回 0（签名对齐 usart_port::periph()） */
    [[nodiscard]] uint32_t periph() const noexcept { return 0; }

    /** @brief 接收缓冲区容量（256 字节） */
    [[nodiscard]] uint16_t rx_buf_capacity() const noexcept { return RX_BUF_SIZE; }

    /**
     * @brief 接收轮询（主循环必须周期调用）
     *
     * 内部 try_getc() 收字节入 rx_buf；连续 3.5 字符时间（Modbus RTU
     * 标准帧间隔）无新字节时置 rx_flag=1（模拟硬件 IDLE 中断判帧）。
     *
     * @return true = 当前有完整帧待处理（rx_flag==1）
     * @note  处理完帧后由用户清 rx_flag/rx_len
     */
    bool poll();

    /**
     * @brief 帧空闲判定阈值（20 字符）
     * @note  软串口为主循环轮询接收：判帧阈值必须大于主循环周期，
     *        否则帧内字节间隙（轮询间隔）会被误判为帧结束。
     *        3.5 字符（Modbus 标准）仅适用于硬件 ISR 场景。
     */
    static constexpr uint8_t FRAME_IDLE_CHARS = 20;

private:
    static constexpr uint16_t RX_BUF_SIZE = 256;   ///< 接收缓冲区（Modbus ADU 上限）

    void _dwt_enable();            // 使能 DWT 周期计数器
    void _delay_cycles(uint32_t cycles);   // 精确忙等 N 周期（溢出安全）

    io_ctrl _tx;
    io_ctrl _rx;
    uint32_t _baud;
    uint32_t _cycles_per_bit;      // 一位周期 = SystemCoreClock / baud
    uint8_t  _rx_prev;             // 上一 RX 状态（边沿检测）
    bool     _initialized;

    // ── 协议桥接（与 usart_port 相同的缓冲视图） ──
    uint8_t        _rx_buf[RX_BUF_SIZE];   // 零堆分配接收缓冲
    uart_buffer_t  _buf;
    uint32_t       _last_rx_cycle;         // 最后收字节时刻（DWT 周期）
    uint32_t       _idle_cycles;           // 帧空闲判定阈值（3.5 字符）
};

#endif /* __cplusplus */
