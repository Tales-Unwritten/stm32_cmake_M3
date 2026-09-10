// ════════════════════════════════════════════════════════════
//  @platform STM32F103xx（STM32F10xxx / Cortex-M3，基于 STM32F1xx HAL 库）
//
//  职责：USART 寄存器级收发驱动（不使用 HAL UART 句柄/状态机）。
//
//  数据流：
//    接收：ISR 收字节（RXNE）→ 空闲帧判帧（IDLE）→ buf.rx_flag = 1
//          → 用户轮询 buffer() 取帧，处理完自清 rx_flag / rx_len
//          → 错误（ORE/FE/NE/PE）自动清空接收帧，等待下一帧
//    发送：send_data()    阻塞发送，完成或超时返回
//          send_data_it() 拷贝进内部 _tx_buf → TXE/TC 中断续发
//          → 全部移出后 buf.tx_busy = 0
//
//  判帧提醒：IDLE = 帧间空闲 ≥1 字节时间。经 USB 虚拟串口（DAP-Link 等）
//  下发时帧间间隙大，IDLE 可能把长帧拆成多段（见 protocol/iap 的说明），
//  此类场景应改走"帧头长度 + rx_len"的驱动式判帧。
// ════════════════════════════════════════════════════════════

#include "inter_usart.hpp"

#include <cstring> // memcpy

// ════════════════════════════════════════════════════════════
//  文件内私有：平台映射表 + 小工具
// ════════════════════════════════════════════════════════════

namespace
{

constexpr uint8_t  USART_COUNT        = 5;     // usart1/usart2/usart3/uart4/uart5
constexpr uint32_t UART_POLL_TIMEOUT  = 0xFFFF; // 阻塞发送单步轮询上限（见 _wait_flag）

// 当前默认实例化为 usart_port<256>；如需其他 RX 缓冲区大小，
// 请在文件底部增加对应显式实例化，并同步调整 g_port_map 类型。
usart_port_base *g_port_map[USART_COUNT] = {};

// ── 平台静态映射表 ──────────────────────────────────────────
// 枚举值即下标（usart1=0 … uart5=4，见头文件 usart_enum_t 说明）。
// 原实现为 irq/periph 两份平行 switch，现并入一表。

struct usart_map_t
{
    USART_TypeDef *periph;
    IRQn_Type      irqn;
};

// 外设指针宏为整型→指针转换，不可用于 constexpr；命名空间级 const 静态初始化等价。
const usart_map_t k_usart_map[USART_COUNT] = {
    {USART1, USART1_IRQn},
    {USART2, USART2_IRQn},
    {USART3, USART3_IRQn},
    {UART4,  UART4_IRQn},
    {UART5,  UART5_IRQn},
};

// 越界编号的回退项（保留旧 switch default 分支的语义）
const usart_map_t k_usart_map_fallback = {USART1, USART1_IRQn};

const usart_map_t &_usart_map(usart_enum_t id)
{
    const uint8_t idx = static_cast<uint8_t>(id);
    return (idx < USART_COUNT) ? k_usart_map[idx] : k_usart_map_fallback;
}

/** @brief 外设编号 → 端口映射表下标；非法编号返回 USART_COUNT（同旧 default） */
uint8_t _usart_index(usart_enum_t id)
{
    const uint8_t idx = static_cast<uint8_t>(id);
    return (idx < USART_COUNT) ? idx : USART_COUNT;
}

// ── 外设时钟开关 ────────────────────────────────────────────
// 保留 switch：__HAL_RCC_xxx_CLK_ENABLE() 是语句宏、不可取址入表。

void _usart_clock_enable(usart_enum_t id)
{
    switch (id)
    {
    case usart1: __HAL_RCC_USART1_CLK_ENABLE(); break;
    case usart2: __HAL_RCC_USART2_CLK_ENABLE(); break;
    case usart3: __HAL_RCC_USART3_CLK_ENABLE(); break;
    case uart4:  __HAL_RCC_UART4_CLK_ENABLE();  break;
    case uart5:  __HAL_RCC_UART5_CLK_ENABLE();  break;
    default:     break; // 非法编号：不动时钟（与旧 default 一致）
    }
}

void _usart_clock_disable(usart_enum_t id)
{
    switch (id)
    {
    case usart1: __HAL_RCC_USART1_CLK_DISABLE(); break;
    case usart2: __HAL_RCC_USART2_CLK_DISABLE(); break;
    case usart3: __HAL_RCC_USART3_CLK_DISABLE(); break;
    case uart4:  __HAL_RCC_UART4_CLK_DISABLE();  break;
    case uart5:  __HAL_RCC_UART5_CLK_DISABLE();  break;
    default:     break;
    }
}

// ── 波特率 → BRR 计算 ───────────────────────────────────────
// USARTDIV = PCLK / (16 × Baud)，而 BRR = USARTDIV × 16 = PCLK / Baud。
// BRR 用 4bit 小数位表示 USARTDIV，为减小量化误差对除法四舍五入：
//     BRR = (PCLK + Baud/2) / Baud
// 例：PCLK=72MHz, 115200 → 625；PCLK=16MHz, 9600 → 1667（1666.67 进位）
// USART1 挂 APB2（PCLK2）；USART2/3、UART4/5 挂 APB1（PCLK1）。
uint32_t _usart_brr(USART_TypeDef *usart, uint32_t baudrate)
{
    const uint32_t pclk = (usart == USART1) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    return (pclk + baudrate / 2U) / baudrate;
}

// ── 标志轮询（阻塞发送用） ──────────────────────────────────
// 先查位、再递减计数；超时返回 false。语义与旧 send_data 内联循环一致。
bool _wait_flag(USART_TypeDef *usart, uint32_t flag)
{
    uint32_t timeout = UART_POLL_TIMEOUT;
    while (!(usart->SR & flag))
    {
        if (--timeout == 0)
            return false;
    }
    return true;
}

// ── RX 事件标志清除（ISR 用） ───────────────────────────────
// F1 固定套路：RXNE/ORE/FE/NE/PE/IDLE 均由"读 SR 后再读 DR"清除。
void _clear_rx_event(USART_TypeDef *usart)
{
    (void)usart->SR;
    (void)usart->DR;
}

} // namespace

// ════════════════════════════════════════════════════════════
//  平台映射（头文件声明导出）
// ════════════════════════════════════════════════════════════

USART_TypeDef *usart_periph_ptr(usart_enum_t id)
{
    return _usart_map(id).periph; // 非法编号回退 USART1（同旧 default）
}

// ════════════════════════════════════════════════════════════
//  生命周期
// ════════════════════════════════════════════════════════════

// ── 构造 / 析构 ─────────────────────────────────────────────

template <uint16_t RX_BUF_SIZE>
usart_port<RX_BUF_SIZE>::usart_port(const UsartPortConfig &cfg)
    : _cfg(cfg), _tx(cfg.tx_port, cfg.tx_pin), _rx(cfg.rx_port, cfg.rx_pin), _initialized(false), _buf{}
{
    _buf.rx_buf = _rx_buf; // 接收视图预先指向成员数组
}

template <uint16_t RX_BUF_SIZE> usart_port<RX_BUF_SIZE>::~usart_port()
{
    deinit();
}

// ── 内部助手 ─────────────────────────────────────────────────

template <uint16_t RX_BUF_SIZE> void usart_port<RX_BUF_SIZE>::_enable_clock()
{
    _usart_clock_enable(_cfg.usart_periph);
}

template <uint16_t RX_BUF_SIZE> IRQn_Type usart_port<RX_BUF_SIZE>::_get_irq() const
{
    return _usart_map(_cfg.usart_periph).irqn;
}

template <uint16_t RX_BUF_SIZE> void usart_port<RX_BUF_SIZE>::_register_port()
{
    const uint8_t idx = _usart_index(_cfg.usart_periph);
    if (idx < USART_COUNT)
        g_port_map[idx] = this;
}

template <uint16_t RX_BUF_SIZE> void usart_port<RX_BUF_SIZE>::_unregister_port()
{
    const uint8_t idx = _usart_index(_cfg.usart_periph);
    if (idx < USART_COUNT)
        g_port_map[idx] = nullptr;
}

// ── init / deinit ────────────────────────────────────────────

template <uint16_t RX_BUF_SIZE> void usart_port<RX_BUF_SIZE>::init()
{
    if (_initialized)
        return;

    // ① 绑定并复位缓冲区视图（零堆分配，_rx_buf 为成员数组）
    _buf.rx_buf = _rx_buf;
    _buf.rx_len = 0;
    _buf.rx_flag = 0;
    _buf.tx_busy = 0;

    // ② 外设时钟
    _enable_clock();

    // ③ GPIO：TX/RX 均为复用推挽输出 + 内部上拉，按 AFIO remap 选脚
    _tx.init(mode_af_pp, pullup, speed_high);
    _tx.set_af(_cfg.af);
    _rx.init(mode_af_pp, pullup, speed_high);
    _rx.set_af(_cfg.af);

    USART_TypeDef *usart = usart_periph_ptr(_cfg.usart_periph);

    // ④ 复位控制寄存器（CR2 含在内：deinit 不清 CR2，re-init 须在此清残留）
    usart->CR1 = 0;
    usart->CR2 = 0;
    usart->CR3 = 0;

    // ⑤ 波特率：BRR = PCLK/Baud 四舍五入，推导见 _usart_brr()
    usart->BRR = _usart_brr(usart, _cfg.baudrate);

    // ⑥ 帧格式与收发使能：8N1、1 停止位（CR2 已复位即默认，无硬件流控）
    usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    usart->CR3 = USART_CR3_EIE; // 错误中断使能（配合 ISR 错误清帧）

    // ⑦ 接收/空闲/校验错误中断
    usart->CR1 |= USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_PEIE;

    // ⑧ NVIC 与端口表注册
    HAL_NVIC_SetPriority(_get_irq(), _cfg.preempt_priority, _cfg.sub_priority);
    HAL_NVIC_EnableIRQ(_get_irq());

    _register_port();
    _initialized = true;
}

template <uint16_t RX_BUF_SIZE> void usart_port<RX_BUF_SIZE>::deinit()
{
    if (!_initialized)
        return;

    _unregister_port(); // 先摘表：此后 ISR 不再路由到本实例

    HAL_NVIC_DisableIRQ(_get_irq());

    // 停用外设（CR2 保持不复位，init 时会连同复位）
    USART_TypeDef *usart = usart_periph_ptr(_cfg.usart_periph);
    usart->CR1 = 0;
    usart->CR3 = 0;

    // GPIO 恢复默认（模拟输入，最低功耗）
    _tx.deinit();
    _rx.deinit();

    // 释放缓冲区视图，复位接收/发送状态
    _buf.rx_buf = nullptr;
    _buf.rx_len = 0;
    _buf.rx_flag = 0;
    _buf.tx_busy = 0;

    _usart_clock_disable(_cfg.usart_periph);
    _initialized = false;
}

// ════════════════════════════════════════════════════════════
//  发送
// ════════════════════════════════════════════════════════════

// ── 阻塞发送（超时保护） ─────────────────────────────────────

template <uint16_t RX_BUF_SIZE> bool usart_port<RX_BUF_SIZE>::send_data(const uint8_t *data, uint16_t len)
{
    if (!_initialized || !data || len == 0)
        return false;

    USART_TypeDef *usart = usart_periph_ptr(_cfg.usart_periph);

    // 关闭发送中断，避免与 send_data_it() 的 TXE/TC 中断互相干扰
    usart->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);

    // 逐字节发送：每字节先等 TXE（数据寄存器空）
    for (uint16_t i = 0; i < len; ++i)
    {
        if (!_wait_flag(usart, USART_SR_TXE))
            return false; // 超时退出：已发字节不回滚（保持旧行为）
        usart->DR = data[i];
    }

    // 等最后一字节完全移出（TC），确保后续切换/关闭前数据已发完
    if (!_wait_flag(usart, USART_SR_TC))
        return false;
    return true;
}

// ── 中断发送（内部拷贝，安全） ───────────────────────────────

template <uint16_t RX_BUF_SIZE> bool usart_port<RX_BUF_SIZE>::send_data_it(const uint8_t *data, uint16_t len)
{
    if (!_initialized || !data || len == 0)
        return false;
    if (_buf.tx_busy)
        return false; // 上一帧未发完
    if (len > TX_BUF_SIZE)
        return false; // 超过内部发送缓冲容量

    // 拷贝进内部缓冲：防止用户指针在 ISR 续发期间被复用/失效
    memcpy(_tx_buf, data, len);
    _buf.tx_data = _tx_buf;
    _buf.tx_len = len;
    _buf.tx_idx = 0;
    _buf.tx_busy = 1;

    USART_TypeDef *usart = usart_periph_ptr(_cfg.usart_periph);
    // TXE 中断逐字节搬运；TC 中断负责最后一字节移出后收尾（tx_busy=0）
    usart->CR1 |= USART_CR1_TXEIE;
    usart->CR1 |= USART_CR1_TCIE;
    return true;
}

// ════════════════════════════════════════════════════════════
//  中断处理
// ════════════════════════════════════════════════════════════

// ── 通用 ISR 处理函数（由各 USARTx_IRQHandler 调用） ─────────
//
//  F1 标志清除规则：
//   - RXNE / ORE / FE / NE / PE / IDLE：读 SR 后再读 DR
//   - TC：向 SR 的 TC 位写 0

static void _uart_isr(usart_port_base *port)
{
    if (!port || !port->is_initialized())
        return;

    USART_TypeDef *usart = (USART_TypeDef *)port->periph();
    uart_buffer_t *buf = port->buffer();
    const uint16_t cap = port->rx_buf_capacity();
    const uint32_t sr = usart->SR; // 中断入口快照一次，以下分支按位处理

    // ── 接收：RXNE（收到 1 字节） ──
    if (sr & USART_SR_RXNE)
    {
        const uint8_t data = (uint8_t)usart->DR; // 读 DR 顺带清 RXNE
        if (buf->rx_len < cap)
        {
            const uint16_t idx = buf->rx_len;
            buf->rx_buf[idx] = data;
            buf->rx_len = idx + 1;
        }
        // 缓冲满：字节已读出并丢弃；帧完整性由上层校验/清空
    }

    // ── 空闲帧：IDLE（一帧结束） ──
    if (sr & USART_SR_IDLE)
    {
        _clear_rx_event(usart); // 读 SR → 读 DR 清 IDLE
        buf->rx_flag = 1;
    }

    // ── 错误：ORE/FE/NE/PE（清空当前接收帧，等待下一帧） ──
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        _clear_rx_event(usart); // 读 SR → 读 DR 清错误标志
        buf->rx_len = 0;
        buf->rx_flag = 0;
    }

    // ── 发送：TXE（可写下一字节 / 已发完则收尾） ──
    if (sr & USART_SR_TXE)
    {
        if (buf->tx_idx < buf->tx_len)
        {
            const uint16_t idx = buf->tx_idx;
            usart->DR = buf->tx_data[idx];
            buf->tx_idx = idx + 1;
        }
        else
        {
            // 全部搬完：停 TXE 中断，等最后一字节移出后由 TC 收尾
            usart->CR1 &= ~USART_CR1_TXEIE;
            usart->CR1 |= USART_CR1_TCIE;
        }
    }

    // ── 发送完成：TC（最后一字节已移出，复位发送状态） ──
    if (sr & USART_SR_TC)
    {
        usart->SR &= ~USART_SR_TC; // TC 写 0 清除
        usart->CR1 &= ~USART_CR1_TCIE;
        buf->tx_busy = 0;
    }
}

// ── 各 USART 中断入口（全部内聚在此，用户无需外部编写） ─────

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

// ════════════════════════════════════════════════════════════
//  显式实例化
//  默认 256 字节接收缓冲区；如需要其他大小，请在此追加：
//    template class usart_port<128>;
//    template class usart_port<512>;
// ════════════════════════════════════════════════════════════

template class usart_port<256>;
// template class usart_port<512>;
