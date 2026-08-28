#include "inter_soft_uart.hpp"
#include "stm32f1xx_hal.h"   // HAL_GetTick（getc 超时，SysTick 1ms）

// ============================================================
//  构造 / 析构
// ============================================================

template <uint16_t RX_BUF_SIZE>
soft_uart_port<RX_BUF_SIZE>::soft_uart_port(const SoftUartConfig &cfg)
    : _tx(cfg.tx_port, cfg.tx_pin)
    , _rx(cfg.rx_port, cfg.rx_pin)
    , _baud(cfg.baud)
    , _cycles_per_bit(0)
    , _rx_prev(1)              // 初始化假定线空闲（HIGH）
    , _initialized(false)
    , _buf{}
    , _last_rx_cycle(0)
    , _idle_cycles(0)
{
    _buf.rx_buf = _rx_buf;     // 绑定静态接收缓冲
}

template <uint16_t RX_BUF_SIZE>
soft_uart_port<RX_BUF_SIZE>::~soft_uart_port()
{
    deinit();
}

// ============================================================
//  DWT 周期计数器（Cortex-M3/4/7 精确时序）
// ============================================================

template <uint16_t RX_BUF_SIZE>
void soft_uart_port<RX_BUF_SIZE>::_dwt_enable()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能 DWT 模块
    DWT->CYCCNT = 0U;                                // 复位计数器
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;            // 使能周期计数
}

template <uint16_t RX_BUF_SIZE>
void soft_uart_port<RX_BUF_SIZE>::_delay_cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles)
    {
        /* 忙等；无符号减法在 32 位溢出时正确回绕 */
    }
}

template <uint16_t RX_BUF_SIZE>
void soft_uart_port<RX_BUF_SIZE>::_wait_until(uint32_t target)
{
    while ((int32_t)(DWT->CYCCNT - target) < 0)
    {
        /* 忙等至绝对时刻 target；int32 差值判断正确处理 32 位回绕 */
    }
}

// ============================================================
//  生命周期
// ============================================================

template <uint16_t RX_BUF_SIZE>
void soft_uart_port<RX_BUF_SIZE>::init()
{
    if (_initialized)
        return;   // 幂等

    _dwt_enable();

    // TX：推挽输出，初始 HIGH（串口空闲电平）
    _tx.init(mode_out_pp, nopull, speed_high);
    _tx.high();

    // RX：上拉输入（悬空时 = HIGH）
    _rx.init(mode_input, pullup);

    _cycles_per_bit = SystemCoreClock / _baud;
    _idle_cycles    = _cycles_per_bit * 10U * FRAME_IDLE_CHARS;   // 20 字符判帧（轮询式软串口专用）
    _rx_prev        = 1;
    _buf.rx_len     = 0;
    _buf.rx_flag    = 0;
    _buf.tx_busy    = 0;
    _initialized    = true;
}

template <uint16_t RX_BUF_SIZE>
void soft_uart_port<RX_BUF_SIZE>::deinit()
{
    if (!_initialized)
        return;

    _tx.deinit();
    _rx.deinit();
    _buf.rx_len  = 0;
    _buf.rx_flag = 0;
    _initialized = false;
}

// ============================================================
//  发送（阻塞）
// ============================================================

template <uint16_t RX_BUF_SIZE>
void soft_uart_port<RX_BUF_SIZE>::putc(uint8_t byte)
{
    if (!_initialized)
        return;

    const uint32_t cpb = _cycles_per_bit;

    // ── 位时序确定性（乱码根治）───────────────────────────────
    // 1) 屏蔽中断：SysTick(1ms) 等 ISR 抢占会把某一位拉长数微秒，
    //    115200 下一位仅 8.7µs，采样点随之偏移 → 单字节位错误
    // 2) 帧锚定时序：每位绝对时刻 = 帧起点 + cpb*i，先写后推进，
    //    GPIO 写入/调用开销不会逐位累积（旧的"写完再延时"方案
    //    每比特固定多出 ~30 周期，10 位累积约 0.5 位 → 尾位漂移）
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // 起始位（LOW）
    _tx.low();
    uint32_t next = DWT->CYCCNT + cpb;
    _wait_until(next);

    // 8 个数据位，低位在前
    for (uint8_t i = 0; i < 8; i++)
    {
        if (byte & (1U << i))
            _tx.high();
        else
            _tx.low();
        next += cpb;
        _wait_until(next);
    }

    // 停止位（HIGH）
    _tx.high();
    next += cpb;
    _wait_until(next);

    __set_PRIMASK(primask);
}

template <uint16_t RX_BUF_SIZE>
void soft_uart_port<RX_BUF_SIZE>::puts(const char *str)
{
    if (!str)
        return;
    while (*str)
        putc((uint8_t)*str++);
}

template <uint16_t RX_BUF_SIZE>
void soft_uart_port<RX_BUF_SIZE>::write(const uint8_t *data, uint16_t len)
{
    if (!data)
        return;
    for (uint16_t i = 0; i < len; i++)
        putc(data[i]);
}

// ============================================================
//  接收：非阻塞轮询（下降沿检测 + 毛刺过滤 + 中点采样）
//
//   1. 读取 RX 引脚状态
//   2. 无下降沿（HIGH→LOW）→ 立即返回 false
//   3. 检测到下降沿 → 等 0.5 位时间到起始位中点
//   4. 毛刺过滤：验证起始位仍为 LOW
//   5. 中点采样 8 个数据位（低位在前）
//   6. 前进到停止位中点，同步边沿检测器
// ============================================================

template <uint16_t RX_BUF_SIZE>
bool soft_uart_port<RX_BUF_SIZE>::try_getc(uint8_t *byte)
{
    if (!_initialized || !byte)
        return false;

    const uint32_t cpb  = _cycles_per_bit;
    const uint32_t half = cpb >> 1;

    // 1. 读取当前引脚状态（始终更新边沿检测器）
    uint8_t cur = (_rx.read() == Hig) ? 1 : 0;

    // 2. 下降沿检测
    if (!(_rx_prev && !cur))
    {
        _rx_prev = cur;
        return false;   // 无起始位
    }

    // 3. 起始位刚刚开始：等待中点
    _rx_prev = cur;
    _delay_cycles(half);

    // 4. 毛刺过滤（假起始位）
    if (_rx.read() == Hig)
        return false;

    // 5. 中点采样 8 个数据位
    uint8_t val = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        _delay_cycles(cpb);
        if (_rx.read() == Hig)
            val |= (uint8_t)(1U << i);   // 低位在前
    }

    // 6. 前进到停止位中点，同步边沿检测器
    _delay_cycles(cpb);
    _rx_prev = (_rx.read() == Hig) ? 1 : 0;

    *byte = val;
    return true;
}

// ============================================================
//  接收：阻塞 + 超时（HAL_GetTick 毫秒精度）
// ============================================================

template <uint16_t RX_BUF_SIZE>
bool soft_uart_port<RX_BUF_SIZE>::getc(uint8_t *byte, uint32_t timeout_ms)
{
    if (!_initialized || !byte)
        return false;

    // timeout_ms == 0：无限等待
    if (timeout_ms == 0)
    {
        while (true)
        {
            if (try_getc(byte))
                return true;
        }
    }

    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start <= timeout_ms)
    {
        if (try_getc(byte))
            return true;
    }
    return false;   // 超时
}

// ============================================================
//  运行时配置
// ============================================================

template <uint16_t RX_BUF_SIZE>
void soft_uart_port<RX_BUF_SIZE>::set_baud(uint32_t baud)
{
    _baud = baud;
    if (_initialized)
    {
        _cycles_per_bit = SystemCoreClock / baud;
        _idle_cycles    = _cycles_per_bit * 10U * FRAME_IDLE_CHARS;
    }
}

// ============================================================
//  协议桥接（对齐 usart_port 接口）
// ============================================================

template <uint16_t RX_BUF_SIZE>
bool soft_uart_port<RX_BUF_SIZE>::send_data(const uint8_t *data, uint16_t len)
{
    if (!_initialized || !data || len == 0)
        return false;

    for (uint16_t i = 0; i < len; i++)
        putc(data[i]);
    return true;
}

template <uint16_t RX_BUF_SIZE>
bool soft_uart_port<RX_BUF_SIZE>::send_data_it(const uint8_t *data, uint16_t len)
{
    // 软串口无硬件中断发送：退化为同步阻塞发送（接口兼容）
    return send_data(data, len);
}

template <uint16_t RX_BUF_SIZE>
bool soft_uart_port<RX_BUF_SIZE>::poll()
{
    if (!_initialized)
        return false;

    // 收完当前可用字节（try_getc 无起始位时立即返回）
    uint8_t b;
    while (try_getc(&b))
    {
        if (_buf.rx_len < RX_BUF_SIZE)
        {
            _buf.rx_buf[_buf.rx_len] = b;
            _buf.rx_len = (uint16_t)(_buf.rx_len + 1);   // volatile 读写分离（C++20 弃用 ++）
        }
        _last_rx_cycle = DWT->CYCCNT;
    }

    // 帧空闲判定：3.5 字符时间无新字节 → 帧就绪（模拟 IDLE）
    if (_buf.rx_len > 0 && (DWT->CYCCNT - _last_rx_cycle) >= _idle_cycles)
    {
        _buf.rx_flag = 1;
    }

    return _buf.rx_flag != 0;
}

// ============================================================
//  显式实例化：默认 256 字节接收缓冲区
//  如需要使用其他大小，请在此追加，例如：
//    template class soft_uart_port<512>;
// ============================================================

template class soft_uart_port<256>;
