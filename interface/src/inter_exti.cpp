// ============================================================
// @platform STM32F103xx（从 GD32F4xx 版本移植，基于 STM32F1xx HAL EXTI 驱动）
//
// ══ 与 GD32F4xx 版的关键实现差异（公开 API 除枚举裁剪/优先级字段拆解外不变）══
//   1. 线配置全部走 HAL EXTI API（HAL_EXTI_SetConfigLine / ClearConfigLine /
//      GenerateSWI / ClearPending），不再直接操作 RTSR/FTSR/IMR/SWIER；
//      仅 ISR 内 PR 检测/清除沿用 HAL_EXTI_IRQHandler 的官方寄存器写法
//      （HAL 回调签名 void(void) 携带不了 user_data，线回调分发表由本类自持，
//      结构对齐 GD32 参考实现）。
//   2. GPIO 端口选择：SYSCFG → AFIO->EXTICR（由 HAL 内部完成），但 AFIO 时钟
//      HAL 不负责，须自行 __HAL_RCC_AFIO_CLK_ENABLE()。
//   3. 线资源裁剪：F103xE 仅 0~15 GPIO + 16 PVD + 17 RTC 闹钟（归 inter_rtc）
//      + 18 USB 设备唤醒；GD32F4 的 19~22（以太网/USB_HS/RTC 时间戳/RTC 唤醒）
//      在 F103 不存在，已删除。
//   4. IRQ/ISR 名称差异：EXTI5_9→EXTI9_5、EXTI10_15→EXTI15_10、
//      LVD_IRQHandler→PVD_IRQHandler、USBFS_WKUP_IRQHandler→USBWakeUp_IRQHandler。
//   5. NVIC：HAL_NVIC_SetPriority(抢占, 子优先级)；ExtiConfig 的 priority 字段
//      拆为 preempt_priority + sub_priority，字段命名对齐 UsartPortConfig。
//   6. enable()/disable() 由 NVIC 组开关改为 EXTI 线 IMR 位门控：F1 组中断
//      （EXTI9_5/EXTI15_10）为多线共享，原版按组关 NVIC 会误伤同组其它实例；
//      deinit() 同样只掩蔽本线、摘表，不关组 NVIC。
// ============================================================

#include "inter_exti.hpp"

#include "inter_nvic.hpp"

// ════════════════════════════════════════════════════════════
//  全局端口映射表（EXTI 线 0~18）
//  线 17（RTC 闹钟）由 inter_rtc 管理，槽位保留 nullptr
// ════════════════════════════════════════════════════════════

namespace
{
constexpr uint8_t EXTI_LINES = 19;
exti_port *g_exti_map[EXTI_LINES] = {};

/** @brief F103xE（VET6）实际引出端口：A~E */
bool _port_valid(GPIO_TypeDef *port)
{
    return port == GPIOA || port == GPIOB || port == GPIOC || port == GPIOD || port == GPIOE;
}

/** @brief GPIO 端口 → HAL EXTI 端口源选择码（EXTI_GPIOA ~ EXTI_GPIOE） */
uint32_t _port_src(GPIO_TypeDef *port)
{
    if (port == GPIOA)
        return EXTI_GPIOA;
    if (port == GPIOB)
        return EXTI_GPIOB;
    if (port == GPIOC)
        return EXTI_GPIOC;
    if (port == GPIOD)
        return EXTI_GPIOD;
    return EXTI_GPIOE;
}

/** @brief EXTI 线号 → HAL 线编码（0~15 带 EXTI_GPIO 属性，16~18 带 EXTI_CONFIG） */
uint32_t _line_encode(uint8_t line)
{
    if (line <= 15)
        return EXTI_LINE_0 + line;
    return EXTI_LINE_16 + (line - 16);
}
} // namespace

void _exti_fire(exti_port *p)
{
    if (!p || !p->is_initialized())
        return;

    // 与 HAL_EXTI_IRQHandler 相同的检测/清除语义（F1 PR 写 1 清位）
    const uint32_t maskline = (1u << p->_line);
    if (EXTI->PR & maskline)
    {
        EXTI->PR = maskline;

        if (p->_cb)
        {
            p->_cb(p->_cb_data);
        }
    }
}

// ════════════════════════════════════════════════════════════
//  构造 / 析构
// ════════════════════════════════════════════════════════════

exti_port::exti_port(const ExtiConfig &cfg)
    : _pin(cfg.port, cfg.pin), _cfg(cfg), _cb(nullptr), _cb_data(nullptr), _initialized(false), _line(0), _hexti{}
{
}

exti_port::~exti_port()
{
    deinit();
}

// ════════════════════════════════════════════════════════════
//  线号解析
// ════════════════════════════════════════════════════════════

uint8_t exti_port::_resolve_line() const
{
    if (_cfg.line_type == exti_line_id::gpio)
    {
        if (_cfg.pin == pin_none)
            return 0xFF; // 无效引脚

        // pin 是一个 bit，找到它是第几位（0~15）
        uint32_t p = _cfg.pin;
        uint8_t n = 0;
        while (p > 1)
        {
            p >>= 1;
            n++;
        }
        return n;
    }
    // 系统线：线号 = 枚举值（16~18）
    return static_cast<uint8_t>(_cfg.line_type);
}

// ════════════════════════════════════════════════════════════
//  IRQ 映射（19 线 → NVIC 通道）
//  0~4 独占通道；5~9 / 10~15 组共享；16 PVD / 18 USB 唤醒
//  17 RTC_Alarm_IRQn（inter_rtc 管理，本类不注册）
// ════════════════════════════════════════════════════════════

IRQn_Type exti_port::_get_irq() const
{
    if (_line <= 4)
        return (IRQn_Type)(EXTI0_IRQn + _line);
    if (_line <= 9)
        return EXTI9_5_IRQn;
    if (_line <= 15)
        return EXTI15_10_IRQn;

    switch (_line)
    {
    case 16:
        return PVD_IRQn; // PVD 低电压检测
    // 17: RTC_Alarm_IRQn（inter_rtc 管理，本类不注册）
    case 18:
        return USBWakeUp_IRQn; // USB 设备唤醒
    default:
        return EXTI0_IRQn; // 不可达（init 已校验线号）
    }
}

// ════════════════════════════════════════════════════════════
//  ISR 注册
// ════════════════════════════════════════════════════════════

void exti_port::_register_isr()
{
    g_exti_map[_line] = this;
}
void exti_port::_unregister_isr()
{
    g_exti_map[_line] = nullptr;
}

// ════════════════════════════════════════════════════════════
//  init / deinit
// ════════════════════════════════════════════════════════════

bool exti_port::init()
{
    if (_initialized)
        return true;

    _line = _resolve_line();

    // ── 参数校验（Release 下断言不生效，显式拦截） ──
    if (_cfg.line_type == exti_line_id::gpio)
    {
        if (_line > 15)
            return false; // pin_none(0xFF) / pinall(16) 均在此拦截
        if (!_port_valid(_cfg.port))
            return false; // 端口必须在 A~E
        _pin.init(mode_input, _cfg.pull);
    }
    else if (_line < 16 || _line > 18)
    {
        return false; // 枚举越界（0~15 段被误当成系统线）
    }

    if (_line == 17)
        return false; // RTC 闹钟线由 inter_rtc 管理
    if (g_exti_map[_line] != nullptr)
        return false; // ⚠️ 线路互斥：同线已有实例

    // GPIO 线经 AFIO->EXTICR 选端口：HAL 只写寄存器，AFIO 时钟须自行使能
    if (_cfg.line_type == exti_line_id::gpio)
        __HAL_RCC_AFIO_CLK_ENABLE();

    // 触发沿：GPIO 线取配置；系统线固定上升沿
    // （PVD：输出上跳 = 电压跌破阈值；USB 唤醒：由外设挂起事件驱动）
    uint32_t trigger = EXTI_TRIGGER_RISING;
    if (_cfg.line_type == exti_line_id::gpio)
    {
        switch (_cfg.trigger)
        {
        case exti_trigger::rising:
            trigger = EXTI_TRIGGER_RISING;
            break;
        case exti_trigger::falling:
            trigger = EXTI_TRIGGER_FALLING;
            break;
        default:
            trigger = EXTI_TRIGGER_RISING_FALLING;
            break;
        }
    }

    // 触发沿（RTSR/FTSR）+ 中断模式（IMR）+ GPIO 端口选择（AFIO->EXTICR）一次完成
    EXTI_ConfigTypeDef cfg = {};
    cfg.Line = _line_encode(_line);
    cfg.Mode = EXTI_MODE_INTERRUPT;
    cfg.Trigger = trigger;
    cfg.GPIOSel = (_cfg.line_type == exti_line_id::gpio) ? _port_src(_cfg.port) : 0;

    HAL_EXTI_SetConfigLine(&_hexti, &cfg);
    HAL_EXTI_ClearPending(&_hexti, EXTI_TRIGGER_RISING_FALLING); // 清残留 pending

    nvic().set_priority(_get_irq(), _cfg.preempt_priority, _cfg.sub_priority);
    nvic().enable(_get_irq()); // 组 NVIC 一次性开启；线级启停用 enable()/disable()
    _register_isr();
    _initialized = true;
    return true;
}

void exti_port::deinit()
{
    if (!_initialized)
        return;

    // IMR/EMR 关本线 + 清触发沿 + 复位 AFIO 端口选择；组 NVIC 保持开启
    // （EXTI9_5/EXTI15_10 多线共享，不能按组关闭，见文件头差异说明第 6 条）
    HAL_EXTI_ClearConfigLine(&_hexti);
    _unregister_isr(); // 摘表，避免同组 ISR 轮询到半初始化实例

    if (_cfg.line_type == exti_line_id::gpio)
        _pin.deinit();

    _cb = nullptr;
    _initialized = false;
}

// ════════════════════════════════════════════════════════════
//  回调 + 线级使能 + 软件触发
// ════════════════════════════════════════════════════════════

void exti_port::on_interrupt(callback_t cb, void *data)
{
    // 关中断保护：防止 ISR 在两次写入之间触发，拿到不一致的 cb + data
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    _cb = cb;
    _cb_data = data;
    __set_PRIMASK(mask);
}

void exti_port::enable()
{
    if (_initialized)
        EXTI->IMR |= (1u << _line);
}
void exti_port::disable()
{
    if (_initialized)
        EXTI->IMR &= ~(1u << _line);
}

void exti_port::software_trigger()
{
    if (!_initialized)
        return;
    // SWIER 置位 → PR 置位；IMR 掩蔽期间不产生中断，enable() 后触发
    HAL_EXTI_GenerateSWI(&_hexti);
}

// ════════════════════════════════════════════════════════════
//  ISR 入口（全部内聚在此）
//  GPIO 线 0~15 + 系统线 16/18（线 17 由 inter_rtc 管理）
// ════════════════════════════════════════════════════════════

extern "C" void EXTI0_IRQHandler(void)
{
    _exti_fire(g_exti_map[0]);
}
extern "C" void EXTI1_IRQHandler(void)
{
    _exti_fire(g_exti_map[1]);
}
extern "C" void EXTI2_IRQHandler(void)
{
    _exti_fire(g_exti_map[2]);
}
extern "C" void EXTI3_IRQHandler(void)
{
    _exti_fire(g_exti_map[3]);
}
extern "C" void EXTI4_IRQHandler(void)
{
    _exti_fire(g_exti_map[4]);
}

extern "C" void EXTI9_5_IRQHandler(void)
{
    for (uint8_t i = 5; i <= 9; i++)
        _exti_fire(g_exti_map[i]);
}

extern "C" void EXTI15_10_IRQHandler(void)
{
    for (uint8_t i = 10; i <= 15; i++)
        _exti_fire(g_exti_map[i]);
}

/* ── 系统事件线（F103 仅 16/18；17 归 inter_rtc） ── */

extern "C" void PVD_IRQHandler(void)
{
    _exti_fire(g_exti_map[16]);
}
extern "C" void USBWakeUp_IRQHandler(void)
{
    _exti_fire(g_exti_map[18]);
}
