// ============================================================
// CAN（inter_can / STM32F1 bxCAN）自测
// ============================================================
// 仅在 CAN_SELFTEST=ON 的固件里由 app_setup() 调用。
// 通信通道：debug_uart（USART1 PA9/PA10 115200，由调用方 DebugPort_Init 初始化）。
//
// 为什么"片内回环"就能验证驱动：
//   bxCAN 的 Loopback 模式（BTR.LBKM=1）在芯片内部把发送器直接接到接收器，
//   发出去的帧会被自己接收、通过验收滤波进入 FIFO0，并自行完成 ACK。
//   因此不接收发器、不接总线，也能跑通「init → send → recv → 逐字节比对」
//   全链路，把"驱动问题"与"接线/对端问题"分离开。
//
// 引脚（回环用例其实不需要外部接线，引脚只是照常配置）：
//   CAN_TX = PB9，CAN_RX = PB8（CAN1 重映射 RM_CAN1_2）
//   若你的板子按 CubeMX 用 PD0/PD1（RM_CAN1_3），改下面三个宏即可。
//
// 可选"真实总线"用例（把 CAN_SELFTEST_BUS_MODE 置 1 后编译）：
//   需板上焊有 CAN 收发器（TJA1050/SN65HVD230 等）且接好总线，固件以
//   normal 模式监听 5s 并打印收到的帧（配合 USB-CAN 调试器发送来验证收发）。
//   默认 0：不跑（没有收发器时该用例必然无帧）。
//
// 栈说明：main 栈仅 0x400，本文件所有 can_port 大对象（~100B）一律放静态存储
//   + placement new（与本工程 inter_usart 的 DMA 存储同款），避免深调用链爆栈。
// ============================================================

#include "can_selftest.hpp"

#include "device_serial.hpp"
#include "inter_can.hpp"

#include "delay.h"

#include <new> // placement new
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// ============================================================
//  可调参数
// ============================================================

#define SELFTEST_CAN_TX_PORT GPIOB
#define SELFTEST_CAN_TX_PIN  pin9
#define SELFTEST_CAN_RX_PORT GPIOB
#define SELFTEST_CAN_RX_PIN  pin8

static constexpr afio_enum_t SELFTEST_CAN_AF = afio_enum_t::RM_CAN1_2;

/// 主用例/冒烟用例的波特率
static constexpr uint32_t SELFTEST_CAN_BAUD = 500000UL;

/// 1 = 附加"真实总线监听"用例（需收发器 + 总线，见文件头）
#ifndef CAN_SELFTEST_BUS_MODE
#define CAN_SELFTEST_BUS_MODE 0
#endif

/// 单帧等待接收的默认超时（ms）
static constexpr uint16_t RX_TIMEOUT_MS = 200;

// ============================================================
//  输出辅助
// ============================================================

namespace
{
uint32_t g_pass = 0;
uint32_t g_fail = 0;

void out(const char *s)
{
    debug_uart.send_data((const uint8_t *)s, (uint16_t)strlen(s));
    debug_uart.send_data((const uint8_t *)"\r\n", 2);
}

bool report(const char *name, bool ok, const char *detail = nullptr)
{
    if (ok)
        g_pass++;
    else
        g_fail++;

    char line[112];
    if (detail != nullptr)
        snprintf(line, sizeof(line), "[CAN] %-26s %-4s %s", name, ok ? "PASS" : "FAIL", detail);
    else
        snprintf(line, sizeof(line), "[CAN] %-26s %s", name, ok ? "PASS" : "FAIL");
    out(line);
    return ok;
}

bool reportf(const char *name, bool ok, const char *fmt, ...)
{
    char det[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(det, sizeof(det), fmt, ap);
    va_end(ap);
    return report(name, ok, det);
}

// ============================================================
//  对象/配置辅助
// ============================================================

CanPortConfig mk_cfg(can_mode mode = can_mode::loopback, uint32_t baud = SELFTEST_CAN_BAUD);

/// 写入式配置：避免 -O0 下每个按值返回各占一个栈槽（会撑大栈帧）
void fill_cfg(CanPortConfig &c, can_mode mode = can_mode::loopback, uint32_t baud = SELFTEST_CAN_BAUD)
{
    // 先把全部字段复位为结构体默认值（含 bs1_tq/bs2_tq/prescaler）；
    // 否则复用同一个 cfg 时会被上一个用例改过的字段"串味"
    c = CanPortConfig{};

    c.periph      = can_id::can1;
    c.tx_port     = SELFTEST_CAN_TX_PORT;
    c.tx_pin      = SELFTEST_CAN_TX_PIN;
    c.rx_port     = SELFTEST_CAN_RX_PORT;
    c.rx_pin      = SELFTEST_CAN_RX_PIN;
    c.af          = SELFTEST_CAN_AF;
    c.baudrate    = baud;
    c.mode        = mode;
    c.filter_bank = 0;
}

CanPortConfig mk_cfg(can_mode mode, uint32_t baud)
{
    CanPortConfig c;
    fill_cfg(c, mode, baud);
    return c;
}

/// 静态缓冲区 + placement new 构造一个 can_port，只取 init() 结果后立刻析构。
/// （参数校验用例很多，逐个放栈会撑大栈帧）
bool init_result(const CanPortConfig &cfg)
{
    alignas(can_port) static unsigned char storage[sizeof(can_port)];
    can_port *p = new (storage) can_port(cfg);
    const bool ok = p->init();
    p->~can_port();
    return ok;
}

/// 期望预分频：PCLK1 / (baud × 总 TQ 数)，默认 12TQ（bs1=8, bs2=3）
uint32_t exp_prescaler(uint32_t baud, uint32_t total_tq = 12)
{
    return (uint32_t)(HAL_RCC_GetPCLK1Freq() / ((uint64_t)baud * total_tq));
}

/// 轮询等待接收就绪（最长 timeout_ms 毫秒）
bool wait_rx(can_port &can, uint16_t timeout_ms)
{
    for (uint16_t i = 0; i < timeout_ms; i++)
    {
        if (can.rx_ready())
            return true;
        delay_ms(1);
    }
    return can.rx_ready();
}

/// 发送一帧并回读比对（ID / 扩展位 / DLC / 数据全部一致才为 true）
bool tx_rx_roundtrip(can_port &can, uint32_t id, bool ext, const uint8_t *tx, uint8_t len,
                     uint16_t timeout_ms = RX_TIMEOUT_MS)
{
    if (!can.send(id, ext, tx, len))
        return false;
    if (!wait_rx(can, timeout_ms))
        return false;

    uint32_t rid = 0;
    bool rext = false;
    uint8_t rbuf[8] = {0};
    uint8_t rlen = 0;
    if (!can.recv(rid, rext, rbuf, rlen))
        return false;
    if (rid != id || rext != ext || rlen != len)
        return false;
    for (uint8_t i = 0; i < len; i++)
        if (rbuf[i] != tx[i])
            return false;
    return true;
}

// ============================================================
//  0) 未初始化 / 参数校验
// ============================================================

void test_uninitialized_and_args()
{
    out("[CAN] ---- 0) 未初始化保护 / 参数校验 ----");

    static can_port can(mk_cfg());
    uint8_t d[8] = {0};
    uint32_t id = 0;
    bool ext = false;
    uint8_t len = 0;
    report("uninit.is_initialized", can.is_initialized() == false);
    report("uninit.rx_ready", can.rx_ready() == false);
    report("uninit.send_false", can.send(0x123, false, d, 8) == false);
    report("uninit.recv_false", can.recv(id, ext, d, len) == false);

    // 非法配置 → init() 必须返回 false
    CanPortConfig c;

    fill_cfg(c);
    c.tx_port = nullptr;
    report("arg.tx_port_null", init_result(c) == false);

    fill_cfg(c);
    c.rx_pin = pin_none;
    report("arg.rx_pin_none", init_result(c) == false);

    fill_cfg(c);
    c.baudrate = 0;
    report("arg.baudrate_zero", init_result(c) == false);

    fill_cfg(c);
    c.filter_bank = 14; // F1 单 CAN 过滤器 bank 上限 13
    report("arg.filter_bank_14", init_result(c) == false);

    fill_cfg(c);
    c.bs1_tq = 17; // 上限 16
    report("arg.bs1_tq_17", init_result(c) == false);

    fill_cfg(c);
    c.bs2_tq = 9; // 上限 8
    report("arg.bs2_tq_9", init_result(c) == false);

    // 468750bps @12TQ：PCLK1/(468750×12) 非整数 → 自动计算必须拒绝
    fill_cfg(c, can_mode::loopback, 468750);
    report("arg.baud_not_divisible", init_result(c) == false, "468750bps 12TQ 不能整除");

    // 1000bps：prescaler 需 3000 > 1024 → 拒绝
    fill_cfg(c, can_mode::loopback, 1000);
    report("arg.baud_too_slow", init_result(c) == false, "1000bps prescaler>1024");

    // 手动指定 prescaler 时不做整除校验，应当成功（用于非标波特率）
    fill_cfg(c, can_mode::loopback, 468750);
    c.prescaler = 6;
    report("arg.prescaler_manual_ok", init_result(c) == true, "手动 prescaler=6 被接受");
}

// ============================================================
//  1) 寄存器白盒校验
// ============================================================

void check_registers(uint32_t exp_pre, uint8_t bs1, uint8_t bs2, can_mode mode)
{
    const uint32_t msr = CAN1->MSR;
    const uint32_t mcr = CAN1->MCR;
    const uint32_t btr = CAN1->BTR;
    const uint32_t esr = CAN1->ESR;

    report("reg.MSR_INAK=0", (msr & CAN_MSR_INAK) == 0, "已退出初始化模式（HAL_CAN_Start 生效）");
    report("reg.MCR_INRQ=0", (mcr & CAN_MCR_INRQ) == 0);
    report("reg.MCR_TTCM=0", (mcr & CAN_MCR_TTCM) == 0);
    report("reg.MCR_ABOM=1", (mcr & CAN_MCR_ABOM) != 0, "自动离线恢复");
    report("reg.MCR_AWUM=0", (mcr & CAN_MCR_AWUM) == 0);
    report("reg.MCR_NART=0", (mcr & CAN_MCR_NART) == 0, "自动重传使能");
    report("reg.MCR_RFLM=0", (mcr & CAN_MCR_RFLM) == 0);
    report("reg.MCR_TXFP=0", (mcr & CAN_MCR_TXFP) == 0);

    const uint32_t brp = (btr & CAN_BTR_BRP);
    const uint32_t ts1 = (btr & CAN_BTR_TS1) >> CAN_BTR_TS1_Pos;
    const uint32_t ts2 = (btr & CAN_BTR_TS2) >> CAN_BTR_TS2_Pos;
    const uint32_t sjw = (btr & CAN_BTR_SJW) >> CAN_BTR_SJW_Pos;
    const bool lbkm = (btr & CAN_BTR_LBKM) != 0;
    const bool silm = (btr & CAN_BTR_SILM) != 0;

    reportf("reg.BTR_BRP", brp == (exp_pre - 1), "prescaler=%lu(期望 %lu)", (unsigned long)(brp + 1),
            (unsigned long)exp_pre);
    reportf("reg.BTR_TS1", ts1 == (uint32_t)(bs1 - 1), "ts1=%lu(期望 %u)", (unsigned long)ts1, (unsigned)(bs1 - 1));
    reportf("reg.BTR_TS2", ts2 == (uint32_t)(bs2 - 1), "ts2=%lu(期望 %u)", (unsigned long)ts2, (unsigned)(bs2 - 1));
    reportf("reg.BTR_SJW", sjw == 0, "sjw=%lu", (unsigned long)sjw);

    // 模式位：loopback → LBKM=1/SILM=0；silent_loopback → 1/1；silent → 0/1；normal → 0/0
    const bool want_lbkm = (mode == can_mode::loopback) || (mode == can_mode::silent_loopback);
    const bool want_silm = (mode == can_mode::silent) || (mode == can_mode::silent_loopback);
    reportf("reg.BTR_mode_bits", lbkm == want_lbkm && silm == want_silm, "LBKM=%d SILM=%d(期望 %d/%d)", (int)lbkm,
            (int)silm, (int)want_lbkm, (int)want_silm);

    // 验收滤波：bank0 已激活 / 掩码模式 / 32 位宽
    report("reg.FA1R.f0_active", (CAN1->FA1R & 0x1u) != 0, "过滤器已使能");
    report("reg.FM1R.f0_mask", (CAN1->FM1R & 0x1u) == 0, "掩码模式");
    report("reg.FS1R.f0_32bit", (CAN1->FS1R & 0x1u) != 0, "32 位宽");

    // 错误状态：无被动错误、无离线；LEC=0（无错误）
    // ESR 位：bit0 EWGF、bit1 EPVF、bit2 BOFF、bit6:4 LEC、bit23:16 TEC、bit31:24 REC
    reportf("reg.ESR_no_error", (esr & (1u << 1)) == 0 && (esr & (1u << 2)) == 0, "LEC=%lu TEC=%lu REC=%lu",
            (unsigned long)((esr >> 4) & 0x7u), (unsigned long)((esr >> 16) & 0xFFu),
            (unsigned long)((esr >> 24) & 0xFFu));
}

// ============================================================
//  2) 回环收发（标准/扩展/ID 边界/DLC 全档）
// ============================================================

void test_loopback_roundtrip(can_port &can)
{
    out("[CAN] ---- 2) 回环收发 ----");
    report("rx.empty_at_start", can.rx_ready() == false);

    // 标准帧 8 字节
    uint8_t d8[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    report("loop.tx_std_8B", can.send(0x123, false, d8, 8));

    const bool rdy = wait_rx(can, RX_TIMEOUT_MS);
    report("loop.rx_ready_after_tx", rdy);

    uint32_t id = 0;
    bool ext = true;
    uint8_t rx[8] = {0};
    uint8_t len = 0;
    const bool got = rdy && can.recv(id, ext, rx, len);
    reportf("loop.rx_std_id", got && id == 0x123, "id=0x%03lX", (unsigned long)id);
    reportf("loop.rx_std_ext", got && !ext, "ext=%d", (int)ext);
    reportf("loop.rx_std_len", got && len == 8, "len=%u", (unsigned)len);
    bool data_ok = got && len == 8;
    for (uint8_t i = 0; data_ok && i < 8; i++)
        data_ok = (rx[i] == d8[i]);
    report("loop.rx_std_data", data_ok, "11 22 33 44 55 66 77 88");
    report("loop.fifo_drained", can.rx_ready() == false);

    // 扩展帧 29 位 ID
    uint8_t dx[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    report("loop.tx_ext_29bit", can.send(0x1FFFFFFF, true, dx, 4));
    const bool rdy2 = wait_rx(can, RX_TIMEOUT_MS);
    const bool got2 = rdy2 && can.recv(id, ext, rx, len);
    reportf("loop.rx_ext_id", got2 && ext && id == 0x1FFFFFFF, "id=0x%08lX ext=%d", (unsigned long)id, (int)ext);
    reportf("loop.rx_ext_len", got2 && len == 4, "len=%u", (unsigned)len);
    report("loop.rx_ext_data", got2 && rx[0] == 0xDE && rx[1] == 0xAD && rx[2] == 0xBE && rx[3] == 0xEF);

    // ID 边界
    uint8_t d1[1] = {0x5A};
    report("loop.id_std_min_0x000", tx_rx_roundtrip(can, 0x000, false, d1, 1));
    report("loop.id_std_max_0x7FF", tx_rx_roundtrip(can, 0x7FF, false, d1, 1));
    report("loop.id_ext_min_0x0", tx_rx_roundtrip(can, 0x00000000, true, d1, 1));
    report("loop.id_ext_max_0x1FFFFFFF", tx_rx_roundtrip(can, 0x1FFFFFFF, true, d1, 1));

    // DLC 0..8 全档
    out("[CAN] ---- 2b) DLC 0..8 ----");
    uint8_t buf[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    for (uint8_t n = 0; n <= 8; n++)
    {
        char name[24];
        snprintf(name, sizeof(name), "loop.dlc_%u", (unsigned)n);
        reportf(name, tx_rx_roundtrip(can, (uint32_t)(0x200 + n), false, buf, n), "len=%u", (unsigned)n);
    }

    // API 侧拒绝（都不应产生 FIFO 帧）
    uint8_t buf9[9] = {0};
    report("arg.send_len9_rejected", can.send(0x1, false, buf9, 9) == false);
    report("arg.send_null_data_rejected", can.send(0x1, false, nullptr, 1) == false);
    report("arg.fifo_still_empty", can.rx_ready() == false);
}

// ============================================================
//  3) 连发管路 + ID 扫描
// ============================================================

void test_burst_and_sweep(can_port &can)
{
    out("[CAN] ---- 3) 连发管路 / ID 扫描 ----");

    // FIFO0 深度 3：连发 3 帧后应能全部按序收回
    uint8_t b[2] = {0xAA, 0xBB};
    bool tx_all = true;
    for (uint8_t i = 0; i < 3; i++)
        tx_all = can.send((uint32_t)(0x300 + i), false, b, 2) && tx_all;
    report("burst.tx_3_frames", tx_all);

    bool in_order = true;
    for (uint8_t i = 0; i < 3; i++)
    {
        uint32_t id = 0;
        bool ext = false;
        uint8_t rx[8] = {0};
        uint8_t len = 0;
        if (!wait_rx(can, RX_TIMEOUT_MS) || !can.recv(id, ext, rx, len))
        {
            in_order = false;
            break;
        }
        if (id != (uint32_t)(0x300 + i) || ext || len != 2 || rx[0] != 0xAA || rx[1] != 0xBB)
            in_order = false;
    }
    report("burst.rx_3_in_order", in_order);
    report("burst.fifo_empty_after", can.rx_ready() == false);

    // 16 帧递增 ID 逐帧回环
    uint8_t s[3] = {0x01, 0x02, 0x03};
    bool sweep_ok = true;
    for (uint16_t i = 0; i < 16; i++)
        if (!tx_rx_roundtrip(can, (uint32_t)(0x100 + i), false, s, 3))
            sweep_ok = false;
    report("sweep.16_frames", sweep_ok);
}

// ============================================================
//  4) 各标准波特率（BRP 与 PCLK1 精确匹配 + 能收发）
// ============================================================

void bitrate_case(can_port &can, uint32_t baud)
{
    const bool init_ok = can.init();
    const uint32_t want_pre = exp_prescaler(baud);
    const uint32_t got_pre = init_ok ? ((CAN1->BTR & CAN_BTR_BRP) + 1u) : 0u;

    uint8_t d[2] = {0xA5, 0x5A};
    const bool rt = init_ok && tx_rx_roundtrip(can, 0x0AB, false, d, 2);

    char name[24];
    snprintf(name, sizeof(name), "bitrate.%luk", (unsigned long)(baud / 1000u));
    reportf(name, init_ok && rt && got_pre == want_pre, "prescaler=%lu(期望 %lu) 12TQ", (unsigned long)got_pre,
            (unsigned long)want_pre);

    can.deinit();
}

void test_bitrates()
{
    out("[CAN] ---- 4) 标准波特率回环 ----");
    static can_port c125(mk_cfg(can_mode::loopback, 125000UL));
    static can_port c250(mk_cfg(can_mode::loopback, 250000UL));
    static can_port c500(mk_cfg(can_mode::loopback, 500000UL));
    static can_port c1M(mk_cfg(can_mode::loopback, 1000000UL));

    bitrate_case(c125, 125000UL);
    bitrate_case(c250, 250000UL);
    bitrate_case(c500, 500000UL);
    bitrate_case(c1M, 1000000UL);
}

// ============================================================
//  5) 各工作模式
// ============================================================

void test_modes()
{
    out("[CAN] ---- 5) 工作模式 ----");

    // silent_loopback：片内回环但 TX 引脚保持隐性（"热自测"）
    {
        static can_port can(mk_cfg(can_mode::silent_loopback, SELFTEST_CAN_BAUD));
        const bool init_ok = can.init();
        report("silent_lb.init", init_ok);
        if (init_ok)
        {
            const uint32_t btr = CAN1->BTR;
            reportf("silent_lb.mode_bits", ((btr & CAN_BTR_LBKM) != 0) && ((btr & CAN_BTR_SILM) != 0), "LBKM=1 SILM=1");
            uint8_t d[1] = {0xC3};
            report("silent_lb.roundtrip", tx_rx_roundtrip(can, 0x0C3, false, d, 1, 500), "片内回环收发");
        }
        can.deinit();
    }

    // silent（只听）：无对端时发送必然完不成 → send() 应返回 false
    {
        static can_port can(mk_cfg(can_mode::silent, SELFTEST_CAN_BAUD));
        const bool init_ok = can.init();
        report("silent.init", init_ok);
        if (init_ok)
        {
            const uint32_t btr = CAN1->BTR;
            reportf("silent.mode_bits", ((btr & CAN_BTR_SILM) != 0) && ((btr & CAN_BTR_LBKM) == 0), "SILM=1 LBKM=0");
            uint8_t d[1] = {0x11};
            report("silent.tx_blocked", can.send(0x111, false, d, 1) == false, "只听模式发送被拒(超时)");
            report("silent.rx_empty", can.rx_ready() == false);
        }
        can.deinit();
    }
}

#if CAN_SELFTEST_BUS_MODE
// ============================================================
//  7) 真实总线监听（需收发器 + 总线）
// ============================================================

void outf(const char *fmt, ...)
{
    char line[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    out(line);
}

void test_real_bus_listen()
{
    out("[CAN] ---- 7) 真实总线监听（normal 500k，5s）----");
    static can_port can(mk_cfg(can_mode::normal, SELFTEST_CAN_BAUD));
    if (!can.init())
    {
        report("bus.init", false);
        return;
    }
    report("bus.init", true, "normal 模式");

    uint32_t n = 0;
    for (uint32_t t = 0; t < 5000u; t++)
    {
        uint32_t id = 0;
        bool ext = false;
        uint8_t rx[8] = {0};
        uint8_t len = 0;
        if (can.recv(id, ext, rx, len))
        {
            outf("[CAN] RX #%lu id=0x%08lX ext=%d len=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
                 (unsigned long)(n++), (unsigned long)id, (int)ext, (unsigned)len, rx[0], rx[1], rx[2], rx[3],
                 rx[4], rx[5], rx[6], rx[7]);
            if (n >= 16u)
                break;
        }
        delay_ms(1);
    }
    reportf("bus.rx_count", true, "共收到 %lu 帧（0 帧也可能是对端没发）", (unsigned long)n);
    can.deinit();
}
#endif

// ============================================================
//  6) deinit / 再初始化
// ============================================================

void test_deinit_reinit(can_port &can)
{
    out("[CAN] ---- 6) deinit / 再初始化 ----");

    can.deinit();
    report("deinit.is_initialized", can.is_initialized() == false);

    uint8_t d[1] = {0x77};
    uint32_t id = 0;
    bool ext = false;
    uint8_t rx[8] = {0};
    uint8_t len = 0;
    report("deinit.send_false", can.send(0x1, false, d, 1) == false);
    report("deinit.rx_ready_false", can.rx_ready() == false);
    report("deinit.recv_false", can.recv(id, ext, rx, len) == false);

    const bool re = can.init();
    report("reinit.ok", re);
    report("reinit.roundtrip", re && tx_rx_roundtrip(can, 0x456, false, d, 1));
}

} // namespace

// ============================================================
//  入口
// ============================================================

int can_selftest_main(void)
{
    out("");
    out("[CAN] ================ inter_can (STM32F1 bxCAN) selftest ================");
    {
        char line[128];
        snprintf(line, sizeof(line),
                 "[CAN] PCLK1=%lu Hz  期望预分频(12TQ): 125k=%lu 250k=%lu 500k=%lu 1M=%lu",
                 (unsigned long)HAL_RCC_GetPCLK1Freq(), (unsigned long)exp_prescaler(125000UL),
                 (unsigned long)exp_prescaler(250000UL), (unsigned long)exp_prescaler(500000UL),
                 (unsigned long)exp_prescaler(1000000UL));
        out(line);
    }

    test_uninitialized_and_args();

    // ── 主用例：500k + loopback ──
    out("[CAN] ---- 1) init + 寄存器白盒校验（500k loopback）----");
    {
        static can_port can(mk_cfg(can_mode::loopback, SELFTEST_CAN_BAUD));
        const bool init_ok = can.init();
        report("init.500k_loopback", init_ok);
        report("init.idempotent", can.init(), "重复 init 必须返回 true");
        reportf("info.is_initialized", can.is_initialized(), "is_initialized=%d", (int)can.is_initialized());
        reportf("info.hw_is_CAN1", can.hw() == (uint32_t)CAN1, "hw=0x%08lX", (unsigned long)can.hw());
        reportf("info.periph_is_can1", can.periph() == can_id::can1, "periph=%d", (int)can.periph());

        if (init_ok)
        {
            check_registers(exp_prescaler(SELFTEST_CAN_BAUD), 8, 3, can_mode::loopback);
            test_loopback_roundtrip(can);
            test_burst_and_sweep(can);
            test_deinit_reinit(can);
            can.deinit(); // 收尾：把硬件交还给后续（smoke）用例
        }
    }

    test_bitrates();
    test_modes();

#if CAN_SELFTEST_BUS_MODE
    test_real_bus_listen();
#endif

    {
        char line[112];
        snprintf(line, sizeof(line), "[CAN] SUMMARY pass=%lu fail=%lu total=%lu", (unsigned long)g_pass,
                 (unsigned long)g_fail, (unsigned long)(g_pass + g_fail));
        out(line);
    }
    if (g_fail == 0)
        out("[CAN] VERDICT: PASS - inter_can 驱动自测全部通过（片内回环全双工已验证）");
    else
    {
        char line[112];
        snprintf(line, sizeof(line), "[CAN] VERDICT: FAIL - %lu 个用例失败，请对照上方 [CAN] ... FAIL 行",
                 (unsigned long)g_fail);
        out(line);
    }
    out("[CAN] DONE");

    return (g_fail == 0) ? 0 : 1;
}

// ============================================================
//  常驻冒烟（app_loop 周期调用）
// ============================================================

void can_selftest_loop(void)
{
    static can_port can(mk_cfg(can_mode::loopback, SELFTEST_CAN_BAUD));
    if (!can.is_initialized() && !can.init())
    {
        out("[CAN] smoke FAIL (init)");
        return;
    }

    uint8_t d[4] = {0xC0, 0xDE, 0x12, 0x34};
    const bool tx_ok = can.send(0x5A5, false, d, 4);
    const bool rx_ok = tx_ok && wait_rx(can, RX_TIMEOUT_MS);
    if (tx_ok && rx_ok)
    {
        out("[CAN] smoke PASS");
        return;
    }

    char line[64];
    snprintf(line, sizeof(line), "[CAN] smoke FAIL tx=%d rx=%d", (int)tx_ok, (int)rx_ok);
    out(line);
}
