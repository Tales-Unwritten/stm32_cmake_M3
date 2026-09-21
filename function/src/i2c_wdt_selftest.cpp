// ============================================================
// 硬件 I2C（i2c_hw_port / device_eeprom）+ 看门狗（wdt_port）自测
// ============================================================
// 仅在 I2C_WDT_SELFTEST=ON 的固件里由 function_init() 调用。
// 通信通道：debug_uart（USART1 PA9/PA10 115200，由调用方 DebugPort_Init 初始化）。
//
// 接线：
//   SCL = PB6，SDA = PB7（I2C1 无重映射）
//   EEPROM：写地址 0xA0 / 读地址 0xA1（7-bit 0x50，即 A2=A1=A0=0）
//
// 注意：
//   - 板上若是 24C32/24C64（双字节内存地址），把 SELFTEST_EEPROM_MODEL 改成
//     eeprom_model::AT24C32 / AT24C64。
//   - 24C04/08/16 的 A0/A1/A2 引脚被内部页地址位占用；本用例只访问地址
//     0x00~0x27（< 256），等价于访问页 0，故 0xA0/0xA1 与 AT24C02 配置同样适用。
//   - 本用例会写入 EEPROM 0x00~0x1F 与 0x20~0x27；写前先备份，测完还原。
//
// 看门狗：
//   两种看门狗一旦启动都无法关闭，因此本函数不返回 —— 末尾进入"持续喂狗"循环。
//   WDT_PROVE_RESET=1（默认）时，会先故意停止喂狗让 IWDG 复位 MCU，用"下次启动读到
//   的复位标志"作为"看门狗确实能复位"的证据；同一启动周期内只做一次（第二次启动
//   读到 IWDG 复位标志后跳过该步），因此不会无限重启。
// ============================================================

#include "i2c_wdt_selftest.hpp"

#include "device_eeprom.hpp"
#include "device_serial.hpp"
#include "inter_i2c_bus.hpp"
#include "inter_i2c_hw.hpp"
#include "inter_wdt.hpp"

#include "delay.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// ============================================================
//  可调参数
// ============================================================

#define SELFTEST_I2C_SCL_PORT GPIOB
#define SELFTEST_I2C_SCL_PIN  pin6
#define SELFTEST_I2C_SDA_PORT GPIOB
#define SELFTEST_I2C_SDA_PIN  pin7

/// 100kHz（内部弱上拉也能跑）；板上确认有 4.7k 外部上拉后可改 400000
static constexpr uint32_t SELFTEST_I2C_SPEED = 100000UL;

/// EEPROM 型号（0xA0/0xA1 对应 A2=A1=A0=0，或 24C08/16 的页 0）
static constexpr eeprom_model SELFTEST_EEPROM_MODEL = eeprom_model::AT24C02;

/// device_eeprom 测试区：0x00 起 32 字节（8 字节页对齐，24C02/24C08 都适用）
static constexpr uint16_t EEP_TEST_ADDR = 0x0000;
static constexpr uint16_t EEP_TEST_LEN  = 32;

/// i2c_write_reg/i2c_read_reg 测试区：0x20 起 24 字节（每档 8 字节独立区）
static constexpr uint8_t HW_TEST_ADDR = 0x20;
static constexpr uint8_t HW_TEST_LEN  = 24;

/// 1 = 额外验证"饿死看门狗会复位"（会让 MCU 复位一次，见文件头说明）
/// 默认 0：只看门狗初始化 + 持续喂狗，固件最后进入常驻喂狗循环（不会触发复位）
#ifndef WDT_PROVE_RESET
#define WDT_PROVE_RESET 0
#endif

// ============================================================
//  输出辅助
// ============================================================

namespace
{
uint32_t g_fail = 0;

void out(const char *s)
{
    debug_uart.send_data((const uint8_t *)s, (uint16_t)strlen(s));
    debug_uart.send_data((const uint8_t *)"\r\n", 2);
}

void outf(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    out(line);
}

void report(const char *name, bool ok, const char *detail = nullptr)
{
    if (!ok)
        g_fail++;
    if (detail != nullptr)
        outf("[I2C] %-24s %-4s %s", name, ok ? "PASS" : "FAIL", detail);
    else
        outf("[I2C] %-24s %s", name, ok ? "PASS" : "FAIL");
}

// I2C 阶段必须定期喂 IWDG：总线异常时每次操作都要等超时（可达百十 ms），
// 整个 I2C 阶段很容易超过看门狗长超时，不喂就会被中途复位。
inline void iwdg_tick()
{
    wdt_port::iwdg_feed();
}

// ============================================================
//  2. 设备配置
// ============================================================

eeprom_config eeprom_cfg()
{
    eeprom_config cfg;
    cfg.model = SELFTEST_EEPROM_MODEL;
    cfg.a2 = 0;
    cfg.a1 = 0;
    cfg.a0 = 0; // A2=A1=A0=0 → 7-bit 0x50 → 写 0xA0 / 读 0xA1
    cfg.write_timeout_ms = 10;
    cfg.wp_pin = nullptr; // 未接 WP 引脚（接了就填 io_ctrl*）
    return cfg;
}

// 参考图案：前 4 字节固定边界值，避免"没写也像成功"
void make_pattern(uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        if (i == 0)
            buf[i] = 0x00;
        else if (i == 1)
            buf[i] = 0xFF;
        else if (i == 2)
            buf[i] = 0x55;
        else if (i == 3)
            buf[i] = 0xAA;
        else
            buf[i] = (uint8_t)(i * 7u + 0x31u);
    }
}

// 逐字节比对，返回首个不一致的下标（无差异返回 -1）
int first_diff(const uint8_t *a, const uint8_t *b, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        if (a[i] != b[i])
            return (int)i;
    }
    return -1;
}

// ============================================================
//  0. 总线物理层诊断
// ============================================================

// 硬件 I2C 的 SCL/SDA 为复用开漏，必须有外部上拉；空闲（无设备拉低）时应读到高电平。
// 若这里读到 0，说明缺少上拉电阻或引脚接错，后续所有收发都不可能成功。
void test_io_levels()
{
    const uint32_t idr = GPIOB->IDR;
    const unsigned scl = (unsigned)((idr >> 6) & 1U); // PB6
    const unsigned sda = (unsigned)((idr >> 7) & 1U); // PB7

    char d[64];
    snprintf(d, sizeof(d), "PB6=%u PB7=%u (1=idle high)", scl, sda);
    report("bus idle levels", (scl == 1U) && (sda == 1U), d);
}

// ============================================================
//  0b. 全总线扫描（定位器件真实地址）
// ============================================================

/// @return 扫描到的器件个数
uint8_t test_scan(i2c_bus &bus)
{
    out("[I2C] ---- bus scan (7-bit 0x08..0x77) ----");

    uint8_t found = 0;
    bus.lock();
    for (uint8_t a = 0x08U; a <= 0x77U; a++)
    {
        iwdg_tick(); // 总线异常时单次操作可能要等超时，别让看门狗咬人
        bus.start();
        bus.write_byte((uint8_t)(a << 1));
        const bool ack = bus.wait_ack(1500); // 超时宽容：慢从机（如 0x28 传感器）也能被扫到
        bus.stop();
        if (ack)
        {
            outf("[I2C]   ACK 0x%02X  (W=0x%02X R=0x%02X)", (unsigned)a, (unsigned)(a << 1),
                 (unsigned)((a << 1) | 1U));
            found++;
        }
    }
    bus.unlock();
    outf("[I2C] scan found %u device(s)", (unsigned)found);
    return found;
}

// ============================================================
//  1c. 硬件 I2C 逐步骤寄存器转储（定位硬件 I2C 故障点）
// ============================================================

void hw_dump(const char *tag)
{
    I2C_TypeDef *p = I2C1;
    outf("[I2C] HW %-14s CR1=0x%04lX CR2=0x%04lX CCR=0x%04lX TRISE=0x%04lX OAR1=0x%04lX SR1=0x%04lX SR2=0x%04lX", tag,
         (unsigned long)p->CR1, (unsigned long)p->CR2, (unsigned long)p->CCR, (unsigned long)p->TRISE,
         (unsigned long)p->OAR1, (unsigned long)p->SR1, (unsigned long)p->SR2);
}

void test_hw_steps(i2c_hw_port &bus)
{
    out("[I2C] ---- HW step-by-step + register dump ----");

    // 引脚/时钟/电平：PB6/PB7 应为 AF 开漏（nibble 0xB）
    outf("[I2C] HW GPIOB CRL=0x%08lX PB6=0x%lX PB7=0x%lX (want 0xF=AF-OD-50M)", (unsigned long)GPIOB->CRL,
         (unsigned long)((GPIOB->CRL >> 24) & 0xFU), (unsigned long)((GPIOB->CRL >> 28) & 0xFU));
    outf("[I2C] HW RCC_APB1ENR=0x%08lX (I2C1EN=bit21)", (unsigned long)RCC->APB1ENR);
    outf("[I2C] HW GPIOB IDR=0x%04lX (PB6=%lu PB7=%lu)", (unsigned long)GPIOB->IDR,
         (unsigned long)((GPIOB->IDR >> 6) & 1U), (unsigned long)((GPIOB->IDR >> 7) & 1U));

    hw_dump("idle");
    bus.start();
    hw_dump("after START");
    bus.write_byte(0xA0);
    hw_dump("after ADDR-W");
    const bool ack = bus.wait_ack(300);
    hw_dump("after wait_ack");
    bus.stop();
    hw_dump("after STOP");
    outf("[I2C] HW addr 0xA0 ack=%d", (int)ack);
}

// ============================================================
//  2b. 写通道探测（判定 EEPROM 是否被写保护）
// ============================================================
//
// AT24Cxx 在 WP 拉高时会保持“读正常、写不进去”：设备地址/字地址仍会 ACK，
// 但数据字节被 NACK。逐字节报告 ACK 就能区分“总线问题”与“写保护”。
// 写回的是刚读到的同一个值，因此不会破坏原有数据。
void write_probe(i2c_bus &bus, device_eeprom &ee, const char *tag)
{
    uint8_t orig = 0;
    if (!ee.read_byte(0, orig))
    {
        outf("[I2C] %s write probe: read addr0 FAIL", tag);
        return;
    }

    bus.lock();
    bus.start();
    bus.write_byte(0xA0);
    const bool a1 = bus.wait_ack(500);
    bool a2 = false;
    bool a3 = false;
    if (a1)
    {
        bus.write_byte(0x00); // 字地址 = 0
        a2 = bus.wait_ack(500);
        bus.write_byte(orig); // 写回同值（非破坏性）
        a3 = bus.wait_ack(500);
    }
    bus.stop();
    bus.unlock();

    outf("[I2C] %s write probe addr=%d word=%d data=%d (orig=0x%02X)", tag, (int)a1, (int)a2, (int)a3, (unsigned)orig);
    if (a1 && a2 && !a3)
        out("[I2C] => data byte NACKed -> EEPROM 写保护（WP 拉高）");
}

// ============================================================
//  1. 地址探测（纯总线原语，直接体现 0xA0/0xA1）
// ============================================================

bool bus_addr_ack(i2c_bus &bus, uint8_t addr8)
{
    bus.start();
    bus.write_byte(addr8);
    const bool ack = bus.wait_ack(500);
    bus.stop();
    return ack;
}

void test_addr_probe(i2c_bus &bus)
{
    out("[I2C] ---- address probe (bus primitives) ----");

    iwdg_tick();
    bus.lock();
    const bool w = bus_addr_ack(bus, 0xA0);
    iwdg_tick();
    const bool r = bus_addr_ack(bus, 0xA1);
    bus.unlock();

    report("addr 0xA0 write ack", w);
    report("addr 0xA1 read  ack", r);
}

// ============================================================
//  2. device_eeprom 经 i2c_bus 抽象接口读写（非破坏性）
// ============================================================

void test_eeprom(device_eeprom &eeprom)
{
    out("[I2C] ---- device_eeprom over i2c_bus ----");
    outf("[I2C] model=%d cap=%u page=%u addr7=0x%02X", (int)eeprom.model(), (unsigned)eeprom.capacity(),
         (unsigned)eeprom.page_size(), (unsigned)eeprom.i2c_addr());

    iwdg_tick();
    report("eeprom.probe", eeprom.probe());

    // 备份原数据
    uint8_t saved[EEP_TEST_LEN];
    iwdg_tick();
    if (!eeprom.read(EEP_TEST_ADDR, saved, EEP_TEST_LEN))
    {
        report("eeprom.read(backup)", false);
        return;
    }
    report("eeprom.read(backup)", true);

    // 写入 + 逐字节回读校验
    uint8_t pat[EEP_TEST_LEN];
    make_pattern(pat, EEP_TEST_LEN);
    iwdg_tick();
    report("eeprom.write_verified", eeprom.write_verified(EEP_TEST_ADDR, pat, EEP_TEST_LEN));

    // 连续读回（read_sequential 走设备级一次传输）
    uint8_t rd[EEP_TEST_LEN];
    memset(rd, 0, sizeof(rd));
    iwdg_tick();
    bool ok = eeprom.read_sequential(EEP_TEST_ADDR, rd, EEP_TEST_LEN);
    const int diff = ok ? first_diff(rd, pat, EEP_TEST_LEN) : -1;
    if (ok && (diff < 0))
    {
        report("eeprom.read_sequential", true);
    }
    else
    {
        char d[48];
        snprintf(d, sizeof(d), "read=%d diff@%d exp=0x%02X got=0x%02X", (int)ok, diff, diff >= 0 ? pat[diff] : 0,
                 diff >= 0 ? rd[diff] : 0);
        report("eeprom.read_sequential", false, d);
    }

    // 还原原始数据（尽量恢复现场）
    iwdg_tick();
    report("eeprom.restore", eeprom.write_verified(EEP_TEST_ADDR, saved, EEP_TEST_LEN));
}

// ============================================================
//  3. i2c_hw_port 设备级 API：i2c_write_reg / i2c_read_reg
//     覆盖 len=1 / 2 / 8 三种接收时序（len==2 走 F1 的 POS 分支）
//     注：这两个 API 返回 void，成败用“回读比对”判定
// ============================================================

void test_hw_reg_api(i2c_hw_port &bus, device_eeprom &eeprom)
{
    out("[I2C] ---- i2c_hw_port::i2c_write_reg / i2c_read_reg ----");

    // 备份（用 device_eeprom：它带返回值，能判定备份是否成功）
    uint8_t saved[HW_TEST_LEN];
    iwdg_tick();
    if (!eeprom.read(HW_TEST_ADDR, saved, HW_TEST_LEN))
    {
        report("hw_reg backup", false);
        return;
    }
    report("hw_reg backup", true);

    static const uint16_t lens[] = {1, 2, 8};
    for (uint8_t k = 0; k < (sizeof(lens) / sizeof(lens[0])); k++)
    {
        const uint16_t len = lens[k];
        const uint8_t  reg = (uint8_t)(HW_TEST_ADDR + (uint8_t)k * 8u);

        uint8_t pat[HW_TEST_LEN];
        uint8_t inv[HW_TEST_LEN];
        make_pattern(pat, len);
        for (uint16_t i = 0; i < len; i++)
            inv[i] = (uint8_t)~pat[i];

        bool ok = true;
        uint8_t rd[HW_TEST_LEN];

        // 正/反图案各写一遍：保证至少有一次写入真的改变了存储单元，
        // 避免“单元原本就是该值”造成的假 PASS。
        for (uint8_t pass = 0; (pass < 2U) && ok; pass++)
        {
            const uint8_t *w = (pass == 0U) ? pat : inv;

            iwdg_tick();
            bus.i2c_write_reg(0x50, reg, w, len);
            delay_ms(10); // EEPROM 内部写周期（tWR ≤ 5ms）

            memset(rd, 0, sizeof(rd));
            iwdg_tick();
            bus.i2c_read_reg(0x50, reg, rd, len);
            ok = (first_diff(rd, w, len) < 0);
        }

        char d[64];
        snprintf(d, sizeof(d), "reg=0x%02X len=%u", (unsigned)reg, (unsigned)len);
        report("hw_reg rw roundtrip", ok, d);
    }

    // 还原（用 device_eeprom：可判定成败）
    iwdg_tick();
    report("hw_reg restore", eeprom.write_verified(HW_TEST_ADDR, saved, HW_TEST_LEN));
}

// ============================================================
//  复位来源（看门狗复位的证据）
// ============================================================

struct reset_cause_t
{
    bool iwdg;
    bool wwdg;
    bool other; // POR / 软件复位 / 引脚复位
};

reset_cause_t read_reset_cause_and_clear()
{
    reset_cause_t rc;
    rc.iwdg = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U);
    rc.wwdg = (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != 0U);
    rc.other = (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0U) || (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0U) ||
               (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    return rc;
}

// ============================================================
//  看门狗验证
// ============================================================

// IWDG：LSI≈40kHz
//   PRESCALER_64  + reload 4095 → (4095+1)*64/40000  ≈ 6.55s
//   PRESCALER_256 + reload 4095 → (4095+1)*256/40000 ≈ 26.2s（最长窗口，用于跑 I2C 测试期间的保护）
//   PRESCALER_64  + reload  624 → (624+1)*64/40000   ≈ 1.00s
static constexpr uint16_t IWDG_RELOAD_LONG  = 4095;
static constexpr uint16_t IWDG_RELOAD_SHORT = 624;

// WWDG：counter=0x7F（最大）；window=0x7F 表示不限制喂狗时刻
static constexpr uint8_t WWDG_COUNTER = 0x7F;

// ── 设计要点（本自测踩过的坑） ────────────────────────────
//  1) 两种看门狗一旦启动都无法软件关闭；IWDG 还会跨系统复位继续计数。
//     因此每个测试循环都必须**同时**喂 IWDG 与 WWDG，否则
//     （如 WWDG 喂狗循环）会因 IWDG 超时而中途异常。
//  2) WWDG 的窗口约束：在 T > W 时刷新会直接触发复位。自测自身的
//     时序抖动很容易踩到，这里统一用 window=0x7F（不限制）避免误触发；
//     窗口功能由 wwdg_init() 的 window 参数暴露，由调用方按需使用。
void wdt_verify()
{
    out("[WDT] ---- IWDG ----");

    // ── 配置 + 寄存器回读 ──────────────────────────────
    wdt_port::iwdg_init(IWDG_PRESCALER_64, IWDG_RELOAD_SHORT);
    {
        char d[80];
        snprintf(d, sizeof(d), "PR=0x%lX RLR=%lu SR=0x%lX timeout~1.0s", (unsigned long)IWDG->PR,
                 (unsigned long)IWDG->RLR, (unsigned long)IWDG->SR);
        report("iwdg cfg readback", (IWDG->PR == IWDG_PRESCALER_64) && (IWDG->RLR == IWDG_RELOAD_SHORT), d);
    }

    // ── 持续喂狗 ~1s（每 50ms 一次，< 1.0s 超时）：能走完即未复位 ─
    for (uint32_t i = 0; i < 20U; i++)
    {
        wdt_port::iwdg_feed();
        delay_ms(50);
    }
    report("iwdg feed 1s survive", true);

    // ── WWDG ──────────────────────────────────────────────
    out("[WDT] ---- WWDG ----");

    // 计数器时钟 = PCLK1 / 4096 / prescaler，必须按实际 PCLK1 计算喂狗间隔
    const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    const uint32_t wwdg_hz = (pclk1 / 4096U) / 8U; // prescaler 8
    const uint32_t tick_us = (wwdg_hz != 0U) ? (1000000U / wwdg_hz) : 0U;
    outf("[WDT] PCLK1=%luHz wwdg_tick=%luus", (unsigned long)pclk1, (unsigned long)tick_us);

    // 先用 window=0x7F（不限制喂狗时刻）验证计数/喂狗通路
    wdt_port::wwdg_init(WWDG_PRESCALER_8, 0x7F, WWDG_COUNTER);
    {
        const bool wdga = ((WWDG->CR & WWDG_CR_WDGA) != 0U);
        const uint32_t wdgtb = (WWDG->CFR & WWDG_CFR_WDGTB);
        const bool tb_ok = (wdgtb == WWDG_PRESCALER_8);
        char d[96];
        snprintf(d, sizeof(d), "CR=0x%02lX CFR=0x%04lX WDGTB=0x%03lX(want 0x%03lX)", (unsigned long)WWDG->CR,
                 (unsigned long)WWDG->CFR, (unsigned long)wdgtb, (unsigned long)WWDG_PRESCALER_8);
        report("wwdg cfg readback", wdga && tb_ok, d);
    }

    // ── 持续喂狗 ~1s（每 10ms 一次）；本循环同时喂 IWDG ─────
    // （IWDG 无法关闭，必须跟着一起喂，否则会在此阶段超时）
    for (uint32_t i = 0; i < 100U; i++)
    {
        wdt_port::wwdg_feed(WWDG_COUNTER);
        wdt_port::iwdg_feed();
        delay_ms(10);
    }
    report("wwdg feed 1s survive", true);
}

// ============================================================
//  常驻喂狗循环
// ============================================================

[[noreturn]] void feed_forever()
{
    out("[WDT] watchdogs armed: feeding forever (heartbeat ~1s)");

    uint32_t n = 0;
    while (1)
    {
        wdt_port::wwdg_feed(WWDG_COUNTER); // 每 10ms，远小于 ~58ms 超时
        wdt_port::iwdg_feed();             // 每 10ms，远小于 1.0s 超时
        delay_ms(10);

        if ((++n % 100U) == 0U)
            outf("[WDT] heartbeat %lu", (unsigned long)(n / 100U));
    }
}
} // namespace

// ============================================================
//  入口
// ============================================================

int i2c_wdt_selftest_main(void)
{
    static i2c_hw_port bus({i2c_hw_id::i2c1, SELFTEST_I2C_SCL_PORT, SELFTEST_I2C_SCL_PIN, SELFTEST_I2C_SDA_PORT,
                            SELFTEST_I2C_SDA_PIN, afio_enum_t::RM_I2C1_DISABLE, SELFTEST_I2C_SPEED});

    out("");
    out("[I2C] ===== hw-I2C (I2C1 PB6/PB7) + EEPROM 0xA0/0xA1 + WDT selftest =====");

    // ── 0) 复位来源 ────────────────────────────────────────
    const reset_cause_t rc = read_reset_cause_and_clear();
#if WDT_PROVE_RESET
    const bool prev_wdt = rc.iwdg || rc.wwdg;
#endif
    if (rc.iwdg)
        out("[WDT] previous reset = IWDG (starvation)");
    else if (rc.wwdg)
        out("[WDT] previous reset = WWDG");
    else if (rc.other)
        out("[WDT] previous reset = power-on / software / pin");
    else
        out("[WDT] previous reset = unknown");

    // IWDG 一旦启动就无法关闭，且会跨系统复位继续计数。不管上次是什么复位，
    // 这里先用**最长窗口**（prescaler 256 + reload 4095 ≈ 26s）把 IWDG 重新装载，
    // 因为总线异常时 I2C 阶段每次操作都要等超时（可达百十 ms），累计很容易超时。
    wdt_port::iwdg_init(IWDG_PRESCALER_256, IWDG_RELOAD_LONG);
    wdt_port::iwdg_feed();

    // ══════════════════════════════════════════════════════
    // 1a) 软件 I2C 参考测试（同一对引脚 PB6/PB7，GPIO 位模拟）
    //     作用：把“驱动问题”和“接线问题”分开——
    //       软件 I2C 能扫到器件 → 硬件接线正常，问题在硬件 I2C 实现；
    //       软件 I2C 也扫不到 → 与驱动无关，是接线/器件问题。
    // ══════════════════════════════════════════════════════
    out("[I2C] ===== SOFT I2C reference (SCL=PB6 SDA=PB7, GPIO bit-bang) =====");
    uint8_t  soft_found = 0;
    uint32_t soft_fail  = 0;
    {
        const uint32_t f0 = g_fail;
        static inter_i2c_bus soft_bus(
            {SELFTEST_I2C_SCL_PORT, SELFTEST_I2C_SDA_PORT, SELFTEST_I2C_SCL_PIN, SELFTEST_I2C_SDA_PIN}, 5);
        soft_bus.init();
        static device_eeprom soft_eeprom(soft_bus, eeprom_cfg());

        test_io_levels();
        iwdg_tick();
        soft_found = test_scan(soft_bus);
        iwdg_tick();
        test_addr_probe(soft_bus);
        test_eeprom(soft_eeprom);
        write_probe(soft_bus, soft_eeprom, "SOFT");

        soft_bus.deinit();
        soft_fail = g_fail - f0;
        outf("[I2C] SOFT SUMMARY fail=%lu", (unsigned long)soft_fail);
    }

    // ══════════════════════════════════════════════════════
    // 1b) 硬件 I2C + EEPROM
    // ══════════════════════════════════════════════════════
    out("[I2C] ===== HW I2C (I2C1 PB6/PB7) =====");
    const uint32_t f1 = g_fail;
    bus.init();
    if (!bus.is_initialized())
    {
        out("[I2C] FAIL bus.init (I2C1 PB6/PB7)");
        return 1;
    }
    outf("[I2C] bus ready: I2C1 SCL=PB6 SDA=PB7 %luHz", (unsigned long)SELFTEST_I2C_SPEED);
    test_hw_steps(bus); // ★ 寄存器转储（定位硬件 I2C 故障点）
    iwdg_tick();

    static device_eeprom eeprom(bus, eeprom_cfg());

    test_io_levels();
    iwdg_tick();
    const uint8_t hw_found = test_scan(bus);
    iwdg_tick();

    // 注：硬件 I2C 的 SCL/SDA 由外设 AF 引脚固定（PB6=SCL, PB7=SDA），
    //     无法像软 I2C 那样交换引脚角色，因此不做“交换重扫”（无意义）。
    //     接线是否接反由软 I2C 阶段的事实判定（扫描到器件 = 接线正确）。
    bus.init(); // 恢复引脚配置，供后续用例使用
    iwdg_tick();

    test_addr_probe(bus);
    test_eeprom(eeprom);
    write_probe(bus, eeprom, "HW  ");
    iwdg_tick();
    test_hw_reg_api(bus, eeprom);

    const uint32_t hw_fail = g_fail - f1;
    outf("[I2C] HW SUMMARY fail=%lu", (unsigned long)hw_fail);
    outf("[I2C] SUMMARY fail=%lu", (unsigned long)g_fail);

    // ★ 一行判决：软/硬两次扫描结果 + 结论
    outf("[I2C] VERDICT soft_scan=%u hw_scan=%u soft_fail=%lu hw_fail=%lu", (unsigned)soft_found, (unsigned)hw_found,
         (unsigned long)soft_fail, (unsigned long)hw_fail);
    if (hw_found > 0U)
        out("[I2C] CONCLUSION: HW I2C works");
    else if (soft_found > 0U)
        out("[I2C] CONCLUSION: wiring OK -> HW I2C driver bug");
    else
        out("[I2C] CONCLUSION: no device answers on either bus -> wiring/device issue");

    out("[I2C] DONE");

    // ── 2) 看门狗 ───────────────────────────────────
    wdt_verify();

    out("[WDT] ---- starvation proof ----");
#if WDT_PROVE_RESET
    if (prev_wdt)
    {
        out("[WDT] previous reset came from a watchdog -> starvation PASS (skipped this cycle)");
    }
    else
    {
        out("[WDT] stop feeding: expect IWDG reset in ~1.0s");
        volatile uint32_t spin = 0; // volatile：防止编译器把空循环优化掉
        while (1)
            spin++; // 故意不喂狗
    }
#else
    out("[WDT] WDT_PROVE_RESET=0 -> skip reset proof");
#endif

    // 看门狗已启动且无法关闭：常驻喂狗
    feed_forever();
}
