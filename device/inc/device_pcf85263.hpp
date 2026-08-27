#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// PCF85263A RTC（I2C，万年历 + 闹钟 + 看门狗定时器）
//
// 来源库 : PCF85263 v0.2.2（Rob Tillaart）
// URL    : https://github.com/RobTillaart/PCF85263
//
// 移植说明:
//  - 无 float/STL/动态内存/异常；软件 I2C（GPIO 模拟），C++17
//  - 参考库为实验性库（仅时间读写）；闹钟/看门狗/BCD-二进制
//    选择等按 PCF85263A 数据手册补充实现。数据手册未能在线
//    核实的位定义均以 TODO 标注，上板前请对照数据手册确认
//  - 寄存器从 0x00 起：0x00 = 百分之一秒（参考库只读 0x01 起
//    7 字节，本移植从 0x00 起一次读 8 字节，含百分位）
//  - 时间/闹钟寄存器默认 BCD 编码（上电默认，与参考库一致）；
//    set_binary_mode() 切换二进制编码（模式选择位 TODO）
//  - year 字段存 0-99（即 2000-2099 年）；也可传 ≥2000 的完整
//    年份自动减 2000（本芯片无 century 位）
//  - 小时按 24 小时制处理：写时强制 bit6=0，读时掩 0x3F
//  - 日寄存器 bit7 = OS 停振标志：getDateTime 返回的 day 不受
//    影响；setDateTime 读改写保留该位；提供 clear_osc_stop_flag()
//  - 寄存器均为单字节指针，使用 inter_i2c_dev 抽象层
// ============================================================

class device_pcf85263
{
public:
    // ── I2C 地址（固定 0x51，I2C 总线只可挂一片）──

    enum Addr : uint8_t
    {
        ADDR_0x51 = 0x51,
    };

    // ── 寄存器地址（RTC 模式，参考库 PCF85263.h）──

    enum Reg : uint8_t
    {
        REG_HUNDREDTHS   = 0x00,   // 百分之一秒 0-99（BCD）
        REG_SECONDS      = 0x01,   // 秒（bit7 = 0）
        REG_MINUTES      = 0x02,   // 分
        REG_HOURS        = 0x03,   // 时（bit6 = 12/24h，本驱动强制 24h）
        REG_DAYS         = 0x04,   // 日（bit7 = OS 停振标志）
        REG_WEEKDAYS     = 0x05,   // 星期（bit2-0）
        REG_MONTHS       = 0x06,   // 月（bit4-0）
        REG_YEARS        = 0x07,   // 年 0-99
        REG_ALARM1       = 0x08,   // 闹钟1：秒/分/时/日/星期 0x08-0x0C
        REG_ALARM2       = 0x0D,   // 闹钟2：分/时/日 0x0D-0x0F
        REG_ALARM_ENABLE = 0x10,   // 闹钟使能寄存器（参考库命名，位定义 TODO）
        REG_TIMESTAMP1   = 0x11,   // 时间戳1 0x11-0x16
        REG_TIMESTAMP2   = 0x17,   // 时间戳2 0x17-0x1C
        REG_TIMESTAMP3   = 0x1D,   // 时间戳3 0x1D-0x22
        REG_TS_MODE      = 0x23,   // 时间戳模式
        REG_OFFSET       = 0x24,   // 偏移校准（aging）
        REG_OSCILLATOR   = 0x25,   // 振荡器控制
        REG_BATTERY      = 0x26,   // 电池切换控制
        REG_PINIO        = 0x27,   // 引脚 IO 配置
        REG_FUNCTION     = 0x28,   // 功能（bit4 = 秒表模式）
        REG_INTA_ENABLE  = 0x29,   // INT A 中断使能
        REG_INTB_ENABLE  = 0x2A,   // INT B 中断使能
        REG_FLAGS        = 0x2B,   // 标志寄存器
        REG_RAMBYTE      = 0x2C,   // 用户 RAM 1 字节
        REG_WATCHDOG     = 0x2D,   // 看门狗定时器（7 位倒数）
        REG_STOP_ENABLE  = 0x2E,   // 停止使能（1 = 停止计时）
        REG_RESETS       = 0x2F,   // 复位控制
    };

    // ── 标志寄存器 0x2B 位定义 ──
    // TODO: bit2-6 定义未核实（推测 PIF/SDF/WDF/AF/TF 等）

    static constexpr uint8_t FLAG_A1F = 0x01;   // bit0 闹钟1 标志
    static constexpr uint8_t FLAG_A2F = 0x02;   // bit1 闹钟2 标志
    // TODO: 停振标志：日寄存器 0x04 bit7 已按数据手册确认；
    //       FLAGS 寄存器 bit7 是否同为 OS 待核实
    static constexpr uint8_t FLAG_OS  = 0x80;

    // ── 闹钟字段使能位（各闹钟寄存器 bit7）──
    // TODO: 使能极性未核实（数据手册 7.4 节）。本移植按
    //       "1 = 该字段参与比较，0 = 忽略该字段" 实现，
    //       若与手册相反只需反转 _alarm_field_mask 逻辑

    static constexpr uint8_t ALARM_FIELD_ENABLE = 0x80;

    // ── 错误码 ──

    enum ErrCode : int
    {
        ERR_OK      = 0,
        ERR_ADDR    = -10,   // 参数非法
        ERR_I2C     = -11,   // I2C 传输失败
        ERR_CONNECT = -12,   // 探测无应答
    };

    // ============================================================
    //  日期时间结构体（BCD 由驱动内部转换）
    // ============================================================

    struct DateTime
    {
        // ⚠ year 为 uint16_t：需求书标注 uint8_t，但"0-99 或 2000+"
        // 两种写法并存时 2000+ 无法装入 uint8_t，故扩为 uint16_t。
        //   写：year = 0-99（即 2000-2099）或 ≥2000 完整年份（如 2024）
        //   读：统一返回完整年份（2000+；本芯片无 century 位，
        //       只能表达 2000-2099）
        uint16_t year;        // 0-99 或 2000+
        uint8_t month;       // 1-12
        uint8_t day;         // 1-31
        uint8_t dow;         // 星期 0-6（0 = 星期日）
        uint8_t hour;        // 0-23（24 小时制）
        uint8_t minute;      // 0-59
        uint8_t second;      // 0-59
        uint8_t hundredths;  // 百分之一秒 0-99（寄存器 0x00）
    };

    // ============================================================
    //  闹钟结构体（enable_xx = 该字段是否参与比较）
    //  闹钟1：5 字段（0x08-0x0C）；闹钟2：分/时/日 3 字段（0x0D-0x0F）
    // ============================================================

    struct Alarm
    {
        uint8_t seconds;      // 0-59（闹钟2 不使用）
        uint8_t minutes;      // 0-59
        uint8_t hours;        // 0-23
        uint8_t days;         // 1-31
        uint8_t weekdays;     // 0-6（闹钟2 不使用——参考库映射中
                              //        闹钟2 无星期字段）
        bool enable_seconds  = true;
        bool enable_minutes  = true;
        bool enable_hours    = true;
        bool enable_days     = true;
        bool enable_weekdays = true;
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit device_pcf85263(inter_i2c_bus* bus, uint8_t addr = ADDR_0x51);

    device_pcf85263(const device_pcf85263&)            = delete;
    device_pcf85263& operator=(const device_pcf85263&) = delete;

    ~device_pcf85263();

    /** @brief 延迟初始化：连接检查（可重复调用） */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool is_connected();
    /** @brief 最近一次操作错误码（0 = 无错误；读取后自动清零） */
    [[nodiscard]] int get_last_error();
    uint8_t get_address() const { return _addr; }

    // ============================================================
    //  日期时间（返回 0 = OK，否则错误码）
    // ============================================================

    int setDateTime(const DateTime& dt);   // 一次事务写 8 字节（0x00-0x07）
    int getDateTime(DateTime& dt);         // 一次事务读 8 字节

    // ============================================================
    //  控制
    // ============================================================

    /// 启动计时（STOP_ENABLE = 0）
    int start();
    /// 停止计时（STOP_ENABLE = 1）
    int stop();
    /// RTC 模式（FUNCTION bit4 = 0，默认）
    int set_rtc_mode();
    /// 秒表模式（FUNCTION bit4 = 1）
    int set_stopwatch_mode();

    /// 时间寄存器编码格式（默认 BCD）
    /// TODO: 模式选择位位置未核实（参考库未实现此功能），
    ///       当前按 FUNCTION 寄存器 bit0 实现，若与数据手册
    ///       不符请修改此函数后重新编译
    int set_binary_mode(bool on);
    /// 驱动当前使用的编码格式（set_binary_mode 后与芯片同步）
    bool get_binary_mode() const { return _binary_mode; }

    // ============================================================
    //  闹钟 1 / 2
    // ============================================================

    int set_alarm1(const Alarm& a);
    int get_alarm1(Alarm& a);
    int set_alarm2(const Alarm& a);
    int get_alarm2(Alarm& a);

    // ============================================================
    //  看门狗定时器（0x2D，7 位倒数计数）
    //  TODO: 时钟选择位未核实（数据手册 7.5 节），当前按
    //        1Hz 默认时钟写入，超时 = count 秒；若需其他
    //        时钟源请先对照手册修改
    // ============================================================

    /// @param count 倒数初值 1-127；0 = 停止看门狗
    int set_watchdog(uint8_t count);
    /// 当前倒数初值（bits 6-0）
    uint8_t get_watchdog();

    // ============================================================
    //  标志 / 中断
    // ============================================================

    uint8_t get_flags();                    // 读失败返回 0，查 get_last_error
    /// 清除指定标志位（mask 位写 0）
    int     clear_flags(uint8_t mask);
    bool    is_alarm1_flag();               // FLAG_A1F
    bool    is_alarm2_flag();               // FLAG_A2F
    /// 停振标志（日寄存器 0x04 bit7，数据手册已确认）
    bool    is_osc_stop_flag();
    /// 清除停振标志（日寄存器 bit7 写 0，读改写保留日值）
    int     clear_osc_stop_flag();
    /// 闹钟中断使能（INT A/B 使能寄存器，位定义 TODO）
    int     set_alarm_interrupts(bool a1ie, bool a2ie);

    // ============================================================
    //  低层寄存器访问（调试 / 未封装功能）
    // ============================================================

    /// 读寄存器（失败返回 0，查 get_last_error）
    uint8_t read_register(uint8_t reg);
    /// 写寄存器
    int     write_register(uint8_t reg, uint8_t value);

private:
    inter_i2c_dev _dev;
    uint8_t       _addr;
    bool          _binary_mode = false;   // 驱动侧编码格式
    int           _error = ERR_OK;

    // ── BCD 转换（参考库算法，无除法表）──

    static uint8_t _dec2bcd(uint8_t value) { return static_cast<uint8_t>(value + 6U * (value / 10U)); }
    static uint8_t _bcd2dec(uint8_t value) { return static_cast<uint8_t>(value - 6U * (value >> 4)); }

    /// 按当前编码格式编码字段值（BCD 或二进制）
    uint8_t _enc(uint8_t value) const { return _binary_mode ? value : _dec2bcd(value); }
    /// 按当前编码格式解码字段值
    uint8_t _dec(uint8_t value) const { return _binary_mode ? value : _bcd2dec(value); }

    /// 闹钟寄存器组写（reg 起 n 个寄存器，一次事务；自动维护 _error）
    void _write_regs(uint8_t reg, const uint8_t* values, uint8_t n);
    /// 闹钟寄存器组读（reg 起 n 个寄存器，一次事务；自动维护 _error）
    bool _read_regs(uint8_t reg, uint8_t* values, uint8_t n);
};

#endif
