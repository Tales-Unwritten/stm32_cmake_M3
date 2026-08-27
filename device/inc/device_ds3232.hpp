#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// DS3232 高精度 RTC（I2C，含 236 字节 SRAM）
//
// 来源库 : DS3232 v0.6.1（Rob Tillaart）
// URL    : https://github.com/RobTillaart/DS3232
//
// 移植说明:
//  - 无 float/STL/动态内存/异常；软件 I2C（GPIO 模拟），C++17
//  - 温度用 0.01°C 定点（m°C = raw × 250，10 位有符号 raw），
//    无 FPU 不引入浮点
//  - 时间/日期用整数结构体（参考库为 uint8_t[7] 数组风格，
//    本移植改为结构体，更清晰）；BCD 编码在驱动内部转换
//  - year 字段存 0-99（即 2000-2099 年）；也可传 ≥2000 的完整
//    年份，内部自动减 2000；2100+ 年份自动置月寄存器 century 位
//  - 小时按 24 小时制处理：写时强制 bit6=0，读时掩 0x3F
//    （芯片支持 12 小时制，本驱动不支持，勿改小时格式位）
//  - init() 显式清 EOSC 位（=0 振荡器启动；VCC 供电时振荡器
//    始终运行，此位影响电池供电，上电默认 0，此处确保无误）
//  - 寄存器均为单字节指针，使用 inter_i2c_dev 抽象层
// ============================================================

class device_ds3232
{
public:
    // ── I2C 地址（固定）──

    enum Addr : uint8_t
    {
        ADDR_0x68 = 0x68,
    };

    // ── 寄存器地址（数据手册 Table 2）──

    enum Reg : uint8_t
    {
        REG_SECONDS        = 0x00,   // 秒 BCD（bit7 = 0）
        REG_MINUTES        = 0x01,   // 分 BCD
        REG_HOURS          = 0x02,   // 时 BCD（bit6 = 12/24h 选择，本驱动强制 24h）
        REG_DAY            = 0x03,   // 星期 1-7（bit2-0）
        REG_DATE           = 0x04,   // 日 1-31 BCD
        REG_MONTH          = 0x05,   // 月 1-12 BCD（bit7 = century）
        REG_YEAR           = 0x06,   // 年 0-99 BCD
        REG_ALARM1_SECONDS = 0x07,   // 闹钟1 秒（bit7 = A1M1）
        REG_ALARM1_MINUTES = 0x08,   // 闹钟1 分（bit7 = A1M2）
        REG_ALARM1_HOURS   = 0x09,   // 闹钟1 时（bit7 = A1M3）
        REG_ALARM1_DAY     = 0x0A,   // 闹钟1 日/星期（bit7 = A1M4, bit6 = DY/DT）
        REG_ALARM2_MINUTES = 0x0B,   // 闹钟2 分（bit7 = A2M2）
        REG_ALARM2_HOURS   = 0x0C,   // 闹钟2 时（bit7 = A2M3）
        REG_ALARM2_DAY     = 0x0D,   // 闹钟2 日/星期（bit7 = A2M4, bit6 = DY/DT）
        REG_CONTROL        = 0x0E,   // 控制
        REG_STATUS         = 0x0F,   // 控制/状态
        REG_AGING_OFFSET   = 0x10,   // 老化校准（有符号 8 位，~0.1ppm/LSB）
        REG_TEMP_MSB       = 0x11,   // 温度 MSB（有符号整数部分）
        REG_TEMP_LSB       = 0x12,   // 温度 LSB（bit7-6 = 0.25°C 小数位）
        REG_SRAM_BASE      = 0x14,   // SRAM 起始（0x14-0xFF，共 236 字节）
    };

    // ── 控制寄存器 0x0E 位定义（数据手册，POR 值 0x1C）──

    static constexpr uint8_t CTRL_EOSC   = 0x80;   // bit7 振荡器使能（0 = 启动）
    static constexpr uint8_t CTRL_BBSQW  = 0x40;   // bit6 电池供电方波
    static constexpr uint8_t CTRL_CONV   = 0x20;   // bit5 强制温度转换
    static constexpr uint8_t CTRL_RS2    = 0x10;   // bit4 方波频率选择
    static constexpr uint8_t CTRL_RS1    = 0x08;   // bit3
    static constexpr uint8_t CTRL_INTCN  = 0x04;   // bit2 1 = INT/SQW 出中断
    static constexpr uint8_t CTRL_A2IE   = 0x02;   // bit1 闹钟2 中断使能
    static constexpr uint8_t CTRL_A1IE   = 0x01;   // bit0 闹钟1 中断使能

    // ── 状态寄存器 0x0F 位定义 ──

    static constexpr uint8_t STAT_OSF     = 0x80;   // bit7 停振标志（写 0 清除）
    static constexpr uint8_t STAT_BB32KHZ = 0x40;   // bit6 电池 32kHz 输出
    static constexpr uint8_t STAT_CRATE1  = 0x20;   // bit5 温度转换速率
    static constexpr uint8_t STAT_CRATE0  = 0x10;   // bit4
    static constexpr uint8_t STAT_EN32KHZ = 0x08;   // bit3 32kHz 输出使能
    static constexpr uint8_t STAT_BSY     = 0x04;   // bit2 温度转换忙
    static constexpr uint8_t STAT_A2F     = 0x02;   // bit1 闹钟2 标志（写 0 清除）
    static constexpr uint8_t STAT_A1F     = 0x01;   // bit0 闹钟1 标志（写 0 清除）

    // ── SRAM 尺寸（寄存器 0x14-0xFF）──

    static constexpr uint16_t SRAM_SIZE = 236;

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
        //   读：统一返回完整年份（2000-2199，century 位自动换算）
        uint16_t year;   // 0-99 或 2000+；2100+ 自动置 century 位
        uint8_t month;   // 1-12
        uint8_t day;     // 1-31
        uint8_t dow;     // 星期 1-7（1 = 星期日）
        uint8_t hour;    // 0-23（24 小时制）
        uint8_t minute;  // 0-59
        uint8_t second;  // 0-59
    };

    // ── 闹钟 1 匹配模式（数据手册 Table 2 Alarm Mask Bits）──

    enum alarm1_mode : uint8_t
    {
        ALARM1_ONCE_PER_SECOND = 0,   // 每秒一次（忽略所有数值）
        ALARM1_MATCH_SECONDS   = 1,   // 秒匹配
        ALARM1_MATCH_MINUTES   = 2,   // 分+秒匹配
        ALARM1_MATCH_HOURS     = 3,   // 时+分+秒匹配
        ALARM1_MATCH_DATE      = 4,   // 日期+时分秒匹配（day_date = 1-31）
        ALARM1_MATCH_DAY       = 5,   // 星期+时分秒匹配（day_date = 1-7）
    };

    struct Alarm1
    {
        uint8_t    seconds;   // 0-59
        uint8_t    minutes;   // 0-59
        uint8_t    hours;     // 0-23
        uint8_t    day_date;  // 1-31（日期）或 1-7（星期，由 mode 决定）
        alarm1_mode mode = ALARM1_ONCE_PER_SECOND;
    };

    // ── 闹钟 2 匹配模式（无秒）──

    enum alarm2_mode : uint8_t
    {
        ALARM2_ONCE_PER_MINUTE = 0,   // 每分钟一次（00 秒）
        ALARM2_MATCH_MINUTES   = 1,   // 分匹配
        ALARM2_MATCH_HOURS     = 2,   // 时+分匹配
        ALARM2_MATCH_DATE      = 3,   // 日期+时分匹配（day_date = 1-31）
        ALARM2_MATCH_DAY       = 4,   // 星期+时分匹配（day_date = 1-7）
    };

    struct Alarm2
    {
        uint8_t    minutes;   // 0-59
        uint8_t    hours;     // 0-23
        uint8_t    day_date;  // 1-31（日期）或 1-7（星期，由 mode 决定）
        alarm2_mode mode = ALARM2_ONCE_PER_MINUTE;
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit device_ds3232(inter_i2c_bus* bus, uint8_t addr = ADDR_0x68);

    device_ds3232(const device_ds3232&)            = delete;
    device_ds3232& operator=(const device_ds3232&) = delete;

    ~device_ds3232();

    /** @brief 延迟初始化：连接检查 + 确保 EOSC=0（可重复调用） */
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

    int setDateTime(const DateTime& dt);   // 一次事务写 7 字节（防进位竞争）
    int getDateTime(DateTime& dt);         // 一次事务读 7 字节

    // ============================================================
    //  温度（0.01°C 定点，无浮点）
    // ============================================================

    /// 温度 m°C（raw10 × 250；如 +25.25°C → +25250）
    int32_t get_temperature_mC();

    // ============================================================
    //  控制 / 状态寄存器
    // ============================================================

    uint8_t get_control_register();             // 读失败返回 0，查 get_last_error
    int     set_control_register(uint8_t value);
    uint8_t get_status_register();              // 读失败返回 0，查 get_last_error
    /// 直接写状态寄存器（写 0 清除对应标志位）
    int     set_status_register(uint8_t value);

    /// 清除指定标志位（mask 位写 0，如 STAT_OSF | STAT_A1F）
    int     clear_status_flags(uint8_t mask);

    bool    is_osc_stop_flag();                 // STAT_OSF
    bool    is_alarm1_flag();                   // STAT_A1F
    bool    is_alarm2_flag();                   // STAT_A2F

    /// 振荡器使能（EOSC=0，读改写）
    int enable_oscillator();
    /// 振荡器禁用（EOSC=1，电池供电时停振，省电）
    int disable_oscillator();

    /// 闹钟中断使能（写 A1IE/A2IE 位，并置 INTCN=1 使 INT/SQW 出中断）
    int set_alarm_interrupts(bool a1ie, bool a2ie);

    // ============================================================
    //  闹钟 1 / 2
    // ============================================================

    int set_alarm1(const Alarm1& a);
    int get_alarm1(Alarm1& a);
    int set_alarm2(const Alarm2& a);
    int get_alarm2(Alarm2& a);

    // ============================================================
    //  老化校准偏移（有符号，≈0.1ppm/LSB，范围 -128..+127）
    // ============================================================

    int     set_aging_offset(int8_t value);
    int8_t  get_aging_offset();

    // ============================================================
    //  SRAM 236 字节（index 0-235 → 寄存器 0x14-0xFF）
    // ============================================================

    int sram_write_byte(uint8_t index, uint8_t value);
    int sram_read_byte(uint8_t index, uint8_t& value);
    int sram_write_block(uint8_t index, const uint8_t* buf, uint16_t len);
    int sram_read_block(uint8_t index, uint8_t* buf, uint16_t len);

private:
    inter_i2c_dev _dev;
    uint8_t       _addr;
    int           _error = ERR_OK;

    // ── BCD 转换（参考库算法，无除法表）──

    static uint8_t _dec2bcd(uint8_t value) { return static_cast<uint8_t>(value + 6U * (value / 10U)); }
    static uint8_t _bcd2dec(uint8_t value) { return static_cast<uint8_t>(value - 6U * (value >> 4)); }

    /// 低层寄存器写（自动维护 _error）
    void _write_reg(uint8_t reg, uint8_t value);
    /// 低层寄存器读（失败返回 0，自动维护 _error）
    uint8_t _read_reg(uint8_t reg);
};

#endif
