#include "device_ds3232.hpp"

// ============================================================
//  构造 & 初始化
// ============================================================

device_ds3232::device_ds3232(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _addr(addr)
{
}

device_ds3232::~device_ds3232()
{
}

void device_ds3232::init()
{
    if (!is_connected())
    {
        _error = ERR_CONNECT;
        return;
    }
    // EOSC 必须为 0：VCC 供电时振荡器始终运行不受此位影响，
    // 但电池供电时 EOSC=1 会停振（数据手册：bit7 上电默认 0，
    // 此处读改写确保无误）
    (void)enable_oscillator();
}

bool device_ds3232::is_connected()
{
    _error = _dev.ping() ? ERR_OK : ERR_CONNECT;
    return _error == ERR_OK;
}

int device_ds3232::get_last_error()
{
    int e = _error;
    _error = ERR_OK;
    return e;
}

// ============================================================
//  低层寄存器读写（inter_i2c_dev 抽象层）
// ============================================================

void device_ds3232::_write_reg(uint8_t reg, uint8_t value)
{
    _dev.freedom_write(reg, value, 1);
    _error = (_dev.lastError() == 0) ? ERR_OK : ERR_I2C;
}

uint8_t device_ds3232::_read_reg(uint8_t reg)
{
    uint64_t v = 0;
    if (!_dev.freedom_read(reg, &v, 1))
    {
        _error = ERR_I2C;
        return 0;
    }
    _error = ERR_OK;
    return static_cast<uint8_t>(v & 0xFF);
}

// ============================================================
//  日期时间（一次事务读写 7 字节，避免逐字节读时进位竞争；
//  数据手册要求写秒后 1 秒内写完其余寄存器，一次写满足该约束）
// ============================================================

int device_ds3232::setDateTime(const DateTime& dt)
{
    if ((dt.month == 0) || (dt.month > 12) ||
        (dt.day == 0) || (dt.day > 31) ||
        (dt.hour > 23) || (dt.minute > 59) || (dt.second > 59))
    {
        _error = ERR_ADDR;
        return _error;
    }

    // 年份归一化：0-99 直接使用；≥2000 自动减 2000；
    // ≥2100 置 century 位（月寄存器 bit7）
    uint16_t yy = dt.year;
    bool century = false;
    if (yy >= 2000) yy -= 2000;
    if (yy >= 100)
    {
        yy -= 100;
        century = true;
    }

    uint8_t month_bcd = _dec2bcd(dt.month);
    if (century) month_bcd |= 0x80;

    // freedom_write：reg 后跟 length 字节，MSB 在前 → 秒在最前
    uint64_t v = (static_cast<uint64_t>(_dec2bcd(dt.second)) << 48)
               | (static_cast<uint64_t>(_dec2bcd(dt.minute)) << 40)
               | (static_cast<uint64_t>(_dec2bcd(dt.hour) & 0x3F) << 32)   // 强制 24h
               | (static_cast<uint64_t>(dt.dow & 0x07) << 24)
               | (static_cast<uint64_t>(_dec2bcd(dt.day) & 0x3F) << 16)
               | (static_cast<uint64_t>(month_bcd) << 8)
               | (static_cast<uint64_t>(_dec2bcd(static_cast<uint8_t>(yy))));

    _dev.freedom_write(REG_SECONDS, v, 7);
    _error = (_dev.lastError() == 0) ? ERR_OK : ERR_I2C;
    return _error;
}

int device_ds3232::getDateTime(DateTime& dt)
{
    uint64_t v = 0;
    if (!_dev.freedom_read(REG_SECONDS, &v, 7))
    {
        _error = ERR_I2C;
        return _error;
    }

    uint8_t seconds = static_cast<uint8_t>((v >> 48) & 0xFF);
    uint8_t minutes = static_cast<uint8_t>((v >> 40) & 0xFF);
    uint8_t hours   = static_cast<uint8_t>((v >> 32) & 0xFF);
    uint8_t dow     = static_cast<uint8_t>((v >> 24) & 0xFF);
    uint8_t date    = static_cast<uint8_t>((v >> 16) & 0xFF);
    uint8_t month   = static_cast<uint8_t>((v >> 8) & 0xFF);
    uint8_t year    = static_cast<uint8_t>(v & 0xFF);

    dt.second = _bcd2dec(seconds & 0x7F);
    dt.minute = _bcd2dec(minutes & 0x7F);
    dt.hour   = _bcd2dec(hours & 0x3F);        // 掩 12/24 位与 AM/PM
    dt.dow    = dow & 0x07;
    dt.day    = _bcd2dec(date & 0x3F);

    bool century = ((month & 0x80) != 0);
    dt.month = _bcd2dec(month & 0x7F);
    dt.year  = static_cast<uint16_t>(2000 + _bcd2dec(year) + (century ? 100 : 0));

    _error = ERR_OK;
    return _error;
}

// ============================================================
//  温度（0.01°C 定点）
// ============================================================

// 10 位有符号（0.25°C/LSB）：MSB 寄存器 8 位整数 + LSB 寄存器高 2 位小数
//   raw10 = (msb << 2) | (lsb >> 6)，补码
//   m°C   = raw10 × 250
int32_t device_ds3232::get_temperature_mC()
{
    uint64_t v = 0;
    if (!_dev.freedom_read(REG_TEMP_MSB, &v, 2))
    {
        _error = ERR_I2C;
        return 0;
    }

    int16_t raw = static_cast<int16_t>((v >> 6) & 0x3FF);   // 10 位无符号取值
    raw = static_cast<int16_t>(raw << 6) >> 6;              // 符号扩展 10→16 位

    _error = ERR_OK;
    return static_cast<int32_t>(raw) * 250;
}

// ============================================================
//  控制 / 状态寄存器
// ============================================================

uint8_t device_ds3232::get_control_register()
{
    return _read_reg(REG_CONTROL);
}

int device_ds3232::set_control_register(uint8_t value)
{
    _write_reg(REG_CONTROL, value);
    return _error;
}

uint8_t device_ds3232::get_status_register()
{
    return _read_reg(REG_STATUS);
}

int device_ds3232::set_status_register(uint8_t value)
{
    _write_reg(REG_STATUS, value);
    return _error;
}

int device_ds3232::clear_status_flags(uint8_t mask)
{
    // 状态寄存器标志位写 0 清除；1 保持
    uint8_t status = _read_reg(REG_STATUS);
    if (_error != ERR_OK) return _error;
    return set_status_register(static_cast<uint8_t>(status & ~mask));
}

bool device_ds3232::is_osc_stop_flag()
{
    return (get_status_register() & STAT_OSF) != 0;
}

bool device_ds3232::is_alarm1_flag()
{
    return (get_status_register() & STAT_A1F) != 0;
}

bool device_ds3232::is_alarm2_flag()
{
    return (get_status_register() & STAT_A2F) != 0;
}

int device_ds3232::enable_oscillator()
{
    uint8_t ctrl = _read_reg(REG_CONTROL);
    if (_error != ERR_OK) return _error;
    return set_control_register(static_cast<uint8_t>(ctrl & ~CTRL_EOSC));
}

int device_ds3232::disable_oscillator()
{
    uint8_t ctrl = _read_reg(REG_CONTROL);
    if (_error != ERR_OK) return _error;
    return set_control_register(static_cast<uint8_t>(ctrl | CTRL_EOSC));
}

int device_ds3232::set_alarm_interrupts(bool a1ie, bool a2ie)
{
    uint8_t ctrl = _read_reg(REG_CONTROL);
    if (_error != ERR_OK) return _error;

    ctrl &= ~(CTRL_A1IE | CTRL_A2IE);
    if (a1ie) ctrl |= CTRL_A1IE;
    if (a2ie) ctrl |= CTRL_A2IE;
    ctrl |= CTRL_INTCN;   // INT/SQW 引脚出中断（而非方波）

    return set_control_register(ctrl);
}

// ============================================================
//  闹钟 1 / 2
// ============================================================

// A1M 掩码位：数据手册 Table 2 ——
//   每秒   : A1M1-4 全 1
//   秒匹配 : A1M2-4 = 1
//   分匹配 : A1M3-4 = 1
//   时匹配 : A1M4 = 1
//   日期匹配: 全 0（DY/DT = 0）
//   星期匹配: 全 0（DY/DT = 1）
int device_ds3232::set_alarm1(const Alarm1& a)
{
    const uint8_t m = static_cast<uint8_t>(a.mode);
    if (m > ALARM1_MATCH_DAY)
    {
        _error = ERR_ADDR;
        return _error;
    }

    uint8_t a1m1 = (m == ALARM1_ONCE_PER_SECOND) ? 1U : 0U;
    uint8_t a1m2 = (m <= ALARM1_MATCH_SECONDS)   ? 1U : 0U;
    uint8_t a1m3 = (m <= ALARM1_MATCH_MINUTES)   ? 1U : 0U;
    uint8_t a1m4 = (m <= ALARM1_MATCH_HOURS)     ? 1U : 0U;
    uint8_t dydt = (m == ALARM1_MATCH_DAY)       ? 1U : 0U;

    uint64_t v = (static_cast<uint64_t>(_dec2bcd(a.seconds) | (a1m1 << 7)) << 24)
               | (static_cast<uint64_t>(_dec2bcd(a.minutes) | (a1m2 << 7)) << 16)
               | (static_cast<uint64_t>((_dec2bcd(a.hours) & 0x3F) | (a1m3 << 7)) << 8)
               | (static_cast<uint64_t>((_dec2bcd(a.day_date) & 0x3F) | (a1m4 << 7) | (dydt << 6)));

    _dev.freedom_write(REG_ALARM1_SECONDS, v, 4);
    _error = (_dev.lastError() == 0) ? ERR_OK : ERR_I2C;
    return _error;
}

int device_ds3232::get_alarm1(Alarm1& a)
{
    uint64_t v = 0;
    if (!_dev.freedom_read(REG_ALARM1_SECONDS, &v, 4))
    {
        _error = ERR_I2C;
        return _error;
    }

    uint8_t s  = static_cast<uint8_t>((v >> 24) & 0xFF);
    uint8_t mi = static_cast<uint8_t>((v >> 16) & 0xFF);
    uint8_t h  = static_cast<uint8_t>((v >> 8) & 0xFF);
    uint8_t d  = static_cast<uint8_t>(v & 0xFF);

    a.seconds  = _bcd2dec(s & 0x7F);
    a.minutes  = _bcd2dec(mi & 0x7F);
    a.hours    = _bcd2dec(h & 0x3F);
    a.day_date = _bcd2dec(d & 0x3F);

    // 由掩码位反推模式
    uint8_t m1 = (s >> 7) & 1;
    uint8_t m2 = (mi >> 7) & 1;
    uint8_t m3 = (h >> 7) & 1;
    uint8_t m4 = (d >> 7) & 1;
    uint8_t dt_bit = (d >> 6) & 1;

    if (m1)                       a.mode = ALARM1_ONCE_PER_SECOND;
    else if (m2)                  a.mode = ALARM1_MATCH_SECONDS;
    else if (m3)                  a.mode = ALARM1_MATCH_MINUTES;
    else if (m4)                  a.mode = ALARM1_MATCH_HOURS;
    else                          a.mode = dt_bit ? ALARM1_MATCH_DAY : ALARM1_MATCH_DATE;

    _error = ERR_OK;
    return _error;
}

int device_ds3232::set_alarm2(const Alarm2& a)
{
    const uint8_t m = static_cast<uint8_t>(a.mode);
    if (m > ALARM2_MATCH_DAY)
    {
        _error = ERR_ADDR;
        return _error;
    }

    uint8_t a2m2 = (m == ALARM2_ONCE_PER_MINUTE) ? 1U : 0U;
    uint8_t a2m3 = (m <= ALARM2_MATCH_MINUTES)   ? 1U : 0U;
    uint8_t a2m4 = (m <= ALARM2_MATCH_HOURS)     ? 1U : 0U;
    uint8_t dydt = (m == ALARM2_MATCH_DAY)       ? 1U : 0U;

    uint64_t v = (static_cast<uint64_t>(_dec2bcd(a.minutes) | (a2m2 << 7)) << 16)
               | (static_cast<uint64_t>((_dec2bcd(a.hours) & 0x3F) | (a2m3 << 7)) << 8)
               | (static_cast<uint64_t>((_dec2bcd(a.day_date) & 0x3F) | (a2m4 << 7) | (dydt << 6)));

    _dev.freedom_write(REG_ALARM2_MINUTES, v, 3);
    _error = (_dev.lastError() == 0) ? ERR_OK : ERR_I2C;
    return _error;
}

int device_ds3232::get_alarm2(Alarm2& a)
{
    uint64_t v = 0;
    if (!_dev.freedom_read(REG_ALARM2_MINUTES, &v, 3))
    {
        _error = ERR_I2C;
        return _error;
    }

    uint8_t mi = static_cast<uint8_t>((v >> 16) & 0xFF);
    uint8_t h  = static_cast<uint8_t>((v >> 8) & 0xFF);
    uint8_t d  = static_cast<uint8_t>(v & 0xFF);

    a.minutes  = _bcd2dec(mi & 0x7F);
    a.hours    = _bcd2dec(h & 0x3F);
    a.day_date = _bcd2dec(d & 0x3F);

    uint8_t m2 = (mi >> 7) & 1;
    uint8_t m3 = (h >> 7) & 1;
    uint8_t m4 = (d >> 7) & 1;
    uint8_t dt_bit = (d >> 6) & 1;

    if (m2)                       a.mode = ALARM2_ONCE_PER_MINUTE;
    else if (m3)                  a.mode = ALARM2_MATCH_MINUTES;
    else if (m4)                  a.mode = ALARM2_MATCH_HOURS;
    else                          a.mode = dt_bit ? ALARM2_MATCH_DAY : ALARM2_MATCH_DATE;

    _error = ERR_OK;
    return _error;
}

// ============================================================
//  老化校准偏移
// ============================================================

int device_ds3232::set_aging_offset(int8_t value)
{
    _write_reg(REG_AGING_OFFSET, static_cast<uint8_t>(value));
    return _error;
}

int8_t device_ds3232::get_aging_offset()
{
    return static_cast<int8_t>(_read_reg(REG_AGING_OFFSET));
}

// ============================================================
//  SRAM 236 字节（index 0-235 → 寄存器 0x14-0xFF）
// ============================================================

int device_ds3232::sram_write_byte(uint8_t index, uint8_t value)
{
    return sram_write_block(index, &value, 1);
}

int device_ds3232::sram_read_byte(uint8_t index, uint8_t& value)
{
    return sram_read_block(index, &value, 1);
}

int device_ds3232::sram_write_block(uint8_t index, const uint8_t* buf, uint16_t len)
{
    if (!buf || (static_cast<uint16_t>(index) + len > SRAM_SIZE))
    {
        _error = ERR_ADDR;
        return _error;
    }

    uint16_t offset = 0;
    while (len > 0)
    {
        // freedom_write 单次 ≤8 字节
        uint8_t n = (len > 8) ? 8 : static_cast<uint8_t>(len);

        // 组包：MSB 在前（先写 index 处的字节）
        uint64_t v = 0;
        for (uint8_t i = 0; i < n; i++)
        {
            v = (v << 8) | buf[offset + i];
        }
        _dev.freedom_write(static_cast<uint8_t>(REG_SRAM_BASE + index + offset), v, n);
        if (_dev.lastError() != 0)
        {
            _error = ERR_I2C;
            return _error;
        }

        offset += n;
        len    -= n;
    }

    _error = ERR_OK;
    return _error;
}

int device_ds3232::sram_read_block(uint8_t index, uint8_t* buf, uint16_t len)
{
    if (!buf || (static_cast<uint16_t>(index) + len > SRAM_SIZE))
    {
        _error = ERR_ADDR;
        return _error;
    }

    uint16_t offset = 0;
    while (len > 0)
    {
        uint8_t n = (len > 8) ? 8 : static_cast<uint8_t>(len);

        uint64_t v = 0;
        if (!_dev.freedom_read(static_cast<uint8_t>(REG_SRAM_BASE + index + offset), &v, n))
        {
            _error = ERR_I2C;
            return _error;
        }
        // 解包：MSB 在前
        for (uint8_t i = 0; i < n; i++)
        {
            buf[offset + i] = static_cast<uint8_t>((v >> (8 * (n - 1 - i))) & 0xFF);
        }

        offset += n;
        len    -= n;
    }

    _error = ERR_OK;
    return _error;
}
