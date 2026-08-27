#include "device_pcf85263.hpp"

// ============================================================
//  构造 & 初始化
// ============================================================

device_pcf85263::device_pcf85263(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _addr(addr)
{
}

device_pcf85263::~device_pcf85263()
{
}

void device_pcf85263::init()
{
    if (!is_connected())
    {
        _error = ERR_CONNECT;
    }
}

bool device_pcf85263::is_connected()
{
    _error = _dev.ping() ? ERR_OK : ERR_CONNECT;
    return _error == ERR_OK;
}

int device_pcf85263::get_last_error()
{
    int e = _error;
    _error = ERR_OK;
    return e;
}

// ============================================================
//  低层寄存器访问
// ============================================================

uint8_t device_pcf85263::read_register(uint8_t reg)
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

int device_pcf85263::write_register(uint8_t reg, uint8_t value)
{
    _dev.freedom_write(reg, value, 1);
    _error = (_dev.lastError() == 0) ? ERR_OK : ERR_I2C;
    return _error;
}

void device_pcf85263::_write_regs(uint8_t reg, const uint8_t* values, uint8_t n)
{
    // 组包：MSB 在前（先写 reg 处字节）
    uint64_t v = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        v = (v << 8) | values[i];
    }
    _dev.freedom_write(reg, v, n);
    _error = (_dev.lastError() == 0) ? ERR_OK : ERR_I2C;
}

bool device_pcf85263::_read_regs(uint8_t reg, uint8_t* values, uint8_t n)
{
    uint64_t v = 0;
    if (!_dev.freedom_read(reg, &v, n))
    {
        _error = ERR_I2C;
        return false;
    }
    for (uint8_t i = 0; i < n; i++)
    {
        values[i] = static_cast<uint8_t>((v >> (8 * (n - 1 - i))) & 0xFF);
    }
    _error = ERR_OK;
    return true;
}

// ============================================================
//  日期时间（一次事务读写 8 字节：0x00 百分位 + 0x01-0x07）
// ============================================================

int device_pcf85263::setDateTime(const DateTime& dt)
{
    if ((dt.month == 0) || (dt.month > 12) ||
        (dt.day == 0) || (dt.day > 31) ||
        (dt.hour > 23) || (dt.minute > 59) || (dt.second > 59) ||
        (dt.hundredths > 99) || (dt.dow > 6))
    {
        _error = ERR_ADDR;
        return _error;
    }

    // 年份归一化：0-99 直接使用；≥2000 自动减 2000（无 century 位）
    uint16_t yy = dt.year;
    if (yy >= 2000) yy -= 2000;
    if (yy > 99)
    {
        _error = ERR_ADDR;
        return _error;
    }

    // 日寄存器 bit7 = OS 停振标志：读改写保留（不因设时间误清标志）
    uint8_t days = read_register(REG_DAYS);
    if (_error != ERR_OK) return _error;
    uint8_t days_value = static_cast<uint8_t>(_enc(dt.day) & 0x3F);
    days_value |= (days & 0x80);   // 保留 OS 位

    uint8_t regs[8];
    regs[0] = static_cast<uint8_t>(_enc(dt.hundredths) & 0x7F);
    regs[1] = static_cast<uint8_t>(_enc(dt.second) & 0x7F);
    regs[2] = static_cast<uint8_t>(_enc(dt.minute) & 0x7F);
    regs[3] = static_cast<uint8_t>(_enc(dt.hour) & 0x3F);     // 强制 24h
    regs[4] = days_value;
    regs[5] = static_cast<uint8_t>(_enc(dt.dow) & 0x07);
    regs[6] = static_cast<uint8_t>(_enc(dt.month) & 0x1F);
    regs[7] = static_cast<uint8_t>(_enc(static_cast<uint8_t>(yy)) & 0xFF);

    _write_regs(REG_HUNDREDTHS, regs, 8);
    return _error;
}

int device_pcf85263::getDateTime(DateTime& dt)
{
    uint8_t regs[8];
    if (!_read_regs(REG_HUNDREDTHS, regs, 8)) return _error;

    dt.hundredths = _dec(regs[0] & 0x7F);
    dt.second     = _dec(regs[1] & 0x7F);
    dt.minute     = _dec(regs[2] & 0x7F);
    dt.hour       = _dec(regs[3] & 0x3F);   // 掩 12/24 位与 AM/PM（本驱动仅 24h）
    dt.day        = _dec(regs[4] & 0x3F);   // 掩 OS 位
    dt.dow        = _dec(regs[5] & 0x07);
    dt.month      = _dec(regs[6] & 0x1F);
    dt.year       = static_cast<uint16_t>(2000 + _dec(regs[7] & 0xFF));

    return _error;
}

// ============================================================
//  控制
// ============================================================

int device_pcf85263::start()
{
    return write_register(REG_STOP_ENABLE, 0);
}

int device_pcf85263::stop()
{
    return write_register(REG_STOP_ENABLE, 1);
}

int device_pcf85263::set_rtc_mode()
{
    uint8_t reg = read_register(REG_FUNCTION);
    if (_error != ERR_OK) return _error;
    return write_register(REG_FUNCTION, static_cast<uint8_t>(reg & ~0x10));   // bit4 = 0
}

int device_pcf85263::set_stopwatch_mode()
{
    uint8_t reg = read_register(REG_FUNCTION);
    if (_error != ERR_OK) return _error;
    return write_register(REG_FUNCTION, static_cast<uint8_t>(reg | 0x10));    // bit4 = 1
}

// TODO: BCD/二进制模式选择位位置未核实。参考库未实现此功能；
//       当前按"FUNCTION 寄存器 bit0"实现（推测值）。上板前请
//       对照 PCF85263A 数据手册 7.2 节确认，若位位置不同，
//       修改下面两行即可
int device_pcf85263::set_binary_mode(bool on)
{
    uint8_t reg = read_register(REG_FUNCTION);
    if (_error != ERR_OK) return _error;

    if (on) reg |= 0x01;
    else    reg &= ~0x01;

    int rv = write_register(REG_FUNCTION, reg);
    if (rv == ERR_OK) _binary_mode = on;
    return rv;
}

// ============================================================
//  闹钟 1 / 2
// ============================================================

// 每个闹钟寄存器：bit7 = 字段使能（1 = 参与比较，TODO 极性），
// 低 7 位 = 字段值（BCD 或二进制，掩码见各寄存器位宽）
int device_pcf85263::set_alarm1(const Alarm& a)
{
    if ((a.seconds > 59) || (a.minutes > 59) || (a.hours > 23) ||
        (a.days == 0) || (a.days > 31) || (a.weekdays > 6))
    {
        _error = ERR_ADDR;
        return _error;
    }

    uint8_t regs[5];
    regs[0] = (_enc(a.seconds) & 0x7F) | (a.enable_seconds  ? ALARM_FIELD_ENABLE : 0);
    regs[1] = (_enc(a.minutes) & 0x7F) | (a.enable_minutes  ? ALARM_FIELD_ENABLE : 0);
    regs[2] = (_enc(a.hours)   & 0x3F) | (a.enable_hours    ? ALARM_FIELD_ENABLE : 0);
    regs[3] = (_enc(a.days)    & 0x3F) | (a.enable_days     ? ALARM_FIELD_ENABLE : 0);
    regs[4] = (_enc(a.weekdays) & 0x07) | (a.enable_weekdays ? ALARM_FIELD_ENABLE : 0);

    _write_regs(REG_ALARM1, regs, 5);
    return _error;
}

int device_pcf85263::get_alarm1(Alarm& a)
{
    uint8_t regs[5];
    if (!_read_regs(REG_ALARM1, regs, 5)) return _error;

    a.seconds         = _dec(regs[0] & 0x7F);
    a.minutes         = _dec(regs[1] & 0x7F);
    a.hours           = _dec(regs[2] & 0x3F);
    a.days            = _dec(regs[3] & 0x3F);
    a.weekdays        = _dec(regs[4] & 0x07);
    a.enable_seconds  = (regs[0] & ALARM_FIELD_ENABLE) != 0;
    a.enable_minutes  = (regs[1] & ALARM_FIELD_ENABLE) != 0;
    a.enable_hours    = (regs[2] & ALARM_FIELD_ENABLE) != 0;
    a.enable_days     = (regs[3] & ALARM_FIELD_ENABLE) != 0;
    a.enable_weekdays = (regs[4] & ALARM_FIELD_ENABLE) != 0;

    return _error;
}

// 闹钟2：分/时/日 3 寄存器（参考库映射 ALARM2 = 0x0D）
// TODO: 数据手册 7.4 节：闹钟2 是否含星期字段（0x10 = ALARM_ENABLE
//       还是 ALARM2 星期）未核实，当前按参考库（无星期字段）实现
int device_pcf85263::set_alarm2(const Alarm& a)
{
    if ((a.minutes > 59) || (a.hours > 23) || (a.days == 0) || (a.days > 31))
    {
        _error = ERR_ADDR;
        return _error;
    }

    uint8_t regs[3];
    regs[0] = (_enc(a.minutes) & 0x7F) | (a.enable_minutes ? ALARM_FIELD_ENABLE : 0);
    regs[1] = (_enc(a.hours)   & 0x3F) | (a.enable_hours   ? ALARM_FIELD_ENABLE : 0);
    regs[2] = (_enc(a.days)    & 0x3F) | (a.enable_days    ? ALARM_FIELD_ENABLE : 0);

    _write_regs(REG_ALARM2, regs, 3);
    return _error;
}

int device_pcf85263::get_alarm2(Alarm& a)
{
    uint8_t regs[3];
    if (!_read_regs(REG_ALARM2, regs, 3)) return _error;

    a.minutes        = _dec(regs[0] & 0x7F);
    a.hours          = _dec(regs[1] & 0x3F);
    a.days           = _dec(regs[2] & 0x3F);
    a.enable_minutes = (regs[0] & ALARM_FIELD_ENABLE) != 0;
    a.enable_hours   = (regs[1] & ALARM_FIELD_ENABLE) != 0;
    a.enable_days    = (regs[2] & ALARM_FIELD_ENABLE) != 0;

    return _error;
}

// ============================================================
//  看门狗定时器（7 位倒数）
// ============================================================

int device_pcf85263::set_watchdog(uint8_t count)
{
    if (count > 127)
    {
        _error = ERR_ADDR;
        return _error;
    }
    // 0 = 停止看门狗（数据手册 7.5 节）；bits 6-0 = 倒数初值
    return write_register(REG_WATCHDOG, count & 0x7F);
}

uint8_t device_pcf85263::get_watchdog()
{
    return read_register(REG_WATCHDOG) & 0x7F;
}

// ============================================================
//  标志 / 中断
// ============================================================

uint8_t device_pcf85263::get_flags()
{
    return read_register(REG_FLAGS);
}

int device_pcf85263::clear_flags(uint8_t mask)
{
    // 标志位写 0 清除；1 保持
    uint8_t flags = read_register(REG_FLAGS);
    if (_error != ERR_OK) return _error;
    return write_register(REG_FLAGS, static_cast<uint8_t>(flags & ~mask));
}

bool device_pcf85263::is_alarm1_flag()
{
    return (get_flags() & FLAG_A1F) != 0;
}

bool device_pcf85263::is_alarm2_flag()
{
    return (get_flags() & FLAG_A2F) != 0;
}

// 停振标志在日寄存器 0x04 bit7（数据手册已确认）
bool device_pcf85263::is_osc_stop_flag()
{
    return (read_register(REG_DAYS) & 0x80) != 0;
}

int device_pcf85263::clear_osc_stop_flag()
{
    uint8_t days = read_register(REG_DAYS);
    if (_error != ERR_OK) return _error;
    return write_register(REG_DAYS, static_cast<uint8_t>(days & ~0x80));
}

// TODO: INT A/B 使能寄存器（0x29/0x2A）的闹钟中断位位置未核实
//       （推测：A1 在 INT A 使能位，A2 在 INT B 使能位）。
//       位位置确认前请谨慎使用，或直接用 write_register 操作
int device_pcf85263::set_alarm_interrupts(bool a1ie, bool a2ie)
{
    uint8_t inta = read_register(REG_INTA_ENABLE);
    if (_error != ERR_OK) return _error;
    uint8_t intb = read_register(REG_INTB_ENABLE);
    if (_error != ERR_OK) return _error;

    // TODO: 以下位位置为推测值（bit0 = 闹钟1，bit1 = 闹钟2）
    inta = static_cast<uint8_t>((inta & ~0x03) | (a1ie ? 0x01 : 0) | (a2ie ? 0x02 : 0));
    intb = static_cast<uint8_t>((intb & ~0x03) | (a1ie ? 0x01 : 0) | (a2ie ? 0x02 : 0));

    int rv = write_register(REG_INTA_ENABLE, inta);
    if (rv != ERR_OK) return rv;
    return write_register(REG_INTB_ENABLE, intb);
}
