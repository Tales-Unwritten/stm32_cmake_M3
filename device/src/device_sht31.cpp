#include "device_sht31.hpp"
#include "systick.h"

// ============================================================
//  加热器冷却时间（毫秒），同参考库 SHT31_HEATER_TIMEOUT
// ============================================================

static constexpr uint32_t SHT31_HEATER_COOLDOWN_MS = 180000UL;

// ============================================================
//  构造
// ============================================================

SHT31::SHT31(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _address(addr)
{
}

SHT31::~SHT31()
{
}

// ============================================================
//  init：地址校验 + 软复位（同参考库 begin()）
// ============================================================

void SHT31::init()
{
    if ((_address != ADDR_0x44) && (_address != ADDR_0x45))
    {
        return;
    }
    (void)reset();
}

// ============================================================
//  连接 / 错误
// ============================================================

bool SHT31::isConnected()
{
    bool ok = _dev.ping();
    if (!ok) _error = ERR_NOT_CONNECT;
    return ok;
}

int SHT31::getLastError()
{
    int e = _error;
    _error = ERR_OK;
    return e;
}

// ============================================================
//  CRC8（poly 0x31, init 0xFF）—— 参考库原样移植
// ============================================================

uint8_t SHT31::crc8(const uint8_t* data, uint8_t len)
{
    const uint8_t POLY = 0x31;
    uint8_t crc = 0xFF;

    for (uint8_t j = len; j; --j)
    {
        crc ^= *data++;
        for (uint8_t i = 8; i; --i)
        {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ POLY)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

// ============================================================
//  命令 / 数据
//
//  SHT3x 无寄存器地址：
//  - writeCmd: freedom_write(高字节, 低字节, 1) 即完整 16 位命令
//  - readBytes: freedom_read(0x00, ...) 的 reg 参数填 0x00 ——
//    0x00 是不完整命令（1 字节 + STOP），SHT3x 忽略后直接返回数据，
//    与参考库 requestFrom（不写任何字节）等效
// ============================================================

bool SHT31::writeCmd(uint16_t cmd)
{
    _dev.freedom_write(static_cast<uint8_t>(cmd >> 8),
                       static_cast<uint8_t>(cmd & 0xFF), 1);
    if (_dev.lastError() != 0)
    {
        _error = ERR_WRITECMD;
        return false;
    }
    _error = ERR_OK;
    return true;
}

bool SHT31::readBytes(uint8_t n, uint8_t* buf)
{
    uint64_t v = 0;
    if (!_dev.freedom_read(0x00, &v, n))
    {
        _error = ERR_READBYTES;
        return false;
    }
    for (uint8_t i = 0; i < n; i++)
    {
        buf[i] = static_cast<uint8_t>(v >> (8 * (n - 1 - i)));
    }
    _error = ERR_OK;
    return true;
}

// ============================================================
//  测量
// ============================================================

bool SHT31::read(bool fast)
{
    if (!writeCmd(fast ? CMD_MEASURE_FAST : CMD_MEASURE_SLOW))
    {
        return false;
    }
    delay_ms(fast ? 4 : 15);   // 数据手册表 4
    return readData(fast);
}

bool SHT31::readData(bool fast)
{
    uint8_t buf[6];   // T_hi T_lo CRC H_hi H_lo CRC
    if (!readBytes(6, buf)) return false;

    if (!fast)
    {
        if (buf[2] != crc8(buf, 2))
        {
            _error = ERR_CRC_TEMP;
            return false;
        }
        if (buf[5] != crc8(buf + 3, 2))
        {
            _error = ERR_CRC_HUM;
            return false;
        }
    }
    _rawTemperature = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
    _rawHumidity    = static_cast<uint16_t>((buf[3] << 8) | buf[4]);
    _lastReadTick   = get_tick();
    return true;
}

bool SHT31::requestData()
{
    if (!writeCmd(CMD_MEASURE_SLOW)) return false;
    _lastRequestTick = get_tick();
    return true;
}

bool SHT31::dataReady()
{
    return (get_tick() - _lastRequestTick) > 15;
}

// ============================================================
//  状态 / 复位
// ============================================================

uint16_t SHT31::readStatus()
{
    if (!writeCmd(CMD_READ_STATUS)) return 0xFFFF;
    uint8_t buf[3];   // status_hi status_lo CRC
    if (!readBytes(3, buf)) return 0xFFFF;
    if (buf[2] != crc8(buf, 2))
    {
        _error = ERR_CRC_STATUS;
        return 0xFFFF;
    }
    return static_cast<uint16_t>((buf[0] << 8) | buf[1]);
}

bool SHT31::clearStatus()
{
    return writeCmd(CMD_CLEAR_STATUS);
}

bool SHT31::reset(bool hard)
{
    bool b = writeCmd(hard ? CMD_HARD_RESET : CMD_SOFT_RESET);
    if (b) delay_ms(1);   // 数据手册表 4
    return b;
}

// ============================================================
//  加热器
// ============================================================

void SHT31::setHeatTimeout(uint8_t seconds)
{
    _heatTimeout = (seconds > 180) ? 180 : seconds;
}

bool SHT31::heatOn()
{
    if (isHeaterOn()) return true;
    if ((_heaterStopTick > 0) && (get_tick() - _heaterStopTick < SHT31_HEATER_COOLDOWN_MS))
    {
        _error = ERR_HEATER_COOLDOWN;
        return false;
    }
    if (!writeCmd(CMD_HEAT_ON))
    {
        _error = ERR_HEATER_ON;
        return false;
    }
    _heaterStartTick = get_tick();
    _heaterOn        = true;
    return true;
}

bool SHT31::heatOff()
{
    // 无条件关闭加热器（参考库注释：忽略 _heaterOn 标志）
    if (!writeCmd(CMD_HEAT_OFF))
    {
        _error = ERR_HEATER_OFF;   // 可能很严重
        return false;
    }
    _heaterStopTick = get_tick();
    _heaterOn       = false;
    return true;
}

bool SHT31::isHeaterOn()
{
    if (!_heaterOn) return false;
    if ((get_tick() - _heaterStartTick) < (static_cast<uint32_t>(_heatTimeout) * 1000UL))
    {
        return true;
    }
    heatOff();
    return false;
}

// ============================================================
//  定点换算（参考库公式，int64 中间量防溢出）
//
//  T_mC = raw × 175/65535 − 45 [°C]
//       → raw × 175000/65535 − 45000 [m°C]        （-45..130°C）
//  RH_x100 = raw × 100/65535 [%]
//       → raw × 10000/65535
// ============================================================

int32_t SHT31::getTemperature_mC() const
{
    return static_cast<int32_t>((static_cast<int64_t>(_rawTemperature) * 175000) / 65535) - 45000;
}

int32_t SHT31::getHumidity_x100() const
{
    return static_cast<int32_t>((static_cast<int64_t>(_rawHumidity) * 10000) / 65535);
}

// ============================================================
//  序列号（SHT3x 电子识别码）
// ============================================================

bool SHT31::getSerialNumber(uint32_t& serial, bool fast)
{
    if (!writeCmd(CMD_GET_SERIAL)) return false;
    delay_ms(1);

    uint8_t buf[6];   // SN_hi SN_lo CRC SN_hi2 SN_lo2 CRC
    if (!readBytes(6, buf)) return false;

    if (!fast)
    {
        if (buf[2] != crc8(buf, 2))
        {
            _error = ERR_SERIAL_CRC;
            return false;
        }
        if (buf[5] != crc8(buf + 3, 2))
        {
            _error = ERR_SERIAL_CRC;
            return false;
        }
    }
    serial = (static_cast<uint32_t>(buf[0]) << 24)
           | (static_cast<uint32_t>(buf[1]) << 16)
           | (static_cast<uint32_t>(buf[3]) << 8)
           |  static_cast<uint32_t>(buf[4]);
    return true;
}

//  -- END OF FILE --
