#include "device_sht4x.hpp"
#include "systick.h"

// ============================================================
//  构造
// ============================================================

SHT4x::SHT4x(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _address(addr)
{
}

SHT4x::~SHT4x()
{
}

// ============================================================
//  init：地址校验 + 软复位（同参考库 begin()）
// ============================================================

void SHT4x::init()
{
    if ((_address < ADDR_0x44) || (_address > ADDR_0x46))
    {
        _error = ERR_INVALID_ADDRESS;
        return;
    }
    (void)reset();
}

// ============================================================
//  连接 / 错误
// ============================================================

bool SHT4x::isConnected()
{
    bool ok = _dev.ping();
    if (!ok) _error = ERR_NOT_CONNECT;
    return ok;
}

int SHT4x::getLastError()
{
    int e = _error;
    _error = ERR_OK;
    return e;
}

// ============================================================
//  测量
// ============================================================

bool SHT4x::read(MeasType type, bool crcCheck)
{
    if (!requestData(type))
    {
        return false;   // _error 已在 requestData() 中设置
    }
    delay_ms(_delay);
    return readData(crcCheck);
}

bool SHT4x::requestData(MeasType type)
{
    // 加热命令的冷却保护
    if (_heatProtection && isHeatCmd(type))
    {
        if (!heatingReady())
        {
            _error = ERR_HEATER_COOLDOWN;
            return false;
        }
    }
    if (!writeCommand(static_cast<uint8_t>(type)))
    {
        return false;
    }
    _lastRequestTick = get_tick();
    setDelay(type);
    if (isHeatCmd(type))
    {
        setHeatInterval(type);
        _lastHeatRequestTick = _lastRequestTick;
    }
    return true;
}

bool SHT4x::dataReady()
{
    return (get_tick() - _lastRequestTick) > _delay;
}

bool SHT4x::readData(bool crcCheck)
{
    uint8_t buf[6];   // T_hi T_lo CRC H_hi H_lo CRC
    if (!readBytes(6, buf))
    {
        _error = ERR_READBYTES;
        return false;
    }

    if (crcCheck)
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

bool SHT4x::reset(bool fast)
{
    bool b = writeCommand(CMD_SOFT_RESET);
    if (b && !fast) delay_ms(1);   // 数据手册表 5
    return b;
}

// ============================================================
//  加热保护
// ============================================================

void SHT4x::setHeatProtection(bool enable)
{
    _heatProtection = enable;
}

bool SHT4x::heatingReady()
{
    return (get_tick() - _lastHeatRequestTick) > _heatInterval;
}

// ============================================================
//  测量类型参数表（数据手册表 5）
// ============================================================

void SHT4x::setDelay(MeasType type)
{
    switch (type)
    {
        case MEAS_SLOW:        _delay = 9; break;
        case MEAS_MEDIUM:      _delay = 5; break;
        case MEAS_FAST:        _delay = 2; break;
        case MEAS_LONG_HIGH:
        case MEAS_LONG_MED:
        case MEAS_LONG_LOW:    _delay = 1100; break;
        case MEAS_SHORT_HIGH:
        case MEAS_SHORT_MED:
        case MEAS_SHORT_LOW:   _delay = 110; break;
    }
}

void SHT4x::setHeatInterval(MeasType type)
{
    // 10% 占空比（参考库注释）
    switch (type)
    {
        case MEAS_LONG_HIGH:
        case MEAS_LONG_MED:
        case MEAS_LONG_LOW:    _heatInterval = 10000; break;
        case MEAS_SHORT_HIGH:
        case MEAS_SHORT_MED:
        case MEAS_SHORT_LOW:   _heatInterval = 1000; break;
        default: break;
    }
}

bool SHT4x::isHeatCmd(MeasType type)
{
    switch (type)
    {
        case MEAS_LONG_HIGH:  case MEAS_SHORT_HIGH:
        case MEAS_LONG_MED:   case MEAS_SHORT_MED:
        case MEAS_LONG_LOW:   case MEAS_SHORT_LOW:
            return true;
        default:
            return false;
    }
}

// ============================================================
//  定点换算（参考库公式，int64 中间量防溢出）
//
//  T_mC = raw × 175/65535 − 45 [°C]
//       → raw × 175000/65535 − 45000 [m°C]
//  RH_x100 = raw × 125/65535 − 6 [%]
//       → raw × 12500/65535 − 600，限幅 0..100%
// ============================================================

int32_t SHT4x::getTemperature_mC() const
{
    return static_cast<int32_t>((static_cast<int64_t>(_rawTemperature) * 175000) / 65535) - 45000;
}

int32_t SHT4x::getHumidity_x100() const
{
    int32_t rh = static_cast<int32_t>((static_cast<int64_t>(_rawHumidity) * 12500) / 65535) - 600;
    if (rh < 0)     rh = 0;
    if (rh > 10000) rh = 10000;
    return rh;
}

// ============================================================
//  序列号（同 SHT3x 电子识别码格式）
// ============================================================

bool SHT4x::getSerialNumber(uint32_t& serial, bool crcCheck)
{
    if (!writeCommand(CMD_GET_SERIAL)) return false;
    delay_ms(1);

    uint8_t buf[6];   // SN_hi SN_lo CRC SN_hi2 SN_lo2 CRC
    if (!readBytes(6, buf)) return false;

    if (crcCheck)
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

// ============================================================
//  I2C 原语
//
//  SHT4x 命令为单字节：
//  - writeCommand: freedom_write(cmd, 0x00, 1) 会在命令后带一个尾随
//    0x00 字节。SHT4x 对未定义命令一律忽略（数据手册），且 freedom_write
//    不检查数据字节的 ACK，尾随字节即使被 NACK 也无害
//  - readBytes: freedom_read(0x00, ...) 的 reg 参数填 0x00 —— 未定义
//    命令被忽略，与参考库 requestFrom（不写任何字节）等效
// ============================================================

bool SHT4x::writeCommand(uint8_t cmd)
{
    _dev.freedom_write(cmd, 0x00, 1);
    if (_dev.lastError() != 0)
    {
        _error = ERR_WRITECMD;
        return false;
    }
    _error = ERR_OK;
    return true;
}

bool SHT4x::readBytes(uint8_t n, uint8_t* buf)
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
//  CRC8（poly 0x31, init 0xFF）—— 参考库原样移植
// ============================================================

uint8_t SHT4x::crc8(const uint8_t* data, uint8_t len)
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

//  -- END OF FILE --
