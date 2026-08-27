#include "device_as5600.hpp"
#include "systick.h"

// ============================================================
//  定点换算
// ============================================================

namespace
{
// raw(12 位) → 0.01° 定点：raw × 879 / 100（int32，任务约定公式；
//   精确值 36000/4096 = 8.7890625°/raw，全量程误差 < 0.01°）
int32_t rawToC01(uint32_t raw)
{
    return static_cast<int32_t>(raw) * 879 / 100;
}

// 0.01° 定点 → raw（四舍五入；负数用补码形式，与原库一致）
uint16_t c01ToRaw(int32_t deg_c01)
{
    if (deg_c01 < 0) deg_c01 = -deg_c01;
    uint32_t raw = (static_cast<uint32_t>(deg_c01) * 100 + 439) / 879;
    return static_cast<uint16_t>(raw & 0x0FFF);
}

// 配置寄存器位段掩码（CONF 高字节 / 低字节）
constexpr uint8_t CONF_POWER_MODE    = 0x03;   // 高字节 [1:0]
constexpr uint8_t CONF_HYSTERESIS    = 0x0C;   // 高字节 [3:2]
constexpr uint8_t CONF_SLOW_FILTER   = 0x03;   // 低字节 [1:0]
constexpr uint8_t CONF_FAST_FILTER   = 0x1C;   // 低字节 [4:2]
constexpr uint8_t CONF_WATCH_DOG     = 0x20;   // 低字节 [5]
}

// ============================================================
//  构造
// ============================================================

AS5600::AS5600(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _addr(addr)
{
}

void AS5600::init()
{
    // 只探测；本驱动默认不写配置寄存器
    (void)isConnected();
}

// ============================================================
//  连接 / 错误
// ============================================================

bool AS5600::isConnected()
{
    return _dev.ping();
}

uint8_t AS5600::getLastError()
{
    return _dev.lastError();
}

// ============================================================
//  角度
// ============================================================

uint16_t AS5600::rawAngle()
{
    return readReg2(REG_RAW_ANGLE) & 0x0FFF;
}

uint16_t AS5600::readAngle()
{
    uint16_t value = readReg2(REG_ANGLE);
    if (_dev.lastError() != 0)
    {
        return _lastReadAngle;   // 读失败返回上次成功值
    }
    value += _offset;
    value &= 0x0FFF;
    _lastReadAngle = static_cast<int16_t>(value);
    return value;
}

int32_t AS5600::getAngle_c01()
{
    return rawToC01(readAngle());
}

// ============================================================
//  软件偏移（0.01° 定点）
// ============================================================

bool AS5600::setOffset(int32_t degrees_c01)
{
    if (degrees_c01 < -36000 || degrees_c01 > 36000) return false;

    bool neg = (degrees_c01 < 0);
    uint16_t offset = c01ToRaw(degrees_c01);
    if (neg) offset = static_cast<uint16_t>((4096 - offset) & 0x0FFF);
    _offset = offset;
    return true;
}

int32_t AS5600::getOffset()
{
    return rawToC01(_offset);
}

bool AS5600::increaseOffset(int32_t degrees_c01)
{
    return setOffset(getOffset() + degrees_c01);
}

// ============================================================
//  状态寄存器
// ============================================================

uint8_t AS5600::readStatus()
{
    return readReg(REG_STATUS);
}

uint8_t AS5600::readAGC()
{
    return readReg(REG_AGC);
}

uint16_t AS5600::readMagnitude()
{
    return readReg2(REG_MAGNITUDE) & 0x0FFF;
}

bool AS5600::magnetDetected()
{
    return (readStatus() & STATUS_MAGNET_DETECT) != 0;
}

bool AS5600::magnetTooStrong()
{
    return (readStatus() & STATUS_MAGNET_HIGH) != 0;
}

bool AS5600::magnetTooWeak()
{
    return (readStatus() & STATUS_MAGNET_LOW) != 0;
}

// ============================================================
//  配置寄存器
// ============================================================

uint8_t AS5600::getZMCO()
{
    return readReg(REG_ZMCO);
}

uint16_t AS5600::getZPosition()
{
    return readReg2(REG_ZPOS) & 0x0FFF;
}

uint16_t AS5600::getMPosition()
{
    return readReg2(REG_MPOS) & 0x0FFF;
}

uint16_t AS5600::getMaxAngle()
{
    return readReg2(REG_MANG) & 0x0FFF;
}

uint16_t AS5600::getConfiguration()
{
    return readReg2(REG_CONF) & 0x3FFF;
}

bool AS5600::setConfiguration(uint16_t value)
{
    if (value > 0x3FFF) return false;
    return writeReg2(REG_CONF, value);
}

bool AS5600::setPowerMode(uint8_t mode)
{
    if (mode > 3) return false;
    uint8_t value = readReg(REG_CONF + 1);
    value &= static_cast<uint8_t>(~CONF_POWER_MODE);
    value |= mode;
    return writeReg(REG_CONF + 1, value);
}

uint8_t AS5600::getPowerMode()
{
    return readReg(REG_CONF + 1) & CONF_POWER_MODE;
}

bool AS5600::setHysteresis(uint8_t hyst)
{
    if (hyst > 3) return false;
    uint8_t value = readReg(REG_CONF + 1);
    value &= static_cast<uint8_t>(~CONF_HYSTERESIS);
    value |= static_cast<uint8_t>(hyst << 2);
    return writeReg(REG_CONF + 1, value);
}

uint8_t AS5600::getHysteresis()
{
    return (readReg(REG_CONF + 1) >> 2) & 0x03;
}

bool AS5600::setSlowFilter(uint8_t mask)
{
    if (mask > 3) return false;
    uint8_t value = readReg(REG_CONF);
    value &= static_cast<uint8_t>(~CONF_SLOW_FILTER);
    value |= mask;
    return writeReg(REG_CONF, value);
}

uint8_t AS5600::getSlowFilter()
{
    return readReg(REG_CONF) & CONF_SLOW_FILTER;
}

bool AS5600::setFastFilter(uint8_t mask)
{
    if (mask > 7) return false;
    uint8_t value = readReg(REG_CONF);
    value &= static_cast<uint8_t>(~CONF_FAST_FILTER);
    value |= static_cast<uint8_t>(mask << 2);
    return writeReg(REG_CONF, value);
}

uint8_t AS5600::getFastFilter()
{
    return (readReg(REG_CONF) >> 2) & 0x07;
}

bool AS5600::setWatchDog(uint8_t on)
{
    if (on > 1) return false;
    uint8_t value = readReg(REG_CONF);
    value &= static_cast<uint8_t>(~CONF_WATCH_DOG);
    value |= static_cast<uint8_t>(on << 5);
    return writeReg(REG_CONF, value);
}

uint8_t AS5600::getWatchDog()
{
    return (readReg(REG_CONF) >> 5) & 0x01;
}

// ============================================================
//  累计位置（整数运算；假设相邻两次读数 < 半圈）
// ============================================================

int32_t AS5600::getCumulativePosition(bool update)
{
    if (update)
    {
        _lastReadAngle = static_cast<int16_t>(readAngle());
        if (_dev.lastError() != 0)
        {
            return _position;   // 读失败返回上次已知位置
        }
    }
    int16_t value = _lastReadAngle;

    if ((_lastPosition > 2048) && (value < (_lastPosition - 2048)))
    {
        // 正向过一圈
        _position = _position + 4096 - _lastPosition + value;
    }
    else if ((value > 2048) && (_lastPosition < (value - 2048)))
    {
        // 反向过一圈
        _position = _position - 4096 - _lastPosition + value;
    }
    else
    {
        _position = _position - _lastPosition + value;
    }
    _lastPosition = value;
    return _position;
}

int32_t AS5600::getRevolutions()
{
    int32_t p = _position >> 12;   // / 4096
    if (p < 0) p++;                // 负数修正（对齐原库 #65）
    return p;
}

int32_t AS5600::resetCumulativePosition(int32_t position)
{
    _lastPosition = static_cast<int16_t>(readAngle());
    int32_t old = _position;
    _position = position;
    return old;
}

// ============================================================
//  角速度（0.01°/s 定点）
//   speed = Δ角度(0.01°) / Δt(s) = Δraw×879/100 × 1000/Δt(ms)
//         = Δraw × 8790 / Δt(ms)          （int64 中间量）
// ============================================================

int32_t AS5600::getAngularSpeed_c01_per_s(bool update)
{
    if (update)
    {
        _lastReadAngle = static_cast<int16_t>(readAngle());
        if (_dev.lastError() != 0) return 0;
    }

    uint32_t now = get_tick();
    uint32_t deltaT = now - _lastMeasurement;
    _lastMeasurement = now;

    int32_t deltaA = static_cast<int32_t>(_lastReadAngle) - _lastAngle;
    _lastAngle = _lastReadAngle;

    // 假设两次测量间旋转 < 180°，修正跳变
    if (deltaA > 2048)       deltaA -= 4096;
    else if (deltaA < -2048) deltaA += 4096;

    if (deltaT == 0) return 0;   // 同一次 tick 内重复调用，无法计算

    return static_cast<int32_t>(
        (static_cast<int64_t>(deltaA) * 8790) / deltaT);
}

// ============================================================
//  电源重启（重新加载 OTP 配置到寄存器）
// ============================================================

void AS5600::resetPOR()
{
    (void)writeReg(REG_BURN, 0x01);
    (void)writeReg(REG_BURN, 0x11);
    (void)writeReg(REG_BURN, 0x10);
    delay_ms(5);
}

// ============================================================
//  I2C 寄存器读写（MSB 优先）
// ============================================================

uint8_t AS5600::readReg(uint8_t reg)
{
    uint64_t v = 0;
    (void)_dev.freedom_read(reg, &v, 1);
    return static_cast<uint8_t>(v & 0xFF);
}

uint16_t AS5600::readReg2(uint8_t reg)
{
    uint64_t v = 0;
    (void)_dev.freedom_read(reg, &v, 2);
    return static_cast<uint16_t>(v & 0xFFFF);
}

bool AS5600::writeReg(uint8_t reg, uint8_t value)
{
    _dev.freedom_write(reg, value, 1);
    return _dev.lastError() == 0;
}

bool AS5600::writeReg2(uint8_t reg, uint16_t value)
{
    _dev.freedom_write(reg, value, 2);
    return _dev.lastError() == 0;
}

//  -- END OF FILE --
