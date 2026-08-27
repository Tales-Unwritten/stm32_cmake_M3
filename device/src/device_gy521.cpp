#include "device_gy521.hpp"
#include "systick.h"

// ============================================================
//  定点换算
//  （int64 中间量防溢出；结果截断为 int32）
// ============================================================

namespace
{
// 加速度：±2g = 16384 LSB/g，量程翻倍则灵敏度减半
//   mg = raw × 1000 / (16384 >> afs)
int32_t rawToAccel_mg(int32_t raw, uint8_t afs)
{
    return (raw * 1000) / (16384 >> afs);
}

// 陀螺仪：±250 °/s = 131 LSB/(°/s)，量程翻倍则灵敏度减半
//   0.001 °/s = raw × (1000 << gfs) / 131
int32_t rawToGyro_mdps(int32_t raw, uint8_t gfs)
{
    return static_cast<int32_t>(
        (static_cast<int64_t>(raw) * (1000 << gfs)) / 131);
}

// 温度：T(°C) = raw / 340 + 36.53
//   m°C = raw × 1000/340 + 36530 = raw × 50/17 + 36530
int32_t rawToTemp_mC(int32_t raw)
{
    return (raw * 50) / 17 + 36530;
}
}

// ============================================================
//  构造
// ============================================================

GY521::GY521(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
{
}

// ============================================================
//  init：唤醒 + 应用量程/DLPF
// ============================================================

void GY521::init()
{
    // 唤醒（上电默认睡眠）：PWR_MGMT_1 = 0x00
    (void)setRegister(REG_PWR_MGMT_1, 0x00);

    // 应用构造函数后设置过的量程 / DLPF（含 reset() 后恢复）
    (void)setAccelRange(_afs);
    (void)setGyroRange(_gfs);
}

// ============================================================
//  连接 / 错误
// ============================================================

bool GY521::isConnected()
{
    return _dev.ping();
}

uint8_t GY521::getLastError()
{
    return _dev.lastError();
}

uint8_t GY521::readWhoAmI()
{
    return getRegister(REG_WHO_AM_I);
}

// ============================================================
//  量程 / 滤波
// ============================================================

bool GY521::setAccelRange(uint8_t range)
{
    if (range > 3) range = 3;
    uint8_t val = getRegister(REG_ACCEL_CONFIG);
    if (_dev.lastError() != 0) return false;

    val &= 0xE7;                // 清 [4:3]
    val |= static_cast<uint8_t>(range << 3);
    if (setRegister(REG_ACCEL_CONFIG, val) != ERR_OK) return false;

    _afs = range;
    return true;
}

uint8_t GY521::getAccelRange()
{
    uint8_t val = getRegister(REG_ACCEL_CONFIG);
    _afs = (val >> 3) & 0x03;
    return _afs;
}

bool GY521::setGyroRange(uint8_t range)
{
    if (range > 3) range = 3;
    uint8_t val = getRegister(REG_GYRO_CONFIG);
    if (_dev.lastError() != 0) return false;

    val &= 0xE7;
    val |= static_cast<uint8_t>(range << 3);
    if (setRegister(REG_GYRO_CONFIG, val) != ERR_OK) return false;

    _gfs = range;
    return true;
}

uint8_t GY521::getGyroRange()
{
    uint8_t val = getRegister(REG_GYRO_CONFIG);
    _gfs = (val >> 3) & 0x03;
    return _gfs;
}

bool GY521::setDLPF(uint8_t mode)
{
    if (mode > 6) return false;
    uint8_t val = getRegister(REG_CONFIG);
    if (_dev.lastError() != 0) return false;

    val &= 0xF8;                // 清 [2:0]
    val |= mode;
    return setRegister(REG_CONFIG, val) == ERR_OK;
}

uint8_t GY521::getDLPF()
{
    return getRegister(REG_CONFIG) & 0x07;
}

// ============================================================
//  读取
// ============================================================

bool GY521::_readBlock(uint8_t reg, uint8_t len, uint64_t* out)
{
    return _dev.freedom_read(reg, out, len);
}

int16_t GY521::read()
{
    if (_throttle)
    {
        uint32_t now = get_tick();
        if ((now - _lastTime) < _throttleTime) return ERR_THROTTLED;
        _lastTime = now;
    }

    // 0x3B 起 6 字节：AX_H AX_L AY_H AY_L AZ_H AZ_L
    uint64_t v = 0;
    if (!_readBlock(REG_ACCEL_XOUT_H, 6, &v)) return ERR_READ;
    _ax = static_cast<int16_t>((v >> 32) & 0xFFFF);
    _ay = static_cast<int16_t>((v >> 16) & 0xFFFF);
    _az = static_cast<int16_t>(v & 0xFFFF);

    // 0x41 起 2 字节：TEMP_H TEMP_L
    if (!_readBlock(REG_TEMP_OUT_H, 2, &v)) return ERR_READ;
    _temperature = static_cast<int16_t>(v & 0xFFFF);

    // 0x43 起 6 字节：GX_H GX_L GY_H GY_L GZ_H GZ_L
    if (!_readBlock(REG_GYRO_XOUT_H, 6, &v)) return ERR_READ;
    _gx = static_cast<int16_t>((v >> 32) & 0xFFFF);
    _gy = static_cast<int16_t>((v >> 16) & 0xFFFF);
    _gz = static_cast<int16_t>(v & 0xFFFF);

    return ERR_OK;
}

int16_t GY521::readAccel()
{
    if (_throttle)
    {
        uint32_t now = get_tick();
        if ((now - _lastTime) < _throttleTime) return ERR_THROTTLED;
        _lastTime = now;
    }

    uint64_t v = 0;
    if (!_readBlock(REG_ACCEL_XOUT_H, 6, &v)) return ERR_READ;
    _ax = static_cast<int16_t>((v >> 32) & 0xFFFF);
    _ay = static_cast<int16_t>((v >> 16) & 0xFFFF);
    _az = static_cast<int16_t>(v & 0xFFFF);
    return ERR_OK;
}

int16_t GY521::readGyro()
{
    if (_throttle)
    {
        uint32_t now = get_tick();
        if ((now - _lastTime) < _throttleTime) return ERR_THROTTLED;
        _lastTime = now;
    }

    uint64_t v = 0;
    if (!_readBlock(REG_GYRO_XOUT_H, 6, &v)) return ERR_READ;
    _gx = static_cast<int16_t>((v >> 32) & 0xFFFF);
    _gy = static_cast<int16_t>((v >> 16) & 0xFFFF);
    _gz = static_cast<int16_t>(v & 0xFFFF);
    return ERR_OK;
}

int16_t GY521::readTemperature()
{
    uint64_t v = 0;
    if (!_readBlock(REG_TEMP_OUT_H, 2, &v)) return ERR_READ;
    _temperature = static_cast<int16_t>(v & 0xFFFF);
    return ERR_OK;
}

// ============================================================
//  定点换算（校准偏移为 raw 计数，先减后换算）
// ============================================================

int32_t GY521::getAccelX_mg()  { return rawToAccel_mg(_ax - _axe, _afs); }
int32_t GY521::getAccelY_mg()  { return rawToAccel_mg(_ay - _aye, _afs); }
int32_t GY521::getAccelZ_mg()  { return rawToAccel_mg(_az - _aze, _afs); }

int32_t GY521::getGyroX_mdps() { return rawToGyro_mdps(_gx - _gxe, _gfs); }
int32_t GY521::getGyroY_mdps() { return rawToGyro_mdps(_gy - _gye, _gfs); }
int32_t GY521::getGyroZ_mdps() { return rawToGyro_mdps(_gz - _gze, _gfs); }

int32_t GY521::getTemperature_mC() { return rawToTemp_mC(_temperature); }

// ============================================================
//  校准（静止平均偏移，raw 计数）
// ============================================================

bool GY521::calibrate(uint16_t times)
{
    if (times == 0) times = 1;

    // 校准期间关闭节流，避免采样被 ERR_THROTTLED 跳过
    bool oldThrottle = _throttle;
    _throttle = false;

    int64_t ax = 0, ay = 0, az = 0;
    int64_t gx = 0, gy = 0, gz = 0;
    uint32_t n = 0;

    for (uint16_t i = 0; i < times; i++)
    {
        if (read() == ERR_OK)
        {
            ax += _ax; ay += _ay; az += _az;
            gx += _gx; gy += _gy; gz += _gz;
            n++;
        }
    }

    _throttle = oldThrottle;

    if (n == 0) return false;

    _axe = static_cast<int32_t>(ax / n);
    _aye = static_cast<int32_t>(ay / n);
    _aze = static_cast<int32_t>(az / n);
    _gxe = static_cast<int32_t>(gx / n);
    _gye = static_cast<int32_t>(gy / n);
    _gze = static_cast<int32_t>(gz / n);
    return true;
}

// ============================================================
//  通用寄存器访问
// ============================================================

uint8_t GY521::setRegister(uint8_t reg, uint8_t value)
{
    _dev.freedom_write(reg, value, 1);
    return (_dev.lastError() == 0) ? ERR_OK : ERR_WRITE;
}

uint8_t GY521::getRegister(uint8_t reg)
{
    uint64_t v = 0;
    (void)_dev.freedom_read(reg, &v, 1);
    return static_cast<uint8_t>(v & 0xFF);
}

void GY521::reset()
{
    // PWR_MGMT_1 bit7 = 设备复位（全部寄存器恢复默认值）
    (void)setRegister(REG_PWR_MGMT_1, 0x80);
    delay_ms(100);              // 复位完成需约 100 ms
    _afs = 0;
    _gfs = 0;
    init();                     // 重新唤醒并恢复量程设置
}

//  -- END OF FILE --
