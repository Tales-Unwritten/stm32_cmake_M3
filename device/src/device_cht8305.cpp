#include "device_cht8305.hpp"
#include "systick.h"

// ============================================================
//  构造
// ============================================================

CHT8305::CHT8305(inter_i2c_bus* bus, uint8_t addr)
    : _bus(bus)
    , _dev(bus, addr)
    , _addr(addr)
{
}

CHT8305::~CHT8305()
{
}

// ============================================================
//  init：地址校验 + 连接检查 + 写配置
// ============================================================

void CHT8305::init()
{
    if ((_addr < ADDR_0x40) || (_addr > ADDR_0x43))
    {
        _error = ERR_ADDR;
        return;
    }
    if (!isConnected())
    {
        _error = ERR_CONNECT;
        return;
    }
    setConfigRegister(buildConfigReg());
    _error = ERR_OK;
}

// ============================================================
//  连接 / 错误
// ============================================================

bool CHT8305::isConnected()
{
    return _dev.ping();
}

int CHT8305::getLastError()
{
    int e = _error;
    _error = ERR_OK;
    return e;
}

// ============================================================
//  两阶段读取（转换触发 + 延时 + 读数据）
//
//  CHT8305 读温度寄存器会触发一次新转换（约 13 ms @14bit），
//  参考库流程: 写寄存器地址 → 延时(_conversionDelay) → 读数据。
//  inter_i2c_dev 的组合事务（写寄存器 + 立即读）无法在中间插入延时，
//  因此用 inter_i2c_bus 原语只写寄存器地址（无数据）触发转换。
//  延时后仍走 _dev.freedom_read() 读回：freedom_read 会再写一次寄存器
//  地址，此时转换已完成——即使该写操作重新触发一次转换，读到的也是
//  上一次已完成的结果（参考库 requestFrom 同样不依赖寄存器指针内容）。
// ============================================================

bool CHT8305::_triggerConversion(uint8_t reg)
{
    bool ok = false;

    _bus->lock();
    _bus->start();
    _bus->write_byte(_addr << 1);
    if (_bus->wait_ack())
    {
        _bus->write_byte(reg);
        if (_bus->wait_ack()) ok = true;
    }
    _bus->stop();
    _bus->unlock();

    if (!ok) _error = ERR_I2C;
    return ok;
}

bool CHT8305::_readConverted(uint8_t reg, uint64_t* data, uint8_t len)
{
    if (!_triggerConversion(reg)) return false;
    delay_ms(_conversionDelay);
    if (!_dev.freedom_read(reg, data, len))
    {
        _error = ERR_I2C;
        return false;
    }
    _error = ERR_OK;
    return true;
}

// ============================================================
//  定点换算（参考库公式，int64 中间量防溢出）
//
//  T_mC  = raw × 165/65535 − 40   [°C]
//        → raw × 165000/65535 − 40000  [m°C]
//  RH_x100 = raw × 100/65535      [%]
//        → raw × 10000/65535
// ============================================================

int32_t CHT8305::rawToTemp_mC(uint16_t raw)
{
    return static_cast<int32_t>((static_cast<int64_t>(raw) * 165000) / 65535) - 40000;
}

int32_t CHT8305::rawToHum_x100(uint16_t raw)
{
    return static_cast<int32_t>((static_cast<int64_t>(raw) * 10000) / 65535);
}

// ============================================================
//  测量
//  参考库限制：距上次读取不足 1 秒直接报 ERR_LASTREAD
//  （芯片推荐读取频率 1 次/秒）；首次读取不受限
// ============================================================

int CHT8305::read()
{
    if ((_lastReadTick != 0) && (get_tick() - _lastReadTick < 1000))
    {
        _error = ERR_LASTREAD;
        return _error;
    }
    _lastReadTick = get_tick();

    uint64_t data = 0;
    if (!_readConverted(REG_TEMPERATURE, &data, 4))   // T_hi T_lo H_hi H_lo
    {
        return _error;
    }

    _temperature_mC = rawToTemp_mC(static_cast<uint16_t>(data >> 16));
    _humidity_x100  = rawToHum_x100(static_cast<uint16_t>(data & 0xFFFF));

    if (_tempOffset_mC != 0) _temperature_mC += _tempOffset_mC;
    if (_humOffset_x100 != 0)
    {
        _humidity_x100 += _humOffset_x100;
        if (_humidity_x100 < 0)     _humidity_x100 = 0;
        if (_humidity_x100 > 10000) _humidity_x100 = 10000;   // 限幅 0..100%
    }
    _error = ERR_OK;
    return _error;
}

int CHT8305::readTemperature()
{
    if ((_lastReadTick != 0) && (get_tick() - _lastReadTick < 1000))
    {
        _error = ERR_LASTREAD;
        return _error;
    }
    _lastReadTick = get_tick();

    uint64_t data = 0;
    if (!_readConverted(REG_TEMPERATURE, &data, 2))
    {
        return _error;
    }
    _temperature_mC = rawToTemp_mC(static_cast<uint16_t>(data));
    if (_tempOffset_mC != 0) _temperature_mC += _tempOffset_mC;

    _error = ERR_OK;
    return _error;
}

int CHT8305::readHumidity()
{
    if ((_lastReadTick != 0) && (get_tick() - _lastReadTick < 1000))
    {
        _error = ERR_LASTREAD;
        return _error;
    }
    _lastReadTick = get_tick();

    // 湿度读取不触发转换（参考库 0.2.3 起无延时），无需两阶段事务
    uint64_t data = 0;
    if (!_dev.freedom_read(REG_HUMIDITY, &data, 2))
    {
        _error = ERR_I2C;
        return _error;
    }
    _humidity_x100 = rawToHum_x100(static_cast<uint16_t>(data));
    if (_humOffset_x100 != 0)
    {
        _humidity_x100 += _humOffset_x100;
        if (_humidity_x100 < 0)     _humidity_x100 = 0;
        if (_humidity_x100 > 10000) _humidity_x100 = 10000;
    }

    _error = ERR_OK;
    return _error;
}

// ============================================================
//  转换延时 / 偏移
// ============================================================

void CHT8305::setConversionDelay(uint8_t cd)
{
    if (cd < 8) cd = 8;   // 参考库实测 7 ms 失败、8 ms 成功
    _conversionDelay = cd;
}

void CHT8305::setTemperatureOffset_mC(int32_t offset_mC)
{
    _tempOffset_mC = offset_mC;
}

void CHT8305::setHumidityOffset_x100(int32_t offset_x100)
{
    _humOffset_x100 = offset_x100;
}

// ============================================================
//  CONFIG 寄存器
// ============================================================

void CHT8305::setConfigRegister(uint16_t bitmask)
{
    _dev.write_16bit(REG_CONFIG, bitmask & ~CFG_RESERVED);   // bit1-0 保留位写 0
}

uint16_t CHT8305::getConfigRegister()
{
    return _dev.read_16bit(REG_CONFIG);
}

void CHT8305::setConfig(const Config& cfg)
{
    _cfg = cfg;
    setConfigRegister(buildConfigReg());
}

uint16_t CHT8305::buildConfigReg() const
{
    uint16_t r = 0;
    if (_cfg.clockStretch)         r |= CFG_CLOCK_STRETCH;
    if (_cfg.heater)               r |= CFG_HEATER;
    if (_cfg.modeBoth)             r |= CFG_MODE;
    if (_cfg.tempRes11bit)         r |= CFG_TEMP_RES;
    if (_cfg.humiRes == 2)         r |= 0x0200;   // 8 bit
    else if (_cfg.humiRes == 1)    r |= 0x0100;   // 11 bit
    r |= static_cast<uint16_t>(_cfg.alertTriggerMode & 0x03) << 6;
    if (_cfg.vccEnable)            r |= CFG_VCC_ENABLE;
    return r;
}

void CHT8305::_setConfigMask(uint16_t mask)
{
    setConfigRegister(getConfigRegister() | mask);
}

void CHT8305::_clrConfigMask(uint16_t mask)
{
    setConfigRegister(getConfigRegister() & ~mask);
}

// ============================================================
//  配置位操作
// ============================================================

void CHT8305::softReset()
{
    // 置位后芯片复位回默认值（0x1004），_cfg 不再代表芯片状态，
    // 需要时请重新 setConfig()
    _setConfigMask(CFG_SOFT_RESET);
}

void CHT8305::setI2CClockStretch(bool on)
{
    if (on) _setConfigMask(CFG_CLOCK_STRETCH);
    else    _clrConfigMask(CFG_CLOCK_STRETCH);
}

bool CHT8305::getI2CClockStretch()
{
    return (getConfigRegister() & CFG_CLOCK_STRETCH) != 0;
}

void CHT8305::setHeaterOn(bool on)
{
    if (on) _setConfigMask(CFG_HEATER);
    else    _clrConfigMask(CFG_HEATER);
}

bool CHT8305::getHeater()
{
    return (getConfigRegister() & CFG_HEATER) != 0;
}

void CHT8305::setMeasurementMode(bool both)
{
    if (both) _setConfigMask(CFG_MODE);
    else      _clrConfigMask(CFG_MODE);
}

bool CHT8305::getMeasurementMode()
{
    return (getConfigRegister() & CFG_MODE) != 0;
}

bool CHT8305::getVCCstatus()
{
    return (getConfigRegister() & CFG_VCCS) != 0;   // 1 = VCC > 2.8V
}

void CHT8305::setTemperatureResolution(uint8_t res)
{
    if (res == 1) _setConfigMask(CFG_TEMP_RES);
    else          _clrConfigMask(CFG_TEMP_RES);
}

uint8_t CHT8305::getTemperatureResolution()
{
    return (getConfigRegister() & CFG_TEMP_RES) ? 1 : 0;
}

void CHT8305::setHumidityResolution(uint8_t res)
{
    _clrConfigMask(CFG_HUMI_RES);
    if (res == 2) _setConfigMask(0x0200);   // 8 bit
    if (res == 1) _setConfigMask(0x0100);   // 11 bit
    // 其它值 = 14 bit（默认）
}

uint8_t CHT8305::getHumidityResolution()
{
    return static_cast<uint8_t>((getConfigRegister() & CFG_HUMI_RES) >> 8);
}

void CHT8305::setVCCenable(bool enable)
{
    if (enable) _setConfigMask(CFG_VCC_ENABLE);
    else        _clrConfigMask(CFG_VCC_ENABLE);
}

bool CHT8305::getVCCenable()
{
    return (getConfigRegister() & CFG_VCC_ENABLE) != 0;
}

// ============================================================
//  报警
// ============================================================

bool CHT8305::setAlertTriggerMode(uint8_t mode)
{
    if (mode > 3) return false;
    _clrConfigMask(CFG_ALERT_MODE);
    _setConfigMask(static_cast<uint16_t>(mode) << 6);
    return true;
}

uint8_t CHT8305::getAlertTriggerMode()
{
    return static_cast<uint8_t>((getConfigRegister() & CFG_ALERT_MODE) >> 6);
}

bool CHT8305::getAlertPendingStatus()
{
    return (getConfigRegister() & CFG_ALERT_PENDING) != 0;
}

bool CHT8305::getAlertHumidityStatus()
{
    return (getConfigRegister() & CFG_ALERT_HUMI) != 0;
}

bool CHT8305::getAlertTemperatureStatus()
{
    return (getConfigRegister() & CFG_ALERT_TEMP) != 0;
}

// ============================================================
//  报警上限写入（参考库公式，int64 中间量）
//
//  湿度码 = RH × 127/100（7 bit）  ← RH 为 %RH×100 → ×127/10000
//  温度码 = (T + 40) × 511/165（9 bit）← T 为 m°C → ×511/165000
// ============================================================

bool CHT8305::setAlertLevels_mC(int32_t t_mC, int32_t rh_x100)
{
    if ((t_mC   < -40000) || (t_mC   > 125000)) return false;   // -40..125°C
    if ((rh_x100 < 0)     || (rh_x100 > 10000)) return false;   // 0..100%

    uint16_t humCode  = static_cast<uint16_t>((static_cast<int64_t>(rh_x100) * 127) / 10000);
    uint16_t tempCode = static_cast<uint16_t>((static_cast<int64_t>(t_mC + 40000) * 511) / 165000);
    _dev.write_16bit(REG_ALERT, static_cast<uint16_t>((humCode << 9) | tempCode));
    return true;
}

int32_t CHT8305::getAlertLevelTemperature_mC()
{
    uint16_t raw  = _dev.read_16bit(REG_ALERT);
    uint16_t code = static_cast<uint16_t>((raw & 0x01FF) << 7);   // 9 bit → 16 bit 刻度
    return rawToTemp_mC(code);
}

int32_t CHT8305::getAlertLevelHumidity_x100()
{
    uint16_t raw = _dev.read_16bit(REG_ALERT);
    return rawToHum_x100(static_cast<uint16_t>(raw & 0xFE00));    // 7 bit → 16 bit 刻度
}

// ============================================================
//  电压（参考库 best guess，含义未定）
// ============================================================

uint16_t CHT8305::getVoltageRaw()
{
    return _dev.read_16bit(REG_VOLTAGE);
}

int32_t CHT8305::getVoltage_mV()
{
    // V = raw × 5.0 / 32768 [V] → mV = raw × 5000 / 32768
    return static_cast<int32_t>((static_cast<int64_t>(getVoltageRaw()) * 5000) / 32768);
}

// ============================================================
//  设备信息（读失败返回 0，参考库返回 0xFFFF）
// ============================================================

uint16_t CHT8305::getManufacturer()
{
    return _dev.read_16bit(REG_MANUFACTURER);
}

uint16_t CHT8305::getVersionID()
{
    return _dev.read_16bit(REG_VERSION);
}

//  -- END OF FILE --
