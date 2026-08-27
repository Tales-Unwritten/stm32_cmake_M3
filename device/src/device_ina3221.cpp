#include "device_ina3221.hpp"
#include "device_serial.hpp"

#include <cstdio>

// ============================================================
//  INA3221 3 通道电流监测驱动（STM32G070 移植版）
//  来源库: Rob Tillaart INA3221_RT Arduino Library v0.4.2 (2024-02-05)
//     URL: https://github.com/RobTillaart/INA3221_RT
//  移植说明:
//    - 无浮点，物理量定点整数
//    - 无电流/功率寄存器：I = Vshunt/R、P = Vbus×I 软件计算
//    - 无 CAL 寄存器；每通道采样电阻存于软件（µΩ）
// ============================================================

// ============================================================
//  构造
// ============================================================

INA3221::INA3221(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _alertPin(nullptr)
{
}

INA3221::INA3221(inter_i2c_bus* bus, uint8_t addr, const Config& cfg)
    : _dev(bus, addr)
    , _cfg(cfg)
    , _alertPin(nullptr)
{
}

INA3221::~INA3221()
{
}

// ============================================================
//  连接 / 错误
// ============================================================

bool INA3221::isConnected()
{
    (void)readRaw(REG_MANUFACTURER_ID);
    return _dev.lastError() == 0;
}

int INA3221::getLastError()
{
    return _dev.lastError();
}

// ============================================================
//  init
// ============================================================

void INA3221::init()
{
    // 1. 配置寄存器 (0x00) — 通道使能 + 平均 + 转换时间 + 模式
    writeRaw(REG_CONFIG, buildConfigReg());

    // 2. Mask/Enable (0x0F) — CNVR + SCC + POL + LEN
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());

    // 3. 读取一次 Mask/Enable 清除可能残留的锁存标志
    (void)readRaw(REG_MASK_ENABLE);

    // 4. 自动配置报警引脚（Config 中指定了 GPIO 端口和引脚时）
    //    零堆分配：io_ctrl 为成员对象（非指针），移动赋值接管引脚所有权
    if (_cfg.alert_port != nullptr && _cfg.alert_pin != pin_none)
    {
        _alertPinAuto = io_ctrl(_cfg.alert_port, _cfg.alert_pin);  // 释放旧引脚 + 接管新引脚
        _alertPinAuto.init(mode_input, pullup);     // 时钟使能 + 上拉输入
    }

    // 注：报警阈值（临界/预警/求和/电源有效）不在此处配置，
    //     复位默认全 0（不触发），由 setCriticalAlert() 等方法设置
}

// ============================================================
//  I2C IO（INA3221 全部 16-bit 寄存器）
// ============================================================

void INA3221::writeRaw(uint8_t reg, uint16_t data)
{
    _dev.write_16bit(reg, data);
}

uint16_t INA3221::readRaw(uint8_t reg)
{
    return _dev.read_16bit(reg);
}

// ============================================================
//  寄存器构建
// ============================================================

uint16_t INA3221::buildConfigReg() const
{
    // Config Register (0x00):
    //   [15]    RST
    //   [14:12] 通道使能（bit14=CH1, bit13=CH2, bit12=CH3）
    //   [11:9]  AVG
    //   [8:6]   VBUSCT
    //   [5:3]   VSHCT
    //   [2:0]   MODE
    uint16_t r = 0;
    r |= static_cast<uint16_t>(_cfg.reset)         << 15;
    r |= (_cfg.ch1_enable ? 1u : 0u)               << 14;
    r |= (_cfg.ch2_enable ? 1u : 0u)               << 13;
    r |= (_cfg.ch3_enable ? 1u : 0u)               << 12;
    r |= static_cast<uint16_t>(_cfg.averaging)     << 9;
    r |= static_cast<uint16_t>(_cfg.busConvTime)   << 6;
    r |= static_cast<uint16_t>(_cfg.shuntConvTime) << 3;
    r |= static_cast<uint16_t>(_cfg.mode);
    return r;
}

uint16_t INA3221::buildMaskEnable() const
{
    // Mask/Enable Register (0x0F):
    //   [15]    CNVR — 转换完成报警使能
    //   [14:12] SCC  — 分流求和通道（bit14=CH1, bit13=CH2, bit12=CH3）
    //   [11]    POL  — 报警极性
    //   [10]    LEN  — 报警锁存
    //   [9:0]   状态标志（只读）
    uint16_t r = 0;
    r |= _cfg.alertMask & 0xF000;                 // bit15 CNVR + bit14-12 SCC
    r |= static_cast<uint16_t>(_cfg.apol)   << 11;
    r |= static_cast<uint16_t>(_cfg.alatch) << 10;
    return r;
}

// ============================================================
//  转换
// ============================================================

int32_t INA3221::rawToShuntVoltage_uV(uint16_t raw)
{
    // 13 位有符号左对齐（bit15-3），LSB = 40 µV
    // 先符号扩展到 16 位再 >>3（bit2-0 恒为 0，算术右移精确）
    int32_t val = static_cast<int32_t>(signExtend<16>(raw));
    return (val >> 3) * 40;
}

int32_t INA3221::rawToBusVoltage_mV(uint16_t raw)
{
    // 13 位无符号左对齐（bit15-3），LSB = 8 mV
    return (raw >> 3) * 8;
}

int32_t INA3221::rawToCurrent_mA(uint16_t rawShunt, uint32_t rShunt_uOhm)
{
    // I = Vshunt / R：µV / µΩ = A；×1000 → mA
    if (rShunt_uOhm == 0)
        return 0;
    int64_t shunt_uV = rawToShuntVoltage_uV(rawShunt);
    return static_cast<int32_t>((shunt_uV * 1000) / rShunt_uOhm);
}

int32_t INA3221::rawToPower_mW(uint16_t rawShunt, uint16_t rawBus, uint32_t rShunt_uOhm)
{
    // P = Vbus × I：mV × mA / 1000 → mW
    if (rShunt_uOhm == 0)
        return 0;
    int64_t shunt_uV = rawToShuntVoltage_uV(rawShunt);
    int64_t current_mA = (shunt_uV * 1000) / rShunt_uOhm;
    int64_t bus_mV = rawToBusVoltage_mV(rawBus);
    return static_cast<int32_t>((bus_mV * current_mA) / 1000);
}

// ============================================================
//  测量 API
// ============================================================

int32_t INA3221::getBusVoltage_mV(uint8_t channel)
{
    if (channel > 2) return 0;
    return rawToBusVoltage_mV(readRaw(busReg(channel)));
}

int32_t INA3221::getShuntVoltage_uV(uint8_t channel)
{
    if (channel > 2) return 0;
    return rawToShuntVoltage_uV(readRaw(shuntReg(channel)));
}

int32_t INA3221::getCurrent_mA(uint8_t channel)
{
    if (channel > 2) return 0;
    return rawToCurrent_mA(readRaw(shuntReg(channel)), _cfg.rShunt_uOhm[channel]);
}

int32_t INA3221::getPower_mW(uint8_t channel)
{
    if (channel > 2) return 0;
    return rawToPower_mW(readRaw(shuntReg(channel)),
                         readRaw(busReg(channel)),
                         _cfg.rShunt_uOhm[channel]);
}

int32_t INA3221::getShuntVoltageSum_uV()
{
    // 15 位有符号左对齐（bit15-1），LSB = 40 µV
    int32_t val = static_cast<int32_t>(signExtend<16>(readRaw(REG_SHUNT_VOLTAGE_SUM)));
    return (val >> 1) * 40;
}

// ============================================================
//  通道配置
// ============================================================

int INA3221::setShuntR(uint8_t channel, uint32_t rShunt_uOhm)
{
    if (channel > 2)
        return ERR_CHANNEL;

    if (rShunt_uOhm < 1000)                 // < 1 mΩ，换算溢出保护
        return ERR_SHUNT_LOW;

    _cfg.rShunt_uOhm[channel] = rShunt_uOhm;
    return ERR_NONE;
}

uint32_t INA3221::getShuntR(uint8_t channel)
{
    if (channel > 2) return 0;
    return _cfg.rShunt_uOhm[channel];
}

int INA3221::enableChannel(uint8_t channel)
{
    if (channel > 2)
        return ERR_CHANNEL;

    switch (channel)
    {
        case 0: _cfg.ch1_enable = true; break;
        case 1: _cfg.ch2_enable = true; break;
        default: _cfg.ch3_enable = true; break;
    }
    writeRaw(REG_CONFIG, buildConfigReg());
    return ERR_NONE;
}

int INA3221::disableChannel(uint8_t channel)
{
    if (channel > 2)
        return ERR_CHANNEL;

    switch (channel)
    {
        case 0: _cfg.ch1_enable = false; break;
        case 1: _cfg.ch2_enable = false; break;
        default: _cfg.ch3_enable = false; break;
    }
    writeRaw(REG_CONFIG, buildConfigReg());
    return ERR_NONE;
}

bool INA3221::isChannelEnabled(uint8_t channel)
{
    if (channel > 2) return false;
    switch (channel)
    {
        case 0: return _cfg.ch1_enable;
        case 1: return _cfg.ch2_enable;
        default: return _cfg.ch3_enable;
    }
}

// ============================================================
//  报警阈值
// ============================================================

int INA3221::setCriticalAlert(uint8_t channel, int32_t microVolt)
{
    if (channel > 2)
        return ERR_CHANNEL;

    // 满量程 ±163.8 mV（13 位有符号，正数最大值 4095 × 40 µV）
    if (microVolt < 0 || microVolt > 163800)
        return ERR_LIMIT_RANGE;

    uint16_t raw = static_cast<uint16_t>((microVolt / 40) << 3);
    writeRaw(criticalReg(channel), raw);
    return ERR_NONE;
}

int32_t INA3221::getCriticalAlert(uint8_t channel)
{
    if (channel > 2) return 0;
    return rawToShuntVoltage_uV(readRaw(criticalReg(channel)));
}

int INA3221::setWarningAlert(uint8_t channel, int32_t microVolt)
{
    if (channel > 2)
        return ERR_CHANNEL;

    if (microVolt < 0 || microVolt > 163800)
        return ERR_LIMIT_RANGE;

    uint16_t raw = static_cast<uint16_t>((microVolt / 40) << 3);
    writeRaw(warningReg(channel), raw);
    return ERR_NONE;
}

int32_t INA3221::getWarningAlert(uint8_t channel)
{
    if (channel > 2) return 0;
    return rawToShuntVoltage_uV(readRaw(warningReg(channel)));
}

int INA3221::setShuntVoltageSumLimit(int32_t microVolt)
{
    // 15 位有符号，满量程 ±16383 × 40 µV = ±655.32 mV
    if (microVolt > 655320 || microVolt < -655320)
        return ERR_LIMIT_RANGE;

    int32_t raw = (microVolt / 40) << 1;
    writeRaw(REG_SHUNT_VOLTAGE_LIMIT, static_cast<uint16_t>(raw & 0xFFFF));
    return ERR_NONE;
}

int32_t INA3221::getShuntVoltageSumLimit()
{
    int32_t val = static_cast<int32_t>(signExtend<16>(readRaw(REG_SHUNT_VOLTAGE_LIMIT)));
    return (val >> 1) * 40;
}

int INA3221::setPowerUpperLimit(int32_t milliVolt)
{
    // 13 位无符号，LSB 8 mV，上限 4095 × 8 = 32760 mV
    if (milliVolt < 0 || milliVolt > 32760)
        return ERR_LIMIT_RANGE;

    uint16_t raw = static_cast<uint16_t>((milliVolt / 8) << 3);
    writeRaw(REG_POWER_VALID_UPPER, raw);
    return ERR_NONE;
}

int32_t INA3221::getPowerUpperLimit()
{
    return (readRaw(REG_POWER_VALID_UPPER) >> 3) * 8;
}

int INA3221::setPowerLowerLimit(int32_t milliVolt)
{
    if (milliVolt < 0 || milliVolt > 32760)
        return ERR_LIMIT_RANGE;

    uint16_t raw = static_cast<uint16_t>((milliVolt / 8) << 3);
    writeRaw(REG_POWER_VALID_LOWER, raw);
    return ERR_NONE;
}

int32_t INA3221::getPowerLowerLimit()
{
    return (readRaw(REG_POWER_VALID_LOWER) >> 3) * 8;
}

// ============================================================
//  报警引脚 / 标志
// ============================================================

void INA3221::bindAlertPin(io_ctrl *pin)
{
    _alertPin = pin;
}

bool INA3221::isAlertAsserted()
{
    // 优先使用 bindAlertPin 绑定的外部引脚
    if (_alertPin)
        return _alertPin->read() == Low;

    // 否则使用 Config 自动配置的引脚（io_ctrl 成员，未配置时无效）
    if (_alertPinAuto.is_initialized())
        return _alertPinAuto.read() == Low;

    return false;
}

INA3221_AlertFlags INA3221::readAlertFlags()
{
    INA3221_AlertFlags f = {};
    uint16_t raw = readRaw(REG_MASK_ENABLE);

    f.conversionReady = (raw & 0x0001) != 0;   // bit 0: CVRF
    f.timingControl   = (raw & 0x0002) != 0;   // bit 1: TCF
    f.powerValid      = (raw & 0x0004) != 0;   // bit 2: PUF
    f.warning3        = (raw & 0x0008) != 0;   // bit 3: WGF3
    f.warning2        = (raw & 0x0010) != 0;   // bit 4: WGF2
    f.warning1        = (raw & 0x0020) != 0;   // bit 5: WGF1
    f.summation       = (raw & 0x0040) != 0;   // bit 6: SF
    f.critical3       = (raw & 0x0080) != 0;   // bit 7: CGF3
    f.critical2       = (raw & 0x0100) != 0;   // bit 8: CGF2
    f.critical1       = (raw & 0x0200) != 0;   // bit 9: CGF1

    return f;
}

void INA3221::clearAlertLatch()
{
    // 读取 Mask/Enable 清除锁存标志，重写配置恢复报警逻辑
    (void)readRaw(REG_MASK_ENABLE);
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
}

uint16_t INA3221::readMaskEnableRaw()
{
    return readRaw(REG_MASK_ENABLE);
}

// ============================================================
//  运行时配置
// ============================================================

void INA3221::setConfig(const Config& cfg)
{
    _cfg = cfg;

    writeRaw(REG_CONFIG,      buildConfigReg());
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
    (void)readRaw(REG_MASK_ENABLE);   // 清除锁存标志
}

void INA3221::setAveraging(AvgSample mode)
    { _cfg.averaging = mode; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA3221::setBusConvTime(ConvTime time)
    { _cfg.busConvTime = time; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA3221::setShuntConvTime(ConvTime time)
    { _cfg.shuntConvTime = time; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA3221::setOperatingMode(OperatingMode mode)
    { _cfg.mode = mode; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA3221::setAlertMask(uint16_t mask)
{
    _cfg.alertMask = mask & 0xF000;   // bit15 CNVR + bit14-12 SCC 可写
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
    (void)readRaw(REG_MASK_ENABLE);   // 清除可能残留的锁存标志
}

void INA3221::reset()
{
    // 写 CONFIG 的 RST 位触发软复位，寄存器恢复默认值
    writeRaw(REG_CONFIG, static_cast<uint16_t>(0x8000u | buildConfigReg()));
}

bool INA3221::isConversionReady()
{
    uint16_t raw = readRaw(REG_MASK_ENABLE);
    return (raw & 0x0001) != 0;       // bit0: CVRF
}

bool INA3221::waitConversionReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) <= timeout_ms)
    {
        if (isConversionReady()) return true;
        delay_ms(1);
    }
    return false;
}

// ============================================================
//  设备信息
// ============================================================

uint16_t INA3221::getManufacturerID()
    { return readRaw(REG_MANUFACTURER_ID); }

uint16_t INA3221::getDieID()
    { return readRaw(REG_DIE_ID); }

// ============================================================
//  调试
// ============================================================

void INA3221::captureSnapshot(INA3221_Snapshot& s)
{
    s.config             = readRaw(REG_CONFIG);
    s.shuntVoltage[0]    = readRaw(REG_SHUNT_VOLTAGE_1);
    s.busVoltage[0]      = readRaw(REG_BUS_VOLTAGE_1);
    s.shuntVoltage[1]    = readRaw(REG_SHUNT_VOLTAGE_2);
    s.busVoltage[1]      = readRaw(REG_BUS_VOLTAGE_2);
    s.shuntVoltage[2]    = readRaw(REG_SHUNT_VOLTAGE_3);
    s.busVoltage[2]      = readRaw(REG_BUS_VOLTAGE_3);
    s.criticalAlert[0]   = readRaw(REG_CRITICAL_ALERT_1);
    s.warningAlert[0]    = readRaw(REG_WARNING_ALERT_1);
    s.criticalAlert[1]   = readRaw(REG_CRITICAL_ALERT_2);
    s.warningAlert[1]    = readRaw(REG_WARNING_ALERT_2);
    s.criticalAlert[2]   = readRaw(REG_CRITICAL_ALERT_3);
    s.warningAlert[2]    = readRaw(REG_WARNING_ALERT_3);
    s.shuntVoltageSum    = readRaw(REG_SHUNT_VOLTAGE_SUM);
    s.shuntVoltageLimit  = readRaw(REG_SHUNT_VOLTAGE_LIMIT);
    s.maskEnable         = readRaw(REG_MASK_ENABLE);
    s.powerValidUpper    = readRaw(REG_POWER_VALID_UPPER);
    s.powerValidLower    = readRaw(REG_POWER_VALID_LOWER);
    s.manufacturerID     = readRaw(REG_MANUFACTURER_ID);
    s.dieID              = readRaw(REG_DIE_ID);
}

void INA3221::dumpRawRegisters(const INA3221_Snapshot& s)
{
    char buf[128];
    int  len;

    len = snprintf(buf, sizeof(buf),
                   "\r\n========== INA3221 RAW ==========\r\n");
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "CONFIG   : 0x%04X\r\n", s.config);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    for (int ch = 0; ch < 3; ch++)
    {
        len = snprintf(buf, sizeof(buf),
                       "CH%d VSHUNT : 0x%04X  VBUS : 0x%04X\r\n",
                       ch + 1, s.shuntVoltage[ch], s.busVoltage[ch]);
        (void)debug_uart.send_data((const uint8_t*)buf, len);
    }

    for (int ch = 0; ch < 3; ch++)
    {
        len = snprintf(buf, sizeof(buf),
                       "CH%d ALERT  : CRIT 0x%04X  WARN 0x%04X\r\n",
                       ch + 1, s.criticalAlert[ch], s.warningAlert[ch]);
        (void)debug_uart.send_data((const uint8_t*)buf, len);
    }

    len = snprintf(buf, sizeof(buf), "VSUM     : 0x%04X  LIMIT 0x%04X\r\n",
                   s.shuntVoltageSum, s.shuntVoltageLimit);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "MASK/EN  : 0x%04X\r\n", s.maskEnable);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "PWRVALID : UPPER 0x%04X  LOWER 0x%04X\r\n",
                   s.powerValidUpper, s.powerValidLower);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "MFG ID   : 0x%04X  DIE ID 0x%04X\r\n",
                   s.manufacturerID, s.dieID);
    (void)debug_uart.send_data((const uint8_t*)buf, len);
}

void INA3221::dumpEngineeringData(const INA3221_Snapshot& s)
{
    char buf[128];
    int  len;

    len = snprintf(buf, sizeof(buf),
                   "\r\n========== INA3221 ENGINEERING ==========\r\n");
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    for (int ch = 0; ch < 3; ch++)
    {
        int32_t shunt_uV   = rawToShuntVoltage_uV(s.shuntVoltage[ch]);
        int32_t bus_mV     = rawToBusVoltage_mV(s.busVoltage[ch]);
        int32_t current_mA = rawToCurrent_mA(s.shuntVoltage[ch], _cfg.rShunt_uOhm[ch]);
        int32_t power_mW   = rawToPower_mW(s.shuntVoltage[ch], s.busVoltage[ch],
                                           _cfg.rShunt_uOhm[ch]);

        len = snprintf(buf, sizeof(buf),
                       "CH%d Bus %ld.%03ld V | Shunt %ld.%03ld mV | "
                       "I %ld.%03ld A | P %ld.%03ld W\r\n",
                       ch + 1,
                       (long)bus_mV / 1000, (long)bus_mV % 1000,
                       (long)shunt_uV / 1000, (long)shunt_uV % 1000,
                       (long)current_mA / 1000, (long)current_mA % 1000,
                       (long)power_mW / 1000, (long)power_mW % 1000);
        (void)debug_uart.send_data((const uint8_t*)buf, len);
    }

    int32_t sum_uV = static_cast<int32_t>(signExtend<16>(s.shuntVoltageSum)) >> 1;
    len = snprintf(buf, sizeof(buf), "VSUM       : %ld.%03ld mV\r\n",
                   (long)(sum_uV * 40) / 1000, (long)(sum_uV * 40) % 1000);
    (void)debug_uart.send_data((const uint8_t*)buf, len);
}

void INA3221::dumpRegisters()
{
    INA3221_Snapshot snap;
    captureSnapshot(snap);
    dumpRawRegisters(snap);
    dumpEngineeringData(snap);
}
