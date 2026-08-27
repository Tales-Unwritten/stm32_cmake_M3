#include "device_ina228.hpp"
#include "device_serial.hpp"

#include <cstdint>
#include <cstdio>

// ============================================================
//  构造
// ============================================================

INA228::INA228(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _alertPin(nullptr)
    , _currentLSB_nA(0)
{
    _updateCurrentLSB();
}

INA228::INA228(inter_i2c_bus* bus, uint8_t addr, const Config& cfg)
    : _dev(bus, addr)
    , _cfg(cfg)
    , _alertPin(nullptr)
    , _currentLSB_nA(0)
{
    _updateCurrentLSB();
}

INA228::~INA228()
{
}

// ============================================================
//  Current LSB（nA 精度，消除整数截断误差）
//  Current_LSB = I_MAX / 2^19
//  currentLSB_nA = maxCurrent_mA × 1e6 / 524288
// ============================================================

void INA228::_updateCurrentLSB()
{
    _currentLSB_nA =
        (static_cast<int64_t>(_cfg.maxCurrent_mA) * 1000000) / 524288;
}

// ============================================================
//  校准（对齐 GitHub 版 setMaxCurrentShunt）
//
//  SHUNT_CAL = 13107.2e6 × Current_LSB(A) × R_shunt(Ω)
//            = maxCurrent_mA × r_shunt_uOhm / 40000
//  ADCRANGE = 1（40.96 mV 档）时 ×4
// ============================================================

int INA228::setMaxCurrentShunt(int32_t maxCurrent_mA, uint32_t rShunt_uOhm)
{
    if (maxCurrent_mA < 1)
        return ERR_MAXCURRENT_LOW;

    if (rShunt_uOhm == 0)
        return ERR_SHUNT_LOW;

    uint64_t shuntCal = (static_cast<uint64_t>(maxCurrent_mA) * rShunt_uOhm) / 40000;

    if (_cfg.adcrange == Adcrange::Range_40mv)
        shuntCal *= 4;

    if (shuntCal > 0xFFFF)
        return ERR_CAL_OVERFLOW;
    if (shuntCal == 0)
        return ERR_CAL_OVERFLOW;

    _cfg.maxCurrent_mA = maxCurrent_mA;
    _cfg.r_shunt_uOhm   = rShunt_uOhm;
    _cfg.shuntCal       = static_cast<uint16_t>(shuntCal);
    _updateCurrentLSB();

    writeRaw(REG_SHUNT_CAL, _cfg.shuntCal, 2);
    return ERR_NONE;
}

// ============================================================
//  连接 / 错误
// ============================================================

bool INA228::isConnected()
{
    (void)readRaw(REG_MANUFACTURER_ID);
    return _dev.lastError() == 0;
}

int INA228::getLastError()
{
    return _dev.lastError();
}

// ============================================================
//  init
// ============================================================

void INA228::init()
{
    // 0. 自动校准：由采样电阻与最大电流推导 SHUNT_CAL + LSB
    //    （默认 163840 mA / 1 mΩ → 0x1000，与原默认值一致）
    if (_cfg.r_shunt_uOhm != 0)
    {
        (void)setMaxCurrentShunt(_cfg.maxCurrent_mA, _cfg.r_shunt_uOhm);
    }

    // 1. 配置寄存器
    writeRaw(REG_CONFIG,     buildConfigReg(),    2);
    writeRaw(REG_ADC_CONFIG, buildADCConfigReg(), 2);

    // 2. 报警配置（ALATCH=1 使能锁存，此后状态位将被锁存）
    writeRaw(REG_DIAG_ALERT, buildDiagAlertCfg(), 2);

    // 3. 校准
    writeRaw(REG_SHUNT_CAL,  _cfg.shuntCal, 2);

    // 4. ⚠️ 上电后 MEMSTAT(bit0) 被芯片内部自检置 1，与寄存器写入顺序无关。
    //    INA228 开漏 ALERT 引脚在锁存模式下只要状态位非零就持续拉低，
    //    即使外部有上拉电阻也拉不回来。读取 DIAG_ALRT 是唯一清除手段。
    //    因此 init 末尾必须读一次，将上电锁存标志清掉以释放 ALERT 引脚。
    (void)readRaw(REG_DIAG_ALERT);

    // 5. 阈值默认不触发，用户调用 set_xxx_limit() 覆盖

    // 6. 自动配置报警引脚（Config 中指定了 GPIO 端口和引脚时）
    //    零堆分配：io_ctrl 为成员对象（非指针），移动赋值接管引脚所有权
    if (_cfg.alert_port != nullptr && _cfg.alert_pin != pin_none)
    {
        _alertPinAuto = io_ctrl(_cfg.alert_port, _cfg.alert_pin);  // 释放旧引脚 + 接管新引脚
        _alertPinAuto.init(mode_input, pullup);     // 时钟使能 + 上拉输入
    }
}

// ============================================================
//  寄存器长度
// ============================================================

uint8_t INA228::regLen(uint8_t reg)
{
    switch (reg)
    {
    case REG_VSHUNT:
    case REG_VBUS:
    case REG_CURRENT:
    case REG_POWER:
        return 3;

    case REG_ENERGY:
    case REG_CHARGE:
        return 5;

    default:
        return 2;
    }
}

// ============================================================
//  Raw I2C IO
// ============================================================

void INA228::writeRaw(uint8_t reg, uint64_t data, uint8_t len)
{
    _dev.freedom_write(reg, data, len);
}

uint64_t INA228::readRaw(uint8_t reg)
{
    uint64_t data = 0;
    _dev.freedom_read(reg, &data, regLen(reg));
    return data;
}

// ============================================================
//  寄存器构建
// ============================================================

uint16_t INA228::buildConfigReg() const
{
    uint16_t r = 0;
    r |= static_cast<uint16_t>(_cfg.reset)    << 15;
    r |= static_cast<uint16_t>(_cfg.retacc)   << 14;
    r |= static_cast<uint16_t>(_cfg.convdly)  << 6;
    r |= static_cast<uint16_t>(_cfg.tempcomp) << 5;
    r |= static_cast<uint16_t>(_cfg.adcrange) << 4;
    r |= static_cast<uint16_t>(_cfg.reserved) << 0;
    return r;
}

uint16_t INA228::buildADCConfigReg() const
{
    uint16_t r = 0;
    r |= static_cast<uint16_t>(_cfg.adcMode)       << 12;
    r |= static_cast<uint16_t>(_cfg.busConvTime)   << 9;
    r |= static_cast<uint16_t>(_cfg.shuntConvTime) << 6;
    r |= static_cast<uint16_t>(_cfg.tempConvTime)  << 3;
    r |= static_cast<uint16_t>(_cfg.averaging);
    return r;
}

uint16_t INA228::buildDiagAlertCfg() const
{
    // ⚠️ 仅写 bit 15-8（配置位）
    // bit 7-0 是只读状态位，写入 0 可清除锁存标志
    uint16_t r = 0;
    r |= static_cast<uint16_t>(_cfg.alatch)    << 15;
    r |= static_cast<uint16_t>(_cfg.cnvr)      << 14;
    r |= static_cast<uint16_t>(_cfg.slowalert) << 13;
    r |= static_cast<uint16_t>(_cfg.apol)      << 12;
    return r;
}

// ============================================================
//  原始值 → 物理量
// ============================================================

int32_t INA228::rawToBusVoltage_mV(uint64_t raw)
{
    // 195.3125 µV/LSB = 25/128 mV/LSB
    int32_t val = signExtend<20>(raw >> 4);
    return (val * 25) >> 7;
}

int32_t INA228::rawToShuntVoltage_uV(uint64_t raw) const
{
    // ADCRANGE=0（163.84 mV 档）: 312.5 nV/LSB = 5/16 µV/LSB
    // ADCRANGE=1（ 40.96 mV 档）:  78.125 nV/LSB = 5/64 µV/LSB
    int32_t val = signExtend<20>(raw >> 4);
    if (_cfg.adcrange == Adcrange::Range_40mv)
        return (val * 5) >> 6;
    return (val * 5) >> 4;
}

int32_t INA228::rawToCurrent_mA(uint64_t raw) const
{
    int32_t val = signExtend<20>(raw >> 4);
    return static_cast<int32_t>(
        (static_cast<int64_t>(val) * _currentLSB_nA) / 1000000);
}

int32_t INA228::rawToTemperature_mC(uint64_t raw)
{
    // 7.8125 m°C/LSB = 1000/128 m°C/LSB
    uint16_t val = static_cast<uint16_t>(raw);
    return (val * 1000) >> 7;
}

int32_t INA228::rawToPower_mW(uint64_t raw) const
{
    // Power(W) = 3.2 × CURRENT_LSB × reg
    // Power(mW) = reg × 3.2 × LSB_nA × 1e-3
    //           = reg × 16 × LSB_nA / 5e6
    int64_t val = static_cast<int64_t>(raw);
    return static_cast<int32_t>((val * 16 * _currentLSB_nA) / 5000000);
}

int64_t INA228::rawToEnergy_mJ(uint64_t raw) const
{
    // ENERGY 寄存器为 40-bit 无符号（数据手册），不做符号扩展
    // Energy(J) = 16 × 3.2 × CURRENT_LSB × reg
    // Energy(mJ) = reg × 32 × LSB_nA / 625000
    uint64_t val = raw;
    return static_cast<int64_t>(
        (val * 32 * static_cast<uint64_t>(_currentLSB_nA)) / 625000);
}

int64_t INA228::rawToCharge_mC(uint64_t raw) const
{
    // CHARGE 寄存器为 40-bit 有符号（数据手册）
    // Charge(C) = CURRENT_LSB × reg
    // Charge(mC) = reg × LSB_nA / 1e6
    int64_t val = signExtend<40>(raw);
    return (val * _currentLSB_nA) / 1000000;
}

// ============================================================
//  测量 API
// ============================================================

int32_t INA228::getBusVoltage_mV()
    { return rawToBusVoltage_mV(readRaw(REG_VBUS)); }

int32_t INA228::getShuntVoltage_uV()
    { return rawToShuntVoltage_uV(readRaw(REG_VSHUNT)); }

int32_t INA228::getCurrent_mA()
    { return rawToCurrent_mA(readRaw(REG_CURRENT)); }

int32_t INA228::getTemperature_mC()
    { return rawToTemperature_mC(readRaw(REG_DIETEMP)); }

int32_t INA228::getPower_mW()
    { return rawToPower_mW(readRaw(REG_POWER)); }

int64_t INA228::getEnergy_mJ()
    { return rawToEnergy_mJ(readRaw(REG_ENERGY)); }

int64_t INA228::getCharge_mC()
    { return rawToCharge_mC(readRaw(REG_CHARGE)); }

// ============================================================
//  阈值配置
// ============================================================

void INA228::set_sovl_limit_mA(int32_t sovl_mA)
{
    int64_t voltage_uV =
        (static_cast<int64_t>(sovl_mA) *
         static_cast<int64_t>(_cfg.r_shunt_uOhm)) / 1000;

    int32_t reg_value = 0;
    if (_cfg.adcrange == Adcrange::Range_163mv)
        reg_value = voltage_uV / 5;
    else
        reg_value = (voltage_uV * 4) / 5;

    if (reg_value > INT16_MAX)  reg_value = INT16_MAX;
    if (reg_value < INT16_MIN)  reg_value = INT16_MIN;

    writeRaw(REG_SOVL, static_cast<uint16_t>(reg_value), 2);
}

void INA228::set_suvl_limit_mA(int32_t suvl_mA)
{
    int64_t voltage_uV =
        (static_cast<int64_t>(suvl_mA) *
         static_cast<int64_t>(_cfg.r_shunt_uOhm)) / 1000;

    int32_t reg_value = 0;
    if (_cfg.adcrange == Adcrange::Range_163mv)
        reg_value = voltage_uV / 5;
    else
        reg_value = (voltage_uV * 4) / 5;

    if (reg_value > INT16_MAX)  reg_value = INT16_MAX;
    if (reg_value < INT16_MIN)  reg_value = INT16_MIN;

    writeRaw(REG_SUVL, static_cast<uint16_t>(reg_value), 2);
}

void INA228::set_bovl_limit_mV(uint16_t bovl_mV)
{
    // 3.125 mV/LSB → reg = mV × 8 / 25
    int32_t reg_value = (static_cast<int32_t>(bovl_mV) * 8) / 25;
    if (reg_value > 0x7FFF)  reg_value = 0x7FFF;
    if (reg_value < 0)       reg_value = 0;
    writeRaw(REG_BOVL, static_cast<uint16_t>(reg_value), 2);
}

void INA228::set_buvl_limit_mV(uint16_t buvl_mV)
{
    int32_t reg_value = (static_cast<int32_t>(buvl_mV) * 8) / 25;
    if (reg_value > 0x7FFF)  reg_value = 0x7FFF;
    if (reg_value < 0)       reg_value = 0;
    writeRaw(REG_BUVL, static_cast<uint16_t>(reg_value), 2);
}

void INA228::set_temp_limit_C(uint8_t temp_C)
{
    // 7.8125 m°C/LSB = 1/128 °C/LSB  →  reg = temp_C × 128
    uint16_t reg_value = static_cast<uint16_t>(temp_C) << 7;
    writeRaw(REG_TEMP_LIMIT, reg_value, 2);
}

void INA228::set_pwr_limit_mW(uint32_t pwr_mW)
{
    // PWR_LIMIT 阈值 LSB = 256 × Power_LSB（数据手册 7.6.1.17）
    // Power(mW) = reg × 16 × LSB_nA / 5e6 × 256
    // → reg = pwr_mW × 5e6 / (16 × 256 × LSB_nA)
    //       = pwr_mW × 5e6 / (4096 × LSB_nA)
    // 修复前缺少 ×256 因子，阈值被写小 256 倍，报警在设定功率的 1/256 就触发。
    if (_currentLSB_nA == 0) return;

    uint64_t num = static_cast<uint64_t>(pwr_mW) * 5000000;
    uint64_t den = static_cast<uint64_t>(_currentLSB_nA) * 4096;
    uint16_t reg_value = static_cast<uint16_t>(num / den);

    writeRaw(REG_PWR_LIMIT, reg_value, 2);
}

void INA228::setShuntTempco(uint16_t ppm)
{
    if (ppm > 16383) ppm = 16383;
    writeRaw(REG_SHUNT_TEMPCO, ppm, 2);
}

void INA228::reset()
{
    // 写 CONFIG 的 RST 位触发软复位，寄存器恢复默认值
    writeRaw(REG_CONFIG, static_cast<uint16_t>(0x8000u | buildConfigReg()), 2);
    _currentLSB_nA = 0;               // 复位后校准丢失
}

// ============================================================
//  报警引脚 / 标志
// ============================================================

void INA228::bindAlertPin(io_ctrl *pin)
{
    _alertPin = pin;
}

bool INA228::isAlertAsserted()
{
    // 优先使用 bindAlertPin 绑定的外部引脚
    if (_alertPin)
        return _alertPin->read() == Low;

    // 否则使用 Config 自动配置的引脚（io_ctrl 成员，未配置时无效）
    if (_alertPinAuto.is_initialized())
        return _alertPinAuto.read() == Low;

    return false;
}

INA228_AlertFlags INA228::readAlertFlags()
{
    INA228_AlertFlags f = {};
    uint16_t raw = static_cast<uint16_t>(readRaw(REG_DIAG_ALERT));

    f.memStatus       = (raw & 0x0001) != 0;
    f.conversionReady = (raw & 0x0002) != 0;
    f.powerOverLimit  = (raw & 0x0004) != 0;
    f.busUnderLimit   = (raw & 0x0008) != 0;
    f.busOverLimit    = (raw & 0x0010) != 0;
    f.shuntUnderLimit = (raw & 0x0020) != 0;
    f.shuntOverLimit  = (raw & 0x0040) != 0;
    f.tempOverLimit   = (raw & 0x0080) != 0;
    f.mathOverflow    = (raw & 0x0200) != 0;
    f.chargeOverflow  = (raw & 0x0400) != 0;
    f.energyOverflow  = (raw & 0x0800) != 0;

    return f;
}

void INA228::clearAlertLatch()
{
    // 连续读取两次：第一次清除锁存标志，第二次确认清除
    (void)readRaw(REG_DIAG_ALERT);
    uint16_t remain = static_cast<uint16_t>(readRaw(REG_DIAG_ALERT));

    // 重新写入配置位，确保报警逻辑状态正确
    writeRaw(REG_DIAG_ALERT, buildDiagAlertCfg(), 2);

    // 如果仍有状态位残留，说明存在持续触发的条件
    // （非锁存模式或硬件故障），此时 ALERT 会一直拉低
    if (remain & 0x00FF)  // bit 7-0 有残留
    {
        // 可在此处设置断点排查具体残留位
        (void)remain;
    }
}

/**
 * @brief 读取原始 DIAG_ALRT 寄存器值（调试用）
 */
uint16_t INA228::readDiagAlertRaw()
{
    return static_cast<uint16_t>(readRaw(REG_DIAG_ALERT));
}

// ============================================================
//  设备信息
// ============================================================

uint16_t INA228::getManufacturerID()
{
    return static_cast<uint16_t>(readRaw(REG_MANUFACTURER_ID));
}

uint16_t INA228::getDeviceID()
{
    return static_cast<uint16_t>(readRaw(REG_DEVICE_ID));
}

// ============================================================
//  调试：Snapshot
// ============================================================

void INA228::captureSnapshot(INA228_Snapshot& s)
{
    s.config        = readRaw(REG_CONFIG);
    s.adcConfig     = readRaw(REG_ADC_CONFIG);
    s.shuntCal      = readRaw(REG_SHUNT_CAL);
    s.diagAlert     = readRaw(REG_DIAG_ALERT);
    s.vshunt        = readRaw(REG_VSHUNT);
    s.vbus          = readRaw(REG_VBUS);
    s.temp          = readRaw(REG_DIETEMP);
    s.current       = readRaw(REG_CURRENT);
    s.power         = readRaw(REG_POWER);
    s.energy        = readRaw(REG_ENERGY);
    s.charge        = readRaw(REG_CHARGE);
    s.sovl          = static_cast<uint16_t>(readRaw(REG_SOVL));
    s.suvl          = static_cast<uint16_t>(readRaw(REG_SUVL));
    s.bovl          = static_cast<uint16_t>(readRaw(REG_BOVL));
    s.buvl          = static_cast<uint16_t>(readRaw(REG_BUVL));
    s.temp_limit    = static_cast<uint16_t>(readRaw(REG_TEMP_LIMIT));
    s.pwr_limit     = static_cast<uint16_t>(readRaw(REG_PWR_LIMIT));
    s.manufacturerID = static_cast<uint16_t>(readRaw(REG_MANUFACTURER_ID));
    s.deviceID       = static_cast<uint16_t>(readRaw(REG_DEVICE_ID));
}

void INA228::dumpRawRegisters(const INA228_Snapshot& s)
{
    char buf[128];
    int  len;

    len = snprintf(buf, sizeof(buf),
                   "\r\n========== INA228 RAW ==========\r\n");
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "CONFIG         : 0x%04X\r\n", s.config);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "ADC_CONFIG     : 0x%04X\r\n", s.adcConfig);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "SHUNT_CAL      : 0x%04X\r\n", s.shuntCal);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "DIAG_ALERT     : 0x%04X\r\n", s.diagAlert);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "DIETEMP        : 0x%04X\r\n", s.temp);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "SOVL           : 0x%04X\r\n", s.sovl);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "SUVL           : 0x%04X\r\n", s.suvl);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "BOVL           : 0x%04X\r\n", s.bovl);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "BUVL           : 0x%04X\r\n", s.buvl);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "TEMP_LIMIT     : 0x%04X\r\n", s.temp_limit);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "PWR_LIMIT      : 0x%04X\r\n", s.pwr_limit);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "VSHUNT         : 0x%06lX\r\n",
                   (unsigned long)(s.vshunt & 0xFFFFFF));
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "VBUS           : 0x%06lX\r\n",
                   (unsigned long)(s.vbus & 0xFFFFFF));
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "CURRENT        : 0x%06lX\r\n",
                   (unsigned long)(s.current & 0xFFFFFF));
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "POWER          : 0x%06lX\r\n",
                   (unsigned long)(s.power & 0xFFFFFF));
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "ENERGY         : 0x%02lX%08lX\r\n",
                   (unsigned long)((s.energy >> 32) & 0xFF),
                   (unsigned long)(s.energy & 0xFFFFFFFF));
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "CHARGE         : 0x%02lX%08lX\r\n",
                   (unsigned long)((s.charge >> 32) & 0xFF),
                   (unsigned long)(s.charge & 0xFFFFFFFF));
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "MANUFACTURER_ID: 0x%04X\r\n", s.manufacturerID);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "DEVICE_ID      : 0x%04X\r\n", s.deviceID);
    (void)debug_uart.send_data((const uint8_t*)buf, len);
}

void INA228::dumpEngineeringData(const INA228_Snapshot& s)
{
    char buf[128];
    int  len;

    int32_t vshunt_uV = rawToShuntVoltage_uV(s.vshunt);
    int32_t vbus_mV   = rawToBusVoltage_mV(s.vbus);
    int32_t temp_mC   = rawToTemperature_mC(s.temp);
    int32_t cur_mA    = rawToCurrent_mA(s.current);
    int32_t pwr_mW    = rawToPower_mW(s.power);
    int64_t eng_mJ    = rawToEnergy_mJ(s.energy);
    int64_t chg_mC    = rawToCharge_mC(s.charge);

    len = snprintf(buf, sizeof(buf),
                   "\r\n========== INA228 ENGINEERING ==========\r\n");
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Bus Voltage    : %ld.%03ld V\r\n",
                   (long)vbus_mV / 1000, (long)vbus_mV % 1000);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Shunt Voltage  : %ld uV\r\n", (long)vshunt_uV);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Temperature    : %ld.%03ld C\r\n",
                   (long)temp_mC / 1000, (long)temp_mC % 1000);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Current        : %ld.%03ld A\r\n",
                   (long)cur_mA / 1000, (long)cur_mA % 1000);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Power          : %ld.%03ld W\r\n",
                   (long)pwr_mW / 1000, (long)pwr_mW % 1000);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Energy         : %ld.%03ld J\r\n",
                   (long)(eng_mJ / 1000),
                   (long)((eng_mJ % 1000) < 0 ? -(eng_mJ % 1000) : (eng_mJ % 1000)));
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Charge         : %ld.%03ld C\r\n",
                   (long)(chg_mC / 1000),
                   (long)((chg_mC % 1000) < 0 ? -(chg_mC % 1000) : (chg_mC % 1000)));
    (void)debug_uart.send_data((const uint8_t*)buf, len);
}

void INA228::dumpRegisters()
{
    INA228_Snapshot snap;
    captureSnapshot(snap);
    dumpRawRegisters(snap);
    dumpEngineeringData(snap);
}
