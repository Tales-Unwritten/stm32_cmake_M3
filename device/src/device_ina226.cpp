
#include "device_ina226.hpp"
#include "device_serial.hpp"

// #include <cstdio>
#include "inter_io_ctrl.hpp"
#include "stdio.h"

// ============================================================
//  构造
// ===========================================================

INA226::INA226(inter_i2c_bus *bus, uint8_t addr) : _dev(bus, addr), _alertPin(nullptr), _currentLSB_nA(0)
{
    _updateCurrentLSB();
}

INA226::INA226(inter_i2c_bus *bus, uint8_t addr, const Config &cfg)
    : _dev(bus, addr), _cfg(cfg), _alertPin(nullptr), _currentLSB_nA(0)
{
    _updateCurrentLSB();
}

INA226::~INA226()
{
}

// ============================================================
//  Current LSB（nA 精度，消除整数截断误差）
//  Current_LSB = I_MAX / 2^15
//  currentLSB_nA = maxCurrent_mA × 1e6 / 32768
// ============================================================

void INA226::_updateCurrentLSB()
{
    _currentLSB_nA = (static_cast<int64_t>(_cfg.maxCurrent_mA) * 1000000) / 32768;
}

// ============================================================
//  校准（对齐 GitHub 版 setMaxCurrentShunt）
//
//  CAL = 0.00512 / (Current_LSB × R_shunt)
//      = 5.12e12 / (LSB_nA × R_uOhm)
//      （LSB_nA 单位 1e-9 A，R_uOhm 单位 1e-6 Ω，乘积单位 1e-15
//        0.00512 / 1e-15 = 5.12e12；注意不是 5.12e9）
//
//  取整后的 CAL 反推实际 LSB，保证读数与芯片内部计算严格自洽：
//    CURRENT(raw) = VSHUNT(raw) × CAL / 2048
// ============================================================

int INA226::setMaxCurrentShunt(int32_t maxCurrent_mA, uint32_t rShunt_uOhm)
{
    if (maxCurrent_mA < 1)
        return ERR_MAXCURRENT_LOW;

    if (rShunt_uOhm < 1000) // < 1 mΩ
        return ERR_SHUNT_LOW;

    // 分流满量程限制：maxCurrent × R ≤ 81.92 mV
    // （-0.02 mV 裕量防数学溢出，与 GitHub 版一致）
    // 单位推导：mA × µΩ = 1e-3 A × 1e-6 Ω = 1 nV
    // 81.92 mV = 81.92e6 nV
    if (static_cast<int64_t>(maxCurrent_mA) * rShunt_uOhm > 81900000)
        return ERR_SHUNTVOLTAGE_HIGH;

    // 名义 LSB（nA），CAL 取整后以实际 LSB 为准
    int64_t lsb_nA = (static_cast<int64_t>(maxCurrent_mA) * 1000000) / 32768;

    uint64_t cal = 5120000000000ULL / (static_cast<uint64_t>(lsb_nA) * rShunt_uOhm);

    // CAL 超出 15 位时 LSB 加倍、CAL 减半（同 GitHub 版 auto-scale）
    while (cal > 0x7FFF)
    {
        lsb_nA *= 2;
        cal >>= 1;
    }

    if (cal == 0)
        return ERR_CAL_OVERFLOW;

    // 用取整后的 CAL 反推实际 LSB，读数与芯片行为一致
    int64_t lsb_actual = 5120000000000LL / (static_cast<int64_t>(cal) * rShunt_uOhm);
    if (lsb_actual <= 0)
        return ERR_CAL_OVERFLOW;

    _cfg.maxCurrent_mA = maxCurrent_mA;
    _cfg.rShunt_uOhm = rShunt_uOhm;
    _cfg.calibration = static_cast<uint16_t>(cal);
    _currentLSB_nA = lsb_actual;

    writeRaw(REG_CALIBRATION, _cfg.calibration);
    return ERR_NONE;
}

// ============================================================
//  连接 / 错误
// ============================================================

bool INA226::isConnected()
{
    (void)readRaw(REG_MANUFACTURER_ID);
    return _dev.lastError() == 0;
}

int INA226::getLastError()
{
    return _dev.lastError();
}

// ============================================================
//  init
// ============================================================

void INA226::init()
{
    // 0. 自动校准模式：由采样电阻与最大电流推导 CAL + LSB
    if (_cfg.rShunt_uOhm != 0)
    {
        (void)setMaxCurrentShunt(_cfg.maxCurrent_mA, _cfg.rShunt_uOhm);
    }

    // 1. 配置寄存器 (0x00)
    writeRaw(REG_CONFIG, buildConfigReg());

    // 2. 校准寄存器 (0x05)
    writeRaw(REG_CALIBRATION, _cfg.calibration);

    // 3. Mask/Enable (0x06) — 写配置位 + 报警功能使能位
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());

    // 4. Alert Limit (0x07)
    writeRaw(REG_ALERT_LIMIT, _cfg.alertLimit);

    // 5. ⚠️ 上电后 AFF 等状态位可能已置位，与寄存器写入顺序无关。
    //    INA226 开漏 ALERT 引脚在锁存模式下只要状态位非零就持续拉低。
    //    读取 Mask/Enable 是唯一清除锁存标志的手段。
    (void)readRaw(REG_MASK_ENABLE);

    // 6. 自动配置报警引脚（Config 中指定了 GPIO 端口和引脚时）
    //    零堆分配：io_ctrl 为成员对象（非指针），移动赋值接管引脚所有权
    if (_cfg.alert_port != nullptr && _cfg.alert_pin != pin_none)
    {
        _alertPinAuto = io_ctrl(_cfg.alert_port, _cfg.alert_pin); // 释放旧引脚 + 接管新引脚
        _alertPinAuto.init(mode_input, pullup);    // 时钟使能 + 上拉输入
    }
}

// ============================================================
//  I2C IO（INA226 全部 16-bit 寄存器）
// ============================================================

void INA226::writeRaw(uint8_t reg, uint16_t data)
{
    _dev.write_16bit(reg, data);
}

uint16_t INA226::readRaw(uint8_t reg)
{
    return _dev.read_16bit(reg);
}

// ============================================================
//  寄存器构建
// ============================================================

uint16_t INA226::buildConfigReg() const
{
    // Config Register (0x00):
    //   [15]    RST
    //   [14:12] 保留 = 0b100
    //   [11:9]  AVG
    //   [8:6]   VBUSCT
    //   [5:3]   VSHCT
    //   [2:0]   MODE
    uint16_t r = 0;
    r |= static_cast<uint16_t>(_cfg.reset) << 15;
    r |= 0x04u << 12; // 保留位 0b100
    r |= static_cast<uint16_t>(_cfg.averaging) << 9;
    r |= static_cast<uint16_t>(_cfg.busConvTime) << 6;
    r |= static_cast<uint16_t>(_cfg.shuntConvTime) << 3;
    r |= static_cast<uint16_t>(_cfg.mode);
    return r;
}

uint16_t INA226::buildMaskEnable() const
{
    // Mask/Enable Register (0x06):
    //   [15:10] 报警功能使能（SOL/SUL/BOL/BUL/POL/CNVR，写入有效）
    //   [9:5]   保留 / 状态（读时有效）
    //   [4:2]   状态标志（只读）
    //   [1]    APOL  — 报警极性
    //   [0]    ALATCH — 报警锁存
    uint16_t r = 0;
    r |= _cfg.alertMask & 0xFC00; // bit15-10 报警使能
    r |= static_cast<uint16_t>(_cfg.apol) << 1;
    r |= static_cast<uint16_t>(_cfg.alatch) << 0;
    return r;
}

// ============================================================
//  转换
// ============================================================

int32_t INA226::rawToShuntVoltage_uV(uint16_t raw)
{
    // 2.5 µV/LSB = 5/2 µV/LSB
    int32_t val = static_cast<int32_t>(signExtend<16>(raw));
    return (val * 5) / 2;
}

int32_t INA226::rawToBusVoltage_mV(uint16_t raw)
{
    // 1.25 mV/LSB = 5/4 mV/LSB
    int32_t val = static_cast<int32_t>(raw);
    return (val * 5) / 4;
}

int32_t INA226::rawToCurrent_mA(uint16_t raw) const
{
    int64_t val = signExtend<16>(raw);
    return static_cast<int32_t>((val * _currentLSB_nA) / 1000000);
}

int32_t INA226::rawToPower_mW(uint16_t raw) const
{
    // Power_LSB = 25 × Current_LSB
    // Power(mW) = raw × 25 × LSB_nA / 1e6
    int64_t val = static_cast<int64_t>(raw);
    return static_cast<int32_t>((val * 25 * _currentLSB_nA) / 1000000);
}

// ============================================================
//  测量 API
// ============================================================

int32_t INA226::getBusVoltage_mV()
{
    return rawToBusVoltage_mV(readRaw(REG_BUS_VOLTAGE));
}

int32_t INA226::getShuntVoltage_uV()
{
    return rawToShuntVoltage_uV(readRaw(REG_SHUNT_VOLTAGE));
}

int32_t INA226::getCurrent_mA()
{
    return rawToCurrent_mA(readRaw(REG_CURRENT));
}

int32_t INA226::getPower_mW()
{
    return rawToPower_mW(readRaw(REG_POWER));
}

// ============================================================
//  报警
// ============================================================

void INA226::bindAlertPin(io_ctrl *pin)
{
    _alertPin = pin;
}

bool INA226::isAlertAsserted()
{
    // 优先使用 bindAlertPin 绑定的外部引脚
    if (_alertPin)
        return _alertPin->read() == Low;

    // 否则使用 Config 自动配置的引脚（io_ctrl 成员，未配置时无效）
    if (_alertPinAuto.is_initialized())
        return _alertPinAuto.read() == Low;

    return false;
}

INA226_AlertFlags INA226::readAlertFlags()
{
    INA226_AlertFlags f = {};
    uint16_t raw = readRaw(REG_MASK_ENABLE);

    f.mathOverflow = (raw & 0x0004) != 0;    // bit 2: OVF
    f.conversionReady = (raw & 0x0008) != 0; // bit 3: CVRF
    f.alertFunction = (raw & 0x0010) != 0;   // bit 4: AFF

    return f;
}

void INA226::clearAlertLatch()
{
    // 读取 Mask/Enable 清除锁存标志，重写配置恢复报警逻辑
    (void)readRaw(REG_MASK_ENABLE);
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
}

uint16_t INA226::readMaskEnableRaw()
{
    return readRaw(REG_MASK_ENABLE);
}

// ============================================================
//  运行时配置
// ============================================================

void INA226::setConfig(const Config &cfg)
{
    _cfg = cfg;
    _updateCurrentLSB();

    // 自动校准模式：由采样电阻与最大电流推导 CAL + LSB
    if (_cfg.rShunt_uOhm != 0)
    {
        (void)setMaxCurrentShunt(_cfg.maxCurrent_mA, _cfg.rShunt_uOhm);
    }

    writeRaw(REG_CONFIG, buildConfigReg());
    writeRaw(REG_CALIBRATION, _cfg.calibration);
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
    writeRaw(REG_ALERT_LIMIT, _cfg.alertLimit);
    (void)readRaw(REG_MASK_ENABLE); // 清除锁存标志
}

void INA226::setAveraging(AvgSample mode)
{
    _cfg.averaging = mode;
    writeRaw(REG_CONFIG, buildConfigReg());
}

void INA226::setBusConvTime(ConvTime time)
{
    _cfg.busConvTime = time;
    writeRaw(REG_CONFIG, buildConfigReg());
}

void INA226::setShuntConvTime(ConvTime time)
{
    _cfg.shuntConvTime = time;
    writeRaw(REG_CONFIG, buildConfigReg());
}

void INA226::setOperatingMode(OperatingMode mode)
{
    _cfg.mode = mode;
    writeRaw(REG_CONFIG, buildConfigReg());
}

void INA226::setCalibration(uint16_t cal)
{
    _cfg.calibration = cal;
    writeRaw(REG_CALIBRATION, cal);
}

void INA226::setAlertLimit(uint16_t limit)
{
    _cfg.alertLimit = limit;
    writeRaw(REG_ALERT_LIMIT, limit);
}

void INA226::setAlertMask(uint16_t mask)
{
    _cfg.alertMask = mask & 0xFC00; // 仅 bit15-10 可写
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
    (void)readRaw(REG_MASK_ENABLE); // 清除可能残留的锁存标志
}

void INA226::reset()
{
    // 写 CONFIG 的 RST 位触发软复位，寄存器恢复默认值
    writeRaw(REG_CONFIG, static_cast<uint16_t>(0x8000u | buildConfigReg()));
    _currentLSB_nA = 0; // 复位后校准丢失
}

bool INA226::isConversionReady()
{
    uint16_t raw = readRaw(REG_MASK_ENABLE);
    return (raw & 0x0008) != 0; // bit3: CVRF
}

bool INA226::waitConversionReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) <= timeout_ms)
    {
        if (isConversionReady())
            return true;
        delay_ms(1);
    }
    return false;
}

// ============================================================
//  设备信息
// ============================================================

uint16_t INA226::getManufacturerID()
{
    return readRaw(REG_MANUFACTURER_ID);
}

uint16_t INA226::getDieID()
{
    return readRaw(REG_DIE_ID);
}

// ============================================================
//  调试
// ============================================================

void INA226::captureSnapshot(INA226_Snapshot &s)
{
    s.config = readRaw(REG_CONFIG);
    s.shuntVoltage = readRaw(REG_SHUNT_VOLTAGE);
    s.busVoltage = readRaw(REG_BUS_VOLTAGE);
    s.power = readRaw(REG_POWER);
    s.current = readRaw(REG_CURRENT);
    s.calibration = readRaw(REG_CALIBRATION);
    s.maskEnable = readRaw(REG_MASK_ENABLE);
    s.alertLimit = readRaw(REG_ALERT_LIMIT);
    s.manufacturerID = readRaw(REG_MANUFACTURER_ID);
    s.dieID = readRaw(REG_DIE_ID);
}

void INA226::dumpRawRegisters(const INA226_Snapshot &s)
{
    char buf[128];
    int len;

    len = snprintf(buf, sizeof(buf), "\r\n========== INA226 RAW ==========\r\n");
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "CONFIG   : 0x%04X\r\n", s.config);
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "VSHUNT   : 0x%04X\r\n", s.shuntVoltage);
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "VBUS     : 0x%04X\r\n", s.busVoltage);
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "CURRENT  : 0x%04X\r\n", s.current);
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "POWER    : 0x%04X\r\n", s.power);
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "CAL      : 0x%04X\r\n", s.calibration);
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "MASK/EN  : 0x%04X\r\n", s.maskEnable);
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "ALERTLIM : 0x%04X\r\n", s.alertLimit);
    (void)debug_uart.send_data((const uint8_t *)buf, len);
}

void INA226::dumpEngineeringData(const INA226_Snapshot &s)
{
    char buf[128];
    int len;

    int32_t shunt_uV = rawToShuntVoltage_uV(s.shuntVoltage);
    int32_t bus_mV = rawToBusVoltage_mV(s.busVoltage);
    int32_t current_mA = rawToCurrent_mA(s.current);
    int32_t power_mW = rawToPower_mW(s.power);

    len = snprintf(buf, sizeof(buf), "\r\n========== INA226 ENGINEERING ==========\r\n");
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "Bus Voltage    : %ld.%03ld V\r\n", (long)bus_mV / 1000, (long)bus_mV % 1000);
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "Shunt Voltage  : %ld.%03ld mV\r\n", (long)shunt_uV / 1000, (long)shunt_uV % 1000);
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "Current        : %ld.%03ld A\r\n", (long)current_mA / 1000,
                   (long)current_mA % 1000);
    (void)debug_uart.send_data((const uint8_t *)buf, len);

    len = snprintf(buf, sizeof(buf), "Power          : %ld.%03ld W\r\n", (long)power_mW / 1000, (long)power_mW % 1000);
    (void)debug_uart.send_data((const uint8_t *)buf, len);
}

void INA226::dumpRegisters()
{
    INA226_Snapshot snap;
    captureSnapshot(snap);
    dumpRawRegisters(snap);
    dumpEngineeringData(snap);
}
