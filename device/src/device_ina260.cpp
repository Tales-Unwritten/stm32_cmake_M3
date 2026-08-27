#include "device_ina260.hpp"
#include "device_serial.hpp"

#include <cstdio>

// ============================================================
//  INA260 电流监测驱动（STM32G070 移植版）
//  来源库: Rob Tillaart INA260 Arduino Library v0.1.2 (2025-02-18)
//     URL: https://github.com/RobTillaart/INA260
//  移植说明:
//    - 无浮点，物理量定点整数；LSB 全部固定（出厂校准）：
//      电流 1.25 mA、总线 1.25 mV、功率 10 mW、分流 2 mΩ
//    - 无 CAL 寄存器、无分流电压寄存器
// ============================================================

// ============================================================
//  构造
// ============================================================

INA260::INA260(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _alertPin(nullptr)
{
}

INA260::INA260(inter_i2c_bus* bus, uint8_t addr, const Config& cfg)
    : _dev(bus, addr)
    , _cfg(cfg)
    , _alertPin(nullptr)
{
}

INA260::~INA260()
{
}

// ============================================================
//  校准
//  INA260 为出厂校准芯片：内置 2 mΩ 分流电阻，无 CAL 寄存器。
//  本方法仅校验参数并记录最大电流，不写任何寄存器。
// ============================================================

int INA260::setMaxCurrentShunt(int32_t maxCurrent_mA, uint32_t rShunt_uOhm)
{
    if (maxCurrent_mA < 1)
        return ERR_MAXCURRENT_LOW;

    if (maxCurrent_mA > 15000)              // 芯片额定连续电流 15 A
        return ERR_MAXCURRENT_HIGH;

    if (rShunt_uOhm != 2000)                // 内置 2 mΩ，不可配置
        return ERR_SHUNT_LOW;

    _maxCurrent_mA = maxCurrent_mA;
    return ERR_NONE;
}

// ============================================================
//  连接 / 错误
// ============================================================

bool INA260::isConnected()
{
    (void)readRaw(REG_MANUFACTURER_ID);
    return _dev.lastError() == 0;
}

int INA260::getLastError()
{
    return _dev.lastError();
}

// ============================================================
//  init
// ============================================================

void INA260::init()
{
    // 1. 配置寄存器 (0x00)
    writeRaw(REG_CONFIG,      buildConfigReg());

    // 2. Mask/Enable (0x06) — 写配置位 + 报警功能使能位
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());

    // 3. Alert Limit (0x07)
    writeRaw(REG_ALERT_LIMIT, _cfg.alertLimit);

    // 4. ⚠️ 上电后 AFF 等状态位可能已置位，与寄存器写入顺序无关。
    //    读取 Mask/Enable 是唯一清除锁存标志的手段。
    (void)readRaw(REG_MASK_ENABLE);

    // 5. 自动配置报警引脚（Config 中指定了 GPIO 端口和引脚时）
    //    零堆分配：io_ctrl 为成员对象（非指针），移动赋值接管引脚所有权
    if (_cfg.alert_port != nullptr && _cfg.alert_pin != pin_none)
    {
        _alertPinAuto = io_ctrl(_cfg.alert_port, _cfg.alert_pin);  // 释放旧引脚 + 接管新引脚
        _alertPinAuto.init(mode_input, pullup);     // 时钟使能 + 上拉输入
    }
}

// ============================================================
//  I2C IO（INA260 全部 16-bit 寄存器）
// ============================================================

void INA260::writeRaw(uint8_t reg, uint16_t data)
{
    _dev.write_16bit(reg, data);
}

uint16_t INA260::readRaw(uint8_t reg)
{
    return _dev.read_16bit(reg);
}

// ============================================================
//  寄存器构建
// ============================================================

uint16_t INA260::buildConfigReg() const
{
    // Config Register (0x00) — 与 INA226 同布局：
    //   [15]    RST
    //   [14:12] 保留 = 0
    //   [11:9]  AVG
    //   [8:6]   VBUSCT
    //   [5:3]   ISHCT（分流/电流转换时间）
    //   [2:0]   MODE
    uint16_t r = 0;
    r |= static_cast<uint16_t>(_cfg.reset)         << 15;
    r |= static_cast<uint16_t>(_cfg.averaging)     << 9;
    r |= static_cast<uint16_t>(_cfg.busConvTime)   << 6;
    r |= static_cast<uint16_t>(_cfg.shuntConvTime) << 3;
    r |= static_cast<uint16_t>(_cfg.mode);
    return r;
}

uint16_t INA260::buildMaskEnable() const
{
    // Mask/Enable Register (0x06) — 与 INA226 同布局：
    //   [15:10] 报警功能使能（SOC/SUC/BOL/BUL/POL/CNVR，写入有效）
    //   [9:5]   保留 / 状态（读时有效）
    //   [4:2]   状态标志（只读）
    //   [1]    APOL  — 报警极性
    //   [0]    ALATCH — 报警锁存
    uint16_t r = 0;
    r |= _cfg.alertMask & 0xFC00;                 // bit15-10 报警使能
    r |= static_cast<uint16_t>(_cfg.apol)   << 1;
    r |= static_cast<uint16_t>(_cfg.alatch) << 0;
    return r;
}

// ============================================================
//  转换（LSB 全部固定）
// ============================================================

int32_t INA260::rawToBusVoltage_mV(uint16_t raw)
{
    // 1.25 mV/LSB = 5/4 mV/LSB
    int32_t val = static_cast<int32_t>(raw);
    return (val * 5) / 4;
}

int32_t INA260::rawToShuntVoltage_uV(uint16_t raw)
{
    // INA260 无分流电压寄存器：Vshunt = I × 2 mΩ
    // = raw × 1.25 mA × 2 mΩ = raw × 2.5 µV = raw × 5/2 µV/LSB
    int32_t val = static_cast<int32_t>(signExtend<16>(raw));
    return (val * 5) / 2;
}

int32_t INA260::rawToCurrent_mA(uint16_t raw)
{
    // 固定 1.25 mA/LSB = 5/4 mA/LSB（出厂校准，与 maxCurrent 无关）
    int32_t val = static_cast<int32_t>(signExtend<16>(raw));
    return (val * 5) / 4;
}

int32_t INA260::rawToPower_mW(uint16_t raw)
{
    // 固定 10 mW/LSB（数据手册/参考库/Adafruit 一致）
    // 注意：不是 25 × Current_LSB（25 × 1.25 mA = 31.25 mW，错误）
    int32_t val = static_cast<int32_t>(raw);
    return val * 10;
}

// ============================================================
//  测量 API
// ============================================================

int32_t INA260::getBusVoltage_mV()
    { return rawToBusVoltage_mV(readRaw(REG_BUS_VOLTAGE)); }

int32_t INA260::getShuntVoltage_uV()
    { return rawToShuntVoltage_uV(readRaw(REG_CURRENT)); }

int32_t INA260::getCurrent_mA()
    { return rawToCurrent_mA(readRaw(REG_CURRENT)); }

int32_t INA260::getPower_mW()
    { return rawToPower_mW(readRaw(REG_POWER)); }

// ============================================================
//  报警
// ============================================================

void INA260::bindAlertPin(io_ctrl *pin)
{
    _alertPin = pin;
}

bool INA260::isAlertAsserted()
{
    // 优先使用 bindAlertPin 绑定的外部引脚
    if (_alertPin)
        return _alertPin->read() == Low;

    // 否则使用 Config 自动配置的引脚（io_ctrl 成员，未配置时无效）
    if (_alertPinAuto.is_initialized())
        return _alertPinAuto.read() == Low;

    return false;
}

INA260_AlertFlags INA260::readAlertFlags()
{
    INA260_AlertFlags f = {};
    uint16_t raw = readRaw(REG_MASK_ENABLE);

    f.mathOverflow    = (raw & 0x0004) != 0;  // bit 2: OVF
    f.conversionReady = (raw & 0x0008) != 0;  // bit 3: CVRF
    f.alertFunction   = (raw & 0x0010) != 0;  // bit 4: AFF

    return f;
}

void INA260::clearAlertLatch()
{
    // 读取 Mask/Enable 清除锁存标志，重写配置恢复报警逻辑
    (void)readRaw(REG_MASK_ENABLE);
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
}

uint16_t INA260::readMaskEnableRaw()
{
    return readRaw(REG_MASK_ENABLE);
}

// ============================================================
//  运行时配置
// ============================================================

void INA260::setConfig(const Config& cfg)
{
    _cfg = cfg;

    writeRaw(REG_CONFIG,      buildConfigReg());
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
    writeRaw(REG_ALERT_LIMIT, _cfg.alertLimit);
    (void)readRaw(REG_MASK_ENABLE);   // 清除锁存标志
}

void INA260::setAveraging(AvgSample mode)
    { _cfg.averaging = mode; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA260::setBusConvTime(ConvTime time)
    { _cfg.busConvTime = time; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA260::setShuntConvTime(ConvTime time)
    { _cfg.shuntConvTime = time; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA260::setOperatingMode(OperatingMode mode)
    { _cfg.mode = mode; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA260::setAlertLimit(uint16_t limit)
    { _cfg.alertLimit = limit; writeRaw(REG_ALERT_LIMIT, limit); }

void INA260::setAlertMask(uint16_t mask)
{
    _cfg.alertMask = mask & 0xFC00;   // 仅 bit15-10 可写
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
    (void)readRaw(REG_MASK_ENABLE);   // 清除可能残留的锁存标志
}

void INA260::reset()
{
    // 写 CONFIG 的 RST 位触发软复位，寄存器恢复默认值
    writeRaw(REG_CONFIG, static_cast<uint16_t>(0x8000u | buildConfigReg()));
}

bool INA260::isConversionReady()
{
    uint16_t raw = readRaw(REG_MASK_ENABLE);
    return (raw & 0x0008) != 0;       // bit3: CVRF
}

bool INA260::waitConversionReady(uint32_t timeout_ms)
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

uint16_t INA260::getManufacturerID()
    { return readRaw(REG_MANUFACTURER_ID); }

uint16_t INA260::getDieID()
    { return readRaw(REG_DIE_ID); }

// ============================================================
//  调试
// ============================================================

void INA260::captureSnapshot(INA260_Snapshot& s)
{
    s.config         = readRaw(REG_CONFIG);
    s.current        = readRaw(REG_CURRENT);
    s.busVoltage     = readRaw(REG_BUS_VOLTAGE);
    s.power          = readRaw(REG_POWER);
    s.maskEnable     = readRaw(REG_MASK_ENABLE);
    s.alertLimit     = readRaw(REG_ALERT_LIMIT);
    s.manufacturerID = readRaw(REG_MANUFACTURER_ID);
    s.dieID          = readRaw(REG_DIE_ID);
}

void INA260::dumpRawRegisters(const INA260_Snapshot& s)
{
    char buf[128];
    int  len;

    len = snprintf(buf, sizeof(buf),
                   "\r\n========== INA260 RAW ==========\r\n");
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "CONFIG   : 0x%04X\r\n", s.config);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "CURRENT  : 0x%04X\r\n", s.current);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "VBUS     : 0x%04X\r\n", s.busVoltage);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "POWER    : 0x%04X\r\n", s.power);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "MASK/EN  : 0x%04X\r\n", s.maskEnable);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "ALERTLIM : 0x%04X\r\n", s.alertLimit);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "MFG ID   : 0x%04X\r\n", s.manufacturerID);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "DIE ID   : 0x%04X\r\n", s.dieID);
    (void)debug_uart.send_data((const uint8_t*)buf, len);
}

void INA260::dumpEngineeringData(const INA260_Snapshot& s)
{
    char buf[128];
    int  len;

    int32_t shunt_uV   = rawToShuntVoltage_uV(s.current);
    int32_t bus_mV     = rawToBusVoltage_mV(s.busVoltage);
    int32_t current_mA = rawToCurrent_mA(s.current);
    int32_t power_mW   = rawToPower_mW(s.power);

    len = snprintf(buf, sizeof(buf),
                   "\r\n========== INA260 ENGINEERING ==========\r\n");
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Bus Voltage    : %ld.%03ld V\r\n",
                   (long)bus_mV / 1000, (long)bus_mV % 1000);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Shunt Voltage  : %ld.%03ld mV\r\n",
                   (long)shunt_uV / 1000, (long)shunt_uV % 1000);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Current        : %ld.%03ld A\r\n",
                   (long)current_mA / 1000, (long)current_mA % 1000);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "Power          : %ld.%03ld W\r\n",
                   (long)power_mW / 1000, (long)power_mW % 1000);
    (void)debug_uart.send_data((const uint8_t*)buf, len);
}

void INA260::dumpRegisters()
{
    INA260_Snapshot snap;
    captureSnapshot(snap);
    dumpRawRegisters(snap);
    dumpEngineeringData(snap);
}
