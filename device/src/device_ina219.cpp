#include "device_ina219.hpp"
#include "device_serial.hpp"

#include <cstdio>

// ============================================================
//  INA219 电流监测驱动（STM32G070 移植版）
//  来源库: Rob Tillaart INA219 Arduino Library v0.4.2 (2021-05-18)
//     URL: https://github.com/RobTillaart/INA219
//  移植说明:
//    - 无浮点，物理量定点整数；校准公式 CAL = 0.04096/(LSB×R)
//    - 12 位 ADC：分流 LSB=10 µV（16 位有符号），总线 LSB=4 mV（13 位左对齐）
//    - 功率 LSB = 20 × Current_LSB
// ============================================================

// ============================================================
//  构造
// ============================================================

INA219::INA219(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _alertPin(nullptr)
    , _currentLSB_nA(0)
{
    _updateCurrentLSB();
}

INA219::INA219(inter_i2c_bus* bus, uint8_t addr, const Config& cfg)
    : _dev(bus, addr)
    , _cfg(cfg)
    , _alertPin(nullptr)
    , _currentLSB_nA(0)
{
    _updateCurrentLSB();
}

INA219::~INA219()
{
}

// ============================================================
//  Current LSB（nA 精度，消除整数截断误差）
//  Current_LSB = I_MAX / 2^15
//  currentLSB_nA = maxCurrent_mA × 1e6 / 32768
// ============================================================

void INA219::_updateCurrentLSB()
{
    _currentLSB_nA =
        (static_cast<int64_t>(_cfg.maxCurrent_mA) * 1000000) / 32768;
}

// ============================================================
//  校准（对齐参考库 setMaxCurrentShunt）
//
//  CAL = 0.04096 / (Current_LSB × R_shunt)
//      = 4.096e13 / (LSB_nA × R_uOhm)   ← 系数 0.04096，非 INA226 的 0.00512
//
//  取整后的 CAL 反推实际 LSB，保证读数与芯片内部计算严格自洽：
//    CURRENT(raw) = VSHUNT(raw) × CAL / 4096   （INA219 12 位 ADC）
// ============================================================

int INA219::setMaxCurrentShunt(int32_t maxCurrent_mA, uint32_t rShunt_uOhm)
{
    if (maxCurrent_mA < 1)
        return ERR_MAXCURRENT_LOW;

    if (rShunt_uOhm < 1000)                 // < 1 mΩ
        return ERR_SHUNT_LOW;

    // 分流满量程由 PGA 增益决定：±40/±80/±160/±320 mV
    // （留 0.02 mV 裕量防数学溢出，与 INA226 版一致）
    // 单位换算：mA × µΩ = nV
    int64_t fs_nV = static_cast<int64_t>(gainFullScale_mV()) * 1000000 - 20000;
    if (static_cast<int64_t>(maxCurrent_mA) * rShunt_uOhm > fs_nV)
        return ERR_SHUNTVOLTAGE_HIGH;

    // 名义 LSB（nA），CAL 取整后以实际 LSB 为准
    int64_t lsb_nA =
        (static_cast<int64_t>(maxCurrent_mA) * 1000000) / 32768;

    uint64_t cal = 40960000000000ULL / (static_cast<uint64_t>(lsb_nA) * rShunt_uOhm);

    // CAL 超出 15 位时 LSB 加倍、CAL 减半（同 INA226 版 auto-scale）
    while (cal > 0x7FFF)
    {
        lsb_nA *= 2;
        cal >>= 1;
    }

    if (cal == 0)
        return ERR_CAL_OVERFLOW;

    // 用取整后的 CAL 反推实际 LSB，读数与芯片行为一致
    int64_t lsb_actual = 40960000000000LL / (static_cast<int64_t>(cal) * rShunt_uOhm);
    if (lsb_actual <= 0)
        return ERR_CAL_OVERFLOW;

    _cfg.maxCurrent_mA = maxCurrent_mA;
    _cfg.rShunt_uOhm   = rShunt_uOhm;
    _cfg.calibration   = static_cast<uint16_t>(cal);
    _currentLSB_nA     = lsb_actual;

    writeRaw(REG_CALIBRATION, _cfg.calibration);
    return ERR_NONE;
}

// ============================================================
//  连接 / 错误
// ============================================================

bool INA219::isConnected()
{
    // INA219 无制造商/Die ID 寄存器，用地址探测
    return _dev.ping();
}

int INA219::getLastError()
{
    return _dev.lastError();
}

// ============================================================
//  init
// ============================================================

void INA219::init()
{
    // 0. 自动校准模式：由采样电阻与最大电流推导 CAL + LSB
    if (_cfg.rShunt_uOhm != 0)
    {
        (void)setMaxCurrentShunt(_cfg.maxCurrent_mA, _cfg.rShunt_uOhm);
    }

    // 1. 配置寄存器 (0x00)
    writeRaw(REG_CONFIG,      buildConfigReg());

    // 2. 校准寄存器 (0x05)
    writeRaw(REG_CALIBRATION, _cfg.calibration);

    // 3. Mask/Enable (0x06) — 写配置位 + 报警功能使能位
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());

    // 4. Alert Limit (0x07)
    writeRaw(REG_ALERT_LIMIT, _cfg.alertLimit);

    // 5. ⚠️ 上电后 AFF 等状态位可能已置位，与寄存器写入顺序无关。
    //    INA219 开漏 ALERT 引脚在锁存模式下只要状态位非零就持续拉低。
    //    读取 Mask/Enable 是唯一清除锁存标志的手段。
    (void)readRaw(REG_MASK_ENABLE);

    // 6. 自动配置报警引脚（Config 中指定了 GPIO 端口和引脚时）
    //    零堆分配：io_ctrl 为成员对象（非指针），移动赋值接管引脚所有权
    if (_cfg.alert_port != nullptr && _cfg.alert_pin != pin_none)
    {
        _alertPinAuto = io_ctrl(_cfg.alert_port, _cfg.alert_pin);  // 释放旧引脚 + 接管新引脚
        _alertPinAuto.init(mode_input, pullup);     // 时钟使能 + 上拉输入
    }
}

// ============================================================
//  I2C IO（INA219 全部 16-bit 寄存器）
// ============================================================

void INA219::writeRaw(uint8_t reg, uint16_t data)
{
    _dev.write_16bit(reg, data);
}

uint16_t INA219::readRaw(uint8_t reg)
{
    return _dev.read_16bit(reg);
}

// ============================================================
//  寄存器构建
// ============================================================

uint16_t INA219::buildConfigReg() const
{
    // Config Register (0x00):
    //   [15]    RST
    //   [14]    保留 = 0
    //   [13]    BRN   — 总线量程（0=16V，1=32V）
    //   [12:11] PGA   — 增益（00=±40mV，01=±80mV，10=±160mV，11=±320mV）
    //   [10:7]  BADC  — 总线 ADC 模式（分辨率/平均）
    //   [6:3]   SADC  — 分流 ADC 模式
    //   [2:0]   MODE
    uint16_t r = 0;
    r |= static_cast<uint16_t>(_cfg.reset)    << 15;
    r |= static_cast<uint16_t>(_cfg.busRange) << 13;
    r |= static_cast<uint16_t>(_cfg.gain)     << 11;
    r |= static_cast<uint16_t>(_cfg.busAdc)   << 7;
    r |= static_cast<uint16_t>(_cfg.shuntAdc) << 3;
    r |= static_cast<uint16_t>(_cfg.mode);
    return r;
}

uint16_t INA219::buildMaskEnable() const
{
    // Mask/Enable Register (0x06) — 与 INA226 同布局：
    //   [15:10] 报警功能使能（SOL/SUL/BOL/BUL/POL/CNVR，写入有效）
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
//  转换
// ============================================================

int32_t INA219::gainFullScale_mV() const
{
    switch (static_cast<uint8_t>(_cfg.gain))
    {
        case 0: return 40;      // G1 ±40 mV
        case 1: return 80;      // G2 ±80 mV
        case 2: return 160;     // G4 ±160 mV
        default: return 320;    // G8 ±320 mV
    }
}

int32_t INA219::rawToShuntVoltage_uV(uint16_t raw)
{
    // 固定 10 µV/LSB，16 位有符号，无需移位
    return static_cast<int32_t>(signExtend<16>(raw)) * 10;
}

int32_t INA219::rawToBusVoltage_mV(uint16_t raw)
{
    // 固定 4 mV/LSB，13 位数据左对齐（bit15-3），bit1-0 为 OVF/CVRF 标志
    // 16V 量程有效 12 位，32V 量程用到第 13 位（LSB 仍为 4 mV）
    return (raw >> 3) * 4;
}

int32_t INA219::rawToCurrent_mA(uint16_t raw) const
{
    int64_t val = signExtend<16>(raw);
    return static_cast<int32_t>((val * _currentLSB_nA) / 1000000);
}

int32_t INA219::rawToPower_mW(uint16_t raw) const
{
    // Power_LSB = 20 × Current_LSB（注意非 INA226 的 25 倍）
    // Power(mW) = raw × 20 × LSB_nA / 1e6
    int64_t val = static_cast<int64_t>(raw);
    return static_cast<int32_t>((val * 20 * _currentLSB_nA) / 1000000);
}

// ============================================================
//  测量 API
// ============================================================

int32_t INA219::getBusVoltage_mV()
    { return rawToBusVoltage_mV(readRaw(REG_BUS_VOLTAGE)); }

int32_t INA219::getShuntVoltage_uV()
    { return rawToShuntVoltage_uV(readRaw(REG_SHUNT_VOLTAGE)); }

int32_t INA219::getCurrent_mA()
    { return rawToCurrent_mA(readRaw(REG_CURRENT)); }

int32_t INA219::getPower_mW()
    { return rawToPower_mW(readRaw(REG_POWER)); }

bool INA219::getMathOverflowFlag()
{
    // 总线寄存器 bit0：OVF 数学溢出（电流/功率寄存器饱和）
    return (readRaw(REG_BUS_VOLTAGE) & 0x0001) != 0;
}

bool INA219::getConversionFlag()
{
    // 总线寄存器 bit1：CVRF 转换完成（参考库 getConversionFlag）
    return (readRaw(REG_BUS_VOLTAGE) & 0x0002) != 0;
}

bool INA219::isConversionReady()
{
    uint16_t raw = readRaw(REG_MASK_ENABLE);
    return (raw & 0x0008) != 0;       // bit3: CVRF
}

// ============================================================
//  报警
// ============================================================

void INA219::bindAlertPin(io_ctrl *pin)
{
    _alertPin = pin;
}

bool INA219::isAlertAsserted()
{
    // 优先使用 bindAlertPin 绑定的外部引脚
    if (_alertPin)
        return _alertPin->read() == Low;

    // 否则使用 Config 自动配置的引脚（io_ctrl 成员，未配置时无效）
    if (_alertPinAuto.is_initialized())
        return _alertPinAuto.read() == Low;

    return false;
}

INA219_AlertFlags INA219::readAlertFlags()
{
    INA219_AlertFlags f = {};
    uint16_t raw = readRaw(REG_MASK_ENABLE);

    f.mathOverflow    = (raw & 0x0004) != 0;  // bit 2: OVF
    f.conversionReady = (raw & 0x0008) != 0;  // bit 3: CVRF
    f.alertFunction   = (raw & 0x0010) != 0;  // bit 4: AFF

    return f;
}

void INA219::clearAlertLatch()
{
    // 读取 Mask/Enable 清除锁存标志，重写配置恢复报警逻辑
    (void)readRaw(REG_MASK_ENABLE);
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
}

uint16_t INA219::readMaskEnableRaw()
{
    return readRaw(REG_MASK_ENABLE);
}

// ============================================================
//  运行时配置
// ============================================================

void INA219::setConfig(const Config& cfg)
{
    _cfg = cfg;
    _updateCurrentLSB();

    // 自动校准模式：由采样电阻与最大电流推导 CAL + LSB
    if (_cfg.rShunt_uOhm != 0)
    {
        (void)setMaxCurrentShunt(_cfg.maxCurrent_mA, _cfg.rShunt_uOhm);
    }

    writeRaw(REG_CONFIG,      buildConfigReg());
    writeRaw(REG_CALIBRATION, _cfg.calibration);
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
    writeRaw(REG_ALERT_LIMIT, _cfg.alertLimit);
    (void)readRaw(REG_MASK_ENABLE);   // 清除锁存标志
}

void INA219::setBusVoltageRange(BusRange range)
    { _cfg.busRange = range; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA219::setGain(Gain gain)
    { _cfg.gain = gain; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA219::setAveraging(AvgSample samples)
{
    // INA219 的平均次数编码在 ADC 字段（12 位 + N 次平均）
    uint8_t code = 0x08 + static_cast<uint8_t>(samples);
    _cfg.busAdc   = static_cast<AdcMode>(code);
    _cfg.shuntAdc = static_cast<AdcMode>(code);
    writeRaw(REG_CONFIG, buildConfigReg());
}

void INA219::setBusADC(AdcMode mode)
    { _cfg.busAdc = mode; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA219::setShuntADC(AdcMode mode)
    { _cfg.shuntAdc = mode; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA219::setADC(AdcMode mode)
{
    _cfg.busAdc   = mode;
    _cfg.shuntAdc = mode;
    writeRaw(REG_CONFIG, buildConfigReg());
}

void INA219::setOperatingMode(OperatingMode mode)
    { _cfg.mode = mode; writeRaw(REG_CONFIG, buildConfigReg()); }

void INA219::setCalibration(uint16_t cal)
    { _cfg.calibration = cal; writeRaw(REG_CALIBRATION, cal); }

void INA219::setAlertLimit(uint16_t limit)
    { _cfg.alertLimit = limit; writeRaw(REG_ALERT_LIMIT, limit); }

void INA219::setAlertMask(uint16_t mask)
{
    _cfg.alertMask = mask & 0xFC00;   // 仅 bit15-10 可写
    writeRaw(REG_MASK_ENABLE, buildMaskEnable());
    (void)readRaw(REG_MASK_ENABLE);   // 清除可能残留的锁存标志
}

void INA219::reset()
{
    // 写 CONFIG 的 RST 位触发软复位，寄存器恢复默认值
    writeRaw(REG_CONFIG, static_cast<uint16_t>(0x8000u | buildConfigReg()));
    _currentLSB_nA = 0;               // 复位后校准丢失
}

bool INA219::waitConversionReady(uint32_t timeout_ms)
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
//  调试
// ============================================================

void INA219::captureSnapshot(INA219_Snapshot& s)
{
    s.config         = readRaw(REG_CONFIG);
    s.shuntVoltage   = readRaw(REG_SHUNT_VOLTAGE);
    s.busVoltage     = readRaw(REG_BUS_VOLTAGE);
    s.power          = readRaw(REG_POWER);
    s.current        = readRaw(REG_CURRENT);
    s.calibration    = readRaw(REG_CALIBRATION);
    s.maskEnable     = readRaw(REG_MASK_ENABLE);
    s.alertLimit     = readRaw(REG_ALERT_LIMIT);
}

void INA219::dumpRawRegisters(const INA219_Snapshot& s)
{
    char buf[128];
    int  len;

    len = snprintf(buf, sizeof(buf),
                   "\r\n========== INA219 RAW ==========\r\n");
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "CONFIG   : 0x%04X\r\n", s.config);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "VSHUNT   : 0x%04X\r\n", s.shuntVoltage);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "VBUS     : 0x%04X\r\n", s.busVoltage);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "CURRENT  : 0x%04X\r\n", s.current);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "POWER    : 0x%04X\r\n", s.power);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "CAL      : 0x%04X\r\n", s.calibration);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "MASK/EN  : 0x%04X\r\n", s.maskEnable);
    (void)debug_uart.send_data((const uint8_t*)buf, len);

    len = snprintf(buf, sizeof(buf), "ALERTLIM : 0x%04X\r\n", s.alertLimit);
    (void)debug_uart.send_data((const uint8_t*)buf, len);
}

void INA219::dumpEngineeringData(const INA219_Snapshot& s)
{
    char buf[128];
    int  len;

    int32_t shunt_uV   = rawToShuntVoltage_uV(s.shuntVoltage);
    int32_t bus_mV     = rawToBusVoltage_mV(s.busVoltage);
    int32_t current_mA = rawToCurrent_mA(s.current);
    int32_t power_mW   = rawToPower_mW(s.power);

    len = snprintf(buf, sizeof(buf),
                   "\r\n========== INA219 ENGINEERING ==========\r\n");
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

    len = snprintf(buf, sizeof(buf), "OVF / CVRF     : %d / %d\r\n",
                   (int)((s.busVoltage & 0x0001) != 0),
                   (int)((s.busVoltage & 0x0002) != 0));
    (void)debug_uart.send_data((const uint8_t*)buf, len);
}

void INA219::dumpRegisters()
{
    INA219_Snapshot snap;
    captureSnapshot(snap);
    dumpRawRegisters(snap);
    dumpEngineeringData(snap);
}
