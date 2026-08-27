#include "device_ads1115.hpp"
#include "systick.h"

// ============================================================
//  来源：Rob Tillaart ADS1X15 v0.6.2
//  移植说明见 device_ads1115.hpp 文件头
//
//  电压换算（定点，无浮点）：
//   ADS1115 是 16-bit 有符号输出，数据手册 LSB = FSR / 2^15
//     ±6.144V → 187.5 µV/LSB      ±4.096V → 125 µV/LSB
//     ±2.048V → 62.5 µV/LSB       ±1.024V → 31.25 µV/LSB
//     ±0.512V → 15.625 µV/LSB     ±0.256V → 7.8125 µV/LSB
//   toVoltage_uV = raw × FSR_uV / 32768
//     raw × FSR_uV 最大 32767 × 6,144,000 ≈ 2.0e11 → 需 64 位中间量
//   toVoltage_mV = raw × FSR_mV / 32768（int32 足够）
// ============================================================

// ============================================================
//  构造 / 初始化
// ============================================================

ADS1115::ADS1115(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _gain(0)
    , _gainMask(0x0000)          // PGA ±6.144V
    , _mode(1)
    , _modeMask(0x0100)          // 单次模式
    , _datarate(4)
    , _datarateMask(4u << 5)     // 128 SPS
    , _compMode(0)               // 传统模式
    , _compPol(1)                // 高有效
    , _compLatch(0)              // 非锁存
    , _compQueConvert(3)         // 禁用比较器
    , _lastRequest(0xFFFF)       // 无请求
    , _error(ERR_OK)
{
}

void ADS1115::init()
{
    if (!isConnected()) return;

    // 写入当前配置但不启动转换（OS = 0）：单次模式下 OS=0 无动作，
    // 连续模式下不改变运行状态；配置在首次 requestADC 时随 CONFIG 写入
    uint16_t config = 0x0000;
    config |= _gainMask;
    config |= _modeMask;
    config |= _datarateMask;
    config |= _compMode      ? 0x0010 : 0x0000;
    config |= _compPol       ? 0x0008 : 0x0000;
    config |= _compLatch     ? 0x0004 : 0x0000;
    config |= (_compQueConvert & 0x03);
    writeRaw(REG_CONFIG, config);
}

bool ADS1115::isConnected()
{
    return _dev.ping();
}

int8_t ADS1115::getError()
{
    int8_t rv = _error;
    _error = ERR_OK;
    return rv;
}

// ============================================================
//  I2C 读写（全部 16-bit 寄存器）
// ============================================================

void ADS1115::writeRaw(uint8_t reg, uint16_t data)
{
    _dev.write_16bit(reg, data);
    if (_dev.lastError() != 0) _error = ERR_I2C;
}

uint16_t ADS1115::readRaw(uint8_t reg)
{
    uint16_t v = _dev.read_16bit(reg);
    if (_dev.lastError() != 0) _error = ERR_I2C;
    return v;
}

// ============================================================
//  配置
// ============================================================

void ADS1115::setGain(uint8_t gain)
{
    switch (gain)
    {
        case 0:  _gain = 0;  _gainMask = 0x0000; break;   // ±6.144V
        case 1:  _gain = 1;  _gainMask = 0x0200; break;   // ±4.096V
        case 2:  _gain = 2;  _gainMask = 0x0400; break;   // ±2.048V
        case 4:  _gain = 4;  _gainMask = 0x0600; break;   // ±1.024V
        case 8:  _gain = 8;  _gainMask = 0x0800; break;   // ±0.512V
        case 16: _gain = 16; _gainMask = 0x0A00; break;   // ±0.256V
        default:  // 无效值 → 最安全的增益 0（对齐原版）
            _gain = 0;
            _gainMask = 0x0000;
            break;
    }
}

uint8_t ADS1115::getGain()
{
    switch (_gainMask)
    {
        case 0x0000: return 0;
        case 0x0200: return 1;
        case 0x0400: return 2;
        case 0x0600: return 4;
        case 0x0800: return 8;
        case 0x0A00: return 16;
    }
    _error = static_cast<int8_t>(ERR_INVALID_GAIN);   // 0xFF → -1，对齐原版
    return static_cast<uint8_t>(ERR_INVALID_GAIN);
}

void ADS1115::setDataRate(uint8_t dataRate)
{
    _datarate = (dataRate > 7) ? 4 : dataRate;   // 无效值 → 默认 4
    _datarateMask = static_cast<uint16_t>(_datarate) << 5;
}

uint8_t ADS1115::getDataRate()
{
    return _datarate;
}

void ADS1115::setMode(Mode mode)
{
    _mode = (mode == Mode::Continuous) ? 0 : 1;
    _modeMask = (_mode == 1) ? 0x0100 : 0x0000;
}

ADS1115::Mode ADS1115::getMode()
{
    if (_mode == 0) return Mode::Continuous;
    if (_mode == 1) return Mode::Single;
    _error = static_cast<int8_t>(ERR_INVALID_MODE);   // 0xFE → -2，对齐原版
    return Mode::Single;
}

// ============================================================
//  电压换算（定点）
// ============================================================

int32_t ADS1115::getMaxVoltage_mV()
{
    switch (_gainMask)
    {
        case 0x0000: return 6144;
        case 0x0200: return 4096;
        case 0x0400: return 2048;
        case 0x0600: return 1024;
        case 0x0800: return 512;
        case 0x0A00: return 256;
    }
    _error = ERR_INVALID_VOLTAGE;
    return ERR_INVALID_VOLTAGE;
}

int32_t ADS1115::toVoltage_uV(int16_t raw)
{
    // uV = raw × FSR_uV / 32768（64 位中间量，见文件头推导）
    int64_t v = static_cast<int64_t>(raw) * (static_cast<int64_t>(getMaxVoltage_mV()) * 1000);
    return static_cast<int32_t>(v / 32768);
}

int32_t ADS1115::toVoltage_mV(int16_t raw)
{
    // mV = raw × FSR_mV / 32768（32767 × 6144 ≈ 2.0e8，int32 足够）
    return (static_cast<int32_t>(raw) * getMaxVoltage_mV()) / 32768;
}

int16_t ADS1115::voltageToRaw_mV(int32_t volts_mV)
{
    // raw = mV × 32768 / FSR_mV（64 位中间量）
    int64_t v = static_cast<int64_t>(volts_mV) * 32768 / getMaxVoltage_mV();
    if (v > 32767)  v = 32767;
    if (v < -32768) v = -32768;
    return static_cast<int16_t>(v);
}

int16_t ADS1115::voltageToRaw_uV(int32_t volts_uV)
{
    int64_t v = static_cast<int64_t>(volts_uV) * 32768
                / (static_cast<int64_t>(getMaxVoltage_mV()) * 1000);
    if (v > 32767)  v = 32767;
    if (v < -32768) v = -32768;
    return static_cast<int16_t>(v);
}

// ============================================================
//  读取
// ============================================================

int16_t ADS1115::readADC(uint8_t pin)
{
    if (pin > 3) return 0;
    uint16_t mode = static_cast<uint16_t>((4 + pin) << 12);   // 单端：pin → MUX
    return _readADC(mode);
}

int16_t ADS1115::readADC_Differential_0_1()
{
    return _readADC(0x0000);   // MUX 差分 AIN0-AIN1
}

int16_t ADS1115::getValue()
{
    return static_cast<int16_t>(readRaw(REG_CONVERSION));
}

// ============================================================
//  异步接口
// ============================================================

void ADS1115::requestADC(uint8_t pin)
{
    if (pin > 3) return;
    _requestADC(static_cast<uint16_t>((4 + pin) << 12));
}

void ADS1115::requestADC_Differential_0_1()
{
    _requestADC(0x0000);
}

bool ADS1115::isBusy()
{
    return !isReady();
}

bool ADS1115::isReady()
{
    uint16_t config = readRaw(REG_CONFIG);
    return (config & 0x8000) != 0;   // OS 位：1 = 转换完成/空闲
}

uint8_t ADS1115::lastRequest()
{
    switch (_lastRequest)
    {
        case 0x4000: return 0x00;
        case 0x5000: return 0x01;
        case 0x6000: return 0x02;
        case 0x7000: return 0x03;
        case 0x0000: return 0x10;   // 差分 0-1
    }
    return 0xFF;   // 无请求 / 非法请求
}

// ============================================================
//  内部
// ============================================================

int16_t ADS1115::_readADC(uint16_t readmode)
{
    _requestADC(readmode);

    if (_mode == 1)
    {
        // 单次模式：轮询 OS 位，超时 = 128 >> datarate + 10 ms（对齐原版）
        // 数据率 0..7 → 转换时间 128..1 ms，+10 ms 裕量
        uint16_t timeout = static_cast<uint16_t>((128u >> _datarate) + 10u);
        uint16_t elapsed = 0;
        while (isBusy())
        {
            if (elapsed++ >= timeout)
            {
                _error = ERR_TIMEOUT;
                return static_cast<int16_t>(ERR_TIMEOUT);
            }
            delay_ms(1);
        }
    }
    else
    {
        // 连续模式：等一次转换周期（8 ms = 最快数据率 128ms/16 的裕量，
        // 对齐原版 _conversionDelay = 8）
        delay_ms(8);
    }
    return getValue();
}

void ADS1115::_requestADC(uint16_t readmode)
{
    // CONFIG 寄存器（bit15 OS 置 1 启动单次转换；连续模式写入无副作用）
    uint16_t config = 0x8000;                      // OS: START
    config |= readmode;                            // bit 12-14 MUX
    config |= _gainMask;                           // bit 9-11 PGA
    config |= _modeMask;                           // bit 8  MODE
    config |= _datarateMask;                       // bit 5-7 DR
    config |= _compMode      ? 0x0010 : 0x0000;    // bit 4  COMP_MODE
    config |= _compPol       ? 0x0008 : 0x0000;    // bit 3  COMP_POL
    config |= _compLatch     ? 0x0004 : 0x0000;    // bit 2  COMP_LATCH
    config |= (_compQueConvert & 0x03);            // bit 1-0 COMP_QUE
    writeRaw(REG_CONFIG, config);

    _lastRequest = readmode;
}

// ============================================================
//  比较器
// ============================================================

void ADS1115::setComparatorMode(CompMode mode)
{
    _compMode = (mode == CompMode::Window) ? 1 : 0;
}

ADS1115::CompMode ADS1115::getComparatorMode()
{
    return (_compMode == 1) ? CompMode::Window : CompMode::Traditional;
}

bool ADS1115::setComparatorOff()
{
    // 读 CONFIG，清 bit1-0（COMP_QUE = 11 = 禁用），写回
    uint16_t config = readRaw(REG_CONFIG) & 0xFFF7;
    writeRaw(REG_CONFIG, config);
    _compQueConvert = 3;
    return _dev.lastError() == 0;
}

void ADS1115::setComparatorPolarity(CompPol pol)
{
    _compPol = (pol == CompPol::ActiveHigh) ? 1 : 0;
}

ADS1115::CompPol ADS1115::getComparatorPolarity()
{
    return (_compPol == 1) ? CompPol::ActiveHigh : CompPol::ActiveLow;
}

void ADS1115::setComparatorLatch(CompLatch latch)
{
    _compLatch = (latch == CompLatch::Latch) ? 1 : 0;
}

ADS1115::CompLatch ADS1115::getComparatorLatch()
{
    return (_compLatch == 1) ? CompLatch::Latch : CompLatch::NonLatch;
}

void ADS1115::setComparatorQueConvert(CompQue que)
{
    _compQueConvert = (static_cast<uint8_t>(que) < 3) ? static_cast<uint8_t>(que) : 3;
}

ADS1115::CompQue ADS1115::getComparatorQueConvert()
{
    return static_cast<CompQue>(_compQueConvert & 0x03);
}

void ADS1115::setComparatorThresholdLow(int16_t lo)
{
    writeRaw(REG_LO_THRESH, static_cast<uint16_t>(lo));
}

void ADS1115::setComparatorThresholdHigh(int16_t hi)
{
    writeRaw(REG_HI_THRESH, static_cast<uint16_t>(hi));
}

int16_t ADS1115::getComparatorThresholdLow()
{
    return static_cast<int16_t>(readRaw(REG_LO_THRESH));
}

int16_t ADS1115::getComparatorThresholdHigh()
{
    return static_cast<int16_t>(readRaw(REG_HI_THRESH));
}

void ADS1115::setComparatorThresholdLow_mV(int32_t lo_mV)
{
    setComparatorThresholdLow(voltageToRaw_mV(lo_mV));
}

void ADS1115::setComparatorThresholdHigh_mV(int32_t hi_mV)
{
    setComparatorThresholdHigh(voltageToRaw_mV(hi_mV));
}
