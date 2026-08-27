#include "device_pca9685.hpp"

// ============================================================
//  通道寄存器基址（0x06 + 4×channel）
// ============================================================

namespace
{
constexpr uint8_t CHANNEL_REG(uint8_t channel) { return 0x06 + (channel * 4); }

// 频率 prescale 定点公式：
//   prescale = round(25e6 / (4096 × freq)) - 1
//            = 6103.5/freq - 1
//   ×8 保持 0.5 精度 → 48828/(freq×8) - 1（与原库一致）
constexpr uint32_t PRESCALE_NUM = 48828;
}

// ============================================================
//  构造
// ============================================================

PCA9685::PCA9685(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _addr(addr)
{
}

void PCA9685::init()
{
    if (!isConnected()) return;
    (void)configure(MODE1_AUTOINCR | MODE1_ALLCALL, MODE2_TOTEMPOLE);
}

// ============================================================
//  连接 / 错误
// ============================================================

bool PCA9685::isConnected()
{
    return _dev.ping();
}

uint8_t PCA9685::getLastError()
{
    return _dev.lastError();
}

// ============================================================
//  配置
// ============================================================

uint8_t PCA9685::configure(uint8_t mode1_mask, uint8_t mode2_mask)
{
    uint8_t r1 = setMode1(mode1_mask);
    uint8_t r2 = setMode2(mode2_mask);
    if ((r1 != ERR_OK) || (r2 != ERR_OK)) return ERR_ERROR;
    return ERR_OK;
}

uint8_t PCA9685::setMode1(uint8_t value)
{
    return _writeRegister(REG_MODE1, value);
}

uint8_t PCA9685::getMode1()
{
    return _readRegister(REG_MODE1);
}

uint8_t PCA9685::setMode2(uint8_t value)
{
    return _writeRegister(REG_MODE2, value);
}

uint8_t PCA9685::getMode2()
{
    return _readRegister(REG_MODE2);
}

// ============================================================
//  PWM 频率
// ============================================================

uint8_t PCA9685::setFrequency(uint16_t freq, int8_t offset)
{
    _freq = freq;
    if (_freq < MIN_FREQ) _freq = MIN_FREQ;
    if (_freq > MAX_FREQ) _freq = MAX_FREQ;

    // prescale = 48828/(freq×8) - 1 + offset
    uint8_t scaler = static_cast<uint8_t>(PRESCALE_NUM / (_freq * 8) - 1);
    scaler = static_cast<uint8_t>(scaler + offset);

    // 数据手册：修改 prescale 前必须置 SLEEP
    uint8_t mode1 = getMode1();
    (void)setMode1(mode1 | MODE1_SLEEP);
    (void)_writeRegister(REG_PRE_SCALER, scaler);
    return setMode1(mode1);   // 恢复 MODE1，退出睡眠
}

uint16_t PCA9685::getFrequency()
{
    uint8_t scaler = _readRegister(REG_PRE_SCALER);
    scaler++;
    _freq = static_cast<uint16_t>(PRESCALE_NUM / scaler / 8);
    return _freq;
}

// ============================================================
//  通道 PWM
// ============================================================

uint8_t PCA9685::setPWM(uint8_t channel, uint16_t onTime, uint16_t offTime)
{
    if (channel >= channelCount()) return ERR_CHANNEL;
    // 0x1000 = 全开/全关位（允许通过 0..4096 传入）
    return _writeChannel(CHANNEL_REG(channel), onTime & 0x1FFF, offTime & 0x1FFF);
}

uint8_t PCA9685::setPWM(uint8_t channel, uint16_t offTime)
{
    return setPWM(channel, 0, offTime);
}

uint8_t PCA9685::getPWM(uint8_t channel, uint16_t* onTime, uint16_t* offTime)
{
    if (channel >= channelCount()) return ERR_CHANNEL;
    return _readChannel(CHANNEL_REG(channel), onTime, offTime);
}

uint8_t PCA9685::write1(uint8_t channel, uint8_t mode)
{
    if (channel >= channelCount()) return ERR_CHANNEL;
    // 全开 = ON 置 FULL_ON 位（0x1000）；全关 = ON/OFF 全零
    if (mode != 0) return _writeChannel(CHANNEL_REG(channel), 0x1000, 0x0000);
    return _writeChannel(CHANNEL_REG(channel), 0x0000, 0x0000);
}

uint8_t PCA9685::read1(uint8_t channel)
{
    if (channel >= channelCount()) return ERR_CHANNEL;

    uint16_t on, off;
    if (_readChannel(CHANNEL_REG(channel), &on, &off) != ERR_OK) return 2;

    if (on  & 0x1000) return 1;   // FULL_ON → 高
    if (off & 0x1000) return 0;   // FULL_OFF → 低

    uint16_t duty = (off - on) & 0x0FFF;
    if (duty == 4095) return 1;   // 100% 占空比 → 高
    if (duty == 0)    return 0;   // 0% → 低
    return 2;                     // 中间值 → 用 getPWM() 读取
}

uint8_t PCA9685::allOFF()
{
    return _writeRegister(REG_ALL_OFF_H, 0x10);   // bit4 = 全关
}

// ============================================================
//  子地址 / 广播地址
// ============================================================

bool PCA9685::enableSubCall(uint8_t nr)
{
    if ((nr == 0) || (nr > 3)) return false;
    uint8_t prev = getMode1();
    uint8_t mask = prev;
    if (nr == 1)      mask |= MODE1_SUB1;
    else if (nr == 2) mask |= MODE1_SUB2;
    else              mask |= MODE1_SUB3;
    if (mask != prev) (void)setMode1(mask);
    return true;
}

bool PCA9685::disableSubCall(uint8_t nr)
{
    if ((nr == 0) || (nr > 3)) return false;
    uint8_t prev = getMode1();
    uint8_t mask = prev;
    if (nr == 1)      mask &= static_cast<uint8_t>(~MODE1_SUB1);
    else if (nr == 2) mask &= static_cast<uint8_t>(~MODE1_SUB2);
    else              mask &= static_cast<uint8_t>(~MODE1_SUB3);
    if (mask != prev) (void)setMode1(mask);
    return true;
}

bool PCA9685::isEnabledSubCall(uint8_t nr)
{
    if ((nr == 0) || (nr > 3)) return false;
    uint8_t mask = getMode1();
    if (nr == 1) return (mask & MODE1_SUB1) != 0;
    if (nr == 2) return (mask & MODE1_SUB2) != 0;
    return (mask & MODE1_SUB3) != 0;
}

bool PCA9685::setSubCallAddress(uint8_t nr, uint8_t address)
{
    if ((nr == 0) || (nr > 3)) return false;
    (void)_writeRegister(0x01 + nr, address);   // SUBADR1..3
    return true;
}

uint8_t PCA9685::getSubCallAddress(uint8_t nr)
{
    if ((nr == 0) || (nr > 3)) return 0;
    return _readRegister(0x01 + nr);
}

bool PCA9685::enableAllCall()
{
    uint8_t prev = getMode1();
    uint8_t mask = prev | MODE1_ALLCALL;
    if (mask != prev) (void)setMode1(mask);
    return true;
}

bool PCA9685::disableAllCall()
{
    uint8_t prev = getMode1();
    uint8_t mask = prev & static_cast<uint8_t>(~MODE1_ALLCALL);
    if (mask != prev) (void)setMode1(mask);
    return true;
}

bool PCA9685::isEnabledAllCall()
{
    return (getMode1() & MODE1_ALLCALL) != 0;
}

bool PCA9685::setAllCallAddress(uint8_t address)
{
    (void)_writeRegister(0x05, address);   // ALLCALLADR
    return true;
}

uint8_t PCA9685::getAllCallAddress()
{
    return _readRegister(0x05);
}

// ============================================================
//  OE 输出使能引脚（低有效）
// ============================================================

void PCA9685::bindOutputEnable(io_ctrl* pin)
{
    _oePin = pin;
}

bool PCA9685::setOutputEnable(bool on)
{
    if (!_oePin) return false;
    _oePin->set(!on);   // 低有效：使能 → 拉低
    return true;
}

uint8_t PCA9685::getOutputEnable()
{
    if (!_oePin) return 1;   // 未绑定视为常使能
    return (_oePin->read() == Low) ? 1 : 0;
}

// ============================================================
//  SMBus 通用软复位
// ============================================================

void PCA9685::softReset()
{
    _dev.softReset();   // 地址 0x00 + 命令 0x06（SMBus 规范）
}

// ============================================================
//  I2C 寄存器读写
// ============================================================

uint8_t PCA9685::_writeRegister(uint8_t reg, uint8_t value)
{
    _dev.freedom_write(reg, value, 1);
    return (_dev.lastError() == 0) ? ERR_OK : ERR_I2C;
}

uint8_t PCA9685::_readRegister(uint8_t reg)
{
    uint64_t v = 0;
    (void)_dev.freedom_read(reg, &v, 1);
    return static_cast<uint8_t>(v & 0xFF);
}

// 通道 4 字节事务：ON_L ON_H OFF_L OFF_H（12 位 + 全开/全关位）
// freedom_write 按打包值从高位字节到低位字节发送：
//   打包 = ON_L<<24 | ON_H<<16 | OFF_L<<8 | OFF_H
uint8_t PCA9685::_writeChannel(uint8_t reg, uint16_t on, uint16_t off)
{
    uint32_t packed =
        static_cast<uint32_t>((on & 0xFF)) << 24 |
        static_cast<uint32_t>((on >> 8) & 0x1F) << 16 |
        static_cast<uint32_t>((off & 0xFF)) << 8 |
        static_cast<uint32_t>((off >> 8) & 0x1F);

    _dev.freedom_write(reg, packed, 4);
    return (_dev.lastError() == 0) ? ERR_OK : ERR_I2C;
}

// 读回：freedom_read 首字节落在打包值最高字节（ON_L）
uint8_t PCA9685::_readChannel(uint8_t reg, uint16_t* on, uint16_t* off)
{
    uint64_t packed = 0;
    if (!_dev.freedom_read(reg, &packed, 4))
    {
        return ERR_I2C;
    }
    *on  = static_cast<uint16_t>(((packed >> 16) & 0x1F) << 8) |
           static_cast<uint16_t>((packed >> 24) & 0xFF);
    *off = static_cast<uint16_t>((packed & 0x1F) << 8) |
           static_cast<uint16_t>((packed >> 8) & 0xFF);
    return ERR_OK;
}

//  -- END OF FILE --
