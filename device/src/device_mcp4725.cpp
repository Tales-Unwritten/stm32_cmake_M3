#include "device_mcp4725.hpp"
#include "systick.h"

// ============================================================
//  来源：Rob Tillaart MCP4725 v0.4.3
//  移植说明见 device_mcp4725.hpp 文件头
//
//  定点换算（无浮点）：
//   百分比：setPercentage 参数 0..10000 = 0.00%..100.00%
//     value = percentage_x100 × 4095 / 10000（int32 足够）
//   电压：满量程默认 5000 mV，线性映射
//     value = volts_mV × 4095 / maxVoltage_mV
// ============================================================

// ============================================================
//  构造 / 初始化
// ============================================================

MCP4725::MCP4725(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _bus(bus)
    , _addr(addr)
    , _lastValue(0)
    , _powerDownMode(PD_NORMAL)
    , _maxVoltage_mV(5000)
{
}

void MCP4725::init()
{
    if (!isConnected()) return;
    _lastValue = readDAC();          // 同步缓存（含 EEPROM 写周期等待）
    _powerDownMode = readPowerDownModeDAC();
}

bool MCP4725::isConnected()
{
    return _dev.ping();
}

uint8_t MCP4725::getAddress()
{
    return _addr;
}

// ============================================================
//  写 DAC
// ============================================================

int MCP4725::setValue(uint16_t value)
{
    if (value == _lastValue) return ERR_OK;
    if (value > MAX_VALUE)   return ERR_VALUE;
    int rv = _writeFastMode(value);
    if (rv == ERR_OK) _lastValue = value;
    return rv;
}

uint16_t MCP4725::getValue()
{
    return _lastValue;
}

int MCP4725::setPercentage(int32_t percentage_x100)
{
    if ((percentage_x100 > 10000) || (percentage_x100 < 0))
        return ERR_VALUE;
    // value = p% × 4095 / 10000（定点，最近整值，对齐原版 round）
    int32_t v = (percentage_x100 * MAX_VALUE) / 10000;
    return setValue(static_cast<uint16_t>(v));
}

int32_t MCP4725::getPercentage_x100()
{
    return (static_cast<int32_t>(_lastValue) * 10000) / MAX_VALUE;
}

void MCP4725::setMaxVoltage_mV(int32_t maxVolts_mV)
{
    _maxVoltage_mV = maxVolts_mV;
}

int32_t MCP4725::getMaxVoltage_mV()
{
    return _maxVoltage_mV;
}

int MCP4725::setVoltage_mV(int32_t volts_mV)
{
    if (volts_mV < 0) return ERR_VALUE;
    // 达到/超过满量程直接置满（避免除法溢出，且数学等价）
    if (volts_mV >= _maxVoltage_mV) return setValue(MAX_VALUE);
    int32_t v = (volts_mV * MAX_VALUE) / _maxVoltage_mV;
    return setValue(static_cast<uint16_t>(v));
}

int32_t MCP4725::getVoltage_mV()
{
    // 线性反推：value × maxVoltage / 4095（int32 足够）
    return (static_cast<int32_t>(_lastValue) * _maxVoltage_mV) / MAX_VALUE;
}

// ============================================================
//  寄存器模式写 / EEPROM
// ============================================================

int MCP4725::writeDAC(uint16_t value, bool eeprom)
{
    if (value > MAX_VALUE) return ERR_VALUE;
    if (!_waitEepromReady(100)) return ERR_REG;   // EEPROM 忙则失败（超时）
    int rv = _writeRegisterMode(value, eeprom ? CMD_DAC_EEPROM : CMD_DAC);
    if (rv == ERR_OK) _lastValue = value;
    return rv;
}

bool MCP4725::ready()
{
    uint8_t buffer[1];
    // 读状态字节：bit7 = EEPROM 写周期完成（1 = 就绪）
    return (_readBytes(buffer, 1) == 1) && ((buffer[0] & 0x80) != 0);
}

uint16_t MCP4725::readDAC()
{
    if (!_waitEepromReady(100)) return _lastValue;
    uint8_t buffer[3];
    if (_readBytes(buffer, 3) != 3) return _lastValue;
    // 读回 3 字节：b1 = DAC 高 8 位，b2 高 4 位 = DAC 低 4 位
    uint16_t value = static_cast<uint16_t>(buffer[1]) << 4;
    value += static_cast<uint16_t>(buffer[2] >> 4);
    return value;
}

uint16_t MCP4725::readEEPROM()
{
    if (!_waitEepromReady(100)) return _lastValue;
    uint8_t buffer[5];
    if (_readBytes(buffer, 5) != 5) return _lastValue;
    // 读回 5 字节：b3 低 4 位 = EEPROM 高 4 位，b4 = EEPROM 低 8 位
    uint16_t value = static_cast<uint16_t>(buffer[3] & 0x0F) << 8;
    value += buffer[4];
    return value;
}

// ============================================================
//  掉电模式
// ============================================================

int MCP4725::writePowerDownMode(uint8_t pdm, bool eeprom)
{
    _powerDownMode = (pdm & 0x03);   // 仅取 PD 位，低层写入时生效
    return writeDAC(_lastValue, eeprom);
}

uint8_t MCP4725::readPowerDownModeEEPROM()
{
    if (!_waitEepromReady(100)) return _powerDownMode;
    uint8_t buffer[4];
    if (_readBytes(buffer, 4) != 4) return _powerDownMode;
    return (buffer[3] >> 5) & 0x03;
}

uint8_t MCP4725::readPowerDownModeDAC()
{
    uint8_t buffer[1];
    if (_readBytes(buffer, 1) != 1) return _powerDownMode;
    return (buffer[0] >> 1) & 0x03;
}

int MCP4725::powerOnReset()
{
    // 通用调用复位：DAC 恢复为 EEPROM 值（P22，实验性）
    int rv = _generalCall(0x06);
    _lastValue = readDAC();   // 同步缓存
    return rv;
}

int MCP4725::powerOnWakeUp()
{
    // 通用调用唤醒：清除掉电模式（P22，实验性）
    int rv = _generalCall(0x09);
    _powerDownMode = readPowerDownModeDAC();
    return rv;
}

// ============================================================
//  I2C 内部实现
// ============================================================

int MCP4725::_writeFastMode(uint16_t value)
{
    // 快速模式 2 字节（数据手册 P18）：
    //   首字节 = [PD1:PD0][D11:D8]，次字节 = [D7:D0]
    // freedom_write(reg = 首字节, data = 次字节, length = 1)
    uint8_t hi = static_cast<uint8_t>((value >> 8) & 0x0F);
    hi |= static_cast<uint8_t>(_powerDownMode << 4);
    _dev.freedom_write(hi, value & 0xFF, 1);
    return (_dev.lastError() == 0) ? ERR_OK : ERR_REG;
}

int MCP4725::_writeRegisterMode(uint16_t value, uint8_t cmd)
{
    // 寄存器模式 3 字节（数据手册 P19）：
    //   首字节 = CMD | [PD1:PD0] << 1
    //   次字节 = [D11:D4]，末字节 = [D3:D0] << 4
    // freedom_write(reg = 首字节, data = 16 位组合, length = 2)
    uint8_t ctrl = static_cast<uint8_t>(cmd | (_powerDownMode << 1));
    uint16_t payload = static_cast<uint16_t>((value >> 4) << 8);
    payload |= static_cast<uint16_t>((value & 0x0F) << 4);
    _dev.freedom_write(ctrl, payload, 2);
    return (_dev.lastError() == 0) ? ERR_OK : ERR_REG;
}

uint8_t MCP4725::_readBytes(uint8_t* buffer, uint8_t length)
{
    // 纯读事务（数据手册 P20）：先发地址 W（不带数据）再 restart 读。
    // 不能用 freedom_read：它会先发一个 reg 字节（被当作 DAC 数据改写输出）
    uint8_t cnt = 0;

    _bus->lock();
    _bus->start();
    _bus->write_byte(_addr << 1);              // 地址 W（无数据）
    if (_bus->wait_ack() == 0)
    {
        _bus->stop();
        _bus->unlock();
        return 0;
    }
    _bus->stop();

    _bus->start();
    _bus->write_byte((_addr << 1) | 0x01);     // 地址 R
    if (_bus->wait_ack() == 0)
    {
        _bus->stop();
        _bus->unlock();
        return 0;
    }
    while (cnt < length)
    {
        buffer[cnt] = _bus->read_byte();
        _bus->write_ack((cnt == (length - 1)) ? 1 : 0);   // 最后一字节 NACK
        cnt++;
    }
    _bus->stop();
    _bus->unlock();
    return cnt;
}

int MCP4725::_generalCall(uint8_t gc)
{
    // 广播地址 0x00（数据手册 P22）
    _bus->lock();
    _bus->start();
    _bus->write_byte(0x00);
    bool ok = _bus->wait_ack() != 0;
    if (ok)
    {
        _bus->write_byte(gc);
        _bus->wait_ack();
    }
    _bus->stop();
    _bus->unlock();
    return ok ? ERR_OK : ERR_NOT_CONNECTED;
}

bool MCP4725::_waitEepromReady(uint32_t timeout_ms)
{
    // EEPROM 写周期最大 50 ms，轮询状态字节 bit7
    while (timeout_ms-- > 0)
    {
        if (ready()) return true;
        delay_ms(1);
    }
    return false;
}
