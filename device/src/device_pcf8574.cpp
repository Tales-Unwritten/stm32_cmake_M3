#include "device_pcf8574.hpp"

// ============================================================
//  PCF8574 I2C 8 位 IO 扩展器驱动
//  来源：Rob Tillaart 的 Arduino 库 "PCF8574" v0.4.5
//    URL：https://github.com/RobTillaart/PCF8574
//  移植说明见 device_pcf8574.hpp 头注释
// ============================================================

// ============================================================
//  构造
// ============================================================

PCF8574::PCF8574(inter_i2c_bus* bus, uint8_t addr)
    : _bus(bus)
    , _dev(bus, addr)
    , _addr(addr)
    , _dataIn(0)
    , _dataOut(0xFF)
    , _err(ERR_NONE)
{
}

PCF8574::PCF8574(inter_i2c_bus* bus, uint8_t addr, const Config& cfg)
    : _bus(bus)
    , _dev(bus, addr)
    , _addr(addr)
    , _dataIn(0)
    , _dataOut(0xFF)
    , _err(ERR_NONE)
    , _cfg(cfg)
{
}

// ============================================================
//  init / 连接探测
// ============================================================

void PCF8574::init()
{
    // 输出初始值（准双向口：写 1 的引脚呈高阻，可被外部拉低后读回）
    write8(_cfg.initialValue);
}

bool PCF8574::isConnected()
{
    return _dev.ping();
}

// ============================================================
//  裸字节 IO
//  PCF8574 无寄存器：读写都是对端口字节的直接传输。
//  时序与 inter_i2c_dev.cpp 保持一致（lock / start / ACK 检查 /
//  stop / unlock）。
// ============================================================

uint8_t PCF8574::read8()
{
    uint8_t rx = 0;
    _err = ERR_NONE;

    _bus->lock();
    _bus->start();
    _bus->write_byte((_addr << 1) | 0x01);
    if (!_bus->wait_ack())
    {
        _err = ERR_I2C;
        goto exit;
    }
    rx = _bus->read_byte();
    _bus->write_ack(1);          // NACK：单字节读结束

exit:
    _bus->stop();
    _bus->unlock();

    if (_err == ERR_NONE)
    {
        _dataIn = rx;            // 出错时保留上次缓存值（与参考库一致）
    }
    return _dataIn;
}

void PCF8574::write8(uint8_t value)
{
    _dataOut = value;
    _err = ERR_NONE;

    _bus->lock();
    _bus->start();
    _bus->write_byte(_addr << 1);
    if (!_bus->wait_ack())
    {
        _err = ERR_I2C;
        goto exit;
    }
    _bus->write_byte(_dataOut);
    if (!_bus->wait_ack())
    {
        _err = ERR_I2C;
        goto exit;
    }

exit:
    _bus->stop();
    _bus->unlock();
}

// ============================================================
//  单引脚操作（读-改-写）
// ============================================================

uint8_t PCF8574::read(uint8_t pin)
{
    if (pin > 7)
    {
        _err = ERR_PIN;
        return 0;
    }
    // ⚠️ 准双向口：读前必须已 write(pin, 1)，
    //    否则引脚输出低电平会短路外部输入信号
    (void)read8();
    return (_dataIn & static_cast<uint8_t>(1u << pin)) ? 1 : 0;
}

void PCF8574::write(uint8_t pin, uint8_t value)
{
    if (pin > 7)
    {
        _err = ERR_PIN;
        return;
    }
    if (value)
    {
        _dataOut |= static_cast<uint8_t>(1u << pin);
    }
    else
    {
        _dataOut &= static_cast<uint8_t>(~(1u << pin));
    }
    write8(_dataOut);
}

void PCF8574::toggle(uint8_t pin)
{
    if (pin > 7)
    {
        _err = ERR_PIN;
        return;
    }
    _dataOut ^= static_cast<uint8_t>(1u << pin);
    write8(_dataOut);
}

// ============================================================
//  错误
// ============================================================

uint8_t PCF8574::getLastError()
{
    uint8_t e = _err;
    _err = ERR_NONE;
    return e;
}
