
#include "inter_i2c_dev.hpp"

inter_i2c_dev::inter_i2c_dev(inter_i2c_bus *bus, uint8_t addr)
    : _bus(bus), _addr(addr)
{
}

// ============================================================
//  16 位寄存器读写
// ============================================================

void inter_i2c_dev::write_16bit(uint8_t reg, uint16_t data)
{
    _err = 0;

    _bus->lock();

    _bus->start();
    _bus->write_byte(_addr << 1);
    if (_bus->wait_ack())
    {
        _bus->write_byte(reg);
        if (_bus->wait_ack())
        {
            _bus->write_byte((data >> 8) & 0xFF);
            _bus->wait_ack();
            _bus->write_byte(data & 0xFF);
            _bus->wait_ack();
        }
        else
        {
            _err = 1;
        }
    }
    else
    {
        _err = 1;
    }
    _bus->stop();
    _bus->unlock();
}

uint16_t inter_i2c_dev::read_16bit(uint8_t reg)
{
    uint16_t data = 0;
    bool success = false;
    _err = 0;

    _bus->lock();

    _bus->start();
    _bus->write_byte(_addr << 1);
    if (!_bus->wait_ack()) { _err = 1; goto exit; }

    _bus->write_byte(reg);
    if (!_bus->wait_ack()) { _err = 1; goto exit; }

    _bus->start();
    _bus->write_byte((_addr << 1) | 0x01);
    if (!_bus->wait_ack()) { _err = 1; goto exit; }

    {
        // 批量接收：软总线走默认逐字节实现，硬件总线走 F1 EV7 时序；
        // 两者都内含末字节 NACK + STOP
        uint8_t raw[2];
        _bus->read_bytes(raw, 2);
        data = (uint16_t)((raw[0] << 8) | raw[1]);
        success = true;
    }

exit:
    // 成功时 read_bytes 已发出 STOP，失败路径才需要收尾
    if (!success) _bus->stop();
    _bus->unlock();
    return data;
}

// ============================================================
//  自由长度写入（1~8 字节）
// ============================================================

void inter_i2c_dev::freedom_write(uint8_t reg, uint64_t data, uint8_t length)
{
    if (length == 0 || length > 8) return;
    _err = 0;

    _bus->lock();

    _bus->start();
    _bus->write_byte(_addr << 1);
    if (!_bus->wait_ack()) { _err = 1; goto exit; }

    _bus->write_byte(reg);
    if (!_bus->wait_ack()) { _err = 1; goto exit; }

    for (int8_t i = length - 1; i >= 0; i--)
    {
        uint8_t byte = (data >> (8 * i)) & 0xFF;
        _bus->write_byte(byte);
        _bus->wait_ack();
    }

exit:
    _bus->stop();
    _bus->unlock();
}

// ============================================================
//  自由长度读取（1~8 字节）
// ============================================================

bool inter_i2c_dev::freedom_read(uint8_t reg, uint64_t *data, uint8_t length)
{
    if (!data || length == 0 || length > 8) return false;
    *data = 0;
    bool success = false;
    _err = 0;

    _bus->lock();

    _bus->start();
    _bus->write_byte(_addr << 1);
    if (!_bus->wait_ack()) { _err = 1; goto exit; }

    _bus->write_byte(reg);
    if (!_bus->wait_ack()) { _err = 1; goto exit; }

    _bus->start();
    _bus->write_byte((_addr << 1) | 0x01);
    if (!_bus->wait_ack()) { _err = 1; goto exit; }

    {
        // 批量接收：软总线走默认逐字节实现，硬件总线走 F1 EV7 时序
        uint8_t raw[8];
        _bus->read_bytes(raw, length);
        for (uint8_t i = 0; i < length; i++)
            *data = (*data << 8) | raw[i];
        success = true;
    }

exit:
    // 成功时 read_bytes 已发出 STOP，失败路径才需要收尾
    if (!success) _bus->stop();
    _bus->unlock();
    return success;
}

uint8_t inter_i2c_dev::lastError()
{
    return _err;
}

// ============================================================
//  设备探测：只发地址字节，检查 ACK
// ============================================================

bool inter_i2c_dev::ping()
{
    _err = 0;

    _bus->lock();
    _bus->start();
    _bus->write_byte(_addr << 1);
    if (!_bus->wait_ack())
    {
        _err = 1;
    }
    _bus->stop();
    _bus->unlock();

    return _err == 0;
}

// ============================================================
//  SMBus 通用软复位：地址 0x00 + 命令 0x06
// ============================================================

void inter_i2c_dev::softReset()
{
    // 写寄存器 0x00、数据 0x06、长度 1 字节，即 SMBus Reset 帧
    freedom_write(0x00, 0x06, 1);
}
