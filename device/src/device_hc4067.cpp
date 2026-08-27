#include "device_hc4067.hpp"

// ============================================================
//  来源：Rob Tillaart I2C_HC4067 v0.1.0
//  移植说明见 device_hc4067.hpp 文件头
//
//  字节协议（PCF8574 无寄存器，写入字节直接输出到端口）：
//   输出字节 bit4-0 = [Enable][S3:S0]
//   - setChannel：切换前先 disable（防鬼影通道）→ 写通道 → enable
//   - 单字节写必须走总线原语：freedom_write 最少发 2 字节，
//     第 2 字节会覆盖端口输出
// ============================================================

// ============================================================
//  构造 / 初始化
// ============================================================

HC4067::HC4067(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _bus(bus)
    , _addr(addr)
    , _channel(0)
    , _enable(false)
    , _lastValue(0x10)   // 与首次写入值不同，强制首写
    , _error(0)
{
}

void HC4067::init()
{
    if (!isConnected()) return;
    disable();   // 对齐原版 begin()：上电先禁用，避免不确定通道导通
}

bool HC4067::isConnected()
{
    return _dev.ping();
}

uint8_t HC4067::getAddress()
{
    return _addr;
}

// ============================================================
//  通道 / 使能
// ============================================================

bool HC4067::setChannel(uint8_t channel, bool disable)
{
    if (channel > MAX_CHANNEL) return false;
    if (channel != _channel)
    {
        _channel = channel;
        if (disable) this->disable();   // 防止切换瞬间出现鬼影通道
        (void)_write();
        this->enable();
    }
    return true;
}

uint8_t HC4067::getChannel()
{
    return _channel;
}

void HC4067::enable()
{
    _enable = true;
    (void)_write();
}

void HC4067::disable()
{
    _enable = false;
    (void)_write();
}

bool HC4067::isEnabled()
{
    return _enable;
}

// ============================================================
//  内部
// ============================================================

int HC4067::_write()
{
    uint8_t value = _channel;
    if (_enable) value |= PIN_ENABLE;

    // 与上次一致则跳过（避免无谓的总线写）
    if (_lastValue == value) return _error;
    _lastValue = value;

    // 单字节写：PCF8574 无寄存器，任何字节都直接输出到端口
    _bus->lock();
    _bus->start();
    _bus->write_byte(_addr << 1);            // W
    bool ok = _bus->wait_ack() != 0;
    if (ok)
    {
        _bus->write_byte(value);
        _bus->wait_ack();
    }
    _bus->stop();
    _bus->unlock();

    _error = ok ? 0 : 1;
    return _error;
}
