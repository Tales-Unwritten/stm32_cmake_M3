#include "device_tca9548.hpp"

// ============================================================
//  TCA9548A I2C 多路复用器驱动
//  来源：Rob Tillaart 的 Arduino 库 "TCA9548" v0.3.2
//    URL：https://github.com/RobTillaart/TCA9548
//  移植说明见 device_tca9548.hpp 头注释
// ============================================================

// ============================================================
//  构造
// ============================================================

TCA9548::TCA9548(inter_i2c_bus* bus, uint8_t addr)
    : _bus(bus)
    , _dev(bus, addr)
    , _mask(0)
    , _err(ERR_NONE)
{
}

// ============================================================
//  init / 连接探测
// ============================================================

void TCA9548::init()
{
    // 默认关闭全部通道（与参考库 begin(0x00) 一致）
    (void)setChannelMask(0x00);
}

bool TCA9548::isConnected()
{
    return _dev.ping();
}

bool TCA9548::isConnected(uint8_t address)
{
    // 探测任意地址：栈上临时构造 inter_i2c_dev（无堆分配）
    inter_i2c_dev probe(_bus, address);
    return probe.ping();
}

bool TCA9548::isConnected(uint8_t address, uint8_t channel)
{
    if (!selectChannel(channel)) return false;
    return isConnected(address);
}

uint8_t TCA9548::find(uint8_t address)
{
    uint8_t mask = 0x00;
    for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++)
    {
        // 多路复用器掉线时也能部分工作（参考库行为）
        selectChannel(ch);
        if (isConnected(address))
        {
            mask |= static_cast<uint8_t>(1u << ch);
        }
    }
    return mask;   // ⚠️ 调用后最后探测的通道保持选中
}

// ============================================================
//  通道控制
// ============================================================

uint8_t TCA9548::channelCount()
{
    return CHANNEL_COUNT;
}

bool TCA9548::enableChannel(uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return false;
    return setChannelMask(_mask | static_cast<uint8_t>(1u << channel));
}

bool TCA9548::disableChannel(uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return false;
    return setChannelMask(_mask & static_cast<uint8_t>(~(1u << channel)));
}

bool TCA9548::selectChannel(uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return false;
    return setChannelMask(static_cast<uint8_t>(1u << channel));
}

bool TCA9548::isEnabled(uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return false;
    return (_mask & static_cast<uint8_t>(1u << channel)) != 0;
}

bool TCA9548::disableAllChannels()
{
    return setChannelMask(0x00);
}

// ============================================================
//  通道掩码
// ============================================================

bool TCA9548::setChannelMask(uint8_t mask)
{
    // 总是写芯片：参考库有"掩码未变则跳过写"的优化，缓存陈旧时会漏写
    // （例如换线后的残留通道状态）。这里每次都写，保证缓存与芯片一致。
    _mask = mask;
    _dev.freedom_write(REG_SELECT, _mask, 1);
    _err = _dev.lastError();
    return _err == ERR_NONE;
}

uint8_t TCA9548::getChannelMask()
{
    return _mask;   // 本地缓存（裁剪了 forced 读芯片模式）
}

// ============================================================
//  错误
// ============================================================

uint8_t TCA9548::getLastError()
{
    uint8_t e = _err;
    _err = ERR_NONE;
    return e;
}
