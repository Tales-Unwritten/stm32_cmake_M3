#include "device_fram.hpp"
#include "systick.h"

// ============================================================
//  构造 & 初始化
// ============================================================

device_fram::device_fram(inter_i2c_bus* bus, uint8_t addr)
    : _bus(bus)
    , _address(addr)
{
}

device_fram::~device_fram()
{
}

void device_fram::init()
{
    if (!is_connected())
    {
        _error = ERR_CONNECT;
        return;
    }
    // 尝试读取器件 ID 推导尺寸（失败不视为错误，可用 set_size_bytes 兜底）
    (void)get_size();
}

bool device_fram::is_connected()
{
    _bus->lock();
    _bus->start();
    _bus->write_byte(_address << 1);
    bool ack = (_bus->wait_ack(500) != 0);
    _bus->stop();
    _bus->unlock();

    _error = ack ? ERR_OK : ERR_CONNECT;
    return ack;
}

int device_fram::get_last_error()
{
    int e = _error;
    _error = ERR_OK;
    return e;
}

// ============================================================
//  基本读写
// ============================================================

int device_fram::write_byte(uint16_t mem_addr, uint8_t value)
{
    return write_block(mem_addr, &value, 1);
}

int device_fram::write_block(uint16_t mem_addr, const uint8_t* buf, uint16_t length)
{
    if (!buf || length == 0)
    {
        _error = ERR_ADDR;
        return _error;
    }
    if ((_size_bytes != 0) && (static_cast<uint32_t>(mem_addr) + length > _size_bytes))
    {
        _error = ERR_ADDR;
        return _error;
    }

    while (length > 0)
    {
        uint16_t chunk = (length > BLOCK_CHUNK) ? BLOCK_CHUNK : length;
        if (!_write_block(mem_addr, buf, chunk))
        {
            return _error;   // _write_block 已置 _error
        }
        mem_addr += chunk;
        buf      += chunk;
        length   -= chunk;
    }

    _error = ERR_OK;
    return _error;
}

int device_fram::read_byte(uint16_t mem_addr, uint8_t& data)
{
    return read_block(mem_addr, &data, 1);
}

int device_fram::read_block(uint16_t mem_addr, uint8_t* buf, uint16_t length)
{
    if (!buf || length == 0)
    {
        _error = ERR_ADDR;
        return _error;
    }
    if ((_size_bytes != 0) && (static_cast<uint32_t>(mem_addr) + length > _size_bytes))
    {
        _error = ERR_ADDR;
        return _error;
    }

    while (length > 0)
    {
        uint16_t chunk = (length > BLOCK_CHUNK) ? BLOCK_CHUNK : length;
        if (!_read_block(mem_addr, buf, chunk))
        {
            return _error;   // _read_block 已置 _error
        }
        mem_addr += chunk;
        buf      += chunk;
        length   -= chunk;
    }

    _error = ERR_OK;
    return _error;
}

int device_fram::update_byte(uint16_t mem_addr, uint8_t value)
{
    uint8_t old = 0;
    if (read_byte(mem_addr, old) != ERR_OK) return _error;
    if (old == value)
    {
        _error = ERR_OK;
        return _error;
    }
    return write_byte(mem_addr, value);
}

// ============================================================
//  类型读写（主机小端序，与参考库一致）
// ============================================================

void device_fram::write8(uint16_t mem_addr, uint8_t value)
{
    (void)write_block(mem_addr, &value, 1);
}

void device_fram::write16(uint16_t mem_addr, uint16_t value)
{
    uint16_t v = value;
    (void)write_block(mem_addr, reinterpret_cast<const uint8_t*>(&v), 2);
}

void device_fram::write32(uint16_t mem_addr, uint32_t value)
{
    uint32_t v = value;
    (void)write_block(mem_addr, reinterpret_cast<const uint8_t*>(&v), 4);
}

void device_fram::write64(uint16_t mem_addr, uint64_t value)
{
    uint64_t v = value;
    (void)write_block(mem_addr, reinterpret_cast<const uint8_t*>(&v), 8);
}

uint8_t device_fram::read8(uint16_t mem_addr)
{
    uint8_t v = 0;
    (void)read_block(mem_addr, &v, 1);
    return v;
}

uint16_t device_fram::read16(uint16_t mem_addr)
{
    uint16_t v = 0;
    (void)read_block(mem_addr, reinterpret_cast<uint8_t*>(&v), 2);
    return v;
}

uint32_t device_fram::read32(uint16_t mem_addr)
{
    uint32_t v = 0;
    (void)read_block(mem_addr, reinterpret_cast<uint8_t*>(&v), 4);
    return v;
}

uint64_t device_fram::read64(uint16_t mem_addr)
{
    uint64_t v = 0;
    (void)read_block(mem_addr, reinterpret_cast<uint8_t*>(&v), 8);
    return v;
}

uint32_t device_fram::clear(uint8_t value)
{
    if (_size_bytes == 0)
    {
        _error = ERR_ADDR;
        return 0;
    }

    uint8_t buf[16];
    for (uint8_t i = 0; i < 16; i++) buf[i] = value;

    uint32_t written = 0;
    uint32_t addr    = 0;
    while (addr < _size_bytes)
    {
        uint32_t chunk = _size_bytes - addr;
        if (chunk > 16) chunk = 16;

        if (!_write_block(static_cast<uint16_t>(addr), buf, static_cast<uint16_t>(chunk)))
        {
            return written;
        }
        addr    += chunk;
        written += chunk;
    }
    _error = ERR_OK;
    return written;
}

// ============================================================
//  器件 ID / 尺寸
// ============================================================

// 器件 ID 事务（参考库 _getMetaData）：
//   START → 0xF8（0x7C<<1）→ ACK → 地址<<1 → ACK → 重复 START →
//   0x79（0x7C<<1 | 1）→ ACK → 读 3 字节 → NACK → STOP
// 返回 24 位：[厂商 12bit][密度 4bit][产品 8bit]
uint32_t device_fram::_get_meta_data()
{
    _bus->lock();
    _bus->start();

    _bus->write_byte(FRAM_SLAVE_ID << 1);
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }
    _bus->write_byte(_address << 1);
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    _bus->start();                        // 重复 START（不带 STOP）
    _bus->write_byte((FRAM_SLAVE_ID << 1) | 0x01);
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    {
        uint32_t value = 0;
        for (uint8_t i = 0; i < 3; i++)
        {
            value = (value << 8) | _bus->read_byte();
            _bus->write_ack((i == 2) ? 1 : 0);   // 末字节 NACK
        }
        _bus->stop();
        _bus->unlock();
        _error = ERR_OK;
        return value;
    }

fail:
    _bus->stop();
    _bus->unlock();
    return 0xFFFFFFFF;
}

uint16_t device_fram::get_manufacturer_id()
{
    uint32_t value = _get_meta_data();
    if (value == 0xFFFFFFFF) return 0;
    return static_cast<uint16_t>((value >> 12) & 0x0FFF);
}

uint16_t device_fram::get_product_id()
{
    uint32_t value = _get_meta_data();
    if (value == 0xFFFFFFFF) return 0;
    return static_cast<uint16_t>(value & 0x0FFF);
}

// 密度码 → 容量（KB），参考库算法：
//   Cypress/Infineon（0x04）：size = 2^密度 × 8 KB（密度 3 = FM24V05 64KB 等）
//   Fujitsu（0x0A）及其他   ：size = 2^密度 × 1 KB（密度 5 = MB85RC256 等）
uint16_t device_fram::get_size()
{
    uint32_t value = _get_meta_data();
    if (value == 0xFFFFFFFF) return 0;

    const uint16_t manufacturer = static_cast<uint16_t>((value >> 12) & 0x0FFF);
    const uint8_t  density      = static_cast<uint8_t>((value >> 8) & 0x0F);

    uint32_t size_kb;
    if (manufacturer == 0x04)
    {
        size_kb = static_cast<uint32_t>(1U << density) * 8U;
    }
    else
    {
        size_kb = static_cast<uint32_t>(1U << density);
    }
    _size_bytes = size_kb * 1024U;
    return static_cast<uint16_t>(size_kb);
}

// ============================================================
//  睡眠 / 唤醒
// ============================================================

// 命令序列（数据手册 P12）：S 0xF8 A 地址 A S 86 A P
bool device_fram::sleep()
{
    _bus->lock();
    _bus->start();

    _bus->write_byte(FRAM_SLAVE_ID << 1);
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }
    _bus->write_byte(_address << 1);
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    _bus->start();                        // 重复 START（不带 STOP）
    _bus->write_byte(FRAM_SLEEP_CMD);     // 睡眠命令字节 0x86
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    _bus->stop();
    _bus->unlock();
    _error = ERR_OK;
    return true;

fail:
    _bus->stop();
    _bus->unlock();
    return false;
}

bool device_fram::wakeup(uint32_t time_recover_us)
{
    // 唤醒 = 普通 ACK 探测；随后等待恢复时间再复测
    bool b = is_connected();
    if (time_recover_us == 0) return b;

    delay_us(time_recover_us);
    return is_connected();
}

// ============================================================
//  单块事务（len ≤ BLOCK_CHUNK）
// ============================================================

bool device_fram::_write_block(uint16_t mem_addr, const uint8_t* obj, uint16_t size)
{
    _bus->lock();
    _bus->start();

    _bus->write_byte(_address << 1);      // R/W = 0
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    _bus->write_byte((mem_addr >> 8) & 0xFF);
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    _bus->write_byte(mem_addr & 0xFF);
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    for (uint16_t i = 0; i < size; i++)
    {
        _bus->write_byte(obj[i]);
        _bus->wait_ack();
    }

    _bus->stop();
    _bus->unlock();
    _error = ERR_OK;
    return true;

fail:
    _bus->stop();
    _bus->unlock();
    _bus->bus_recovery();
    return false;
}

bool device_fram::_read_block(uint16_t mem_addr, uint8_t* obj, uint16_t size)
{
    _bus->lock();
    _bus->start();

    _bus->write_byte(_address << 1);      // R/W = 0
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    _bus->write_byte((mem_addr >> 8) & 0xFF);
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    _bus->write_byte(mem_addr & 0xFF);
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    _bus->start();                        // 重复 START 进入读模式
    _bus->write_byte((_address << 1) | 0x01);
    if (!_bus->wait_ack()) { _error = ERR_I2C; goto fail; }

    for (uint16_t i = 0; i < size; i++)
    {
        obj[i] = _bus->read_byte();
        _bus->write_ack((i == size - 1) ? 1 : 0);   // 末字节 NACK
    }

    _bus->stop();
    _bus->unlock();
    _error = ERR_OK;
    return true;

fail:
    _bus->stop();
    _bus->unlock();
    _bus->bus_recovery();
    return false;
}
