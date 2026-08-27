#include "device_24lc1025.hpp"

// ============================================================
//  构造 & 初始化
// ============================================================

device_24lc1025::device_24lc1025(inter_i2c_bus* bus, uint8_t addr)
    : _bus(bus)
    , _addr(addr)
{
}

device_24lc1025::~device_24lc1025()
{
}

void device_24lc1025::init()
{
    if (!is_connected())
    {
        _error = ERR_CONNECT;
    }
}

bool device_24lc1025::is_connected()
{
    _bus->lock();
    _bus->start();
    _bus->write_byte(_addr << 1);
    bool ack = (_bus->wait_ack(500) != 0);
    _bus->stop();
    _bus->unlock();

    _error = ack ? ERR_OK : ERR_CONNECT;
    return ack;
}

int device_24lc1025::get_last_error()
{
    int e = _error;
    _error = ERR_OK;
    return e;
}

// ============================================================
//  内部：地址 / 事务原语
// ============================================================

// 17 位地址 → 7 位器件地址：
//   低 16 位走两个地址字节；bit16（块选择 B0）进器件地址 bit2
//   块 0（0x00000~0x0FFFF）→ 基础地址；块 1（0x10000~0x1FFFF）→ 基础地址 | 0x04
uint8_t device_24lc1025::_device_addr(uint32_t mem_addr) const
{
    if (mem_addr >= BLOCK_SIZE)
    {
        return static_cast<uint8_t>(_addr | 0x04);
    }
    return _addr;
}

bool device_24lc1025::_begin_write(uint32_t mem_addr)
{
    _bus->start();
    _bus->write_byte(_device_addr(mem_addr) << 1);   // R/W = 0
    if (!_bus->wait_ack()) return false;

    _bus->write_byte((mem_addr >> 8) & 0xFF);
    if (!_bus->wait_ack()) return false;

    _bus->write_byte(mem_addr & 0xFF);
    if (!_bus->wait_ack()) return false;
    return true;
}

bool device_24lc1025::_wait_write_cycle()
{
    // ACK 轮询：连续发 START + 器件地址（基础地址即可，两块的 ACK 属于同一芯片），
    // 直到芯片应答；超时由 _write_timeout_ms 控制
    const uint16_t max_attempts = static_cast<uint16_t>(_write_timeout_ms) * 5;

    for (uint16_t i = 0; i < max_attempts; i++)
    {
        _bus->start();
        _bus->write_byte(_addr << 1);
        if (_bus->wait_ack(100))
        {
            _bus->stop();
            return true;
        }
        _bus->stop();
    }
    return false;
}

int device_24lc1025::_write_chunk(uint32_t addr, const uint8_t* buf, uint16_t len)
{
    _bus->lock();

    if (!_begin_write(addr))
    {
        _bus->stop();
        _bus->unlock();
        _bus->bus_recovery();
        return ERR_I2C;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        _bus->write_byte(buf[i]);
        _bus->wait_ack();
    }

    _bus->stop();
    _bus->unlock();

    // 等待内部写入周期结束
    if (!_wait_write_cycle()) return ERR_I2C;
    return ERR_OK;
}

uint32_t device_24lc1025::_read_chunk(uint32_t addr, uint8_t* buf, uint32_t len)
{
    _bus->lock();

    if (!_begin_write(addr))          // 伪写设置地址指针
    {
        _bus->stop();
        _bus->unlock();
        _bus->bus_recovery();
        return 0;
    }

    _bus->start();                    // 重复 START 进入读模式
    _bus->write_byte((_device_addr(addr) << 1) | 0x01);
    if (!_bus->wait_ack())
    {
        _bus->stop();
        _bus->unlock();
        _bus->bus_recovery();
        return 0;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        buf[i] = _bus->read_byte();
        _bus->write_ack((i == len - 1) ? 1 : 0);   // 末字节 NACK
    }

    _bus->stop();
    _bus->unlock();
    return len;
}

// ============================================================
//  写 API
// ============================================================

int device_24lc1025::write_byte(uint32_t mem_addr, uint8_t value)
{
    return write_block(mem_addr, &value, 1);
}

int device_24lc1025::write_block(uint32_t mem_addr, const uint8_t* buffer, uint32_t length)
{
    if (!buffer || length == 0)
    {
        _error = ERR_ADDR;
        return _error;
    }
    if (mem_addr + length > DEVICE_SIZE)
    {
        _error = ERR_ADDR;
        return _error;
    }

    while (length > 0)
    {
        // 页边界（128 B）与块边界（64 KB）取近者切分
        uint32_t page_boundary  = ((mem_addr / PAGE_SIZE) + 1) * PAGE_SIZE;
        uint32_t block_boundary = (mem_addr < BLOCK_SIZE) ? BLOCK_SIZE : DEVICE_SIZE;
        uint32_t boundary       = (page_boundary < block_boundary) ? page_boundary : block_boundary;

        uint32_t chunk = boundary - mem_addr;
        if (chunk > length) chunk = length;

        int rv = _write_chunk(mem_addr, buffer, chunk);
        if (rv != ERR_OK)
        {
            _error = rv;
            return _error;
        }

        mem_addr += chunk;
        buffer   += chunk;
        length   -= chunk;
    }

    _error = ERR_OK;
    return _error;
}

int device_24lc1025::set_block(uint32_t mem_addr, uint8_t value, uint32_t length)
{
    if (length == 0)
    {
        _error = ERR_ADDR;
        return _error;
    }
    if (mem_addr + length > DEVICE_SIZE)
    {
        _error = ERR_ADDR;
        return _error;
    }

    // 栈缓冲填满整页，按页重复写（避免逐字节写）
    uint8_t buf[PAGE_SIZE];
    for (uint16_t i = 0; i < PAGE_SIZE; i++) buf[i] = value;

    while (length > 0)
    {
        uint32_t page_boundary  = ((mem_addr / PAGE_SIZE) + 1) * PAGE_SIZE;
        uint32_t block_boundary = (mem_addr < BLOCK_SIZE) ? BLOCK_SIZE : DEVICE_SIZE;
        uint32_t boundary       = (page_boundary < block_boundary) ? page_boundary : block_boundary;

        uint32_t chunk = boundary - mem_addr;
        if (chunk > length) chunk = length;

        int rv = _write_chunk(mem_addr, buf, static_cast<uint16_t>(chunk));
        if (rv != ERR_OK)
        {
            _error = rv;
            return _error;
        }

        mem_addr += chunk;
        length   -= chunk;
    }

    _error = ERR_OK;
    return _error;
}

int device_24lc1025::update_byte(uint32_t mem_addr, uint8_t value)
{
    // 写前比较：相同则跳过（省一次写周期）
    if (read_byte(mem_addr) == value)
    {
        _error = ERR_OK;
        return _error;
    }
    return write_byte(mem_addr, value);
}

// ============================================================
//  读 API
// ============================================================

uint8_t device_24lc1025::read_byte(uint32_t mem_addr)
{
    if (mem_addr >= DEVICE_SIZE)
    {
        _error = ERR_ADDR;
        return 0xFF;
    }
    uint8_t value;
    uint32_t n = read_block(mem_addr, &value, 1);
    if (n != 1)
    {
        return 0xFF;   // _error 已由 read_block 设置
    }
    _error = ERR_OK;
    return value;
}

uint32_t device_24lc1025::read_block(uint32_t mem_addr, uint8_t* buffer, uint32_t length)
{
    if (!buffer || length == 0)
    {
        _error = ERR_ADDR;
        return 0;
    }
    if (mem_addr + length > DEVICE_SIZE)
    {
        _error = ERR_ADDR;
        return 0;
    }

    uint32_t rv = 0;
    while (length > 0)
    {
        // 读无页限制，但地址指针在块内回绕，必须在 0x10000 处切分
        uint32_t boundary = (mem_addr < BLOCK_SIZE) ? BLOCK_SIZE : DEVICE_SIZE;
        uint32_t chunk    = boundary - mem_addr;
        if (chunk > length) chunk = length;

        uint32_t n = _read_chunk(mem_addr, buffer, chunk);
        if (n != chunk)
        {
            _error = ERR_I2C;
            return rv + n;
        }

        mem_addr += n;
        buffer   += n;
        length   -= n;
        rv       += n;
    }

    _error = ERR_OK;
    return rv;
}

// ============================================================
//  写 + 回读校验
// ============================================================

bool device_24lc1025::write_byte_verify(uint32_t mem_addr, uint8_t value)
{
    if (write_byte(mem_addr, value) != ERR_OK) return false;
    return (read_byte(mem_addr) == value);
}

bool device_24lc1025::write_block_verify(uint32_t mem_addr, const uint8_t* buffer, uint32_t length)
{
    if (write_block(mem_addr, buffer, length) != ERR_OK) return false;

    // 固定栈缓冲分块回读比对（避免大块一次性回读占栈）
    uint8_t buf[PAGE_SIZE];
    uint32_t offset = 0;
    while (offset < length)
    {
        uint32_t chunk = length - offset;
        if (chunk > PAGE_SIZE) chunk = PAGE_SIZE;

        if (read_block(mem_addr + offset, buf, chunk) != chunk) return false;
        for (uint32_t i = 0; i < chunk; i++)
        {
            if (buf[i] != buffer[offset + i]) return false;
        }
        offset += chunk;
    }
    return true;
}

bool device_24lc1025::set_block_verify(uint32_t mem_addr, uint8_t value, uint32_t length)
{
    if (set_block(mem_addr, value, length) != ERR_OK) return false;

    uint8_t buf[PAGE_SIZE];
    uint32_t offset = 0;
    while (offset < length)
    {
        uint32_t chunk = length - offset;
        if (chunk > PAGE_SIZE) chunk = PAGE_SIZE;

        if (read_block(mem_addr + offset, buf, chunk) != chunk) return false;
        for (uint32_t i = 0; i < chunk; i++)
        {
            if (buf[i] != value) return false;
        }
        offset += chunk;
    }
    return true;
}

// ============================================================
//  写周期等待配置
// ============================================================

void device_24lc1025::set_write_timeout_ms(uint8_t ms)
{
    _write_timeout_ms = (ms == 0) ? 1 : ms;
}
