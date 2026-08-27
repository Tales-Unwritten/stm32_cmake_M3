#include "device_eeprom.hpp"

// ============================================================================
//  型号参数表（编译时常量，零 RAM 开销）
// ============================================================================

const device_eeprom::_model_info device_eeprom::_model_table[] = {
    //  model,        capacity, page, dual, page_addr, page_bits
    { /* AT24C02 */    256,      8,   false, false,    0 },
    { /* AT24C04 */    512,     16,   false, true,     1 },   // P0 在 I2C 地址 bit1
    { /* AT24C08 */   1024,     16,   false, true,     2 },   // P1P0 在 I2C 地址 bit2-1
    { /* AT24C16 */   2048,     16,   false, true,     3 },   // P2P1P0 在 I2C 地址 bit3-1
    { /* AT24C32 */   4096,     32,   true,  false,    0 },
    { /* AT24C64 */   8192,     32,   true,  false,    0 },
    { /* UNKNOWN */      0,      0,   false, false,    0 },   // fallback
};


// ============================================================================
//  构造 & 初始化
// ============================================================================

device_eeprom::device_eeprom(inter_i2c_bus &bus, const eeprom_config &cfg)
    : _bus(bus)
    , _cfg(cfg)
    , _i2c_base(0)
    , _capacity(0)
    , _page_size(0)
    , _dual_addr(false)
    , _page_addressed(false)
    , _page_bits(0)
    , _initialized(false)
{
    init();
}


void device_eeprom::init()
{
    uint8_t idx = static_cast<uint8_t>(_cfg.model);
    if (idx > 6) idx = 6;

    const _model_info &info = _model_table[idx];

    _capacity        = info.capacity;
    _page_size       = info.page_size;
    _dual_addr       = info.dual_addr;
    _page_addressed  = info.page_addressed;
    _page_bits       = info.page_bits;

    // 计算基础 I2C 7-bit 地址
    // 公式：0x50 | (A2<<3) | (A1<<2) | (A0<<1)
    // 对于页寻址型号（24C04/08/16），页位在计算实际设备地址时才加入
    _i2c_base = 0x50
              | ((_cfg.a2 & 0x01) << 3)
              | ((_cfg.a1 & 0x01) << 2)
              | ((_cfg.a0 & 0x01) << 1);

    // 初始化 WP 引脚（开漏输出，默认高电平 = 写保护）
    if (_cfg.wp_pin && !_cfg.wp_pin->is_initialized())
    {
        _cfg.wp_pin->init(mode_out_pp, nopull, speed_low);
        _cfg.wp_pin->high();  // 默认写保护
    }

    _initialized = true;
}


// ============================================================================
//  L1 — 内部协议层
// ============================================================================


uint8_t device_eeprom::_page_device_addr(uint16_t mem_addr) const
{
    if (!_page_addressed) return _i2c_base;

    // 从内存地址中提取页号
    // 对于 256 字节页：page = mem_addr >> 8
    uint8_t page = (mem_addr >> 8) & ((1 << _page_bits) - 1);

    // 将页号嵌入 I2C 器件地址
    // 页位占据 Bit3~Bit1（原 A2/A1/A0 位置），按从低到高排列
    // page 的 bit0 → I2C addr bit1, bit1 → bit2, bit2 → bit3
    return _i2c_base | ((page & 0x07) << 1);
}


uint8_t device_eeprom::_page_offset(uint16_t mem_addr) const
{
    if (!_page_addressed) return 0xFF;  // 不使用
    return static_cast<uint8_t>(mem_addr & 0xFF);
}


bool device_eeprom::_begin_write(uint16_t mem_addr)
{
    _bus.start();

    if (_page_addressed)
    {
        // 页寻址型号：I2C 地址携带页位，再发页内偏移
        uint8_t dev_addr = _page_device_addr(mem_addr);
        uint8_t offset   = _page_offset(mem_addr);

        _bus.write_byte(dev_addr & 0xFE);  // R/W = 0
        if (!_bus.wait_ack()) return false;

        _bus.write_byte(offset);
        if (!_bus.wait_ack()) return false;
    }
    else if (_dual_addr)
    {
        // 双字节地址型号
        _bus.write_byte(_i2c_base & 0xFE);  // R/W = 0
        if (!_bus.wait_ack()) return false;

        _bus.write_byte((mem_addr >> 8) & 0xFF);
        if (!_bus.wait_ack()) return false;

        _bus.write_byte(mem_addr & 0xFF);
        if (!_bus.wait_ack()) return false;
    }
    else
    {
        // 单字节地址型号（24C02）
        _bus.write_byte(_i2c_base & 0xFE);  // R/W = 0
        if (!_bus.wait_ack()) return false;

        _bus.write_byte(mem_addr & 0xFF);
        if (!_bus.wait_ack()) return false;
    }
    return true;
}


bool device_eeprom::_begin_read(uint16_t mem_addr)
{
    // 先发送"伪写"设置内存地址指针
    if (!_begin_write(mem_addr))
    {
        _bus.stop();
        return false;
    }

    // 重复 START 进入读模式
    _bus.start();

    uint8_t read_addr;
    if (_page_addressed)
    {
        read_addr = _page_device_addr(mem_addr) | 0x01;  // R/W = 1
    }
    else
    {
        read_addr = _i2c_base | 0x01;  // R/W = 1
    }

    _bus.write_byte(read_addr);
    if (!_bus.wait_ack())
    {
        _bus.stop();
        return false;
    }
    return true;
}


bool device_eeprom::_wait_write_cycle()
{
    // ACK 轮询：连续发送 START + 器件地址，直到芯片 ACK
    // 超时由 _cfg.write_timeout_ms 控制
    // 每次尝试约 200μs（取决于 I2C 总线速度），所以循环次数 = timeout_ms * 5

    const uint16_t max_attempts = static_cast<uint16_t>(_cfg.write_timeout_ms) * 5;

    for (uint16_t i = 0; i < max_attempts; i++)
    {
        _bus.start();

        uint8_t dev_addr;
        if (_page_addressed)
            dev_addr = _page_device_addr(0) & 0xFE;  // 任选一页试探
        else
            dev_addr = _i2c_base & 0xFE;

        _bus.write_byte(dev_addr);
        if (_bus.wait_ack(100))
        {
            _bus.stop();
            return true;
        }
        _bus.stop();
    }
    return false;  // 超时
}


bool device_eeprom::_write_paged(uint16_t addr, const uint8_t *buf, uint16_t len)
{
    uint16_t remaining = len;
    uint16_t offset    = 0;

    while (remaining > 0)
    {
        // 计算当前页内剩余空间
        uint16_t page_boundary;
        if (_page_addressed)
            page_boundary = ((addr >> 8) + 1) << 8;
        else
            page_boundary = ((addr / _page_size) + 1) * _page_size;

        uint16_t chunk = page_boundary - addr;
        if (chunk > remaining) chunk = remaining;
        if (chunk > _page_size) chunk = _page_size;

        // 发送当前页的数据
        _bus.lock();

        if (!_begin_write(addr))
        {
            _bus.stop();
            _bus.unlock();
            _bus.bus_recovery();
            return false;
        }

        for (uint16_t i = 0; i < chunk; i++)
        {
            _bus.write_byte(buf[offset + i]);
            if (!_bus.wait_ack())
            {
                _bus.stop();
                _bus.unlock();
                return false;
            }
        }

        _bus.stop();
        _bus.unlock();

        // 等待内部写入完成
        if (!_wait_write_cycle()) return false;

        addr      += chunk;
        offset    += chunk;
        remaining -= chunk;
    }
    return true;
}


// ============================================================================
//  L1 — 公开协议层 API
// ============================================================================


bool device_eeprom::read(uint16_t mem_addr, uint8_t *buf, uint16_t len)
{
    if (!_initialized || !buf || len == 0) return false;
    if (mem_addr + len > _capacity && _capacity > 0) return false;

    _bus.lock();

    if (!_begin_read(mem_addr))
    {
        _bus.unlock();
        _bus.bus_recovery();
        return false;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        buf[i] = _bus.read_byte();

        // 最后一个字节发 NACK，其余发 ACK
        _bus.write_ack((i == len - 1) ? 1 : 0);
    }

    _bus.stop();
    _bus.unlock();
    return true;
}


bool device_eeprom::write(uint16_t mem_addr, const uint8_t *buf, uint16_t len)
{
    if (!_initialized || !buf || len == 0) return false;
    if (mem_addr + len > _capacity && _capacity > 0) return false;

    return _write_paged(mem_addr, buf, len);
}


bool device_eeprom::read_byte(uint16_t mem_addr, uint8_t &data)
{
    return read(mem_addr, &data, 1);
}


bool device_eeprom::write_byte(uint16_t mem_addr, uint8_t data)
{
    return write(mem_addr, &data, 1);
}


bool device_eeprom::fill_byte(uint16_t start, uint8_t value, uint16_t len)
{
    if (!_initialized || len == 0) return false;
    if (start + len > _capacity && _capacity > 0) return false;

    // 使用栈缓冲避免逐字节写入（性能优化）
    // 最大页大小 32 字节（24C32/64），用固定上限避免 VLA
    uint8_t buf[32];
    for (uint8_t i = 0; i < _page_size; i++) buf[i] = value;

    uint16_t remaining = len;
    uint16_t addr      = start;

    while (remaining > 0)
    {
        uint16_t page_boundary;
        if (_page_addressed)
            page_boundary = ((addr >> 8) + 1) << 8;
        else
            page_boundary = ((addr / _page_size) + 1) * _page_size;

        uint16_t chunk = page_boundary - addr;
        if (chunk > remaining) chunk = remaining;
        if (chunk > _page_size) chunk = _page_size;

        if (!_write_paged(addr, buf, chunk)) return false;

        addr      += chunk;
        remaining -= chunk;
    }
    return true;
}


bool device_eeprom::probe()
{
    if (!_initialized) return false;

    _bus.lock();

    _bus.start();
    _bus.write_byte(_i2c_base & 0xFE);
    bool ack = _bus.wait_ack(500);
    _bus.stop();

    _bus.unlock();
    return ack;
}


void device_eeprom::bus_recovery()
{
    _bus.lock();
    _bus.bus_recovery();
    _bus.unlock();
}


// ============================================================================
//  L2 — 应用便利层
// ============================================================================


bool device_eeprom::dump(uint8_t *buf)
{
    if (!_initialized || !buf || _capacity == 0) return false;

    return read_sequential(0, buf, _capacity);
}


bool device_eeprom::write_verified(uint16_t addr, const uint8_t *buf, uint16_t len)
{
    if (!write(addr, buf, len)) return false;

    // 逐字节回读校验
    uint8_t verify_byte;
    for (uint16_t i = 0; i < len; i++)
    {
        if (!read_byte(addr + i, verify_byte)) return false;
        if (verify_byte != buf[i]) return false;
    }
    return true;
}


bool device_eeprom::update_byte(uint16_t mem_addr, uint8_t data)
{
    uint8_t old = 0;
    if (!read_byte(mem_addr, old)) return false;
    if (old == data) return true;   // 相同则跳过写，省一个写周期
    return write_byte(mem_addr, data);
}


uint16_t device_eeprom::update_block(uint16_t mem_addr, const uint8_t *buf, uint16_t len)
{
    if (!_initialized || !buf || len == 0) return 0;
    if (mem_addr + len > _capacity && _capacity > 0) return 0;

    uint16_t written = 0;
    uint16_t remaining = len;
    uint16_t addr = mem_addr;
    const uint8_t *p = buf;

    // 按硬件页粒度处理：读整页旧数据，页内有任一字节变化才整页重写。
    // 页最大 32B（AT24C32/64），64B 栈缓冲留裕量。
    uint8_t page_buf[64];

    while (remaining > 0)
    {
        uint16_t chunk = _page_size - (addr % _page_size);   // 当前页剩余容量
        if (chunk > remaining) chunk = remaining;

        if (!read(addr, page_buf, chunk))
            return written;              // 读失败：返回已写字节数

        bool same = true;
        for (uint16_t i = 0; i < chunk; i++)
        {
            if (page_buf[i] != p[i]) { same = false; break; }
        }

        if (!same)
        {
            if (!write(addr, p, chunk))
                return written;          // 写失败：返回已写字节数
            written += chunk;
        }

        addr += chunk;
        p    += chunk;
        remaining -= chunk;
    }
    return written;
}


bool device_eeprom::is_blank(uint16_t addr, uint16_t len)
{
    if (!_initialized || len == 0) return false;
    if (addr + len > _capacity && _capacity > 0) return false;

    // 分块读取，检查是否全为 0xFF
    uint8_t buf[32];
    uint16_t remaining = len;
    uint16_t offset    = addr;

    while (remaining > 0)
    {
        uint16_t chunk = (remaining > 32) ? 32 : remaining;
        if (!read(offset, buf, chunk)) return false;

        for (uint16_t i = 0; i < chunk; i++)
        {
            if (buf[i] != 0xFF) return false;
        }

        offset    += chunk;
        remaining -= chunk;
    }
    return true;
}


bool device_eeprom::read_sequential(uint16_t start_addr, uint8_t *buf, uint16_t len)
{
    // 与 read() 相同实现：AT24Cxx 的随机读取后本身就是连续读取
    // 区别：此方法明确语义为"利用 auto-increment 的一次连续传输"
    return read(start_addr, buf, len);
}


void device_eeprom::wp_enable()
{
    if (_cfg.wp_pin && _cfg.wp_pin->is_initialized())
        _cfg.wp_pin->high();
}


void device_eeprom::wp_disable()
{
    if (_cfg.wp_pin && _cfg.wp_pin->is_initialized())
        _cfg.wp_pin->low();
}


bool device_eeprom::detect_model(eeprom_model &detected)
{
    if (!_initialized) return false;

    // ── 步骤 1：探测器件存在 ──────────────────────────────────
    if (!probe()) return false;

    // ── 步骤 2：保存地址 0 的数据（非破坏性检测） ──────────────
    uint8_t saved_byte;
    if (!read_byte(0, saved_byte))
    {
        // 如果连地址 0 都读不出来，器件可能异常
        return false;
    }

    // ── 步骤 3：用写入回读测试容量边界 ────────────────────────
    // 算法：在地址 0 写标记值，然后依次在各容量边界写入不同值
    // 如果地址 0 被覆盖 → 说明写入在边界处发生了回绕 → 容量小于该边界

    // 按容量从小到大排序的检测序列
    static const struct {
        uint16_t capacity;
        uint16_t test_addr;   // 边界地址 = capacity
        eeprom_model model;
    } test_sequence[] = {
        {  256,   256, eeprom_model::AT24C02 },
        {  512,   512, eeprom_model::AT24C04 },
        { 1024,  1024, eeprom_model::AT24C08 },
        { 2048,  2048, eeprom_model::AT24C16 },
        { 4096,  4096, eeprom_model::AT24C32 },
        { 8192,  8192, eeprom_model::AT24C64 },
    };

    const uint8_t marker_zero  = 0xA5;  // 地址 0 的标志值
    const uint8_t marker_bound = 0x5A;  // 边界地址的标志值（与 marker_zero 不同）

    detected = eeprom_model::UNKNOWN;

    // 先在地址 0 写入标记
    wp_disable();
    if (!write_byte(0, marker_zero))
    {
        goto restore;
    }

    // 按容量从小到大测试
    for (uint8_t i = 0; i < 6; i++)
    {
        // 在边界地址写入（如果芯片容量 ≥ test_addr，此写入不影响地址 0）
        if (!write_byte(test_sequence[i].test_addr, marker_bound))
        {
            // 写入失败：可能是芯片不支持该地址（容量不足时的回绕地址
            // 可能是有效的地址 0）
            // 检查地址 0 是否被污染
            uint8_t check;
            if (read_byte(0, check) && check != marker_zero)
            {
                // 地址 0 被覆盖 → 发生了回绕 → 容量不足
                detected = test_sequence[i].model;
                goto restore;
            }
            // 如果地址 0 仍是 marker_zero，继续检测
            continue;
        }

        // 回读地址 0 的值
        uint8_t check;
        if (!read_byte(0, check))
        {
            goto restore;
        }

        if (check != marker_zero)
        {
            // 地址 0 的值变了 → 发生回绕 → 容量 = test_sequence[i].capacity
            detected = test_sequence[i].model;
            goto restore;
        }
        // 否则未回绕，继续下一个更大的容量
    }

    // 所有边界均未回绕 → 容量 >= 8192 → 是 24C64（最大的）
    detected = eeprom_model::AT24C64;

restore:
    // ── 步骤 4：恢复地址 0 的原始数据 ──────────────────────────
    write_byte(0, saved_byte);
    _wait_write_cycle();
    wp_enable();

    return detected != eeprom_model::UNKNOWN;
}


// ============================================================================
//  CRC-16-CCITT (polynomial 0x1021, initial 0xFFFF)
//  用于 L3 结构化存取的校验
// ============================================================================

uint16_t device_eeprom::_crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc = (crc << 1);
    }
    return crc;
}


uint16_t device_eeprom::_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
        crc = _crc16_update(crc, data[i]);
    return crc;
}
