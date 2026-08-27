#include "inter_i2c_bus.hpp"
#include "inter_io_ctrl.hpp"
// ============================================================
//  构造函数：通过初始化列表构建 io_ctrl 对象
// ============================================================

inter_i2c_bus::inter_i2c_bus(const I2C_Bus_Info_t &cfg, uint8_t delay_us)
    : _scl(cfg.PORT_SCL, cfg.SCL), _sda(cfg.PORT_SDA, cfg.SDA), _delay_val(delay_us), _busy(0)
{
}

// ============================================================
//  初始化：开漏输出 + 内部上拉
// ============================================================

void inter_i2c_bus::init()
{
    // SCL: 开漏输出，上拉，高速
    _scl.init(mode_out_od, pullup, speed_high);

    // SDA: 开漏输出，上拉，高速
    _sda.init(mode_out_od, pullup, speed_high);

    // 释放总线：SCL = 1, SDA = 1
    _write_scl(1);
    _write_sda(1);
}

void inter_i2c_bus::deinit()
{
    _scl.deinit();
    _sda.deinit();
    _busy = 0;
}

// ============================================================
//  延时
// ============================================================

void inter_i2c_bus::_delay()
{
    delay_us(_delay_val);
}

// ============================================================
//  底层 GPIO 操作
// ============================================================

void inter_i2c_bus::_write_scl(uint8_t BitValue)
{
    if (BitValue)
        _scl.high();
    else
        _scl.low();

    _delay();
}

void inter_i2c_bus::_write_sda(uint8_t BitValue)
{
    if (BitValue)
        _sda.high();
    else
        _sda.low();

    _delay();
}

polarity inter_i2c_bus::_read_sda()
{
    return _sda.read();
}

// ============================================================
//  总线控制
// ============================================================

void inter_i2c_bus::start()
{
    _write_sda(1);
    _write_scl(1);
    _write_sda(0);
    _write_scl(0);
}

void inter_i2c_bus::stop()
{
    _write_sda(0);
    _write_scl(1);
    _write_sda(1);
}

uint8_t inter_i2c_bus::scan(uint8_t *found, uint8_t max_count, uint8_t start_addr, uint8_t end_addr)
{

    if (!found || max_count == 0)
    {
        return 0;
    }
    uint8_t count = 0;
    lock();
    for (uint16_t addr = start_addr; addr <= end_addr && count < max_count; addr++)
    {
        start();
        write_byte((uint8_t)(addr << 1));
        if (wait_ack())
        {
            found[count++] = (uint8_t)addr;
        }
        stop();
    }
    unlock();
    return count;
}

// ============================================================
//  数据传输
// ============================================================

void inter_i2c_bus::write_byte(uint8_t ByteValue)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        _write_sda(ByteValue & (0x80 >> i));
        _write_scl(1);
        _write_scl(0);
    }
}

uint8_t inter_i2c_bus::read_byte()
{
    uint8_t ByteValue = 0;

    // 释放 SDA（开漏模式下写 1 即释放，由上拉电阻拉高）
    _write_sda(1);

    for (uint8_t i = 0; i < 8; i++)
    {
        _write_scl(1);
        if (_read_sda() == Hig)
            ByteValue |= (0x80 >> i);
        _write_scl(0);
    }
    return ByteValue;
}

uint8_t inter_i2c_bus::wait_ack(uint16_t timeout)
{
    // 释放 SDA，等待从机拉低
    _write_sda(1);
    _write_scl(1);

    while (_read_sda() == Hig)
    {
        if (--timeout == 0)
        {
            _write_scl(0);
            return 0; // 超时，无应答
        }
    }

    _write_scl(0);
    return 1; // 收到应答
}

void inter_i2c_bus::write_ack(uint8_t AckValue)
{
    // AckValue: 0 = ACK, 1 = NACK
    _write_sda(AckValue);
    _write_scl(1);
    _write_scl(0);
}

// ============================================================
//  互斥
// ============================================================

void inter_i2c_bus::lock()
{
    // IRQ-safe spinlock：关中断保证原子性
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    while (_busy)
    {
        __set_PRIMASK(mask); // 短暂恢复中断，让 ISR 运行
        __disable_irq();
    }
    _busy = 1;
    __set_PRIMASK(mask);
}

void inter_i2c_bus::unlock()
{
    _busy = 0;
}

uint8_t inter_i2c_bus::is_busy()
{
    return _busy;
}

// ============================================================
//  总线恢复：发送 9 个时钟脉冲 + STOP
// ============================================================

void inter_i2c_bus::bus_recovery()
{
    // 先释放 SDA
    _write_sda(1);

    // 发送 9 个时钟脉冲，让从机释放总线
    for (int i = 0; i < 9; i++)
    {
        _write_scl(1);
        _write_scl(0);
    }

    // 发送 STOP 条件
    stop();
}
