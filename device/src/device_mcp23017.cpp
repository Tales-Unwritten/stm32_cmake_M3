#include "device_mcp23017.hpp"

// ============================================================
//  MCP23017 I2C 16 位 IO 扩展器驱动
//  来源：Rob Tillaart 的 Arduino 库 "MCP23017_RT" v0.9.3
//    URL：https://github.com/RobTillaart/MCP23017_RT
//  移植说明见 device_mcp23017.hpp 头注释
// ============================================================

// ============================================================
//  构造
// ============================================================

MCP23017::MCP23017(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _error(ERR_NONE)
{
}

MCP23017::MCP23017(inter_i2c_bus* bus, uint8_t addr, const Config& cfg)
    : _dev(bus, addr)
    , _cfg(cfg)
    , _error(ERR_NONE)
{
}

// ============================================================
//  连接 / 错误
// ============================================================

bool MCP23017::isConnected()
{
    return _dev.ping();
}

uint8_t MCP23017::getLastError()
{
    uint8_t e = _error;
    _error = ERR_NONE;
    return e;
}

// ============================================================
//  init
// ============================================================

void MCP23017::init()
{
    // 写 IOCON = 0x00：强制 BANK=0（寄存器映射与头文件 enum Reg 一致）
    // 且 SEQOP=0（地址指针自增，16 位 API 连续读/写 A、B 两个字节依赖它）。
    // 参考库假定复位默认值不写 IOCON，这里显式写一次更稳妥；
    // IOCON 在两种 BANK 模式下地址都是 0x0A，因此该写入始终有效。
    (void)writeReg(REG_IOCON, 0x00);

    if (_cfg.pullups)
    {
        (void)writeReg(REG_PUR_A, 0xFF);
        (void)writeReg(REG_PUR_B, 0xFF);
    }
}

// ============================================================
//  低层寄存器访问
// ============================================================

bool MCP23017::writeReg(uint8_t reg, uint8_t value)
{
    if (reg > REG_OLAT_B)
    {
        _error = ERR_REGISTER;
        return false;
    }
    _dev.freedom_write(reg, value, 1);
    _error = (_dev.lastError() == 0) ? ERR_NONE : ERR_I2C;
    return _error == ERR_NONE;
}

uint8_t MCP23017::readReg(uint8_t reg)
{
    if (reg > REG_OLAT_B)
    {
        _error = ERR_REGISTER;
        return 0;
    }
    uint64_t v = 0;
    if (!_dev.freedom_read(reg, &v, 1))
    {
        _error = ERR_I2C;
        return 0;
    }
    _error = ERR_NONE;
    return static_cast<uint8_t>(v);
}

bool MCP23017::writeReg16(uint8_t reg, uint16_t value)
{
    if (reg > REG_OLAT_B)
    {
        _error = ERR_REGISTER;
        return false;
    }
    _dev.write_16bit(reg, value);
    _error = (_dev.lastError() == 0) ? ERR_NONE : ERR_I2C;
    return _error == ERR_NONE;
}

uint16_t MCP23017::readReg16(uint8_t reg)
{
    if (reg > REG_OLAT_B)
    {
        _error = ERR_REGISTER;
        return 0;
    }
    uint16_t v = _dev.read_16bit(reg);
    _error = (_dev.lastError() == 0) ? ERR_NONE : ERR_I2C;
    return v;
}

// ============================================================
//  单引脚 API
// ============================================================

bool MCP23017::pinMode1(uint8_t pin, Mode mode)
{
    if (pin > 15)
    {
        _error = ERR_PIN;
        return false;
    }
    if ((mode != Mode::Input) && (mode != Mode::InputPullup) &&
        (mode != Mode::Output))
    {
        _error = ERR_VALUE;
        return false;
    }

    uint8_t ddrReg = (pin > 7) ? REG_DDR_B : REG_DDR_A;
    uint8_t mask   = static_cast<uint8_t>(1u << (pin & 0x07));

    uint8_t val = readReg(ddrReg);
    if (_error != ERR_NONE) return false;

    // ⚠️ REV D 芯片 GPA7/GPB7 作输入时行为异常（见参考库 README）
    if ((mode == Mode::Input) || (mode == Mode::InputPullup))
    {
        val |= mask;                     // 1 = 输入
    }
    else
    {
        val &= static_cast<uint8_t>(~mask);   // 0 = 输出
    }
    return writeReg(ddrReg, val);
}

bool MCP23017::write1(uint8_t pin, uint8_t value)
{
    if (pin > 15)
    {
        _error = ERR_PIN;
        return false;
    }
    uint8_t gpioReg = (pin > 7) ? REG_GPIO_B : REG_GPIO_A;
    uint8_t mask    = static_cast<uint8_t>(1u << (pin & 0x07));

    uint8_t val = readReg(gpioReg);
    if (_error != ERR_NONE) return false;

    uint8_t pre = val;
    if (value)
    {
        val |= mask;
    }
    else
    {
        val &= static_cast<uint8_t>(~mask);
    }
    // 无变化不写（读-改-写优化，参考库行为）
    if (pre == val) return true;
    return writeReg(gpioReg, val);
}

uint8_t MCP23017::read1(uint8_t pin)
{
    if (pin > 15)
    {
        _error = ERR_PIN;
        return INVALID_READ;
    }
    uint8_t gpioReg = (pin > 7) ? REG_GPIO_B : REG_GPIO_A;

    uint8_t val = readReg(gpioReg);
    if (_error != ERR_NONE) return INVALID_READ;

    return (val & static_cast<uint8_t>(1u << (pin & 0x07))) ? 1 : 0;
}

bool MCP23017::setPolarity(uint8_t pin, bool reversed)
{
    if (pin > 15)
    {
        _error = ERR_PIN;
        return false;
    }
    uint8_t polReg = (pin > 7) ? REG_POL_B : REG_POL_A;
    uint8_t mask   = static_cast<uint8_t>(1u << (pin & 0x07));

    uint8_t val = readReg(polReg);
    if (_error != ERR_NONE) return false;

    if (reversed)
    {
        val |= mask;
    }
    else
    {
        val &= static_cast<uint8_t>(~mask);
    }
    return writeReg(polReg, val);
}

bool MCP23017::getPolarity(uint8_t pin, bool& reversed)
{
    if (pin > 15)
    {
        _error = ERR_PIN;
        return false;
    }
    uint8_t polReg = (pin > 7) ? REG_POL_B : REG_POL_A;

    uint8_t val = readReg(polReg);
    if (_error != ERR_NONE) return false;

    reversed = (val & static_cast<uint8_t>(1u << (pin & 0x07))) != 0;
    return true;
}

bool MCP23017::setPullup(uint8_t pin, bool pullup)
{
    if (pin > 15)
    {
        _error = ERR_PIN;
        return false;
    }
    uint8_t purReg = (pin > 7) ? REG_PUR_B : REG_PUR_A;
    uint8_t mask   = static_cast<uint8_t>(1u << (pin & 0x07));

    uint8_t val = readReg(purReg);
    if (_error != ERR_NONE) return false;

    if (pullup)
    {
        val |= mask;
    }
    else
    {
        val &= static_cast<uint8_t>(~mask);
    }
    return writeReg(purReg, val);
}

bool MCP23017::getPullup(uint8_t pin, bool& pullup)
{
    if (pin > 15)
    {
        _error = ERR_PIN;
        return false;
    }
    uint8_t purReg = (pin > 7) ? REG_PUR_B : REG_PUR_A;

    uint8_t val = readReg(purReg);
    if (_error != ERR_NONE) return false;

    pullup = (val & static_cast<uint8_t>(1u << (pin & 0x07))) != 0;
    return true;
}

// ============================================================
//  8 引脚（端口）API
// ============================================================

bool MCP23017::pinMode8(uint8_t port, uint8_t mask)
{
    if (port > 1)
    {
        _error = ERR_PORT;
        return false;
    }
    return writeReg((port == 0) ? REG_DDR_A : REG_DDR_B, mask);
}

bool MCP23017::write8(uint8_t port, uint8_t value)
{
    if (port > 1)
    {
        _error = ERR_PORT;
        return false;
    }
    return writeReg((port == 0) ? REG_GPIO_A : REG_GPIO_B, value);
}

uint8_t MCP23017::read8(uint8_t port)
{
    if (port > 1)
    {
        _error = ERR_PORT;
        return INVALID_READ;
    }
    return readReg((port == 0) ? REG_GPIO_A : REG_GPIO_B);
}

// ============================================================
//  16 引脚 API
//  位序：高 8 位 = Port A（pin 8..15），低 8 位 = Port B（pin 0..7）
// ============================================================

bool MCP23017::pinMode16(uint16_t mask)
{
    return writeReg16(REG_DDR_A, mask);
}

bool MCP23017::write16(uint16_t value)
{
    return writeReg16(REG_GPIO_A, value);
}

uint16_t MCP23017::read16()
{
    return readReg16(REG_GPIO_A);
}

// ============================================================
//  中断（单引脚）
//  INTCON=1 时与 DEFVAL 比较（RISING/FALLING），INTCON=0 时与
//  上次读到的值比较（CHANGE）。参考库行为。
// ============================================================

bool MCP23017::enableInterrupt(uint8_t pin, IntMode mode)
{
    if (pin > 15)
    {
        _error = ERR_PIN;
        return false;
    }

    uint8_t intconReg  = (pin > 7) ? REG_INTCON_B  : REG_INTCON_A;
    uint8_t defvalReg  = (pin > 7) ? REG_DEFVAL_B  : REG_DEFVAL_A;
    uint8_t gpintenReg = (pin > 7) ? REG_GPINTEN_B : REG_GPINTEN_A;
    uint8_t mask       = static_cast<uint8_t>(1u << (pin & 0x07));

    uint8_t intcon = readReg(intconReg);
    if (_error != ERR_NONE) return false;
    uint8_t preIntcon = intcon;

    if (mode == IntMode::Change)
    {
        // 与上一次读到的值比较
        intcon &= static_cast<uint8_t>(~mask);
    }
    else
    {
        // 与 DEFVAL 比较
        intcon |= mask;

        uint8_t defval = readReg(defvalReg);
        if (_error != ERR_NONE) return false;
        uint8_t preDefval = defval;

        if (mode == IntMode::Rising)
        {
            defval &= static_cast<uint8_t>(~mask);   // 比较 0
        }
        else if (mode == IntMode::Falling)
        {
            defval |= mask;                          // 比较 1
        }
        if (preDefval != defval)
        {
            if (!writeReg(defvalReg, defval)) return false;
        }
    }
    if (preIntcon != intcon)
    {
        if (!writeReg(intconReg, intcon)) return false;
    }

    // 最后使能中断位
    uint8_t gpinten = readReg(gpintenReg);
    if (_error != ERR_NONE) return false;
    uint8_t preGpinten = gpinten;
    gpinten |= mask;
    if (preGpinten != gpinten)
    {
        return writeReg(gpintenReg, gpinten);
    }
    return true;   // 该引脚中断已使能
}

bool MCP23017::disableInterrupt(uint8_t pin)
{
    if (pin > 15)
    {
        _error = ERR_PIN;
        return false;
    }
    uint8_t gpintenReg = (pin > 7) ? REG_GPINTEN_B : REG_GPINTEN_A;
    uint8_t mask       = static_cast<uint8_t>(1u << (pin & 0x07));

    uint8_t val = readReg(gpintenReg);
    if (_error != ERR_NONE) return false;

    uint8_t pre = val;
    val &= static_cast<uint8_t>(~mask);
    if (pre == val) return true;   // 该引脚中断已禁用
    return writeReg(gpintenReg, val);
}
