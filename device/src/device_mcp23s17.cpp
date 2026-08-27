#include "device_mcp23s17.hpp"

// ============================================================
//  MCP23S17 SPI 16 位 IO 扩展器
//  来源：Rob Tillaart "MCP23S17" v0.8.2
//    URL：https://github.com/RobTillaart/MCP23S17
//  移植说明见 device_mcp23s17.hpp 头注释
// ============================================================

// ============================================================
//  构造 / 连接 / 错误
// ============================================================

MCP23S17::MCP23S17(spi_bus &spi, uint8_t address)
    : _spi(spi)
    , _address(static_cast<uint8_t>((address & 0x07u) << 1))   // 左移 1 位
    , _error(ERR_NONE)
{
}

bool MCP23S17::isConnected()
{
    // SPI 版无 ID 寄存器且无 ACK 机制，无法可靠探测（参考库行为）
    _error = ERR_NONE;
    return true;
}

uint8_t MCP23S17::getLastError()
{
    uint8_t e = _error;
    _error = ERR_NONE;
    return e;
}

// ============================================================
//  begin
// ============================================================

bool MCP23S17::begin(bool pullup)
{
    // 读 IOCON：清 SEQOP（地址自增，16 位 API 依赖）+ 顺带清 BANK
    // （强制 BANK=0，寄存器映射与头文件 enum Reg 一致；参考库只清
    //  SEQOP，此处与 I2C 版 MCP23017 显式写 IOCON=0x00 的意图一致，
    //  但保留 HAEN 等其他位）
    uint8_t reg = readReg(REG_IOCON);
    if (_error != ERR_NONE) return false;
    if (reg & (IOCON_SEQOP | IOCON_BANK))
    {
        reg &= static_cast<uint8_t>(~(IOCON_SEQOP | IOCON_BANK));
        if (!writeReg(REG_IOCON, reg)) return false;
    }

    if (pullup)
    {
        // 全部 16 脚使能内部上拉（参考库 begin 行为）
        if (!writeReg(REG_PUR_A, 0xFF)) return false;
        if (!writeReg(REG_PUR_B, 0xFF)) return false;
    }
    return true;
}

// ============================================================
//  单引脚 API
// ============================================================

bool MCP23S17::pinMode1(uint8_t pin, Mode mode)
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

bool MCP23S17::write1(uint8_t pin, uint8_t value)
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

uint8_t MCP23S17::read1(uint8_t pin)
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

bool MCP23S17::setPolarity(uint8_t pin, bool reversed)
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

bool MCP23S17::getPolarity(uint8_t pin, bool& reversed)
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

bool MCP23S17::setPullup(uint8_t pin, bool pullup)
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

bool MCP23S17::getPullup(uint8_t pin, bool& pullup)
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

bool MCP23S17::pinMode8(uint8_t port, uint8_t mask)
{
    if (port > 1)
    {
        _error = ERR_PORT;
        return false;
    }
    return writeReg((port == 0) ? REG_DDR_A : REG_DDR_B, mask);
}

bool MCP23S17::write8(uint8_t port, uint8_t value)
{
    if (port > 1)
    {
        _error = ERR_PORT;
        return false;
    }
    return writeReg((port == 0) ? REG_GPIO_A : REG_GPIO_B, value);
}

uint8_t MCP23S17::read8(uint8_t port)
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
//  （依赖 begin() 已清 SEQOP，寄存器地址自增连读/写两字节）
// ============================================================

bool MCP23S17::pinMode16(uint16_t mask)
{
    return writeReg16(REG_DDR_A, mask);
}

bool MCP23S17::write16(uint16_t value)
{
    return writeReg16(REG_GPIO_A, value);
}

uint16_t MCP23S17::read16()
{
    return readReg16(REG_GPIO_A);
}

// ============================================================
//  中断（单引脚）
//  INTCON=1 时与 DEFVAL 比较（RISING/FALLING），INTCON=0 时与
//  上次读到的值比较（CHANGE）。参考库行为。
// ============================================================

bool MCP23S17::enableInterrupt(uint8_t pin, IntMode mode)
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

bool MCP23S17::disableInterrupt(uint8_t pin)
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

// ============================================================
//  IOCON 控制
// ============================================================

bool MCP23S17::enableControlRegister(uint8_t mask)
{
    uint8_t reg = readReg(REG_IOCON);
    if (_error != ERR_NONE) return false;
    reg |= mask;
    return writeReg(REG_IOCON, reg);
}

bool MCP23S17::disableControlRegister(uint8_t mask)
{
    uint8_t reg = readReg(REG_IOCON);
    if (_error != ERR_NONE) return false;
    reg &= static_cast<uint8_t>(~mask);
    return writeReg(REG_IOCON, reg);
}

// ============================================================
//  低层寄存器访问
//  单字节：控制字节 + 寄存器地址 + [数据]
//  16 位：地址自增连读/写两字节（MSB = Port A 在前）
// ============================================================

bool MCP23S17::writeReg(uint8_t reg, uint8_t value)
{
    if (reg > REG_OLAT_B)
    {
        _error = ERR_REGISTER;
        return false;
    }
    _spi.cs_select();
    _spi.transfer_byte(OPCODE_WRITE | _address);   // 0x40 | addr<<1 | 0
    _spi.transfer_byte(reg);
    _spi.transfer_byte(value);
    _spi.cs_deselect();
    _error = ERR_NONE;
    return true;
}

uint8_t MCP23S17::readReg(uint8_t reg)
{
    if (reg > REG_OLAT_B)
    {
        _error = ERR_REGISTER;
        return 0;
    }
    _spi.cs_select();
    _spi.transfer_byte(OPCODE_READ | _address);    // 0x41 | addr<<1 | 1
    _spi.transfer_byte(reg);
    uint8_t rv = _spi.transfer_byte(0xFF);
    _spi.cs_deselect();
    _error = ERR_NONE;
    return rv;
}

bool MCP23S17::writeReg16(uint8_t reg, uint16_t value)
{
    if (reg > REG_OLAT_B)
    {
        _error = ERR_REGISTER;
        return false;
    }
    _spi.cs_select();
    _spi.transfer_byte(OPCODE_WRITE | _address);
    _spi.transfer_byte(reg);
    _spi.transfer_byte(static_cast<uint8_t>(value >> 8));   // 高字节 = Port A
    _spi.transfer_byte(static_cast<uint8_t>(value & 0xFF)); // 低字节 = Port B
    _spi.cs_deselect();
    _error = ERR_NONE;
    return true;
}

uint16_t MCP23S17::readReg16(uint8_t reg)
{
    if (reg > REG_OLAT_B)
    {
        _error = ERR_REGISTER;
        return 0;
    }
    _spi.cs_select();
    _spi.transfer_byte(OPCODE_READ | _address);
    _spi.transfer_byte(reg);
    uint16_t regA = _spi.transfer_byte(0xFF);
    uint16_t regB = _spi.transfer_byte(0xFF);
    _spi.cs_deselect();
    _error = ERR_NONE;
    return static_cast<uint16_t>((regA << 8) | regB);
}
