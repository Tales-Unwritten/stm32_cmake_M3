#include "device_lcd1602.hpp"
#include "systick.h"

#include <cstring>

// ============================================================
//  HD44780 命令 / 选项
// ============================================================

namespace
{
constexpr uint8_t CMD_CLEARDISPLAY   = 0x01;
constexpr uint8_t CMD_RETURNHOME     = 0x02;
constexpr uint8_t CMD_ENTRYMODESET   = 0x04;
constexpr uint8_t CMD_DISPLAYCONTROL = 0x08;
constexpr uint8_t CMD_CURSORSHIFT    = 0x10;
constexpr uint8_t CMD_FUNCTIONSET    = 0x20;
constexpr uint8_t CMD_SETCGRAMADDR   = 0x40;
constexpr uint8_t CMD_SETDDRAMADDR   = 0x80;

constexpr uint8_t OPT_ENTRYLEFT      = 0x02;
constexpr uint8_t OPT_DISPLAYON      = 0x04;
constexpr uint8_t OPT_CURSORON       = 0x02;
constexpr uint8_t OPT_BLINKON        = 0x01;
constexpr uint8_t OPT_DISPLAYMOVE    = 0x08;
constexpr uint8_t OPT_MOVERIGHT      = 0x04;
constexpr uint8_t OPT_2LINE          = 0x08;

//  PCF8574 引脚映射（标准背板）
constexpr uint8_t PCF_RS = 0x01;   // P0
constexpr uint8_t PCF_RW = 0x02;   // P1（恒低，只写）
constexpr uint8_t PCF_EN = 0x04;   // P2
constexpr uint8_t PCF_BL = 0x08;   // P3 背光
constexpr uint8_t PCF_D4 = 0x10;   // P4
constexpr uint8_t PCF_D5 = 0x20;   // P5
constexpr uint8_t PCF_D6 = 0x40;   // P6
constexpr uint8_t PCF_D7 = 0x80;   // P7
}

// ============================================================
//  构造
// ============================================================

I2C_LCD::I2C_LCD(inter_i2c_bus* bus, uint8_t addr, uint8_t cols, uint8_t rows)
    : _dev(bus, addr)
    , _bus(bus)
    , _addr(addr)
    , _cols(cols)
    , _rows(rows)
    , _displayControl(CMD_DISPLAYCONTROL)
    , _backlight(PCF_BL)
    , _position(0)
    , _row(0)
    , _error(ERR_OK)
{
}

// ============================================================
//  init：上电等待 + 4 位初始化序列（HD44780 数据手册 Figure 24）
// ============================================================

void I2C_LCD::init()
{
    // 上电等待：HD44780 要求 VCC 稳定后 >40 ms 才能接收初始化命令。
    // 与原库一致，阻塞至系统启动后 100 ms；若 init() 在启动后
    // 才被调用（t >= 100）则不额外阻塞。
    uint32_t t = get_tick();
    if (t < 100) delay_ms(100 - t);

    // 全部引脚拉低（含背光关）
    _pcfWrite(0x00);

    // 强制进入 4 位模式（3 次 0x03 + 1 次 0x02）
    _write4bits(0x03);
    delay_ms(5);            // 数据手册 > 4.1 ms
    _write4bits(0x03);
    delay_us(200);          // 数据手册 > 100 us
    _write4bits(0x03);
    delay_us(200);
    _write4bits(0x02);      // 切换 4 位接口
    delay_us(200);

    // 功能设置：4 位 / 2 行 / 5x8 点阵
    _send(CMD_FUNCTIONSET | OPT_2LINE, false);

    // 显示开（含背光位刷新）
    display();

    // 清屏
    clear();

    // 输入模式：光标右移、显示不滚动
    _send(CMD_ENTRYMODESET | OPT_ENTRYLEFT, false);
}

// ============================================================
//  连接 / 错误
// ============================================================

bool I2C_LCD::isConnected()
{
    bool ok = _dev.ping();
    _error = _dev.lastError();
    return ok;
}

uint8_t I2C_LCD::getLastError()
{
    uint8_t e = _error;
    _error = ERR_OK;
    return e;
}

// ============================================================
//  背光 / 显示
// ============================================================

void I2C_LCD::setBacklight(bool on)
{
    // 只更新背光位并强制刷新一次端口字节。
    // （原库会连带切换显示开关，此处改为无副作用版本）
    _backlight = on ? PCF_BL : 0x00;
    _send(_displayControl, false);
}

void I2C_LCD::display()
{
    _displayControl |= OPT_DISPLAYON;
    _send(_displayControl, false);
}

void I2C_LCD::noDisplay()
{
    _displayControl &= static_cast<uint8_t>(~OPT_DISPLAYON);
    _send(_displayControl, false);
}

// ============================================================
//  定位 / 光标
// ============================================================

void I2C_LCD::clear()
{
    _send(CMD_CLEARDISPLAY, false);
    _position = 0;
    _row = 0;
    delay_ms(2);            // 清屏需约 1.52 ms
}

void I2C_LCD::home()
{
    _send(CMD_RETURNHOME, false);
    _position = 0;
    _row = 0;
    delay_us(1600);         // 数据手册 1520 us
}

bool I2C_LCD::setCursor(uint8_t col, uint8_t row)
{
    if ((col >= _cols) || (row >= _rows))
    {
        _error = ERR_COLUMN_ROW;
        return false;
    }

    // 行偏移：奇数行 +0x40，行 2（0b10）另 +列数
    //  16x2: 0x00 0x40 | 20x4: 0x00 0x40 0x14 0x54
    uint8_t offset = 0;
    if (row & 0x01) offset += 0x40;
    if (row & 0x02) offset += _cols;
    offset += col;

    _position = col;
    _row = row;
    _send(CMD_SETDDRAMADDR | offset, false);
    return true;
}

void I2C_LCD::blink()
{
    _displayControl |= OPT_BLINKON;
    _send(_displayControl, false);
}

void I2C_LCD::noBlink()
{
    _displayControl &= static_cast<uint8_t>(~OPT_BLINKON);
    _send(_displayControl, false);
}

void I2C_LCD::cursor()
{
    _displayControl |= OPT_CURSORON;
    _send(_displayControl, false);
}

void I2C_LCD::noCursor()
{
    _displayControl &= static_cast<uint8_t>(~OPT_CURSORON);
    _send(_displayControl, false);
}

void I2C_LCD::scrollDisplayLeft()
{
    _send(CMD_CURSORSHIFT | OPT_DISPLAYMOVE, false);
}

void I2C_LCD::scrollDisplayRight()
{
    _send(CMD_CURSORSHIFT | OPT_DISPLAYMOVE | OPT_MOVERIGHT, false);
}

// ============================================================
//  文本输出
// ============================================================

void I2C_LCD::sendChar(char c)
{
    uint8_t ch = static_cast<uint8_t>(c);

    // '\n' → 下一行行首（原库其余控制字符处理未移植）
    if (ch == '\n')
    {
        if (_row < _rows - 1) (void)setCursor(0, _row + 1);
        return;
    }

    // 行尾溢出保护
    if (_position < _cols)
    {
        _send(ch, true);
        _position++;
    }
}

void I2C_LCD::sendString(const char* s)
{
    if (!s) return;
    while (*s) sendChar(*s++);
}

void I2C_LCD::createChar(uint8_t index, const uint8_t* charmap)
{
    if (!charmap) return;

    _send(CMD_SETCGRAMADDR | ((index & 0x07) << 3), false);
    for (uint8_t i = 0; i < 8; i++) _send(charmap[i], true);

    // CGRAM 写入会移动地址指针，写回 DDRAM 恢复光标位置
    uint8_t offset = 0;
    if (_row & 0x01) offset += 0x40;
    if (_row & 0x02) offset += _cols;
    offset += _position;
    _send(CMD_SETDDRAMADDR | offset, false);
}

void I2C_LCD::center(uint8_t row, const char* message)
{
    if (!message) return;
    uint8_t len = static_cast<uint8_t>(strlen(message));
    (void)setCursor((_cols >= len) ? (_cols - len) / 2 : 0, row);
    sendString(message);
}

void I2C_LCD::right(uint8_t col, uint8_t row, const char* message)
{
    if (!message) return;
    uint8_t len = static_cast<uint8_t>(strlen(message));
    (void)setCursor((col >= len) ? (col - len) : 0, row);
    sendString(message);
}

void I2C_LCD::repeat(char c, uint8_t times)
{
    while (times--) sendChar(c);
}

// ============================================================
//  底层 PCF8574 写（裸字节，无寄存器地址）
//  事务：START → 地址(W) → 数据 → STOP
// ============================================================

void I2C_LCD::_pcfWrite(uint8_t value)
{
    _bus->lock();
    _bus->start();
    _bus->write_byte(_addr << 1);
    if (!_bus->wait_ack())
    {
        _error = ERR_I2C;
        _bus->stop();
        _bus->unlock();
        return;
    }
    _bus->write_byte(value);
    if (!_bus->wait_ack()) _error = ERR_I2C;   // 数据字节 NACK 也记为 I2C 错误
    _bus->stop();
    _bus->unlock();
}

// ============================================================
//  HD44780 4 位协议
// ============================================================

void I2C_LCD::_send(uint8_t value, bool dataFlag)
{
    // 高 4 位 / 低 4 位各一次 EN 脉冲（EN 高 → 数据就绪，低 → 锁存）
    uint8_t base = _backlight;
    if (dataFlag) base |= PCF_RS;

    uint8_t msn = base | (value & 0xF0);
    uint8_t lsn = base | (value << 4);

    _pcfWrite(msn | PCF_EN);
    _pcfWrite(msn);
    _pcfWrite(lsn | PCF_EN);
    _pcfWrite(lsn);
}

void I2C_LCD::_write4bits(uint8_t value)
{
    // 初始化序列专用：只发高 4 位（D4..D7），RS/RW 恒低
    uint8_t cmd = 0;
    if (value & 0x01) cmd |= PCF_D4;
    if (value & 0x02) cmd |= PCF_D5;
    if (value & 0x04) cmd |= PCF_D6;
    if (value & 0x08) cmd |= PCF_D7;

    _pcfWrite(cmd | PCF_EN);
    _pcfWrite(cmd);
}

//  -- END OF FILE --
