#include "delay.h"
#include "device_oled.hpp"
#include "device_oledfont.hpp" // 字模数据（内含 F6x8 / F8X16 / F16x16）

//==============================================================================
// 私有辅助
//==============================================================================
namespace
{
/** @brief 无符号 32 位整数转十进制字符串（无前导零），返回字符数 */
uint8_t u32ToStr(char *buf, uint32_t num)
{
    uint8_t writeIdx = 0;
    if (num == 0)
    {
        buf[writeIdx++] = '0';
    }
    else
    {
        while (num > 0)
        {
            buf[writeIdx++] = static_cast<char>('0' + (num % 10u));
            num /= 10u;
        }
    }
    // 原地逆序
    for (uint8_t head = 0, tail = static_cast<uint8_t>(writeIdx - 1); head < tail; ++head, --tail)
    {
        char t = buf[head];
        buf[head] = buf[tail];
        buf[tail] = t;
    }
    buf[writeIdx] = '\0';
    return writeIdx;
}
} // namespace

//==============================================================================
// 构造
//==============================================================================
Oled::Oled(inter_i2c_dev &dev, Controller ctrl) noexcept
    : _dev(dev), _colOffset(static_cast<uint8_t>(ctrl))
{
}

//==============================================================================
// 底层 I2C 原语
//==============================================================================
void Oled::writeCmd(uint8_t cmd)
{
    _dev.freedom_write(kCtrlCmd, static_cast<uint64_t>(cmd), 1);
}

void Oled::writeData(const uint8_t *data, uint16_t len)
{
    constexpr uint8_t kMaxChunk = 8; // freedom_write 单次最多 8 字节

    while (len > 0)
    {
        uint8_t chunk = (len > kMaxChunk) ? kMaxChunk : static_cast<uint8_t>(len);
        // 打包进 uint64_t：按 freedom_write 约定 data[0] 放最高字节（先发），
        // 否则发送顺序会被逐字节反转（字模左右镜像）
        uint64_t packed = 0;
        for (uint8_t i = 0; i < chunk; ++i)
        {
            packed = (packed << 8) | static_cast<uint64_t>(data[i]);
        }
        _dev.freedom_write(kCtrlData, packed, chunk);

        data += chunk;
        len -= chunk;
    }
}

void Oled::setCursor(uint8_t col, uint8_t page)
{
    // 加控制器列偏移：SSD1306 为 0；SH1106(1.3") 可见区从第 2 列开始，需 +2
    const uint8_t c = static_cast<uint8_t>(col + _colOffset);
    writeCmd(static_cast<uint8_t>(0xB0 | (page & 0x07)));       // 页地址
    writeCmd(static_cast<uint8_t>(0x10 | ((c & 0xF0) >> 4)));   // 列高半字节
    writeCmd(static_cast<uint8_t>(c & 0x0F));                   // 列低半字节
}

//==============================================================================
// 初始化
//==============================================================================
void Oled::init()
{
    delay_ms(50); // 上电等待

    // SSD1306 上电初始化序列（命令 + 参数按发送顺序排列）
    static const uint8_t kInitSeq[] = {
        0xAE,       // 关闭显示
        0x20, 0x10, // 内存寻址模式 = 页寻址
        0xB0,       // 页起始地址 = 0
        0xC8,       // COM 输出扫描方向（上下翻转）
        0x00, 0x10, // 列起始地址 = 0
        0x40,       // 显示起始行 = 0
        0x81, 0xFF, // 对比度 = 0xFF
        0xA1,       // 段重映射（左右翻转）
        0xA6,       // 正常显示（非反相）
        0xA8, 0x3F, // 多路复用比 = 1/64
        0xA4,       // 输出跟随 RAM
        0xD3, 0x00, // 显示偏移 = 0
        0xD5, 0xF0, // 时钟分频 / 振荡频率
        0xD9, 0x22, // 预充电周期
        0xDA, 0x12, // COM 引脚硬件配置
        0xDB, 0x20, // VCOMH 取消选择电平
        0x8D, 0x14, // 电荷泵使能
        0xAF,       // 打开显示
    };

    for (uint8_t cmd : kInitSeq)
    {
        writeCmd(cmd);
    }

    delay_ms(100);
}

//==============================================================================
// 清屏与电源控制
//==============================================================================
void Oled::clear()
{
    static const uint8_t kZeros[8] = {0};

    for (uint8_t page = 0; page < 8; ++page)
    {
        setCursor(0, page);
        for (uint8_t i = 0; i < 16; ++i) // 128 / 8 = 16 次
        {
            writeData(kZeros, 8);
        }
    }
    delay_ms(100);
}

void Oled::displayOn()
{
    writeCmd(0x8D); // 电荷泵设置
    writeCmd(0x14); // 使能
    writeCmd(0xAF); // 打开显示
}

void Oled::displayOff()
{
    writeCmd(0x8D);
    writeCmd(0x10); // 禁能电荷泵
    writeCmd(0xAE); // 关闭显示
}

//==============================================================================
// 文字渲染
//==============================================================================
void Oled::drawAscii8x16(uint8_t x, uint8_t y, const char *str)
{
    uint8_t pixelCol = static_cast<uint8_t>(x * 8);
    uint8_t page = static_cast<uint8_t>(y * 2);

    while (*str != '\0')
    {
        if (pixelCol > 120) // 宽度不够则换行
        {
            pixelCol = 0;
            page += 2;
        }

        uint8_t code = static_cast<uint8_t>(*str) - 32;
        if (code > 94)
            code = 0; // 越界保护 → 空格

        // 上半行（8 字节）
        setCursor(pixelCol, page);
        writeData(&F8X16[code * 16], 8);

        // 下半行（8 字节）
        setCursor(pixelCol, static_cast<uint8_t>(page + 1));
        writeData(&F8X16[code * 16 + 8], 8);

        pixelCol = static_cast<uint8_t>(pixelCol + 8);
        ++str;
    }
}

void Oled::drawAscii6x8(uint8_t x, uint8_t y, const char *str)
{
    uint8_t pixelCol = static_cast<uint8_t>(x * 6);
    uint8_t page = y;

    while (*str != '\0')
    {
        if (pixelCol > 120) // 6px × 21 字符 = 126，再放会越过第 127 列
        {
            pixelCol = 0;
            ++page;
        }

        uint8_t code = static_cast<uint8_t>(*str) - 32;
        if (code > 94)
            code = 0;

        setCursor(pixelCol, page);
        writeData(&F6x8[code][0], 6);

        pixelCol = static_cast<uint8_t>(pixelCol + 6);
        ++str;
    }
}

void Oled::print(uint8_t x, uint8_t y, const char *str, Font font)
{
    if (str == nullptr)
        return;

    if (font == Font::F8x16)
        drawAscii8x16(x, y, str);
    else
        drawAscii6x8(x, y, str);
}

void Oled::printInt(uint8_t x, uint8_t y, int32_t num, Font font)
{
    char buf[12]; // 最坏："-2147483648\0"
    uint8_t idx = 0;

    if (num < 0)
    {
        buf[idx++] = '-';
        // 通过无符号取反规避 INT32_MIN 的 UB
        uint32_t absVal = 0u - static_cast<uint32_t>(num);
        idx = static_cast<uint8_t>(idx + u32ToStr(buf + idx, absVal));
    }
    else
    {
        idx = static_cast<uint8_t>(idx + u32ToStr(buf + idx, static_cast<uint32_t>(num)));
    }
    buf[idx] = '\0';

    print(x, y, buf, font);
}

void Oled::printUint(uint8_t x, uint8_t y, uint32_t num, Font font)
{
    char buf[11]; // 最坏："4294967295\0"
    u32ToStr(buf, num);
    print(x, y, buf, font);
}

//==============================================================================
// 图形
//==============================================================================
void Oled::drawGlyph(uint8_t x, uint8_t y, uint8_t glyphIndex)
{
    uint16_t offset = static_cast<uint16_t>(32u * glyphIndex);

    setCursor(x, y);
    writeData(&F16x16[offset], 16);

    setCursor(x, static_cast<uint8_t>(y + 1));
    writeData(&F16x16[offset + 16], 16);
}

void Oled::drawBitmap(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *bitmap)
{
    if (bitmap == nullptr || width == 0 || height == 0)
        return;

    for (uint8_t pageRow = 0; pageRow < height; ++pageRow)
    {
        setCursor(x, static_cast<uint8_t>(y + pageRow));
        writeData(bitmap + static_cast<uint16_t>(pageRow) * width, width);
    }
}
