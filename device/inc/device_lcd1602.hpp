#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// LCD1602 / LCD2004 字符屏（PCF8574 I2C 背板，HD44780 4 位模式）
//
// 移植自 Rob Tillaart I2C_LCD v0.2.6
//   URL: https://github.com/RobTillaart/I2C_LCD
//
// 移植说明：
//  - 仅支持标准 PCF8574 背板默认引脚映射
//    （P0=RS P1=RW P2=EN P3=背光 P4..P7=D4..D7），
//    原库 config() 自定义引脚映射未移植
//  - 原库基于 Arduino Print（printf 风格）→ 简化为 sendChar / sendString，
//    特殊字符处理仅保留 '\n'（换行）
//  - 行地址偏移算法与原库一致：奇数行 +0x40，行 2 +列数
//    （兼容 16x2 / 20x4，16x4 / 10x4 亦可用）
//  - setBacklight 不再连带切换显示开关（原库会调用 display()/noDisplay()），
//    只刷新背光位，避免意外关屏副作用
//  - 背光极性固定为正逻辑（位=1 亮），原库极性参数未移植
//  - 上电等待与原库一致：阻塞至系统启动后 100 ms（HD44780 初始化时序要求）
// ============================================================

class I2C_LCD
{
public:
    // ── I2C 地址 ──────────────────────────────────────────

    enum Addr : uint8_t
    {
        ADDR_0x27 = 0x27,   // PCF8574 常用地址
        ADDR_0x3F = 0x3F,   // PCF8574A 常用地址
    };

    // ── 错误码 ────────────────────────────────────────────

    enum ErrCode : uint8_t
    {
        ERR_OK         = 0x00,   // 无错误
        ERR_I2C        = 0x01,   // NACK / 总线错误（对齐 inter_i2c_dev 语义）
        ERR_COLUMN_ROW = 0x81,   // setCursor 行列越界（对齐原库）
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit I2C_LCD(inter_i2c_bus* bus, uint8_t addr = ADDR_0x27,
                     uint8_t cols = 16, uint8_t rows = 2);

    I2C_LCD(const I2C_LCD&) = delete;
    I2C_LCD& operator=(const I2C_LCD&) = delete;

    /**
     * @brief 初始化：上电等待 + HD44780 4 位初始化序列 + 显示开 + 清屏
     * @note  阻塞约 100 ms（上电时序），应在上电后调用一次
     */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    /** @brief I2C 探测（仅发地址检查 ACK） */
    [[nodiscard]] bool isConnected();

    /** @brief 最近一次操作的错误码（0 = 无错误；读取后自动清零） */
    [[nodiscard]] uint8_t getLastError();

    // ============================================================
    //  背光 / 显示
    // ============================================================

    void setBacklight(bool on);   // 只切换背光位，不影响显示开关
    void display();
    void noDisplay();

    // ============================================================
    //  定位 / 光标
    // ============================================================

    void clear();                 // 清屏并回到 (0,0)
    void home();                  // 光标回到 (0,0)
    bool setCursor(uint8_t col, uint8_t row);
    void blink();
    void noBlink();
    void cursor();
    void noCursor();
    void scrollDisplayLeft();
    void scrollDisplayRight();

    // ============================================================
    //  文本输出
    // ============================================================

    void sendChar(char c);                 // '\n' = 换行，其余为字符
    void sendString(const char* s);
    void createChar(uint8_t index, const uint8_t* charmap);   // 0..7 自定义字符
    void center(uint8_t row, const char* message);
    void right(uint8_t col, uint8_t row, const char* message);
    void repeat(char c, uint8_t times);

    // ============================================================
    //  查询
    // ============================================================

    uint8_t getColumn() const { return _position; }
    uint8_t getRow()    const { return _row; }

private:
    // ── 底层：PCF8574 裸字节写（无寄存器地址） ────────────
    void _pcfWrite(uint8_t value);

    // ── HD44780 4 位协议 ──────────────────────────────────
    void _send(uint8_t value, bool dataFlag);   // dataFlag: true=数据 false=命令
    void _write4bits(uint8_t value);

    inter_i2c_dev _dev;          // 探测用（isConnected / lastError）
    inter_i2c_bus* _bus;         // 裸字节写需直接操作总线（PCF8574 无寄存器）
    uint8_t _addr;

    uint8_t _cols;               // 列数（默认 16）
    uint8_t _rows;               // 行数（默认 2）
    uint8_t _displayControl;     // 显示控制寄存器值（bit0=BLINK bit1=CURSOR bit2=DISPLAY）
    uint8_t _backlight;          // 背光位掩码（PCF8574 P3 = 0x08）
    uint8_t _position;           // 当前列（0.._cols-1）
    uint8_t _row;                // 当前行（0.._rows-1）
    uint8_t _error;              // 最近错误码（getLastError 读取后清零）
};

#endif
