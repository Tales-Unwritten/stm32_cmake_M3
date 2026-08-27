#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
//  来源：Rob Tillaart PCF8591 v0.4.2（Arduino 库）
//     URL: https://github.com/RobTillaart/PCF8591
//  移植说明：
//   - PCF8591 无寄存器地址概念：首字节为控制字节
//     （bit6 DAC 使能 | bit5-4 输入模式 | bit2 INCR | bit1-0 通道）
//   - 写 = 控制字节 + DAC 值（2 字节）→ _dev.freedom_write()
//   - 读 = 写控制字节 + 读 2 字节 → _dev.freedom_read()
//     （读回的第一个字节是前一次转换结果，必须丢弃）
//   - 8-bit ADC/DAC，无需浮点换算
//   - 差分/比较器模式读数按 int8_t 符号解释
// ============================================================

class PCF8591
{
public:

    // ── I2C 地址（A0-A2 决定 0x48~0x4F） ──────────────────

    enum Addr : uint8_t
    {
        ADDR_0x48 = 0x48, ADDR_0x49 = 0x49,
        ADDR_0x4A = 0x4A, ADDR_0x4B = 0x4B,
        ADDR_0x4C = 0x4C, ADDR_0x4D = 0x4D,
        ADDR_0x4E = 0x4E, ADDR_0x4F = 0x4F,
    };

    // ── 模拟输入模式（控制字节 bit5-4，数据手册图 4） ─────

    enum Mode : uint8_t
    {
        FOUR_SINGLE_CHANNEL = 0x00,   // 4 路单端（默认）
        THREE_DIFFERENTIAL  = 0x01,   // 3 路差分
        MIXED               = 0x02,   // 混合模式
        TWO_DIFFERENTIAL    = 0x03,   // 2 路差分
    };

    // ── 错误码（对齐原版；底层 I2C 错误另见 lastError） ───

    enum ErrCode : int
    {
        ERR_OK            = 0x00,
        ERR_PIN           = 0x81,   // 引脚错误
        ERR_I2C           = 0x82,   // I2C 通信错误
        ERR_MODE          = 0x83,   // 模式参数非法
        ERR_CHANNEL       = 0x84,   // 通道参数非法
        ERR_ADDRESS       = 0x85,   // 地址参数非法
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit PCF8591(inter_i2c_bus* bus, uint8_t addr = ADDR_0x48);

    PCF8591(const PCF8591&) = delete;
    PCF8591& operator=(const PCF8591&) = delete;

    /** @brief 初始化：探测设备，若在线则写 DAC = 0（对齐原版 begin(0)） */
    void init();

    /** @brief I2C 探测：仅发地址检查 ACK */
    [[nodiscard]] bool isConnected();

    uint8_t getAddress();

    // ============================================================
    //  ADC 部分（8-bit）
    // ============================================================

    /** @brief 读指定通道（0..3），mode 见 Mode 枚举；失败返回上次缓存值 */
    uint8_t read(uint8_t channel, uint8_t mode = FOUR_SINGLE_CHANNEL);

    /** @brief 一次读 4 路（INCR 模式），返回 ERR_OK 或错误码 */
    uint8_t read4();

    /** @brief 最近一次 read4() 的通道值缓存 */
    uint8_t lastRead(uint8_t channel);

    /** @brief 差分读数（int8_t 符号解释），原版为实验性 API */
    int readComparator_01();   // 通道 0, 模式 TWO_DIFFERENTIAL
    int readComparator_23();   // 通道 1, 模式 TWO_DIFFERENTIAL
    int readComparator_03();   // 通道 0, 模式 THREE_DIFFERENTIAL
    int readComparator_13();   // 通道 1, 模式 THREE_DIFFERENTIAL

    // ── INCR（地址自增） ──────────────────────────────────

    void enableINCR();
    void disableINCR();
    [[nodiscard]] bool isINCREnabled();

    // ============================================================
    //  DAC 部分（8-bit）
    // ============================================================

    void enableDAC();
    void disableDAC();
    [[nodiscard]] bool isDACEnabled();

    /** @brief 写 DAC 值，成功返回 true */
    bool write(uint8_t value = 0);

    /** @brief 最近一次成功写入的 DAC 值 */
    uint8_t lastWrite();

    // ============================================================
    //  错误
    // ============================================================

    /** @brief 读取并清零错误码 */
    [[nodiscard]] int lastError();

private:

    inter_i2c_dev _dev;
    uint8_t       _addr;
    uint8_t       _control;   // 控制字节缓存
    uint8_t       _dac;       // 最近成功写入的 DAC 值
    uint8_t       _adc[4];    // 4 通道最近读数
    int           _error;
};

#endif /* __cplusplus */
