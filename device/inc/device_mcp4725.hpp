#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_i2c_bus.hpp"
#include <cstdint>

// ============================================================
//  来源：Rob Tillaart MCP4725 v0.4.3（Arduino 库）
//     URL: https://github.com/RobTillaart/MCP4725
//  移植说明：
//   - MCP4725 无寄存器地址概念：写入的任何字节都被当作
//     DAC 数据（fast mode 2 字节 / 寄存器模式 3 字节）
//   - 写事务（2/3 字节）用 _dev.freedom_write()（reg 作首字节）
//   - 读事务必须先「只发地址不发数据」再读，freedom_read 会
//     先发 reg 字节（= 改写 DAC 值），因此读必须直接使用
//     inter_i2c_bus 原语（项目 device_eeprom 同款做法）
//   - 禁止 float：setPercentage 用 0.01% 定点、电压用 mV 定点
//   - EEPROM 写周期等待：ready() 轮询状态字节 bit7，
//     内部等待带超时（delay.hpp），不依赖 millis()
// ============================================================

class MCP4725
{
public:

    // ── I2C 地址（A0-A2 决定 0x60~0x67） ──────────────────

    enum Addr : uint8_t
    {
        ADDR_0x60 = 0x60, ADDR_0x61 = 0x61,
        ADDR_0x62 = 0x62, ADDR_0x63 = 0x63,
        ADDR_0x64 = 0x64, ADDR_0x65 = 0x65,
        ADDR_0x66 = 0x66, ADDR_0x67 = 0x67,
    };

    // ── 命令字节（寄存器模式，数据手册 P19） ──────────────
    //   命令字节 = CMD | (PD 模式 << 1)；写 EEPROM 时 CMD=0x60

    enum Reg : uint8_t
    {
        CMD_DAC       = 0x40,   // 写 DAC（不上电保存）
        CMD_DAC_EEPROM = 0x60,  // 写 DAC + EEPROM（上电保持）
    };

    // ── 掉电模式（PD 位） ─────────────────────────────────

    enum PowerDownMode : uint8_t
    {
        PD_NORMAL   = 0x00,   // 正常输出
        PD_GND_1K   = 0x01,   // 1KΩ 下拉到地
        PD_GND_100K = 0x02,   // 100KΩ 下拉到地
        PD_GND_500K = 0x03,   // 500KΩ 下拉到地
    };

    // ── 常量 ──────────────────────────────────────────────

    static constexpr uint16_t MAX_VALUE = 4095;   // 12-bit 满量程
    static constexpr uint16_t MIDPOINT  = 2048;   // 中点值

    // ── 错误码（对齐原版） ────────────────────────────────

    enum ErrCode : int
    {
        ERR_OK            = 0,
        ERR_VALUE         = -999,   // 值超出范围
        ERR_REG           = -998,   // 命令字节错误
        ERR_NOT_CONNECTED = -997,   // 设备不在线
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit MCP4725(inter_i2c_bus* bus, uint8_t addr = ADDR_0x60);

    MCP4725(const MCP4725&) = delete;
    MCP4725& operator=(const MCP4725&) = delete;

    /** @brief 初始化：探测设备，读回 DAC 与掉电模式缓存 */
    void init();

    /** @brief I2C 探测：仅发地址检查 ACK */
    [[nodiscard]] bool isConnected();

    uint8_t getAddress();

    // ============================================================
    //  写 DAC
    // ============================================================

    /** @brief 快速模式写值 0..4095（不写 EEPROM） */
    int setValue(uint16_t value = 0);

    /** @brief 最近一次成功写入的值（缓存） */
    uint16_t getValue();

    /** @brief 输出百分比，0..10000 = 0.00%..100.00%（定点） */
    int setPercentage(int32_t percentage_x100);
    int32_t getPercentage_x100();

    /** @brief 设置满量程电压（默认 5000 mV） */
    void setMaxVoltage_mV(int32_t maxVolts_mV = 5000);
    int32_t getMaxVoltage_mV();

    /** @brief 按电压写 DAC（mV 定点，线性映射） */
    int setVoltage_mV(int32_t volts_mV);
    /** @brief 当前输出对应的电压（mV，按满量程线性反推） */
    int32_t getVoltage_mV();

    // ============================================================
    //  寄存器模式写 / EEPROM
    // ============================================================

    /**
     * @brief 写 DAC（寄存器模式，3 字节事务）
     * @param value  0..4095
     * @param eeprom true = 同时写入 EEPROM（上电保持）
     * @return ERR_OK / ERR_VALUE；写前自动等待 EEPROM 就绪（超时 100ms）
     */
    int writeDAC(uint16_t value, bool eeprom = false);

    /** @brief EEPROM 写周期是否完成（读状态字节 bit7） */
    [[nodiscard]] bool ready();

    /** @brief 从芯片读回 DAC 值（3 字节读） */
    uint16_t readDAC();

    /** @brief 从芯片读回 EEPROM 值（5 字节读） */
    uint16_t readEEPROM();

    // ============================================================
    //  掉电模式
    // ============================================================

    /** @brief 设置掉电模式（可同时写 EEPROM），用上次 DAC 值重写 */
    int writePowerDownMode(uint8_t pdm, bool eeprom = false);

    /** @brief 读 EEPROM 中的掉电模式 */
    uint8_t readPowerDownModeEEPROM();

    /** @brief 读当前 DAC 掉电模式 */
    uint8_t readPowerDownModeDAC();

    /** @brief 通用调用复位：DAC 恢复为 EEPROM 值（P22，实验性） */
    int powerOnReset();

    /** @brief 通用调用唤醒：清除掉电模式（P22，实验性） */
    int powerOnWakeUp();

private:

    // ── I2C ───────────────────────────────────────────────

    int      _writeFastMode(uint16_t value);              // 2 字节
    int      _writeRegisterMode(uint16_t value, uint8_t cmd);  // 3 字节
    uint8_t  _readBytes(uint8_t* buffer, uint8_t length); // 纯读（不先发数据）
    int      _generalCall(uint8_t gc);                    // 广播地址 0x00
    bool     _waitEepromReady(uint32_t timeout_ms);       // 轮询 ready()

    // ── 成员 ──────────────────────────────────────────────

    inter_i2c_dev  _dev;
    inter_i2c_bus *_bus;
    uint8_t        _addr;

    uint16_t _lastValue;        // 缓存最近成功写入值
    uint8_t  _powerDownMode;    // 当前掉电模式
    int32_t  _maxVoltage_mV;    // 满量程电压（mV 定点）
};

#endif /* __cplusplus */
