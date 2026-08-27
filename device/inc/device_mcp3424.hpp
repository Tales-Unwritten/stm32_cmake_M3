#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_i2c_bus.hpp"
#include <cstdint>

// ============================================================
//  来源：Rob Tillaart MCP3424 v0.2.0（Arduino 库）
//     URL: https://github.com/RobTillaart/MCP3424
//  移植说明：
//   - 仅移植 MCP3424（4 通道、最高 18-bit）；MCP3421/22/23/25~28
//     派生型号可用 getMaxChannels() 约束通道数模拟
//   - MCP3424 无寄存器地址概念：I2C 写入的任何字节都被当作
//     配置字节，读取则是「纯读」3/4 字节（最后 1 字节 = 状态）。
//     因此配置写与数据读均直接使用 inter_i2c_bus 原语
//     （项目 device_eeprom 同款做法），_dev 仅用于 ping
//   - 禁止 float：readVolts/MilliVolts/MicroVolts 改为定点
//     uV/mV 返回（LSB = 15.625µV × 2^(18-bits)，见 cpp 推导）
//   - 无 millis()：转换延时/查询由调用方轮询，无阻塞等待
// ============================================================

class MCP3424
{
public:

    // ── I2C 地址（A1:A0 决定 0x68~0x6F） ──────────────────

    enum Addr : uint8_t
    {
        ADDR_0x68 = 0x68, ADDR_0x69 = 0x69,
        ADDR_0x6A = 0x6A, ADDR_0x6B = 0x6B,
        ADDR_0x6C = 0x6C, ADDR_0x6D = 0x6D,
        ADDR_0x6E = 0x6E, ADDR_0x6F = 0x6F,
    };

    // ── 配置字节位（唯一可写寄存器，无地址，直接写） ──────
    //   bit7    RDY：单次模式写 1 = 启动转换；读回 0 = 转换完成
    //   bit6-5  通道 C1:C0（0..3）
    //   bit4    模式 1 = 连续（默认），0 = 单次
    //   bit3-2  分辨率 00=12bit 01=14bit 10=16bit 11=18bit
    //   bit1-0  增益 00=x1 01=x2 10=x4 11=x8

    enum class Resolution : uint8_t
    {
        Bits12 = 12, Bits14 = 14, Bits16 = 16, Bits18 = 18,
    };

    enum class Gain : uint8_t
    {
        X1 = 1, X2 = 2, X4 = 4, X8 = 8,
    };

    enum class Mode : uint8_t
    {
        Continuous = 0,   // 连续转换（默认）
        SingleShot = 1,   // 单次转换
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit MCP3424(inter_i2c_bus* bus, uint8_t addr = ADDR_0x68);

    MCP3424(const MCP3424&) = delete;
    MCP3424& operator=(const MCP3424&) = delete;

    /** @brief 初始化：探测设备，若在线则写入当前配置（默认连续/12bit/x1/ch0） */
    void init();

    /** @brief I2C 探测：仅发地址检查 ACK */
    [[nodiscard]] bool isConnected();

    uint8_t getAddress();
    uint8_t getMaxChannels();   // 4（MCP3424）

    /** @brief 最近一次总线操作错误码（0 = 无错误） */
    [[nodiscard]] uint8_t lastError();

    // ============================================================
    //  数据读取
    // ============================================================

    /** @brief 读转换结果（有符号，按当前分辨率符号扩展） */
    int32_t read();

    /** @brief 单次模式：触发一次转换（等同 setSingleShotMode） */
    void requestSingleShot();

    /**
     * @brief 查询转换是否完成（读回状态字节 RDY == 0）
     * @note  本函数会执行一次读事务并更新 _config 缓存
     */
    [[nodiscard]] bool isReady();

    /** @brief 读原始值并换算为 µV（定点，见 cpp 推导） */
    int32_t readVoltage_uV();

    /** @brief 读原始值并换算为 mV（定点） */
    int32_t readVoltage_mV();

    // ============================================================
    //  配置
    // ============================================================

    /** @brief 选择通道 0..3（非法返回 false） */
    bool setChannel(uint8_t channel = 0);
    uint8_t getChannel();

    /** @brief 设置增益 x1/x2/x4/x8（非法返回 false） */
    bool setGain(Gain gain);
    uint8_t getGain();

    /** @brief 设置分辨率 12/14/16/18 bit（非法返回 false） */
    bool setResolution(Resolution bits);
    uint8_t getResolution();

    /** @brief 当前分辨率的典型转换时间（ms）：{5,17,67,267} */
    uint16_t getConversionDelay();

    /** @brief 连续模式（默认） */
    void setContinuousMode();
    /** @brief 单次模式（同时触发一次转换） */
    void setSingleShotMode();
    /** @brief 0 = 连续，1 = 单次 */
    uint8_t getMode();

private:

    // ── I2C 原语（MCP3424 无寄存器地址） ──────────────────

    bool    writeConfig();   // 写 1 字节配置
    int32_t readRaw();       // 纯读 3/4 字节（最后 1 字节 = 状态）

    // ── 符号扩展 ──────────────────────────────────────────

    int32_t signExtend(int32_t rv, uint8_t bits) const;

    // ── 成员 ───────────────────────────────────────────────

    inter_i2c_dev  _dev;
    inter_i2c_bus *_bus;
    uint8_t        _addr;

    uint8_t  _channel;   // 0..3
    uint8_t  _gain;      // 1/2/4/8
    uint8_t  _bits;      // 12/14/16/18
    uint8_t  _config;    // 配置字节（含 RDY 状态位）
    int32_t  _raw;       // 最近一次读到的原始值
};

#endif /* __cplusplus */
