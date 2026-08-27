#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_i2c_bus.hpp"
#include <cstdint>

// ============================================================
//  来源：Rob Tillaart I2C_HC4067 v0.1.0（Arduino 库，header-only）
//     URL: https://github.com/RobTillaart/I2C_HC4067
//  移植说明：
//   - 通过 PCF8574（I2C 8 位 IO 扩展器）控制 CD74HC4067
//     16 通道多路复用器，接线（README）：
//       PCF8574 P0..P3 → HC4067 S0..S3（通道选择）
//       PCF8574 P4    → HC4067 使能引脚 E
//       PCF8574 P5..P7 空闲
//   - PCF8574 无寄存器概念：写入的任何字节直接输出到端口，
//     单字节写必须使用 inter_i2c_bus 原语（freedom_write
//     最少发 2 字节，第 2 字节会覆盖端口），_dev 仅用于 ping
//   - 端口类 API（setChannel/enable/disable/isEnabled）
//     与原版保持一致
// ============================================================

class HC4067
{
public:

    // ── PCF8574 I2C 地址（A0-A2 决定 0x20~0x27） ──────────

    enum Addr : uint8_t
    {
        ADDR_0x20 = 0x20, ADDR_0x21 = 0x21,
        ADDR_0x22 = 0x22, ADDR_0x23 = 0x23,
        ADDR_0x24 = 0x24, ADDR_0x25 = 0x25,
        ADDR_0x26 = 0x26, ADDR_0x27 = 0x27,
    };

    // ── 端口位定义（PCF8574 输出字节） ────────────────────

    static constexpr uint8_t MAX_CHANNEL = 15;   // 通道 0..15
    static constexpr uint8_t PIN_ENABLE   = 0x10;  // P4 → HC4067 使能引脚

    // ============================================================
    //  构造
    // ============================================================

    explicit HC4067(inter_i2c_bus* bus, uint8_t addr = ADDR_0x20);

    HC4067(const HC4067&) = delete;
    HC4067& operator=(const HC4067&) = delete;

    /** @brief 初始化：探测设备，若在线则 disable()（对齐原版 begin） */
    void init();

    /** @brief I2C 探测：仅发地址检查 ACK */
    [[nodiscard]] bool isConnected();

    uint8_t getAddress();

    // ============================================================
    //  通道 / 使能（端口类 API，与原版一致）
    // ============================================================

    /**
     * @brief 选择通道 0..15（非法返回 false）
     * @param disable 切换前先 disable 防鬼影通道（默认开启）
     * @note  调用后设备恢复使能；通道未变化时直接返回 true
     */
    bool setChannel(uint8_t channel, bool disable = true);

    /** @brief 当前通道（0..15，禁用状态下仍保留） */
    uint8_t getChannel();

    /** @brief 使能多路复用（输出通道有效） */
    void enable();

    /** @brief 禁用多路复用（所有通道断开） */
    void disable();

    [[nodiscard]] bool isEnabled();

private:

    /** @brief 写 1 字节到 PCF8574（value = 通道 | 使能位），返回错误码 */
    int _write();

    inter_i2c_dev  _dev;
    inter_i2c_bus *_bus;
    uint8_t        _addr;

    uint8_t  _channel;    // 当前通道 0..15
    bool     _enable;     // 使能状态缓存
    uint8_t  _lastValue;  // 最近写入值缓存（避免重复写）
    int      _error;
};

#endif /* __cplusplus */
