#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
//  TCA9548A I2C 多路复用器驱动（8 通道 I2C MUX）
// ============================================================
//  来源：Rob Tillaart 的 Arduino 库 "TCA9548"
//  版本：0.3.2（2021-03-16）
//    URL：https://github.com/RobTillaart/TCA9548
//
//  移植说明（对照参考库 0.3.2）：
//    - 只移植核心 API；裁剪了 PCA954x 派生类、reset 引脚
//      （setResetPin/reset）、forced 模式（getChannelMask() 只返回
//      本地缓存，不读芯片）
//    - 寄存器 0x00 为 8 位通道选择寄存器：bit n = 1 选中通道 n，
//      可多通道同时使能（SDA/SCL 并发连接）；selectChannel() 排他
//    - setChannelMask() 总是写芯片，去掉了参考库"掩码未变则跳过"
//      的优化，保证本地缓存与芯片状态一致（init() 可靠关闭全部通道）
//    - 错误处理对齐本项目：getLastError() 返回并清零
//    - 无浮点、无堆分配，I2C 访问仅依赖 inter_i2c_dev
// ============================================================

class TCA9548
{
public:

    // ── I2C 地址（A2 A1 A0 决定）─────────────────────────

    enum Addr : uint8_t
    {
        ADDR_0x70 = 0x70, ADDR_0x71 = 0x71,
        ADDR_0x72 = 0x72, ADDR_0x73 = 0x73,
        ADDR_0x74 = 0x74, ADDR_0x75 = 0x75,
        ADDR_0x76 = 0x76, ADDR_0x77 = 0x77,
    };

    // ── 寄存器地址 ────────────────────────────────────────

    enum Reg : uint8_t
    {
        REG_SELECT = 0x00,   // 通道选择寄存器（bit n = 1 选中通道 n）
    };

    // ── 错误码 ────────────────────────────────────────────

    enum ErrCode : uint8_t
    {
        ERR_NONE = 0,   // 无错误
        ERR_I2C  = 1,   // 总线错误 / 地址 NACK（与 inter_i2c_dev::lastError 一致）
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit TCA9548(inter_i2c_bus* bus, uint8_t addr = ADDR_0x70);

    TCA9548(const TCA9548&) = delete;
    TCA9548& operator=(const TCA9548&) = delete;

    /** @brief 初始化：关闭全部通道（与参考库 begin(0x00) 一致） */
    void init();

    // ============================================================
    //  连接探测
    // ============================================================

    /** @brief 探测多路复用器本身 */
    [[nodiscard]] bool isConnected();

    /** @brief 探测总线上任意地址（不改变当前通道选择） */
    [[nodiscard]] bool isConnected(uint8_t address);

    /** @brief 先选中通道，再探测该地址 */
    [[nodiscard]] bool isConnected(uint8_t address, uint8_t channel);

    /**
     * @brief 在全部通道上逐个探测 address，返回有 ACK 的通道掩码
     * @note  调用后最后探测的通道保持选中；多路复用器掉线时也能部分工作
     */
    uint8_t find(uint8_t address);

    // ============================================================
    //  通道控制（channel = 0..7）
    // ============================================================

    uint8_t channelCount();   // 固定 8

    bool enableChannel(uint8_t channel);    // 非排他使能
    bool disableChannel(uint8_t channel);
    bool selectChannel(uint8_t channel);    // 只使能该通道（排他）
    [[nodiscard]] bool isEnabled(uint8_t channel);
    bool disableAllChannels();

    bool    setChannelMask(uint8_t mask);   // 位掩码，可一次设置多个通道
    uint8_t getChannelMask(); // 本地缓存

    // ============================================================
    //  错误
    // ============================================================

    /** @brief 最近一次操作错误码（读取后自动清零） */
    [[nodiscard]] uint8_t getLastError();

private:

    inter_i2c_bus *_bus;     // 探测任意地址用
    inter_i2c_dev  _dev;
    uint8_t        _mask;    // 缓存当前通道掩码
    uint8_t        _err;     // 0 = 无错误

    static constexpr uint8_t CHANNEL_COUNT = 8;
};

#endif
