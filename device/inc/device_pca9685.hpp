#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_io_ctrl.hpp"
#include <cstdint>

// ============================================================
// PCA9685 16 通道 PWM / LED 驱动（12 位，I2C）
//
// 移植自 Rob Tillaart PCA9685_RT v0.7.4
//   URL: https://github.com/RobTillaart/PCA9685_RT
//
// 移植说明：
//  - 通道寄存器（4 字节：ON_L/ON_H/OFF_L/OFF_H）通过
//    inter_i2c_dev::freedom_write/read 一次事务完成（≤8 字节），
//    字节顺序由打包位序控制（ON_L 先发）
//  - 频率 prescale 整数公式与原库一致（"定点化"）：
//      prescale = 25e6/(4096×freq) - 1 ≈ 48828/(freq×8) - 1
//    （×8 保持 0.5 精度；offset 参数用于校准晶振偏差）
//  - OE 引脚：原库用 Arduino pinMode/digitalWrite → 改为绑定
//    io_ctrl 对象（bindOutputEnable，低有效，不绑定则视为使能）
//  - I2C_SoftwareReset 简化为 SMBus 通用软复位（地址 0x00 数据 0x06，
//    原库 method 0；method 1 的 0x03/0xA5/0x5A 序列未移植）
//  - getFrequency() 始终读寄存器返回实际频率（原库 cache 参数省略）
//  - 错误码：setMode1/setPWM 等返回详细错误码（0x00=OK）；
//    getLastError() 返回总线错误（0=OK 1=NACK，对齐 inter_i2c_dev 语义）
// ============================================================

class PCA9685
{
public:
    // ── I2C 地址 ──────────────────────────────────────────

    enum Addr : uint8_t
    {
        ADDR_0x40 = 0x40, ADDR_0x41 = 0x41, ADDR_0x42 = 0x42,
        ADDR_0x43 = 0x43, ADDR_0x44 = 0x44, ADDR_0x45 = 0x45,
        ADDR_0x46 = 0x46, ADDR_0x47 = 0x47,
    };

    // ── 寄存器地址 ────────────────────────────────────────

    enum Reg : uint8_t
    {
        REG_MODE1      = 0x00,
        REG_MODE2      = 0x01,
        REG_ALL_ON_L   = 0xFA,
        REG_ALL_OFF_H  = 0xFD,
        REG_PRE_SCALER = 0xFE,
        // 通道 n：0x06 + 4n
    };

    // ── MODE1 位 ──────────────────────────────────────────

    static constexpr uint8_t MODE1_RESTART  = 0x80;
    static constexpr uint8_t MODE1_EXTCLK   = 0x40;
    static constexpr uint8_t MODE1_AUTOINCR = 0x20;
    static constexpr uint8_t MODE1_SLEEP    = 0x10;
    static constexpr uint8_t MODE1_SUB1     = 0x08;
    static constexpr uint8_t MODE1_SUB2     = 0x04;
    static constexpr uint8_t MODE1_SUB3     = 0x02;
    static constexpr uint8_t MODE1_ALLCALL  = 0x01;

    // ── MODE2 位 ──────────────────────────────────────────

    static constexpr uint8_t MODE2_INVERT    = 0x10;
    static constexpr uint8_t MODE2_ACK       = 0x08;
    static constexpr uint8_t MODE2_TOTEMPOLE = 0x04;
    static constexpr uint8_t MODE2_OUTNE     = 0x03;

    // ── 频率范围（数据手册 P25） ──────────────────────────

    static constexpr uint16_t MIN_FREQ = 24;
    static constexpr uint16_t MAX_FREQ = 1526;

    // ── 错误码（对齐原库） ────────────────────────────────

    enum ErrCode : uint8_t
    {
        ERR_OK      = 0x00,
        ERR_I2C     = 0xFC,
        ERR_MODE    = 0xFD,
        ERR_CHANNEL = 0xFE,
        ERR_ERROR   = 0xFF,
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit PCA9685(inter_i2c_bus* bus, uint8_t addr = ADDR_0x40);

    PCA9685(const PCA9685&) = delete;
    PCA9685& operator=(const PCA9685&) = delete;

    /**
     * @brief 初始化：探测 + 写默认 MODE1/MODE2 掩码
     *        （AUTOINCR|ALLCALL，TOTEMPOLE，同原库 begin 默认值）
     */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool isConnected();
    [[nodiscard]] uint8_t getLastError();
    uint8_t getAddress() const { return _addr; }
    uint8_t channelCount() const { return 16; }

    // ============================================================
    //  配置
    // ============================================================

    /** @brief 同时写 MODE1/MODE2，返回 ErrCode */
    uint8_t configure(uint8_t mode1_mask, uint8_t mode2_mask);
    uint8_t setMode1(uint8_t value);
    uint8_t getMode1();
    uint8_t setMode2(uint8_t value);
    uint8_t getMode2();

    // ============================================================
    //  PWM 频率（24..1526 Hz）
    // ============================================================

    /**
     * @brief 设置全局 PWM 频率
     * @param freq   24..1526 Hz（越界自动钳位）
     * @param offset prescale 修正值（晶振偏差校准，可为负）
     * @note  prescale = 48828/(freq×8) - 1 + offset；
     *        写入时序：MODE1 置 SLEEP → 写 prescale → 恢复 MODE1
     */
    uint8_t setFrequency(uint16_t freq, int8_t offset = 0);

    /** @brief 读 prescale 寄存器返回实际频率（含 offset 修正） */
    uint16_t getFrequency();

    // ============================================================
    //  通道 PWM（12 位；onTime/offTime = 0x1000 表示全开/全关位）
    // ============================================================

    uint8_t setPWM(uint8_t channel, uint16_t onTime, uint16_t offTime);
    uint8_t setPWM(uint8_t channel, uint16_t offTime);   // onTime = 0
    uint8_t getPWM(uint8_t channel, uint16_t* onTime, uint16_t* offTime);

    /** @brief 通道置高/低（mode: 非 0 = 全开，0 = 全关） */
    uint8_t write1(uint8_t channel, uint8_t mode);
    /** @brief 返回 1=全开 0=全关 2=PWM（需 getPWM 看具体值） */
    uint8_t read1(uint8_t channel);

    /** @brief 全部通道输出关闭（ALL_OFF_H 全关位） */
    uint8_t allOFF();

    // ============================================================
    //  子地址 / 广播地址（SMBus 特性）
    // ============================================================

    bool enableSubCall(uint8_t nr);          // nr = 1..3
    bool disableSubCall(uint8_t nr);
    bool isEnabledSubCall(uint8_t nr);
    bool setSubCallAddress(uint8_t nr, uint8_t address);
    uint8_t getSubCallAddress(uint8_t nr);

    bool enableAllCall();
    bool disableAllCall();
    bool isEnabledAllCall();
    bool setAllCallAddress(uint8_t address);
    uint8_t getAllCallAddress();

    // ============================================================
    //  OE 输出使能引脚（低有效）
    // ============================================================

    /**
     * @brief 绑定外部 OE 引脚（io_ctrl，调用方负责 init 为输出）
     * @param pin 已初始化的 io_ctrl 对象；nullptr = 解除绑定（视为常使能）
     */
    void bindOutputEnable(io_ctrl* pin);

    /** @brief 使能/禁用输出（on=true → 引脚拉低） */
    bool setOutputEnable(bool on);

    /** @brief 返回 1=输出使能 0=禁用（未绑定引脚时返回 1） */
    uint8_t getOutputEnable();

    // ============================================================
    //  SMBus 通用软复位（地址 0x00 + 命令 0x06）
    // ============================================================

    void softReset();

private:
    // ── I2C 寄存器读写 ────────────────────────────────────

    uint8_t _writeRegister(uint8_t reg, uint8_t value);
    uint8_t _readRegister(uint8_t reg);
    uint8_t _writeChannel(uint8_t reg, uint16_t on, uint16_t off);
    uint8_t _readChannel(uint8_t reg, uint16_t* on, uint16_t* off);

    inter_i2c_dev _dev;
    uint8_t _addr;
    io_ctrl* _oePin = nullptr;   // 绑定的 OE 引脚
    uint16_t _freq = 200;        // 最近设置的频率（getFrequency 读寄存器为准）
};

#endif
