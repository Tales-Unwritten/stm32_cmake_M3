#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// AS5600 磁角度编码器（12 位，0.0879°/LSB）
//
// 移植自 Rob Tillaart AS5600 v0.6.7
//   URL: https://github.com/RobTillaart/AS5600
//
// 移植说明：
//  - 无 FPU：角度 0.01° 定点，raw × 879 / 100（int32）
//    （= 36000/4096 的近似，全量程误差 < 0.01°）
//  - 方向引脚（SW DIRECTION）未移植 → 不做方向反转
//  - PWM 输出相关（setOutputMode / setPWMFrequency）未移植（任务约定）
//  - ZPOS/MPOS 只读不写（OTP 参数，写需烧录操作，风险自负）
//  - 角速度原库用 micros()，本驱动用 get_tick（1 ms 分辨率），
//    两次测量间隔须 < 180° 旋转（原库同样假设）
//  - 错误处理简化为 inter_i2c_dev 语义（getLastError：0=OK 1=NACK）
//  - 累计位置计数（整数运算，无漂移补偿）保留
// ============================================================

class AS5600
{
public:
    // ── I2C 地址 ──────────────────────────────────────────

    enum Addr : uint8_t
    {
        ADDR_0x36 = 0x36,   // AS5600 固定地址
        ADDR_0x40 = 0x40,   // AS5600L 默认地址
    };

    // ── 寄存器地址 ────────────────────────────────────────

    enum Reg : uint8_t
    {
        REG_ZMCO      = 0x00,   // 烧录次数计数（只读）
        REG_ZPOS      = 0x01,   // 零位（2 字节，只读）
        REG_MPOS      = 0x03,   // 最大位置（2 字节，只读）
        REG_MANG      = 0x05,   // 最大角度（2 字节）
        REG_CONF      = 0x07,   // 配置（2 字节）
        REG_STATUS    = 0x0B,   // 状态（只读）
        REG_RAW_ANGLE = 0x0C,   // 原始角度（2 字节，12 位）
        REG_ANGLE     = 0x0E,   // 角度（2 字节，12 位，含 ZPOS 补偿）
        REG_AGC       = 0x1A,   // 自动增益（只读）
        REG_MAGNITUDE = 0x1B,   // 磁通量幅度（2 字节，只读）
        REG_BURN      = 0xFF,   // 电源重启命令
    };

    // ── 状态位（0x0B） ────────────────────────────────────

    static constexpr uint8_t STATUS_MAGNET_HIGH   = 0x08;   // 磁场过强
    static constexpr uint8_t STATUS_MAGNET_LOW    = 0x10;   // 磁场过弱
    static constexpr uint8_t STATUS_MAGNET_DETECT = 0x20;   // 检测到磁铁

    // ============================================================
    //  构造
    // ============================================================

    explicit AS5600(inter_i2c_bus* bus, uint8_t addr = ADDR_0x36);

    AS5600(const AS5600&) = delete;
    AS5600& operator=(const AS5600&) = delete;

    /**
     * @brief 初始化：只做 I2C 探测
     * @note  本驱动默认不写任何配置/OTP 寄存器；如需调整
     *        CONF 位段请用 setPowerMode 等显式调用
     */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool isConnected();
    [[nodiscard]] uint8_t getLastError();

    // ============================================================
    //  角度
    // ============================================================

    /** @brief 原始角度 12 位（0x0C，不含 ZPOS 补偿） */
    uint16_t rawAngle();

    /** @brief 角度 12 位（0x0E，含 ZPOS 补偿 + 软件偏移） */
    uint16_t readAngle();

    /** @brief 角度 0.01° 定点 = readAngle() × 879 / 100 */
    int32_t getAngle_c01();

    // ============================================================
    //  软件偏移（0.01° 定点，绝对值 ≤ 36000）
    // ============================================================

    bool setOffset(int32_t degrees_c01);
    int32_t getOffset();
    bool increaseOffset(int32_t degrees_c01);

    // ============================================================
    //  状态寄存器
    // ============================================================

    uint8_t readStatus();
    uint8_t readAGC();
    uint16_t readMagnitude();
    [[nodiscard]] bool magnetDetected();
    [[nodiscard]] bool magnetTooStrong();
    [[nodiscard]] bool magnetTooWeak();

    // ============================================================
    //  配置寄存器（ZPOS/MPOS 只读；CONF 位段可写）
    // ============================================================

    uint8_t getZMCO();
    uint16_t getZPosition();      // 只读
    uint16_t getMPosition();      // 只读
    uint16_t getMaxAngle();
    uint16_t getConfiguration();
    bool setConfiguration(uint16_t value);    // 0..0x3FFF

    bool setPowerMode(uint8_t mode);          // 0=正常 1..3=低功耗
    uint8_t getPowerMode();
    bool setHysteresis(uint8_t hyst);         // 0..3
    uint8_t getHysteresis();
    bool setSlowFilter(uint8_t mask);         // 0..3（16x/8x/4x/2x）
    uint8_t getSlowFilter();
    bool setFastFilter(uint8_t mask);         // 0..7（无/LSB6/7/9/18/21/24/10）
    uint8_t getFastFilter();
    bool setWatchDog(uint8_t on);             // 0=关 1=开（自动低功耗）
    uint8_t getWatchDog();

    // ============================================================
    //  累计位置（整数运算；须足够频繁地读角度）
    // ============================================================

    int32_t getCumulativePosition(bool update = true);
    int32_t getRevolutions();
    int32_t resetCumulativePosition(int32_t position = 0);

    // ============================================================
    //  角速度（0.01°/s 定点）
    // ============================================================

    /**
     * @brief 近似角速度（0.01°/s 定点）
     * @param update true = 先读一次角度再计算
     * @note  两次测量间隔内旋转不得超过 180°（否则无法判向）
     */
    int32_t getAngularSpeed_c01_per_s(bool update = true);

    // ============================================================
    //  电源重启（重新加载 OTP 配置）
    // ============================================================

    void resetPOR();

private:
    // ── I2C 寄存器读写（freedom 1/2 字节，MSB 优先） ──────
    uint8_t  readReg(uint8_t reg);
    uint16_t readReg2(uint8_t reg);
    bool     writeReg(uint8_t reg, uint8_t value);
    bool     writeReg2(uint8_t reg, uint16_t value);

    inter_i2c_dev _dev;
    uint8_t _addr;

    uint16_t _offset = 0;           // 软件偏移（raw 计数）
    int32_t  _position = 0;         // 累计位置（raw 计数）
    int16_t  _lastPosition = 0;     // 上次角度（raw）
    int16_t  _lastAngle = 0;        // 角速度用上次角度（raw）
    int16_t  _lastReadAngle = 0;    // 最近一次成功读的角度（raw）
    uint32_t _lastMeasurement = 0;  // get_tick 时间戳
};

#endif
