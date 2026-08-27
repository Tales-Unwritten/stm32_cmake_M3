#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// GY521 模块 / MPU6050 6 轴 IMU（加速度计 + 陀螺仪 + 温度）
//
// 移植自 Rob Tillaart GY521 v0.6.2
//   URL: https://github.com/RobTillaart/GY521
//
// 移植说明：
//  - STM32G070 无 FPU，禁止 float：物理量定点化
//      加速度    → mg（×1000，±2g 时 16384 LSB/g）
//      角速度    → 0.001 °/s（×1000，±250 °/s 时 131 LSB/(°/s)）
//      温度      → m°C（T = raw/340 + 36.53）
//  - 不移植姿态解算：atan 俯仰/横滚、角速度积分、互补滤波、
//    normalize 归一化、pitch/roll/yaw（任务约定：不做融合算法）
//  - calibrate() 简化为各轴原始值平均偏移（raw 计数，减去后换算），
//    不做原库的重力方向三角函数补偿（sin/cos）
//  - 14 字节连续输出寄存器分 3 次读（accel 6 + temp 2 + gyro 6），
//    因为 inter_i2c_dev::freedom_read 单次上限 8 字节
//  - 读节流（throttle）保留，默认 10 ms，防止软件 I2C 下读太频繁
// ============================================================

class GY521
{
public:
    // ── I2C 地址 ──────────────────────────────────────────

    enum Addr : uint8_t
    {
        ADDR_0x68 = 0x68,   // AD0 接地
        ADDR_0x69 = 0x69,   // AD0 接 VCC
    };

    // ── 寄存器地址 ────────────────────────────────────────

    enum Reg : uint8_t
    {
        REG_CONFIG       = 0x1A,   // DLPF
        REG_GYRO_CONFIG  = 0x1B,   // 陀螺仪量程 [4:3]
        REG_ACCEL_CONFIG = 0x1C,   // 加速度量程 [4:3]
        REG_ACCEL_XOUT_H = 0x3B,   // 加速度 6 字节块起始
        REG_TEMP_OUT_H   = 0x41,   // 温度 2 字节
        REG_GYRO_XOUT_H  = 0x43,   // 陀螺仪 6 字节块起始
        REG_PWR_MGMT_1   = 0x6B,   // 电源管理
        REG_WHO_AM_I     = 0x75,   // 应返回 0x68
    };

    // ── 错误码（语义对齐原库） ─────────────────────────────

    enum Err : int16_t
    {
        ERR_OK            = 0,
        ERR_THROTTLED     = 1,    // 节流中（不是错误）
        ERR_READ          = -1,   // I2C 读失败
        ERR_WRITE         = -2,   // I2C 写失败
        ERR_NOT_CONNECTED = -3,
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit GY521(inter_i2c_bus* bus, uint8_t addr = ADDR_0x68);

    GY521(const GY521&) = delete;
    GY521& operator=(const GY521&) = delete;

    /**
     * @brief 初始化：唤醒芯片（PWR_MGMT_1 = 0）并应用量程/DLPF 设置
     * @note  芯片上电默认睡眠；必须先 init() 才能读数
     */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool isConnected();
    [[nodiscard]] uint8_t getLastError();

    /** @brief WHO_AM_I 寄存器（正常应返回 0x68） */
    uint8_t readWhoAmI();

    // ============================================================
    //  量程 / 滤波（须在读数前设置）
    // ============================================================

    /** @brief 加速度量程 0..3 = ±2g / ±4g / ±8g / ±16g */
    bool setAccelRange(uint8_t range);
    uint8_t getAccelRange();

    /** @brief 陀螺仪量程 0..3 = ±250 / ±500 / ±1000 / ±2000 °/s */
    bool setGyroRange(uint8_t range);
    uint8_t getGyroRange();

    /** @brief 数字低通滤波 0..6（数据手册 P13 reg26） */
    bool setDLPF(uint8_t mode);
    uint8_t getDLPF();

    // ============================================================
    //  读取（返回 Err；成功为 ERR_OK）
    // ============================================================

    int16_t read();             // 加速度 + 温度 + 陀螺仪（一次读完 3 块）
    int16_t readAccel();        // 仅加速度
    int16_t readGyro();         // 仅陀螺仪
    int16_t readTemperature();  // 仅温度（不节流）

    // ============================================================
    //  定点换算结果（read 成功后有效）
    // ============================================================

    int32_t getAccelX_mg();     // 加速度 mg（×1000）
    int32_t getAccelY_mg();
    int32_t getAccelZ_mg();
    int32_t getGyroX_mdps();    // 角速度 0.001 °/s（×1000）
    int32_t getGyroY_mdps();
    int32_t getGyroZ_mdps();
    int32_t getTemperature_mC();// 温度 m°C（×1000）

    // ── 原始 16 位有符号值（调试用） ──────────────────────

    int16_t getRawAccelX() const { return _ax; }
    int16_t getRawAccelY() const { return _ay; }
    int16_t getRawAccelZ() const { return _az; }
    int16_t getRawGyroX()  const { return _gx; }
    int16_t getRawGyroY()  const { return _gy; }
    int16_t getRawGyroZ()  const { return _gz; }
    int16_t getRawTemperature() const { return _temperature; }

    // ============================================================
    //  校准（简化为原始值平均偏移）
    // ============================================================

    /**
     * @brief 静止采样 times 次，求各轴原始值平均作为偏移（raw 计数）
     * @note  须先 setAccelRange()/setGyroRange() 再调用；
     *        换算公式自动减去偏移。不做原库的重力方向补偿
     */
    bool calibrate(uint16_t times);

    // ============================================================
    //  通用寄存器访问
    // ============================================================

    uint8_t setRegister(uint8_t reg, uint8_t value);   // 返回 Err
    uint8_t getRegister(uint8_t reg);                  // 值经返回值，错误看 getLastError()

    /** @brief 软复位（寄存器恢复默认，需重新 init()） */
    void reset();

    // ============================================================
    //  读节流
    // ============================================================

    void setThrottle(bool on = true) { _throttle = on; }
    bool getThrottle() const { return _throttle; }
    void setThrottleTime(uint16_t ms) { _throttleTime = ms; }
    uint16_t getThrottleTime() const { return _throttleTime; }

private:
    // ── I2C 读块（freedom_read 长度 ≤ 8，MSB 优先） ───────
    bool _readBlock(uint8_t reg, uint8_t len, uint64_t* out);

    inter_i2c_dev _dev;
    uint8_t _afs = 0;            // 加速度量程 0..3
    uint8_t _gfs = 0;            // 陀螺仪量程 0..3

    int16_t _ax = 0, _ay = 0, _az = 0;    // 加速度原始值
    int16_t _gx = 0, _gy = 0, _gz = 0;    // 陀螺仪原始值
    int16_t _temperature = 0;             // 温度原始值

    int32_t _axe = 0, _aye = 0, _aze = 0; // 加速度校准偏移（raw 计数）
    int32_t _gxe = 0, _gye = 0, _gze = 0; // 陀螺仪校准偏移（raw 计数）

    bool     _throttle = true;
    uint16_t _throttleTime = 10;          // 默认 10 ms
    uint32_t _lastTime = 0;               // get_tick 时间戳
};

#endif
