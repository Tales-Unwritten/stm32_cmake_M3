#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
//  来源：Rob Tillaart ADS1X15 v0.6.2（Arduino 库）
//     URL: https://github.com/RobTillaart/ADS1X15
//  移植说明：
//   - 仅移植 ADS1115（16-bit，8/16/32/64/128/250/475/860 SPS），
//     不含 ADS1015/1013/1113 等派生型号；如需要 12-bit 可加模板参数
//   - 禁止 float：toVoltage() 改为定点 uV/mV 返回（LSB = FSR/2^15，
//     见 device_ads1115.cpp 换算注释）
//   - 超时等待用 systick.h 轮询，不依赖 Arduino millis()
//   - 比较器阈值保留「原始寄存器值」API，另加 _mV/_uV 换算版
// ============================================================

class ADS1115
{
public:

    // ── I2C 地址 ──────────────────────────────────────────

    enum Addr : uint8_t
    {
        ADDR_0x48 = 0x48, ADDR_0x49 = 0x49,
        ADDR_0x4A = 0x4A, ADDR_0x4B = 0x4B,
    };

    // ── 寄存器地址 ────────────────────────────────────────

    enum Reg : uint8_t
    {
        REG_CONVERSION = 0x00,   // 转换结果 (R, 16-bit 有符号)
        REG_CONFIG     = 0x01,   // 配置寄存器 (R/W)
        REG_LO_THRESH  = 0x02,   // 低阈值 (R/W, 原始 16-bit)
        REG_HI_THRESH  = 0x03,   // 高阈值 (R/W, 原始 16-bit)
    };

    // ── PGA 增益（对应满量程，bit 9-11）────────────────────

    enum class Gain : uint8_t
    {
        FSR_6144mV = 0,   // ±6.144V  187.5 µV/LSB
        FSR_4096mV = 1,   // ±4.096V  125   µV/LSB
        FSR_2048mV = 2,   // ±2.048V  62.5  µV/LSB
        FSR_1024mV = 4,   // ±1.024V  31.25 µV/LSB
        FSR_512mV  = 8,   // ±0.512V  15.625 µV/LSB
        FSR_256mV  = 16,  // ±0.256V  7.8125 µV/LSB
    };

    // ── 数据率（ADS1115 实际 SPS，bit 5-7）────────────────

    enum class DataRate : uint8_t
    {
        SPS_8 = 0, SPS_16 = 1, SPS_32 = 2, SPS_64 = 3,
        SPS_128 = 4, SPS_250 = 5, SPS_475 = 6, SPS_860 = 7,
    };

    // ── 工作模式（bit 8）──────────────────────────────────

    enum class Mode : uint8_t
    {
        Continuous = 0,   // 连续转换
        Single     = 1,   // 单次转换（默认）
    };

    // ── 比较器配置（bit 0-4）──────────────────────────────

    enum class CompMode : uint8_t  { Traditional = 0, Window = 1 };
    enum class CompPol : uint8_t   { ActiveLow = 0, ActiveHigh = 1 };
    enum class CompLatch : uint8_t { NonLatch = 0, Latch = 1 };
    enum class CompQue : uint8_t
    {
        After1Conv = 0,   // 1 次转换后触发
        After2Conv = 1,   // 2 次转换后触发
        After4Conv = 2,   // 4 次转换后触发
        Disable    = 3,   // 禁用比较器（默认）
    };

    // ── 错误码（对齐原版）──────────────────────────────────
    // 注：底层用 int16_t 容纳 0xFF/0xFE；getError() 返回 int8_t 时
    //     0xFF/0xFE 会被转成 -1/-2，与原版 Arduino 行为一致

    enum ErrCode : int16_t
    {
        ERR_OK                = 0,
        ERR_INVALID_VOLTAGE   = -100,  // 增益非法，无法换算电压
        ERR_TIMEOUT           = -101,  // 单次转换超时
        ERR_I2C               = -102,  // I2C 总线错误
        ERR_INVALID_GAIN      = 0xFF,  // getGain() 非法增益
        ERR_INVALID_MODE      = 0xFE,  // getMode() 非法模式
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit ADS1115(inter_i2c_bus* bus, uint8_t addr = ADDR_0x48);

    ADS1115(const ADS1115&) = delete;
    ADS1115& operator=(const ADS1115&) = delete;

    /** @brief 初始化：探测设备，若在线则写入当前配置（OS=0，不启动转换） */
    void init();

    /** @brief I2C 探测：仅发地址检查 ACK */
    [[nodiscard]] bool isConnected();

    /** @brief 读取并清零错误码（ERR_OK = 0） */
    [[nodiscard]] int8_t getError();

    // ============================================================
    //  配置
    // ============================================================

    /**
     * @brief 设置 PGA 增益（参数 0/1/2/4/8/16，无效值映射到 0）
     * @note  仅更新本地配置，下次 requestADC/readADC 生效
     */
    void setGain(uint8_t gain = 0);
    /** @brief 返回当前增益参数；非法状态返回 ERR_INVALID_GAIN */
    uint8_t getGain();

    /** @brief 设置数据率（0..7，无效值映射到 4） */
    void setDataRate(uint8_t dataRate = 4);
    uint8_t getDataRate();

    /** @brief 设置单次/连续模式（默认单次） */
    void setMode(Mode mode = Mode::Single);
    Mode getMode();

    // ============================================================
    //  转换结果 → 电压（定点，禁止 float）
    // ============================================================

    /** @brief 当前增益下的满量程电压（mV） */
    int32_t getMaxVoltage_mV();

    /** @brief 原始值 → µV（LSB = FSR/2^15，见 cpp 推导） */
    int32_t toVoltage_uV(int16_t raw);
    /** @brief 原始值 → mV */
    int32_t toVoltage_mV(int16_t raw);

    /** @brief 电压(mV) → 寄存器原始值（写阈值用，限幅 ±32767） */
    int16_t voltageToRaw_mV(int32_t volts_mV);
    /** @brief 电压(µV) → 寄存器原始值（写阈值用，限幅 ±32767） */
    int16_t voltageToRaw_uV(int32_t volts_uV);

    // ============================================================
    //  读取（同步：内部等待转换完成）
    // ============================================================

    /** @brief 读单端通道 0..3，返回 16-bit 原始值；超时返回 ERR_TIMEOUT */
    int16_t readADC(uint8_t pin = 0);

    /** @brief 读差分 AIN0-AIN1 */
    int16_t readADC_Differential_0_1();

    /** @brief 读转换结果寄存器（上一次转换） */
    int16_t getValue();

    // ============================================================
    //  异步接口：requestADC → isBusy/isReady → getValue
    // ============================================================

    void requestADC(uint8_t pin = 0);
    void requestADC_Differential_0_1();

    /** @brief 转换进行中？（读 CONFIG 的 OS 位） */
    [[nodiscard]] bool isBusy();
    /** @brief 转换完成？ */
    [[nodiscard]] bool isReady();

    /** @brief 最近一次请求：0x0[0..3] 单端 / 0x10 差分0-1 / 0xFF 无请求 */
    uint8_t lastRequest();

    // ============================================================
    //  比较器（阈值按寄存器原始值，另附 mV/uV 换算版）
    // ============================================================

    void setComparatorMode(CompMode mode);
    CompMode getComparatorMode();

    /** @brief 关闭比较器（读 CONFIG 清 bit1-0 后写回） */
    bool setComparatorOff();

    void setComparatorPolarity(CompPol pol);
    CompPol getComparatorPolarity();

    void setComparatorLatch(CompLatch latch);
    CompLatch getComparatorLatch();

    void setComparatorQueConvert(CompQue que);
    CompQue getComparatorQueConvert();

    /** @brief 低阈值（寄存器原始值，16-bit 有符号） */
    void setComparatorThresholdLow(int16_t lo);
    /** @brief 高阈值（寄存器原始值，16-bit 有符号） */
    void setComparatorThresholdHigh(int16_t hi);
    int16_t getComparatorThresholdLow();
    int16_t getComparatorThresholdHigh();

    /** @brief 低阈值（mV，自动换算为寄存器原始值） */
    void setComparatorThresholdLow_mV(int32_t lo_mV);
    /** @brief 高阈值（mV，自动换算为寄存器原始值） */
    void setComparatorThresholdHigh_mV(int32_t hi_mV);

private:

    // ── I2C 读写（全部 16-bit 寄存器） ─────────────────────

    void     writeRaw(uint8_t reg, uint16_t data);
    uint16_t readRaw(uint8_t reg);

    // ── 内部 ───────────────────────────────────────────────

    int16_t _readADC(uint16_t readmode);      // 请求 + 等待 + 读结果
    void    _requestADC(uint16_t readmode);   // 组 CONFIG 并写入

    // ── 成员 ───────────────────────────────────────────────

    inter_i2c_dev _dev;

    uint8_t  _gain;           // PGA 参数 0/1/2/4/8/16
    uint16_t _gainMask;       // 寄存器位 9-11
    uint8_t  _mode;           // 0=连续 1=单次
    uint16_t _modeMask;       // 寄存器位 8
    uint8_t  _datarate;       // 0..7
    uint16_t _datarateMask;   // 寄存器位 5-7

    uint8_t  _compMode;       // 0=传统 1=窗口
    uint8_t  _compPol;        // 0=低有效 1=高有效
    uint8_t  _compLatch;      // 0=非锁存 1=锁存
    uint8_t  _compQueConvert; // 0..3

    uint16_t _lastRequest;    // 最近一次 MUX 请求（0xFFFF = 无）
    int8_t   _error;
};

#endif /* __cplusplus */
