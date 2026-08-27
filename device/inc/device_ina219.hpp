#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_io_ctrl.hpp"
#include <cstdint>

// ============================================================
//  INA219 电流监测驱动（STM32G070 移植版）
//  ─────────────────────────────────────────────────────────
//  来源库: Rob Tillaart INA219 Arduino Library v0.4.2 (2021-05-18)
//     URL: https://github.com/RobTillaart/INA219
//  移植说明:
//    - 无浮点：物理量定点整数（µV/mV/mA/mW），Current LSB 沿用
//      device_ina226.cpp 的 nA 定点法（_currentLSB_nA）
//    - 12 位 ADC；分流电压 LSB = 10 µV（16 位有符号，无需移位）
//    - 总线电压 LSB = 4 mV（13 位左对齐，寄存器 bit15-3，须 >>3）
//    - CAL = 0.04096/(LSB×R_shunt)（注意系数 0.04096，非 INA226 的 0.00512）
//    - 功率 LSB = 20 × Current_LSB（非 INA226 的 25 倍）
//    - 无 MANUFACTURER_ID / DIE_ID 寄存器（与 INA226/INA260 不同）
//    - I2C 地址 0x40~0x4F（A0~A2 + ALERT 脚，共 16 个）
// ============================================================

// ============================================================
// INA219 Snapshot
// ============================================================

struct INA219_Snapshot
{
    uint16_t config;
    uint16_t shuntVoltage;
    uint16_t busVoltage;
    uint16_t power;
    uint16_t current;
    uint16_t calibration;
    uint16_t maskEnable;
    uint16_t alertLimit;
};

// ============================================================
//  Alert 标志位（Mask/Enable 寄存器 bit 4-2 状态字段）
//  读后自动清除（锁存模式）或实时反映（非锁存模式）
// ============================================================

struct INA219_AlertFlags
{
    bool mathOverflow;     // bit 2: OVF 数学溢出
    bool conversionReady;  // bit 3: CVRF 转换完成
    bool alertFunction;    // bit 4: AFF 报警功能激活
};

// ============================================================

class INA219
{
public:

    // ── I2C 地址（0x40~0x4F，A0~A2 + ALERT 脚） ─────────────

    enum Addr : uint8_t
    {
        ADDR_0x40 = 0x40, ADDR_0x41 = 0x41, ADDR_0x42 = 0x42, ADDR_0x43 = 0x43,
        ADDR_0x44 = 0x44, ADDR_0x45 = 0x45, ADDR_0x46 = 0x46, ADDR_0x47 = 0x47,
        ADDR_0x48 = 0x48, ADDR_0x49 = 0x49, ADDR_0x4A = 0x4A, ADDR_0x4B = 0x4B,
        ADDR_0x4C = 0x4C, ADDR_0x4D = 0x4D, ADDR_0x4E = 0x4E, ADDR_0x4F = 0x4F,
    };

    // ── 寄存器地址 ────────────────────────────────────────

    enum Reg : uint8_t
    {
        REG_CONFIG           = 0x00,   // 配置寄存器 (R/W, reset=0x399F)
        REG_SHUNT_VOLTAGE    = 0x01,   // 分流电压 (R, 16-bit 有符号, 10 µV/LSB)
        REG_BUS_VOLTAGE      = 0x02,   // 总线电压 (R, 13-bit 左对齐, 4 mV/LSB, bit1-0=标志)
        REG_POWER            = 0x03,   // 功率     (R, 16-bit 无符号, 20×LSB/LSB)
        REG_CURRENT          = 0x04,   // 电流     (R, 16-bit 有符号)
        REG_CALIBRATION      = 0x05,   // 校准     (R/W)
        REG_MASK_ENABLE      = 0x06,   // 掩码/使能 (R/W, reset=0x0000)
        REG_ALERT_LIMIT      = 0x07,   // 报警阈值 (R/W)
        // INA219 无 MANUFACTURER_ID(0xFE) / DIE_ID(0xFF) 寄存器
    };

    // ── CONFIG 寄存器枚举 ─────────────────────────────────

    enum class Reset : uint8_t    { Normal = 0, SoftReset = 1 };
    enum class BusRange : uint8_t { V16 = 0, V32 = 1 };         // bit13 BRN
    enum class Gain : uint8_t     { G1 = 0, G2 = 1, G4 = 2, G8 = 3 }; // bits12-11 PGA，±40/±80/±160/±320 mV

    // 12 位 ADC；分辨率/平均合并在同一个 4 位字段（BADC/SADC）
    // 注意：代码 4~7 为保留值，本枚举不暴露
    enum class AdcMode : uint8_t
    {
        // 单次转换（含转换时间）
        B9  = 0x00,   // 9-bit,  84 µs
        B10 = 0x01,   // 10-bit, 148 µs
        B11 = 0x02,   // 11-bit, 276 µs
        B12 = 0x03,   // 12-bit, 532 µs
        // 12 位 + N 次平均（每次 532 µs）
        N1   = 0x08,  // 1 次,    532 µs
        N2   = 0x09,  // 2 次,    1.06 ms
        N4   = 0x0A,  // 4 次,    2.13 ms
        N8   = 0x0B,  // 8 次,    4.26 ms
        N16  = 0x0C,  // 16 次,   8.51 ms
        N32  = 0x0D,  // 32 次,   17.02 ms
        N64  = 0x0E,  // 64 次,   34.05 ms
        N128 = 0x0F,  // 128 次,  68.10 ms
    };

    // 平均次数（setAveraging 用，映射到 AdcMode 的 N1~N128）
    enum class AvgSample : uint8_t
    {
        N1 = 0, N2 = 1, N4 = 2, N8 = 3,
        N16 = 4, N32 = 5, N64 = 6, N128 = 7,
    };
    enum class OperatingMode : uint8_t
    {
        PowerDown          = 0x00,
        ShuntTrigger       = 0x01, BusTrigger         = 0x02,
        ShuntBusTrigger    = 0x03,
        Shutdown           = 0x04,   // ADC 关闭（等同 PowerDown）
        ShuntContinuous    = 0x05, BusContinuous      = 0x06,
        ShuntBusContinuous = 0x07,
    };

    // ── Mask/Enable 配置位 ────────────────────────────────

    enum class Alatch : uint8_t  { Transparent = 0, Latch = 1 };
    enum class Apol : uint8_t    { ActiveLow = 0, ActiveHigh = 1 };

    // ── 报警功能使能位（Mask/Enable 寄存器 bit15-10）──
    // 对齐参考库的 INA219_SHUNT_OVER_VOLTAGE 等定义（与 INA226 同布局）
    static constexpr uint16_t ALERT_SHUNT_OVER_VOLTAGE  = 0x8000; // bit15 SOL
    static constexpr uint16_t ALERT_SHUNT_UNDER_VOLTAGE = 0x4000; // bit14 SUL
    static constexpr uint16_t ALERT_BUS_OVER_VOLTAGE    = 0x2000; // bit13 BOL
    static constexpr uint16_t ALERT_BUS_UNDER_VOLTAGE   = 0x1000; // bit12 BUL
    static constexpr uint16_t ALERT_POWER_OVER_LIMIT    = 0x0800; // bit11 POL
    static constexpr uint16_t ALERT_CONVERSION_READY    = 0x0400; // bit10 CNVR
    // 读回状态标志位（Mask/Enable 寄存器 bit4-2）
    static constexpr uint16_t FLAG_ALERT_FUNCTION       = 0x0010; // bit4  AFF
    static constexpr uint16_t FLAG_CONVERSION_READY     = 0x0008; // bit3  CVRF
    static constexpr uint16_t FLAG_MATH_OVERFLOW        = 0x0004; // bit2  OVF

    // ============================================================
    //  Config 结构体（用户可定制）
    // ============================================================

    struct Config
    {
        // ── CONFIG 寄存器 ──
        Reset         reset         = Reset::Normal;
        BusRange      busRange      = BusRange::V32;    // 32 V 量程（芯片复位默认）
        Gain          gain          = Gain::G4;         // ±160 mV（芯片复位默认）
        AdcMode       busAdc        = AdcMode::N16;     // 总线 ADC：16 次平均
        AdcMode       shuntAdc      = AdcMode::N16;     // 分流 ADC：16 次平均
        OperatingMode mode          = OperatingMode::ShuntBusContinuous;

        // ── 报警配置（Mask/Enable 寄存器 bit 1-0） ──
        Alatch        alatch        = Alatch::Latch;        // 锁存使能
        Apol          apol          = Apol::ActiveLow;      // 开漏低有效

        // ── 报警引脚自动配置（0=不使用） ──
        GPIO_TypeDef* alert_port    = nullptr;         // GPIO 端口，如 GPIOB
        pin_enum_t    alert_pin     = pin_none;         // 引脚掩码，如 GPIO_PIN_5

        // ── 校准参数 ──
        // 与 INA226 相同：rShunt_uOhm 非 0 → 自动校准；
        // 保持 0 → 使用 calibration 原值（需自行保证自洽）
        uint32_t      rShunt_uOhm   = 0;        // 采样电阻 µΩ（0 = 手动校准模式）
        uint16_t      calibration   = 0x0800;   // CAL 寄存器值（手动模式使用；自动模式被覆盖）
        uint16_t      alertLimit    = 0x0000;   // Alert Limit 寄存器值

        // ── 报警功能使能位（Mask/Enable 寄存器 bit15-10，0 = 全部禁用）──
        uint16_t      alertMask     = 0;

        // ── 用户参数 ──
        int32_t       maxCurrent_mA = 81;      // 预期最大电流 mA（推导 Current LSB）
    };

    // ── 校准错误码（setMaxCurrentShunt 返回值，语义对齐 INA226）──
    enum ErrCode : int
    {
        ERR_NONE              = 0,       // 成功
        ERR_SHUNTVOLTAGE_HIGH = 1,       // 最大电流 × 采样电阻 > PGA 满量程（40/80/160/320 mV）
        ERR_MAXCURRENT_LOW    = 2,       // 最大电流过小
        ERR_SHUNT_LOW         = 3,       // 采样电阻过小
        ERR_CAL_OVERFLOW      = 4,       // CAL 寄存器溢出（LSB 加倍后仍超 15 位）
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit INA219(inter_i2c_bus* bus, uint8_t addr = ADDR_0x40);
    explicit INA219(inter_i2c_bus* bus, uint8_t addr, const Config& cfg);

    INA219(const INA219&) = delete;
    INA219& operator=(const INA219&) = delete;

    ~INA219();

    void init();

    // ============================================================
    //  校准（对齐参考库 setMaxCurrentShunt，整数版）
    // ============================================================

    /**
     * @brief 由最大电流与采样电阻自动计算 Current LSB 与 CAL 寄存器并写入芯片
     * @param maxCurrent_mA 预期最大电流 mA（决定 Current LSB = maxCurrent / 2^15）
     * @param rShunt_uOhm   采样电阻 µΩ（如 10 mΩ 填 10000）
     * @return ErrCode，0 = 成功；失败时芯片状态不变
     * @note  INA219: CAL = 0.04096/(LSB×R)，分流满量程由 PGA 增益决定
     */
    int setMaxCurrentShunt(int32_t maxCurrent_mA, uint32_t rShunt_uOhm);

    /** @brief 是否已校准（自动或手动） */
    [[nodiscard]] bool isCalibrated() const { return _currentLSB_nA != 0; }

    /** @brief 当前 Current LSB（nA/LSB，仅内部换算用，调试时参考） */
    int64_t getCurrentLSB_nA() const { return _currentLSB_nA; }

    // ============================================================
    //  连接 / 错误
    // ============================================================

    /** @brief I2C 探测（INA219 无 ID 寄存器，用地址探测） */
    [[nodiscard]] bool isConnected();

    /** @brief 最近一次寄存器操作错误码（0 = 无错误；读取前自动清零） */
    [[nodiscard]] int getLastError();

    // ============================================================
    //  测量 API
    // ============================================================

    int32_t getBusVoltage_mV();       // (raw>>3) × 4 mV（不处理溢出标志，见 getMathOverflowFlag）
    int32_t getShuntVoltage_uV();     // raw × 10 µV
    int32_t getCurrent_mA();          // raw × LSB
    int32_t getPower_mW();            // raw × 20 × LSB

    /** @brief 总线寄存器 bit0：数学溢出（电流/功率寄存器饱和） */
    [[nodiscard]] bool getMathOverflowFlag();

    /** @brief 总线寄存器 bit1：本次转换完成（参考库 getConversionFlag） */
    [[nodiscard]] bool getConversionFlag();

    /** @brief 读 Mask/Enable bit3：本次转换是否完成（与 INA226 同源） */
    [[nodiscard]] bool isConversionReady();

    // ============================================================
    //  报警引脚 / 标志
    // ============================================================

    void bindAlertPin(io_ctrl *pin);
    [[nodiscard]] bool isAlertAsserted();

    /** @brief 便捷方法：等同 isAlertAsserted() */
    [[nodiscard]] bool CheckAlert() { return isAlertAsserted(); }

    [[nodiscard]] INA219_AlertFlags readAlertFlags();
    void clearAlertLatch();

    /** @brief 读取 Mask/Enable 原始值（调试用） */
    uint16_t readMaskEnableRaw();

    // ============================================================
    //  运行时配置
    // ============================================================

    void setConfig(const Config& cfg);

    /** @brief 总线电压量程（16 V / 32 V） */
    void setBusVoltageRange(BusRange range);

    /** @brief PGA 增益（±40/±80/±160/±320 mV），改增益后需重新校准 */
    void setGain(Gain gain);

    /** @brief 平均次数（同时作用于总线/分流 ADC，保持 12 位） */
    void setAveraging(AvgSample samples);

    /** @brief 总线 ADC 模式（分辨率/平均，见 AdcMode） */
    void setBusADC(AdcMode mode);

    /** @brief 分流 ADC 模式（分辨率/平均，见 AdcMode） */
    void setShuntADC(AdcMode mode);

    /** @brief 总线/分流 ADC 同时设置 */
    void setADC(AdcMode mode);

    void setOperatingMode(OperatingMode mode);
    void setCalibration(uint16_t cal);
    void setAlertLimit(uint16_t limit);

    /** @brief 设置报警功能使能位（见 Config::alertMask，读后原值返回） */
    void setAlertMask(uint16_t mask);

    /** @brief 软复位芯片（寄存器恢复默认值，需重新 init/校准） */
    void reset();

    /**
     * @brief 等待转换完成
     * @param timeout_ms 超时上限（默认 600 ms）
     * @return true = 转换完成；false = 超时
     */
    bool waitConversionReady(uint32_t timeout_ms = 600);

    // ============================================================
    //  调试
    // ============================================================

    void dumpRegisters();

private:

    inter_i2c_dev _dev;
    Config         _cfg;
    io_ctrl       *_alertPin    = nullptr;   // 外部报警引脚（bindAlertPin 绑定）
    io_ctrl        _alertPinAuto{nullptr, pin_none};      // 自动配置的报警引脚（Config::alert_port/alert_pin，零堆分配）

    int64_t _currentLSB_nA;              // Current LSB (nA/LSB，64 位避免截断误差)

    // ── I2C 读写（INA219 全部 16-bit 寄存器） ────────────

    void     writeRaw(uint8_t reg, uint16_t data);
    uint16_t readRaw(uint8_t reg);

    // ── 寄存器构建 ────────────────────────────────────────

    uint16_t buildConfigReg()   const;
    uint16_t buildMaskEnable()  const;   // 仅配置位

    // ── 符号扩展 ──────────────────────────────────────────

    template<uint8_t Bits>
    static constexpr int64_t signExtend(uint64_t raw)
    {
        constexpr int shift = 64 - Bits;
        return (static_cast<int64_t>(raw << shift)) >> shift;
    }

    // ── 转换 ──────────────────────────────────────────────

    int32_t gainFullScale_mV() const;             // PGA 增益对应满量程 40/80/160/320 mV

    static int32_t rawToShuntVoltage_uV(uint16_t raw);  // 10 µV/LSB
    static int32_t rawToBusVoltage_mV(uint16_t raw);    // 4 mV/LSB（13 位左对齐）
    int32_t        rawToCurrent_mA(uint16_t raw) const; // raw × LSB_nA / 1e6
    int32_t        rawToPower_mW(uint16_t raw) const;   // raw × 20 × LSB_nA / 1e6

    // ── 更新 Current LSB ──────────────────────────────────

    void _updateCurrentLSB();

    // ── 调试 ──────────────────────────────────────────────

    void captureSnapshot(INA219_Snapshot& snap);
    void dumpRawRegisters(const INA219_Snapshot& snap);
    void dumpEngineeringData(const INA219_Snapshot& snap);
};

#endif
