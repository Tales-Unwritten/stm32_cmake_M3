#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_io_ctrl.hpp"
#include <cstdint>

// ============================================================
//  INA260 电流监测驱动（STM32G070 移植版）
//  ─────────────────────────────────────────────────────────
//  来源库: Rob Tillaart INA260 Arduino Library v0.1.2 (2025-02-18)
//     URL: https://github.com/RobTillaart/INA260
//  移植说明:
//    - 无浮点：物理量定点整数（µV/mV/mA/mW）
//    - 芯片出厂校准：内置 2 mΩ 分流电阻，LSB 固定不可配：
//        电流 LSB = 1.25 mA、总线 LSB = 1.25 mV、功率 LSB = 10 mW
//      （参考库/Adafruit 均按此实现；注意功率 LSB 不是 25×Current_LSB）
//    - 无 CAL 寄存器、无分流电压寄存器（分流电压由电流 × 2 mΩ 推导，
//      等效 2.5 µV/LSB）
//    - setMaxCurrentShunt 仅做参数校验（rShunt 固定 2000 µΩ），
//      不写任何寄存器
//    - 有 MANUFACTURER_ID (0x5449) / DIE_ID (0x2270) 寄存器
//    - I2C 地址 0x40~0x47（A0~A2 三引脚，共 8 个）
// ============================================================

// ============================================================
// INA260 Snapshot
// ============================================================

struct INA260_Snapshot
{
    uint16_t config;
    uint16_t current;
    uint16_t busVoltage;
    uint16_t power;
    uint16_t maskEnable;
    uint16_t alertLimit;
    uint16_t manufacturerID;
    uint16_t dieID;
};

// ============================================================
//  Alert 标志位（Mask/Enable 寄存器 bit 4-2 状态字段）
//  读后自动清除（锁存模式）或实时反映（非锁存模式）
// ============================================================

struct INA260_AlertFlags
{
    bool mathOverflow;     // bit 2: OVF 数学溢出
    bool conversionReady;  // bit 3: CVRF 转换完成
    bool alertFunction;    // bit 4: AFF 报警功能激活
};

// ============================================================

class INA260
{
public:

    // ── I2C 地址（0x40~0x47，A0~A2 三引脚） ────────────────

    enum Addr : uint8_t
    {
        ADDR_0x40 = 0x40, ADDR_0x41 = 0x41, ADDR_0x42 = 0x42, ADDR_0x43 = 0x43,
        ADDR_0x44 = 0x44, ADDR_0x45 = 0x45, ADDR_0x46 = 0x46, ADDR_0x47 = 0x47,
    };

    // ── 寄存器地址 ────────────────────────────────────────

    enum Reg : uint8_t
    {
        REG_CONFIG           = 0x00,   // 配置寄存器 (R/W)
        REG_CURRENT          = 0x01,   // 电流 (R, 16-bit 有符号, 1.25 mA/LSB)
        REG_BUS_VOLTAGE      = 0x02,   // 总线电压 (R, 16-bit 无符号, 1.25 mV/LSB)
        REG_POWER            = 0x03,   // 功率 (R, 16-bit 无符号, 10 mW/LSB)
        // INA260 无分流电压寄存器、无 CAL 寄存器（出厂校准）
        REG_MASK_ENABLE      = 0x06,   // 掩码/使能 (R/W)
        REG_ALERT_LIMIT      = 0x07,   // 报警阈值 (R/W)
        REG_MANUFACTURER_ID  = 0xFE,   // 制造商 ID (R, 0x5449)
        REG_DIE_ID           = 0xFF,   // 芯片 ID   (R, 0x2270)
    };

    // ── CONFIG 寄存器枚举（布局与 INA226 相同） ─────────────

    enum class Reset : uint8_t    { Normal = 0, SoftReset = 1 };
    enum class AvgSample : uint8_t
    {
        N1 = 0, N4 = 1, N16 = 2, N64 = 3,
        N128 = 4, N256 = 5, N512 = 6, N1024 = 7,
    };
    enum class ConvTime : uint8_t
    {
        Us140 = 0, Us204 = 1, Us332 = 2, Us588 = 3,
        Us1100 = 4, Us2116 = 5, Us4156 = 6, Us8244 = 7,
    };
    enum class OperatingMode : uint8_t
    {
        Shutdown           = 0x00,
        ShuntTrigger       = 0x01, BusTrigger         = 0x02,
        ShuntBusTrigger    = 0x03,
        Shutdown2          = 0x04,
        ShuntContinuous    = 0x05, BusContinuous      = 0x06,
        ShuntBusContinuous = 0x07,
    };

    // ── Mask/Enable 配置位 ────────────────────────────────

    enum class Alatch : uint8_t  { Transparent = 0, Latch = 1 };
    enum class Apol : uint8_t    { ActiveLow = 0, ActiveHigh = 1 };

    // ── 报警功能使能位（Mask/Enable 寄存器 bit15-10）──
    // 对齐参考库的 INA260_SHUNT_OVER_CURRENT 等定义
    static constexpr uint16_t ALERT_SHUNT_OVER_CURRENT  = 0x8000; // bit15 SOC
    static constexpr uint16_t ALERT_SHUNT_UNDER_CURRENT = 0x4000; // bit14 SUC
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
        AvgSample     averaging     = AvgSample::N256;
        ConvTime      busConvTime   = ConvTime::Us1100;
        ConvTime      shuntConvTime = ConvTime::Us1100;
        OperatingMode mode          = OperatingMode::ShuntBusContinuous;

        // ── 报警配置（Mask/Enable 寄存器 bit 1-0） ──
        Alatch        alatch        = Alatch::Latch;        // 锁存使能
        Apol          apol          = Apol::ActiveLow;      // 开漏低有效

        // ── 报警引脚自动配置（0=不使用） ──
        GPIO_TypeDef* alert_port    = nullptr;         // GPIO 端口，如 GPIOB
        pin_enum_t    alert_pin     = pin_none;         // 引脚掩码，如 GPIO_PIN_5

        // ── 报警 ──
        uint16_t      alertLimit    = 0x0000;   // Alert Limit 寄存器值

        // ── 报警功能使能位（Mask/Enable 寄存器 bit15-10，0 = 全部禁用）──
        uint16_t      alertMask     = 0;

        // 注：INA260 无校准寄存器（内置 2 mΩ 分流、出厂校准），
        //     Config 中没有 rShunt/calibration/maxCurrent 字段
    };

    // ── 校准错误码（setMaxCurrentShunt 返回值）──
    enum ErrCode : int
    {
        ERR_NONE            = 0,       // 成功
        ERR_MAXCURRENT_LOW  = 1,       // 最大电流 < 1 mA
        ERR_MAXCURRENT_HIGH = 2,       // 最大电流 > 15 A（芯片额定连续电流）
        ERR_SHUNT_LOW       = 3,       // rShunt ≠ 2000 µΩ（内置 2 mΩ 固定）
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit INA260(inter_i2c_bus* bus, uint8_t addr = ADDR_0x40);
    explicit INA260(inter_i2c_bus* bus, uint8_t addr, const Config& cfg);

    INA260(const INA260&) = delete;
    INA260& operator=(const INA260&) = delete;

    ~INA260();

    void init();

    // ============================================================
    //  校准（INA260 出厂校准，仅参数校验，不写寄存器）
    // ============================================================

    /**
     * @brief INA260 为出厂校准芯片（内置 2 mΩ 分流），本方法仅做参数校验
     * @param maxCurrent_mA 预期最大电流 mA（1~15000，仅记录供应用参考）
     * @param rShunt_uOhm   必须为 2000（内置 2 mΩ，不可配置）
     * @return ErrCode，0 = 成功；不写任何寄存器
     * @note  电流/功率 LSB 固定：1.25 mA / 10 mW，不受本方法影响
     */
    int setMaxCurrentShunt(int32_t maxCurrent_mA, uint32_t rShunt_uOhm);

    /** @brief 芯片出厂已校准，恒为 true */
    [[nodiscard]] bool isCalibrated() const { return true; }

    /** @brief 固定 Current LSB（1.25 mA = 1250000 nA） */
    int64_t getCurrentLSB_nA() const { return 1250000; }

    /** @brief 最近一次 setMaxCurrentShunt 的最大电流 mA */
    int32_t getMaxCurrent_mA() const { return _maxCurrent_mA; }

    /** @brief 内置采样电阻：固定 2000 µΩ（2 mΩ） */
    uint32_t getShunt_uOhm() const { return 2000; }

    // ============================================================
    //  连接 / 错误
    // ============================================================

    /** @brief I2C 探测：读 ManufacturerID 并检查总线错误 */
    [[nodiscard]] bool isConnected();

    /** @brief 最近一次寄存器操作错误码（0 = 无错误；读取前自动清零） */
    [[nodiscard]] int getLastError();

    // ============================================================
    //  测量 API
    // ============================================================

    int32_t getBusVoltage_mV();       // raw × 1.25 mV
    int32_t getShuntVoltage_uV();     // 由电流推导：raw × 2.5 µV（无专用寄存器）
    int32_t getCurrent_mA();          // raw × 1.25 mA
    int32_t getPower_mW();            // raw × 10 mW

    // ============================================================
    //  报警引脚 / 标志
    // ============================================================

    void bindAlertPin(io_ctrl *pin);
    [[nodiscard]] bool isAlertAsserted();

    /** @brief 便捷方法：等同 isAlertAsserted() */
    [[nodiscard]] bool CheckAlert() { return isAlertAsserted(); }

    [[nodiscard]] INA260_AlertFlags readAlertFlags();
    void clearAlertLatch();

    /** @brief 读取 Mask/Enable 原始值（调试用） */
    uint16_t readMaskEnableRaw();

    // ============================================================
    //  运行时配置
    // ============================================================

    void setConfig(const Config& cfg);
    void setAveraging(AvgSample mode);
    void setBusConvTime(ConvTime time);
    void setShuntConvTime(ConvTime time);
    void setOperatingMode(OperatingMode mode);
    void setAlertLimit(uint16_t limit);

    /** @brief 设置报警功能使能位（见 Config::alertMask，读后原值返回） */
    void setAlertMask(uint16_t mask);

    /** @brief 软复位芯片（寄存器恢复默认值，需重新 init） */
    void reset();

    /** @brief 读 Mask/Enable bit3：本次转换是否完成 */
    [[nodiscard]] bool isConversionReady();

    /**
     * @brief 等待转换完成
     * @param timeout_ms 超时上限（默认 600 ms，与参考库一致）
     * @return true = 转换完成；false = 超时
     */
    bool waitConversionReady(uint32_t timeout_ms = 600);

    // ============================================================
    //  设备信息
    // ============================================================

    uint16_t getManufacturerID();
    uint16_t getDieID();

    // ============================================================
    //  调试
    // ============================================================

    void dumpRegisters();

private:

    inter_i2c_dev _dev;
    Config         _cfg;
    io_ctrl       *_alertPin    = nullptr;   // 外部报警引脚（bindAlertPin 绑定）
    io_ctrl        _alertPinAuto{nullptr, pin_none};      // 自动配置的报警引脚（Config::alert_port/alert_pin，零堆分配）

    int32_t _maxCurrent_mA = 15000;    // 应用层最大电流记录（仅参考）

    // ── I2C 读写（INA260 全部 16-bit 寄存器） ────────────

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

    // ── 转换（LSB 全部固定） ──────────────────────────────

    static int32_t rawToBusVoltage_mV(uint16_t raw);    // 1.25 mV/LSB = 5/4
    static int32_t rawToShuntVoltage_uV(uint16_t raw);  // 2.5 µV/LSB = 5/2（电流寄存器推导）
    static int32_t rawToCurrent_mA(uint16_t raw);       // 1.25 mA/LSB = 5/4
    static int32_t rawToPower_mW(uint16_t raw);         // 10 mW/LSB

    // ── 调试 ──────────────────────────────────────────────

    void captureSnapshot(INA260_Snapshot& snap);
    void dumpRawRegisters(const INA260_Snapshot& snap);
    void dumpEngineeringData(const INA260_Snapshot& snap);
};

#endif
