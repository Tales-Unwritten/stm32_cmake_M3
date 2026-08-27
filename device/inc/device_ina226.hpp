
#pragma once

#include "stm32f103xe.h"
#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_io_ctrl.hpp"
#include "stdint.h"

// ============================================================
// INA226 Snapshot
// ============================================================

struct INA226_Snapshot
{
    uint16_t config;
    uint16_t shuntVoltage;
    uint16_t busVoltage;
    uint16_t power;
    uint16_t current;
    uint16_t calibration;
    uint16_t maskEnable;
    uint16_t alertLimit;
    uint16_t manufacturerID;
    uint16_t dieID;
};

// ============================================================
//  Alert 标志位（Mask/Enable 寄存器 bit 4-0 状态字段）
//  读后自动清除（锁存模式）或实时反映（非锁存模式）
// ============================================================

struct INA226_AlertFlags
{
    bool mathOverflow;    // bit 2: OVF 数学溢出
    bool conversionReady; // bit 3: CVRF 转换完成
    bool alertFunction;   // bit 4: AFF 报警功能激活
};

// ============================================================

class INA226
{
  public:
    // ── I2C 地址 ──────────────────────────────────────────

    enum Addr : uint8_t
    {
        ADDR_0x40 = 0x40,
        ADDR_0x41 = 0x41,
        ADDR_0x44 = 0x44,
        ADDR_0x45 = 0x45,
    };

    // ── 寄存器地址 ────────────────────────────────────────

    enum Reg : uint8_t
    {
        REG_CONFIG = 0x00,          // 配置寄存器 (R/W, reset=0x4127)
        REG_SHUNT_VOLTAGE = 0x01,   // 分流电压 (R, 16-bit 有符号)
        REG_BUS_VOLTAGE = 0x02,     // 总线电压 (R, 16-bit 无符号)
        REG_POWER = 0x03,           // 功率     (R, 16-bit 无符号)
        REG_CURRENT = 0x04,         // 电流     (R, 16-bit 有符号)
        REG_CALIBRATION = 0x05,     // 校准     (R/W)
        REG_MASK_ENABLE = 0x06,     // 掩码/使能 (R/W, reset=0x0000)
        REG_ALERT_LIMIT = 0x07,     // 报警阈值 (R/W)
        REG_MANUFACTURER_ID = 0xFE, // 制造商 ID (R, 0x5449)
        REG_DIE_ID = 0xFF,          // 芯片 ID   (R, 0x2260)
    };

    // ── CONFIG 寄存器枚举 ─────────────────────────────────

    enum class Reset : uint8_t
    {
        Normal = 0,
        SoftReset = 1
    };
    enum class AvgSample : uint8_t
    {
        N1 = 0,
        N4 = 1,
        N16 = 2,
        N64 = 3,
        N128 = 4,
        N256 = 5,
        N512 = 6,
        N1024 = 7,
    };
    enum class ConvTime : uint8_t
    {
        Us140 = 0,
        Us204 = 1,
        Us332 = 2,
        Us588 = 3,
        Us1100 = 4,
        Us2116 = 5,
        Us4156 = 6,
        Us8244 = 7,
    };
    enum class OperatingMode : uint8_t
    {
        PowerDown = 0x00,
        ShuntTrigger = 0x01,
        BusTrigger = 0x02,
        ShuntBusTrigger = 0x03,
        Shutdown = 0x04,
        ShuntContinuous = 0x05,
        BusContinuous = 0x06,
        ShuntBusContinuous = 0x07,
    };

    // ── Mask/Enable 配置位 ────────────────────────────────

    enum class Alatch : uint8_t
    {
        Transparent = 0,
        Latch = 1
    };
    enum class Apol : uint8_t
    {
        ActiveLow = 0,
        ActiveHigh = 1
    };

    // ── 报警功能使能位（Mask/Enable 寄存器 bit15-10，供 Config::alertMask 使用）──
    // 对齐 GitHub 版 INA226.h 的 INA226_SHUNT_OVER_VOLTAGE 等定义
    static constexpr uint16_t ALERT_SHUNT_OVER_VOLTAGE = 0x8000;  // bit15 SOL
    static constexpr uint16_t ALERT_SHUNT_UNDER_VOLTAGE = 0x4000; // bit14 SUL
    static constexpr uint16_t ALERT_BUS_OVER_VOLTAGE = 0x2000;    // bit13 BOL
    static constexpr uint16_t ALERT_BUS_UNDER_VOLTAGE = 0x1000;   // bit12 BUL
    static constexpr uint16_t ALERT_POWER_OVER_LIMIT = 0x0800;    // bit11 POL
    static constexpr uint16_t ALERT_CONVERSION_READY = 0x0400;    // bit10 CNVR
    // 读回状态标志位（Mask/Enable 寄存器 bit4-2）
    static constexpr uint16_t FLAG_ALERT_FUNCTION = 0x0010;   // bit4  AFF
    static constexpr uint16_t FLAG_CONVERSION_READY = 0x0008; // bit3  CVRF
    static constexpr uint16_t FLAG_MATH_OVERFLOW = 0x0004;    // bit2  OVF

    // ============================================================
    //  Config 结构体（用户可定制）
    // ============================================================

    struct Config
    {
        // ── CONFIG 寄存器 ──
        Reset reset = Reset::Normal;
        AvgSample averaging = AvgSample::N256;
        ConvTime busConvTime = ConvTime::Us1100;
        ConvTime shuntConvTime = ConvTime::Us1100;
        OperatingMode mode = OperatingMode::ShuntBusContinuous;

        // ── 报警配置（Mask/Enable 寄存器 bit 1-0） ──
        Alatch alatch = Alatch::Latch; // 锁存使能
        Apol apol = Apol::ActiveLow;   // 开漏低有效

        // ── 报警引脚自动配置（0=不使用） ──
        GPIO_TypeDef *alert_port = nullptr; // GPIO 端口，如 GPIOB
        pin_enum_t alert_pin = pin_none;   // 引脚掩码，如 GPIO_PIN_5

        // ── 校准参数 ──
        // 两种方式任选其一：
        //   a) 自动：rShunt_uOhm 非 0 → init()/setConfig() 用 maxCurrent_mA 与采样电阻
        //      自动计算 CAL 寄存器与 Current LSB（推荐，与 GitHub 版 setMaxCurrentShunt 等效）
        //   b) 手动：rShunt_uOhm 保持 0 → 直接使用下方 calibration 原值（需自行保证与
        //      采样电阻、maxCurrent_mA 自洽，否则电流/功率读数系统性错误）
        uint32_t rShunt_uOhm = 0;      // 采样电阻 µΩ（0 = 手动校准模式）
        uint16_t calibration = 0x0800; // CAL 寄存器值（手动模式使用；自动模式被覆盖）
        uint16_t alertLimit = 0x0000;  // Alert Limit 寄存器值

        // ── 报警功能使能位（Mask/Enable 寄存器 bit15-10，0 = 全部禁用）──
        //   bit15 SOL 分流过压 | bit14 SUL 分流欠压 | bit13 BOL 总线过压
        //   bit12 BUL 总线欠压 | bit11 POL 功率超限 | bit10 CNVR 转换完成
        // 只有在此使能的报警才会拉低 ALERT 引脚（配合 alert_port/alert_pin）
        uint16_t alertMask = 0;

        // ── 用户参数 ──
        int32_t maxCurrent_mA = 81; // 预期最大电流 mA（推导 Current LSB）
    };

    // ── 校准错误码（setMaxCurrentShunt 返回值，语义对齐 GitHub 版）──
    enum ErrCode : int
    {
        ERR_NONE = 0,              // 成功
        ERR_SHUNTVOLTAGE_HIGH = 1, // 最大电流 × 采样电阻 > 81.9 mV（分流超量程）
        ERR_MAXCURRENT_LOW = 2,    // 最大电流过小
        ERR_SHUNT_LOW = 3,         // 采样电阻过小
        ERR_CAL_OVERFLOW = 4,      // CAL 寄存器溢出（LSB 加倍后仍超 16 位）
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit INA226(inter_i2c_bus *bus, uint8_t addr = ADDR_0x40);
    explicit INA226(inter_i2c_bus *bus, uint8_t addr, const Config &cfg);

    INA226(const INA226 &) = delete;
    INA226 &operator=(const INA226 &) = delete;

    ~INA226();

    void init();

    // ============================================================
    //  校准（对齐 GitHub 版 setMaxCurrentShunt）
    // ============================================================

    /**
     * @brief 由最大电流与采样电阻自动计算 Current LSB 与 CAL 寄存器并写入芯片
     * @param maxCurrent_mA 预期最大电流 mA（决定 Current LSB = maxCurrent / 2^15）
     * @param rShunt_uOhm   采样电阻 µΩ（如 10 mΩ 填 10000）
     * @return ErrCode，0 = 成功；失败时芯片状态不变
     */
    int setMaxCurrentShunt(int32_t maxCurrent_mA, uint32_t rShunt_uOhm);

    /** @brief 是否已校准（自动或手动） */
    [[nodiscard]] bool isCalibrated() const
    {
        return _currentLSB_nA != 0;
    }

    /** @brief 当前 Current LSB（nA/LSB，仅内部换算用，调试时参考） */
    int64_t getCurrentLSB_nA() const
    {
        return _currentLSB_nA;
    }

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

    int32_t getBusVoltage_mV();
    int32_t getShuntVoltage_uV();
    int32_t getCurrent_mA();
    int32_t getPower_mW();

    // ============================================================
    //  报警引脚 / 标志
    // ============================================================

    void bindAlertPin(io_ctrl *pin);
    [[nodiscard]] bool isAlertAsserted();

    /** @brief 便捷方法：等同 isAlertAsserted() */
    [[nodiscard]] bool CheckAlert()
    {
        return isAlertAsserted();
    }

    [[nodiscard]] INA226_AlertFlags readAlertFlags();
    void clearAlertLatch();

    /** @brief 读取 Mask/Enable 原始值（调试用） */
    uint16_t readMaskEnableRaw();

    // ============================================================
    //  运行时配置
    // ============================================================

    void setConfig(const Config &cfg);
    void setAveraging(AvgSample mode);
    void setBusConvTime(ConvTime time);
    void setShuntConvTime(ConvTime time);
    void setOperatingMode(OperatingMode mode);
    void setCalibration(uint16_t cal);
    void setAlertLimit(uint16_t limit);

    /** @brief 设置报警功能使能位（见 Config::alertMask，读后原值返回） */
    void setAlertMask(uint16_t mask);

    /** @brief 软复位芯片（寄存器恢复默认值，需重新 init/校准） */
    void reset();

    /** @brief 读 Mask/Enable bit3：本次转换是否完成 */
    [[nodiscard]] bool isConversionReady();

    /**
     * @brief 等待转换完成
     * @param timeout_ms 超时上限（默认 600 ms，与 GitHub 版一致）
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
    Config _cfg;
    io_ctrl *_alertPin = nullptr; // 外部报警引脚（bindAlertPin 绑定）
    io_ctrl _alertPinAuto{nullptr, pin_none};  // 自动配置的报警引脚（Config::alert_port/alert_pin，零堆分配）

    int64_t _currentLSB_nA; // Current LSB (nA/LSB，64 位避免截断误差)

    // ── I2C 读写（INA226 全部 16-bit 寄存器） ────────────

    void writeRaw(uint8_t reg, uint16_t data);
    uint16_t readRaw(uint8_t reg);

    // ── 寄存器构建 ────────────────────────────────────────

    uint16_t buildConfigReg() const;
    uint16_t buildMaskEnable() const; // 仅配置位

    // ── 符号扩展 ──────────────────────────────────────────

    template <uint8_t Bits> static constexpr int64_t signExtend(uint64_t raw)
    {
        constexpr int shift = 64 - Bits;
        return (static_cast<int64_t>(raw << shift)) >> shift;
    }

    // ── 转换 ──────────────────────────────────────────────

    static int32_t rawToShuntVoltage_uV(uint16_t raw); // 2.5 µV/LSB = 5/2
    static int32_t rawToBusVoltage_mV(uint16_t raw);   // 1.25 mV/LSB = 5/4
    int32_t rawToCurrent_mA(uint16_t raw) const;       // raw × LSB_nA / 1e6
    int32_t rawToPower_mW(uint16_t raw) const;         // raw × 25 × LSB_nA / 1e6

    // ── 更新 Current LSB ──────────────────────────────────

    void _updateCurrentLSB();

    // ── 调试 ──────────────────────────────────────────────

    void captureSnapshot(INA226_Snapshot &snap);
    void dumpRawRegisters(const INA226_Snapshot &snap);
    void dumpEngineeringData(const INA226_Snapshot &snap);
};

#endif
