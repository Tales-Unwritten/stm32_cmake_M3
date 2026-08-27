#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_io_ctrl.hpp"
#include <cstdint>

// ============================================================
//  INA3221 3 通道电流监测驱动（STM32G070 移植版）
//  ─────────────────────────────────────────────────────────
//  来源库: Rob Tillaart INA3221_RT Arduino Library v0.4.2 (2024-02-05)
//     URL: https://github.com/RobTillaart/INA3221_RT
//  移植说明:
//    - 无浮点：物理量定点整数（µV/mV/mA/mW）
//    - 3 通道独立分流/总线电压寄存器：
//        分流电压 LSB = 40 µV（13 位有符号左对齐，须 >>3）
//        总线电压 LSB = 8 mV（13 位无符号左对齐，须 >>3）
//    - 无电流/功率寄存器、无 CAL 寄存器：电流由软件计算 I = Vshunt/R
//      （每通道采样电阻 µΩ 存于 Config，与参考库 setShuntR 等效）
//    - 配置寄存器含通道使能（bit14-12，CH1/CH2/CH3）
//    - Mask/Enable：bit15 CNVR、bit14-12 SCC 求和通道、bit11 POL、bit10 LEN，
//      标志位 bit9-0（CGF/WGF/SF/PUF/TCF/CVRF，读后清除）
//    - 有 MANUFACTURER_ID (0x5449) / DIE_ID (0x3220) 寄存器
//    - I2C 地址 0x40~0x43（A0~A1 两引脚，共 4 个）
// ============================================================

// ============================================================
// INA3221 Snapshot
// ============================================================

struct INA3221_Snapshot
{
    uint16_t config;
    uint16_t shuntVoltage[3];     // 通道 1~3 分流电压
    uint16_t busVoltage[3];       // 通道 1~3 总线电压
    uint16_t criticalAlert[3];    // 通道 1~3 临界报警阈值
    uint16_t warningAlert[3];     // 通道 1~3 预警阈值
    uint16_t shuntVoltageSum;     // 分流电压求和
    uint16_t shuntVoltageLimit;   // 求和上限（临界）
    uint16_t maskEnable;
    uint16_t powerValidUpper;
    uint16_t powerValidLower;
    uint16_t manufacturerID;
    uint16_t dieID;
};

// ============================================================
//  Alert 标志位（Mask/Enable 寄存器 bit 9-0 状态字段，读后清除）
// ============================================================

struct INA3221_AlertFlags
{
    bool conversionReady;   // bit 0: CVRF 转换完成
    bool timingControl;     // bit 1: TCF 时序控制
    bool powerValid;        // bit 2: PUF 电源有效
    bool warning3;          // bit 3: WGF3 通道 3 预警
    bool warning2;          // bit 4: WGF2 通道 2 预警
    bool warning1;          // bit 5: WGF1 通道 1 预警
    bool summation;         // bit 6: SF 求和报警
    bool critical3;         // bit 7: CGF3 通道 3 临界
    bool critical2;         // bit 8: CGF2 通道 2 临界
    bool critical1;         // bit 9: CGF1 通道 1 临界
};

// ============================================================

class INA3221
{
public:

    // ── I2C 地址（0x40~0x43，A0~A1 两引脚） ────────────────

    enum Addr : uint8_t
    {
        ADDR_0x40 = 0x40, ADDR_0x41 = 0x41,
        ADDR_0x42 = 0x42, ADDR_0x43 = 0x43,
    };

    // ── 寄存器地址 ────────────────────────────────────────

    enum Reg : uint8_t
    {
        REG_CONFIG             = 0x00,   // 配置寄存器 (R/W, reset=0x7127)
        REG_SHUNT_VOLTAGE_1    = 0x01,   // 通道 1 分流电压 (R, 40 µV/LSB)
        REG_BUS_VOLTAGE_1      = 0x02,   // 通道 1 总线电压 (R, 8 mV/LSB)
        REG_SHUNT_VOLTAGE_2    = 0x03,
        REG_BUS_VOLTAGE_2      = 0x04,
        REG_SHUNT_VOLTAGE_3    = 0x05,
        REG_BUS_VOLTAGE_3      = 0x06,
        REG_CRITICAL_ALERT_1   = 0x07,   // 通道 1 临界报警阈值 (R/W)
        REG_WARNING_ALERT_1    = 0x08,   // 通道 1 预警阈值 (R/W)
        REG_CRITICAL_ALERT_2   = 0x09,
        REG_WARNING_ALERT_2    = 0x0A,
        REG_CRITICAL_ALERT_3   = 0x0B,
        REG_WARNING_ALERT_3    = 0x0C,
        REG_SHUNT_VOLTAGE_SUM  = 0x0D,   // 分流电压求和 (R, 15-bit 有符号)
        REG_SHUNT_VOLTAGE_LIMIT= 0x0E,   // 求和临界阈值 (R/W)
        REG_MASK_ENABLE        = 0x0F,   // 掩码/使能 (R/W)
        REG_POWER_VALID_UPPER  = 0x10,   // 电源有效上限 (R/W, 8 mV/LSB)
        REG_POWER_VALID_LOWER  = 0x11,   // 电源有效下限 (R/W)
        REG_MANUFACTURER_ID    = 0xFE,   // 制造商 ID (R, 0x5449)
        REG_DIE_ID             = 0xFF,   // 芯片 ID   (R, 0x3220)
    };

    // ── CONFIG 寄存器枚举 ─────────────────────────────────

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
        PowerDown          = 0x00,
        ShuntTrigger       = 0x01, BusTrigger         = 0x02,
        ShuntBusTrigger    = 0x03,
        PowerDown2         = 0x04,
        ShuntContinuous    = 0x05, BusContinuous      = 0x06,
        ShuntBusContinuous = 0x07,
    };

    // ── Mask/Enable 配置位 ────────────────────────────────

    enum class Alatch : uint8_t  { Transparent = 0, Latch = 1 };   // bit10 LEN
    enum class Apol : uint8_t    { ActiveLow = 0, ActiveHigh = 1 }; // bit11 POL

    // 报警使能/求和通道位（Mask/Enable 寄存器 bit15-12，供 Config::alertMask）
    static constexpr uint16_t ALERT_CONVERSION_READY = 0x8000; // bit15 CNVR 转换完成报警使能
    static constexpr uint16_t SCC_CH1                = 0x1000; // bit12 通道 1 参与分流求和
    static constexpr uint16_t SCC_CH2                = 0x2000; // bit13 通道 2 参与分流求和
    static constexpr uint16_t SCC_CH3                = 0x4000; // bit14 通道 3 参与分流求和
    // 读回状态标志位（Mask/Enable 寄存器 bit9-0，读后清除）
    static constexpr uint16_t FLAG_CRITICAL_1        = 0x0200; // bit9  CGF1
    static constexpr uint16_t FLAG_CRITICAL_2        = 0x0100; // bit8  CGF2
    static constexpr uint16_t FLAG_CRITICAL_3        = 0x0080; // bit7  CGF3
    static constexpr uint16_t FLAG_SUMMATION         = 0x0040; // bit6  SF
    static constexpr uint16_t FLAG_WARNING_1         = 0x0020; // bit5  WGF1
    static constexpr uint16_t FLAG_WARNING_2         = 0x0010; // bit4  WGF2
    static constexpr uint16_t FLAG_WARNING_3         = 0x0008; // bit3  WGF3
    static constexpr uint16_t FLAG_POWER_VALID       = 0x0004; // bit2  PUF
    static constexpr uint16_t FLAG_TIMING_CONTROL    = 0x0002; // bit1  TCF
    static constexpr uint16_t FLAG_CONVERSION_READY  = 0x0001; // bit0  CVRF

    // ============================================================
    //  Config 结构体（用户可定制）
    // ============================================================

    struct Config
    {
        // ── CONFIG 寄存器 ──
        Reset         reset         = Reset::Normal;
        bool          ch1_enable    = true;   // bit14 通道 1
        bool          ch2_enable    = true;   // bit13 通道 2
        bool          ch3_enable    = true;   // bit12 通道 3
        AvgSample     averaging     = AvgSample::N256;
        ConvTime      busConvTime   = ConvTime::Us1100;
        ConvTime      shuntConvTime = ConvTime::Us1100;
        OperatingMode mode          = OperatingMode::ShuntBusContinuous;

        // ── Mask/Enable 配置位 ──
        Alatch        alatch        = Alatch::Latch;        // bit10 LEN 锁存使能
        Apol          apol          = Apol::ActiveLow;      // bit11 POL 报警极性

        // ── 报警引脚自动配置（0=不使用） ──
        GPIO_TypeDef* alert_port    = nullptr;         // GPIO 端口，如 GPIOB
        pin_enum_t    alert_pin     = pin_none;         // 引脚掩码，如 GPIO_PIN_5

        // ── Mask/Enable 功能位（bit15 CNVR + bit14-12 SCC）──
        //   默认 0x7000：三个通道全部参与分流求和（数据手册复位值）
        uint16_t      alertMask     = 0x7000;

        // ── 每通道采样电阻（软件换算电流用，µΩ；0 = 未配置）──
        uint32_t      rShunt_uOhm[3] = {100000, 100000, 100000};   // 默认 0.1 Ω

        // 注：INA3221 无 CAL 寄存器（电流由软件计算），Config 无校准字段；
        //     报警阈值（临界/预警/求和/电源有效）通过运行时方法设置
    };

    // ── 错误码 ──
    enum ErrCode : int
    {
        ERR_NONE        = 0,       // 成功
        ERR_CHANNEL     = 1,       // 通道号超出 0..2
        ERR_SHUNT_LOW   = 2,       // 采样电阻 < 1 mΩ（除法溢出保护）
        ERR_LIMIT_RANGE = 3,       // 阈值超出寄存器量程
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit INA3221(inter_i2c_bus* bus, uint8_t addr = ADDR_0x40);
    explicit INA3221(inter_i2c_bus* bus, uint8_t addr, const Config& cfg);

    INA3221(const INA3221&) = delete;
    INA3221& operator=(const INA3221&) = delete;

    ~INA3221();

    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    /** @brief I2C 探测：读 ManufacturerID 并检查总线错误 */
    [[nodiscard]] bool isConnected();

    /** @brief 最近一次寄存器操作错误码（0 = 无错误；读取前自动清零） */
    [[nodiscard]] int getLastError();

    // ============================================================
    //  测量 API（channel = 0..2；无效通道返回 0）
    // ============================================================

    /** @brief 通道总线电压 (raw>>3) × 8 mV，量程 0~26 V */
    int32_t getBusVoltage_mV(uint8_t channel);

    /** @brief 通道分流电压 (raw>>3) × 40 µV，满量程 ±163.8 mV */
    int32_t getShuntVoltage_uV(uint8_t channel);

    /** @brief 通道电流（软件计算 I = Vshunt/R，需先配置采样电阻） */
    int32_t getCurrent_mA(uint8_t channel);

    /** @brief 通道功率（软件计算 P = Vbus × I） */
    int32_t getPower_mW(uint8_t channel);

    /** @brief 分流电压求和（参与通道由 SCC 位决定），±655.32 mV */
    int32_t getShuntVoltageSum_uV();

    // ============================================================
    //  通道配置
    // ============================================================

    /** @brief 设置通道采样电阻（µΩ，如 10 mΩ 填 10000），≥ 1 mΩ */
    int setShuntR(uint8_t channel, uint32_t rShunt_uOhm);

    /** @brief 读取通道采样电阻（无效通道返回 0） */
    uint32_t getShuntR(uint8_t channel);

    /** @brief 使能通道（立即写 CONFIG，保留其余配置） */
    int enableChannel(uint8_t channel);

    /** @brief 禁用通道 */
    int disableChannel(uint8_t channel);

    /** @brief 通道是否使能 */
    bool isChannelEnabled(uint8_t channel);

    // ============================================================
    //  报警阈值
    // ============================================================

    /** @brief 通道临界报警阈值（µV，0~163800；|Vshunt| ≥ 阈值触发） */
    int setCriticalAlert(uint8_t channel, int32_t microVolt);
    int32_t getCriticalAlert(uint8_t channel);

    /** @brief 通道预警阈值（µV，0~163800） */
    int setWarningAlert(uint8_t channel, int32_t microVolt);
    int32_t getWarningAlert(uint8_t channel);

    /** @brief 分流求和临界阈值（µV，±655320 内） */
    int setShuntVoltageSumLimit(int32_t microVolt);
    int32_t getShuntVoltageSumLimit();

    /** @brief 电源有效上限（mV，0~32760，8 mV 步进） */
    int setPowerUpperLimit(int32_t milliVolt);
    int32_t getPowerUpperLimit();

    /** @brief 电源有效下限（mV，0~32760，8 mV 步进） */
    int setPowerLowerLimit(int32_t milliVolt);
    int32_t getPowerLowerLimit();

    // ============================================================
    //  报警引脚 / 标志
    // ============================================================

    void bindAlertPin(io_ctrl *pin);
    [[nodiscard]] bool isAlertAsserted();

    /** @brief 便捷方法：等同 isAlertAsserted() */
    [[nodiscard]] bool CheckAlert() { return isAlertAsserted(); }

    /** @brief 读取标志位（读后清除锁存标志），见 INA3221_AlertFlags */
    [[nodiscard]] INA3221_AlertFlags readAlertFlags();
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

    /** @brief 设置 Mask/Enable 功能位（bit15 CNVR + bit14-12 SCC） */
    void setAlertMask(uint16_t mask);

    /** @brief 软复位芯片（寄存器恢复默认值，需重新 init） */
    void reset();

    /** @brief 读 Mask/Enable bit0：本次转换周期是否完成 */
    [[nodiscard]] bool isConversionReady();

    /**
     * @brief 等待转换完成
     * @param timeout_ms 超时上限（默认 3000 ms；默认配置
     *        N256×3 通道×2×1100 µs ≈ 1.69 s/周期）
     * @return true = 转换完成；false = 超时
     */
    bool waitConversionReady(uint32_t timeout_ms = 3000);

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

    // ── I2C 读写（INA3221 全部 16-bit 寄存器） ────────────

    void     writeRaw(uint8_t reg, uint16_t data);
    uint16_t readRaw(uint8_t reg);

    // ── 寄存器地址推导 ────────────────────────────────────

    static uint8_t shuntReg(uint8_t channel)    { return 0x01 + channel * 2; }
    static uint8_t busReg(uint8_t channel)      { return 0x02 + channel * 2; }
    static uint8_t criticalReg(uint8_t channel) { return 0x07 + channel * 2; }
    static uint8_t warningReg(uint8_t channel)  { return 0x08 + channel * 2; }

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

    static int32_t rawToShuntVoltage_uV(uint16_t raw);      // (raw>>3) × 40，13 位有符号
    static int32_t rawToBusVoltage_mV(uint16_t raw);        // (raw>>3) × 8，13 位无符号
    static int32_t rawToCurrent_mA(uint16_t rawShunt, uint32_t rShunt_uOhm);
    static int32_t rawToPower_mW(uint16_t rawShunt, uint16_t rawBus, uint32_t rShunt_uOhm);

    // ── 调试 ──────────────────────────────────────────────

    void captureSnapshot(INA3221_Snapshot& snap);
    void dumpRawRegisters(const INA3221_Snapshot& snap);
    void dumpEngineeringData(const INA3221_Snapshot& snap);
};

#endif
