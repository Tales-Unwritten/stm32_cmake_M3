#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_io_ctrl.hpp"
#include <cstdint>

// ============================================================
// Snapshot
// ============================================================

struct INA228_Snapshot
{
    uint16_t config;          // 0x00 配置
    uint16_t adcConfig;       // 0x01 ADC 配置
    uint16_t shuntCal;        // 0x02 分流校准
    uint16_t shunt_tempco;    // 0x03 分流温度系数

    uint64_t vshunt;          // 0x04 分流电压 (24bit)
    uint64_t vbus;            // 0x05 总线电压 (24bit)
    uint16_t temp;            // 0x06 芯片温度 (16bit)
    uint64_t current;         // 0x07 电流结果 (24bit)
    uint64_t power;           // 0x08 功率结果 (24bit)
    uint64_t energy;          // 0x09 电能累积 (40bit)
    uint64_t charge;          // 0x0A 电荷累积 (40bit)
    uint16_t diagAlert;       // 0x0B 诊断与报警标志

    uint16_t sovl;            // 0x0C 分流过压阈值
    uint16_t suvl;            // 0x0D 分流欠压阈值
    uint16_t bovl;            // 0x0E 总线过压阈值
    uint16_t buvl;            // 0x0F 总线欠压阈值
    uint16_t temp_limit;      // 0x10 温度过限阈值
    uint16_t pwr_limit;       // 0x11 功率过限阈值

    uint16_t manufacturerID;  // 0x3E 制造商 ID
    uint16_t deviceID;        // 0x3F 器件 ID
};

// ============================================================
//  Alert 标志位（DIAG_ALRT 寄存器 bit 7-0 状态字段）
//  读后自动清除（锁存模式）或实时反映（非锁存模式）
// ============================================================

struct INA228_AlertFlags
{
    bool memStatus;       // bit 0: 存储器校验错误
    bool conversionReady; // bit 1: 转换完成
    bool powerOverLimit;  // bit 2: 功率超限
    bool busUnderLimit;   // bit 3: 总线欠压
    bool busOverLimit;    // bit 4: 总线过压
    bool shuntUnderLimit; // bit 5: 分流欠压
    bool shuntOverLimit;  // bit 6: 分流过压
    bool tempOverLimit;   // bit 7: 温度超限
    bool mathOverflow;    // bit 9: 数学溢出
    bool chargeOverflow;  // bit 10: 电荷溢出
    bool energyOverflow;  // bit 11: 能量溢出
};

// ============================================================

class INA228
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

    // ── CONFIG 寄存器枚举 ─────────────────────────────────

    enum class Reset : uint8_t    { Normal = 0, SoftReset = 1 };
    enum class Retacc : uint8_t   { Normal = 0, Clear = 1 };
    enum class Convdly : uint8_t  { Ms0 = 0, Ms2 = 1, Ms510 = 0xFF };
    enum class Tempcomp : uint8_t { Disable = 0, Enable = 1 };
    enum class Adcrange : uint8_t { Range_163mv = 0, Range_40mv = 1 };
    enum class Reserved : uint8_t { Reserved_0 = 0 };

    // ── DIAG_ALRT 配置位（bit 15-8，可写） ────────────────

    enum class Alatch : uint8_t     { Disable = 0, Enable = 1 };
    enum class Cnvr : uint8_t       { Disable = 0, Enable = 1 };
    enum class Slowalert : uint8_t  { Disable = 0, Enable = 1 };
    enum class Apol : uint8_t       { ActiveLow = 0, ActiveHigh = 1 };

    // ── 寄存器地址 ────────────────────────────────────────

    enum Reg : uint8_t
    {
        REG_CONFIG           = 0x00, REG_ADC_CONFIG   = 0x01,
        REG_SHUNT_CAL        = 0x02, REG_SHUNT_TEMPCO = 0x03,
        REG_VSHUNT           = 0x04, REG_VBUS         = 0x05,
        REG_DIETEMP          = 0x06, REG_CURRENT      = 0x07,
        REG_POWER            = 0x08, REG_ENERGY       = 0x09,
        REG_CHARGE           = 0x0A, REG_DIAG_ALERT   = 0x0B,
        REG_SOVL             = 0x0C, REG_SUVL         = 0x0D,
        REG_BOVL             = 0x0E, REG_BUVL         = 0x0F,
        REG_TEMP_LIMIT       = 0x10, REG_PWR_LIMIT    = 0x11,
        REG_MANUFACTURER_ID  = 0x3E, REG_DEVICE_ID    = 0x3F,
    };

    // ── ADC 模式 ──────────────────────────────────────────

    enum class ADCMode : uint8_t
    {
        Shutdown                    = 0x00,
        Trig_BusV                   = 0x01, Trig_ShuntV             = 0x02,
        Trig_BusV_ShuntV            = 0x03, Trig_Temp               = 0x04,
        Trig_BusV_Temp              = 0x05, Trig_ShuntV_Temp        = 0x06,
        Trig_BusV_ShuntV_Temp       = 0x07,
        Shutdown2                   = 0x08,
        Cont_BusV                   = 0x09, Cont_ShuntV             = 0x0A,
        Cont_BusV_ShuntV            = 0x0B, Cont_Temp               = 0x0C,
        Cont_BusV_Temp              = 0x0D, Cont_ShuntV_Temp        = 0x0E,
        Cont_BusV_ShuntV_Temp       = 0x0F,
    };

    enum class ConvTime : uint8_t
    {
        Us50 = 0, Us84 = 1, Us150 = 2, Us280 = 3,
        Us540 = 4, Us1052 = 5, Us2074 = 6, Us4120 = 7,
    };

    enum class AvgSample : uint8_t
    {
        N1 = 0, N4 = 1, N16 = 2, N64 = 3,
        N128 = 4, N256 = 5, N512 = 6, N1024 = 7,
    };

    // ============================================================
    //  Config 结构体（用户可定制）
    // ============================================================

    struct Config
    {
        // ── CONFIG 寄存器 ──
        Reset     reset      = Reset::Normal;
        Retacc    retacc     = Retacc::Normal;
        Convdly   convdly    = Convdly::Ms0;
        Tempcomp  tempcomp   = Tempcomp::Disable;
        Adcrange  adcrange   = Adcrange::Range_163mv;
        Reserved  reserved   = Reserved::Reserved_0;

        // ── ADC 寄存器 ──
        ADCMode   adcMode        = ADCMode::Cont_BusV_ShuntV_Temp;
        ConvTime  busConvTime    = ConvTime::Us1052;
        ConvTime  shuntConvTime  = ConvTime::Us1052;
        ConvTime  tempConvTime   = ConvTime::Us280;
        AvgSample averaging      = AvgSample::N128;

        // ── 报警配置（bit 15-8，可写） ──
        Alatch    alatch     = Alatch::Enable;       // 锁存报警
        Cnvr      cnvr       = Cnvr::Disable;        // 转换完成不触发
        Slowalert slowalert  = Slowalert::Disable;   // 不用平均数据比较
        Apol      apol       = Apol::ActiveLow;      // 开漏低有效

        // ── 报警引脚自动配置（0=不使用） ──
        GPIO_TypeDef* alert_port = nullptr;         // GPIO 端口，如 GPIOB
        pin_enum_t    alert_pin  = pin_none;         // 引脚掩码，如 GPIO_PIN_5

        // ── 校准参数 ──
        // r_shunt_uOhm 非 0 时，init()/setConfig() 用 maxCurrent_mA 与采样电阻
        // 自动计算 SHUNT_CAL（= 13107.2e6 × LSB × R，ADCRANGE=1 时 ×4），
        // 等效 GitHub 版 setMaxCurrentShunt；保持 0 则使用下方 shuntCal 原值
        uint16_t shuntCal       = 0x1000;
        uint32_t r_shunt_uOhm   = 1000;     // 采样电阻 µΩ（0 = 手动校准模式）
        int32_t  maxCurrent_mA  = 163840;   // 最大电流 mA（用于 LSB 计算）
    };

    // ── 校准错误码（setMaxCurrentShunt 返回值）──
    enum ErrCode : int
    {
        ERR_NONE           = 0,       // 成功
        ERR_MAXCURRENT_LOW = 1,       // 最大电流过小
        ERR_SHUNT_LOW      = 2,       // 采样电阻过小
        ERR_CAL_OVERFLOW   = 3,       // SHUNT_CAL 超出 16 位
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit INA228(inter_i2c_bus* bus, uint8_t addr = ADDR_0x40);
    explicit INA228(inter_i2c_bus* bus, uint8_t addr, const Config& cfg);
    ~INA228();

    void init();

    // ============================================================
    //  校准（对齐 GitHub 版 setMaxCurrentShunt）
    // ============================================================

    /**
     * @brief 由最大电流与采样电阻自动计算 Current LSB 与 SHUNT_CAL 并写入芯片
     * @param maxCurrent_mA 预期最大电流 mA（决定 Current LSB = maxCurrent / 2^19）
     * @param rShunt_uOhm   采样电阻 µΩ（如 1 mΩ 填 1000）
     * @return ErrCode，0 = 成功；失败时芯片状态不变
     * @note  SHUNT_CAL = 13107.2e6 × LSB × R；ADCRANGE=1（40.96 mV 档）时 ×4
     */
    int setMaxCurrentShunt(int32_t maxCurrent_mA, uint32_t rShunt_uOhm);

    /** @brief 是否已校准 */
    [[nodiscard]] bool isCalibrated() const { return _currentLSB_nA != 0; }

    /** @brief 当前 Current LSB（nA/LSB） */
    int64_t getCurrentLSB_nA() const { return _currentLSB_nA; }

    // ============================================================
    //  连接 / 错误
    // ============================================================

    /** @brief I2C 探测：读 ManufacturerID 并检查总线错误 */
    [[nodiscard]] bool isConnected();

    /** @brief 最近一次寄存器操作错误码（0 = 无错误） */
    [[nodiscard]] int getLastError();

    // ============================================================
    //  测量 API
    // ============================================================

    int32_t getBusVoltage_mV();
    int32_t getShuntVoltage_uV();
    int32_t getCurrent_mA();
    int32_t getTemperature_mC();
    int32_t getPower_mW();
    int64_t getEnergy_mJ();
    int64_t getCharge_mC();

    // ============================================================
    //  阈值配置
    // ============================================================

    void set_sovl_limit_mA(int32_t sovl_mA);
    void set_suvl_limit_mA(int32_t suvl_mA);
    void set_bovl_limit_mV(uint16_t bovl_mV);
    void set_buvl_limit_mV(uint16_t buvl_mV);
    void set_temp_limit_C(uint8_t temp_C);
    void set_pwr_limit_mW(uint32_t pwr_mW);

    /** @brief 写 SHUNT_TEMPCO 寄存器（0..16383 ppm；配合 Config::tempcomp 使用） */
    void setShuntTempco(uint16_t ppm);

    /** @brief 软复位芯片（寄存器恢复默认值，需重新 init/校准） */
    void reset();

    // ============================================================
    //  报警引脚 / 标志
    // ============================================================

    /**
     * @brief 绑定外部 ALERT 引脚（开漏输出，需外部上拉）
     * @param pin  已初始化为 INPUT + PULLUP 的 io_ctrl 对象
     * @note  调用时机：init() 之后。传入 nullptr 解除绑定。
     */
    void bindAlertPin(io_ctrl *pin);

    /**
     * @brief 读取 DIAG_ALRT 寄存器，解析为结构化标志
     * @note  锁存模式下，读取会自动清除锁存的报警位
     */
    [[nodiscard]] INA228_AlertFlags readAlertFlags();

    /**
     * @brief 快速检查 ALERT 引脚是否被拉低（轮询用）
     * @note  需先通过 bindAlertPin() 绑定引脚
     */
    [[nodiscard]] bool isAlertAsserted();

    /** @brief 便捷方法：等同 isAlertAsserted() */
    [[nodiscard]] bool CheckAlert() { return isAlertAsserted(); }

    /**
     * @brief 解锁报警（退出锁存状态）
     * @note  读取 DIAG_ALRT → 写 0x0001 到配置位 = 软复位报警状态机
     */
    void clearAlertLatch();

    /** @brief 读取 DIAG_ALRT 原始值（调试用，排查持续报警原因） */
    uint16_t readDiagAlertRaw();

    // ============================================================
    //  设备信息
    // ============================================================

    uint16_t getManufacturerID();
    uint16_t getDeviceID();

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

    // ── I2C 读写 ──────────────────────────────────────────

    void     writeRaw(uint8_t reg, uint64_t data, uint8_t len);
    uint64_t readRaw(uint8_t reg);
    static uint8_t regLen(uint8_t reg);

    // ── 寄存器构建 ────────────────────────────────────────

    uint16_t buildConfigReg()    const;
    uint16_t buildADCConfigReg() const;
    uint16_t buildDiagAlertCfg() const;   // 仅配置位 15-8

    // ── 符号扩展 ──────────────────────────────────────────

    template<uint8_t Bits>
    static constexpr int64_t signExtend(uint64_t raw)
    {
        constexpr int shift = 64 - Bits;
        return (static_cast<int64_t>(raw << shift)) >> shift;
    }

    // ── 原始值 → 物理量 ───────────────────────────────────

    static int32_t rawToBusVoltage_mV(uint64_t raw);
    int32_t rawToShuntVoltage_uV(uint64_t raw) const;   // 按 ADCRANGE 区分 LSB
    int32_t rawToCurrent_mA(uint64_t raw) const;
    static int32_t rawToTemperature_mC(uint64_t raw);
    int32_t rawToPower_mW(uint64_t raw) const;
    int64_t rawToEnergy_mJ(uint64_t raw) const;         // 40-bit 无符号
    int64_t rawToCharge_mC(uint64_t raw) const;         // 40-bit 有符号

    // ── 更新 Current LSB ──────────────────────────────────

    void _updateCurrentLSB();

    // ── 调试 ──────────────────────────────────────────────

    void captureSnapshot(INA228_Snapshot& snap);
    void dumpRawRegisters(const INA228_Snapshot& snap);
    void dumpEngineeringData(const INA228_Snapshot& snap);
};

#endif
