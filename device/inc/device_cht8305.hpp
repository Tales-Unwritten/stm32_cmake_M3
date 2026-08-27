#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_i2c_bus.hpp"
#include <cstdint>

// ============================================================
// CHT8305 温湿度传感器（I2C）
//
// 来源库 : CHT8305 v0.2.4（Rob Tillaart）
// URL    : https://github.com/RobTillaart/CHT8305
//
// 移植说明:
//  - 禁止 float/double：温度以 m°C（×1000）、湿度以 %RH×100 定点返回
//  - 读温度寄存器会触发一次新转换（约 13 ms @14bit，参考库默认等待 14 ms）；
//    参考库限制 1 秒内只能调一次 read 系函数（本移植保留）
//  - 转换触发 = "写寄存器地址（无数据）→ 延时 → 读数据"两阶段事务，
//    inter_i2c_dev 的组合事务（写寄存器 + 立即读）无法插入延时，
//    故触发写用 inter_i2c_bus 原语实现（见 _triggerConversion 注释）
//  - ⚠ 电阻测量：参考库 v0.2.4 未提供独立的电阻测量 API。
//    寄存器 0x04 在 README 中标注为 VOLTAGE，公式 V = raw×5V/32768
//    为 best guess（参考库原注释 "meaning of this function is unclear"），
//    此处提供 getVoltageRaw()/getVoltage_mV() 原样移植；
//    如需电阻测量请对照 CHT8305 数据手册确认 0x04 实际含义
//  - 无动态内存 / STL / 异常，C++17，Cortex-M0+
// ============================================================

class CHT8305
{
public:

    // ── I2C 地址（AD0 引脚决定）──

    enum Addr : uint8_t
    {
        ADDR_0x40 = 0x40,   // AD0 = GND（默认）
        ADDR_0x41 = 0x41,   // AD0 = VCC
        ADDR_0x42 = 0x42,   // AD0 = SDA
        ADDR_0x43 = 0x43,   // AD0 = SCL
    };

    // ── 寄存器地址 ──

    enum Reg : uint8_t
    {
        REG_TEMPERATURE  = 0x00,   // 温度（读触发转换）
        REG_HUMIDITY     = 0x01,   // 湿度
        REG_CONFIG       = 0x02,   // 配置 (R/W)
        REG_ALERT        = 0x03,   // 报警阈值 (R/W)
        REG_VOLTAGE      = 0x04,   // 电压（含义不明，见文件头说明）
        REG_MANUFACTURER = 0xFE,   // 制造商 ID (R, 期望 0x5959)
        REG_VERSION      = 0xFF,   // 版本 ID (R, 测试返回 0x8305)
    };

    // ── CONFIG 寄存器位掩码 ──

    static constexpr uint16_t CFG_SOFT_RESET    = 0x8000;   // bit15 软复位（写 1 触发）
    static constexpr uint16_t CFG_CLOCK_STRETCH = 0x4000;   // bit14 时钟延展
    static constexpr uint16_t CFG_HEATER        = 0x2000;   // bit13 加热器（自行控时序）
    static constexpr uint16_t CFG_MODE          = 0x1000;   // bit12 1=同测 T+H, 0=单测
    static constexpr uint16_t CFG_VCCS          = 0x0800;   // bit11 VCC 状态 (R, 1=>2.8V)
    static constexpr uint16_t CFG_TEMP_RES      = 0x0400;   // bit10 1=11bit, 0=14bit
    static constexpr uint16_t CFG_HUMI_RES      = 0x0300;   // bit9-8 10=8bit, 01=11bit, 00=14bit
    static constexpr uint16_t CFG_ALERT_MODE    = 0x00C0;   // bit7-6 报警触发模式
    static constexpr uint16_t CFG_ALERT_PENDING = 0x0020;   // bit5 报警挂起 (R)
    static constexpr uint16_t CFG_ALERT_HUMI    = 0x0010;   // bit4 湿度报警 (R)
    static constexpr uint16_t CFG_ALERT_TEMP    = 0x0008;   // bit3 温度报警 (R)
    static constexpr uint16_t CFG_VCC_ENABLE    = 0x0004;   // bit2 VCC 测量使能
    static constexpr uint16_t CFG_RESERVED      = 0x0003;   // bit1-0 保留（写 0）

    // ── 错误码（同参考库）──

    enum ErrCode : int
    {
        ERR_OK       = 0,
        ERR_ADDR     = -10,    // 地址不在 0x40..0x43
        ERR_I2C      = -11,    // I2C 传输失败
        ERR_CONNECT  = -12,    // 探测无应答
        ERR_BUFSIZE  = -13,    // 读回字节数不符（本移植不产生，保留兼容）
        ERR_LASTREAD = -20,    // 距上次读取不足 1 秒
        ERR_GENERIC  = -999,
    };

    // ============================================================
    //  Config 结构体（默认 = 芯片复位值 0x1004：同测 T+H + VCC 使能）
    // ============================================================

    struct Config
    {
        bool    clockStretch     = false;   // bit14 时钟延展
        bool    heater           = false;   // bit13 加热器（用户负责时序）
        bool    modeBoth         = true;    // bit12 1=同测 T+H, 0=单测
        bool    tempRes11bit     = false;   // bit10 1=11bit, 0=14bit（默认）
        uint8_t humiRes          = 0;       // bit9-8 2=8bit, 1=11bit, 0=14bit（默认）
        uint8_t alertTriggerMode = 0;       // bit7-6 0=T|H, 1=T, 2=H, 3=T&H
        bool    vccEnable        = true;    // bit2 VCC 测量使能（默认）
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit CHT8305(inter_i2c_bus* bus, uint8_t addr = ADDR_0x40);

    CHT8305(const CHT8305&) = delete;
    CHT8305& operator=(const CHT8305&) = delete;

    ~CHT8305();

    /** @brief 初始化：地址校验 + 连接检查 + 写配置寄存器 */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool isConnected();
    /** @brief 最近一次操作错误码（0 = 无错误；读取后自动清零） */
    [[nodiscard]] int getLastError();
    uint8_t getAddress() const { return _addr; }

    // ============================================================
    //  测量
    //  ⚠ 参考库限制：read/readTemperature/readHumidity 互斥，
    //    1 秒内只能调用其中一个（数据手册推荐 1 次/秒）
    // ============================================================

    int read();                // 同时读温度 + 湿度（4 字节一次读回）
    int readTemperature();     // 只读温度（触发转换 + 延时）
    int readHumidity();        // 只读湿度（不触发转换，更快）

    /** @brief 上次测量温度（m°C ×1000） */
    int32_t getTemperature_mC() const { return _temperature_mC; }
    /** @brief 上次测量湿度（%RH×100） */
    int32_t getHumidity_x100()  const { return _humidity_x100; }
    /** @brief 上次读取时刻（get_tick） */
    uint32_t lastRead() const { return _lastReadTick; }

    // ── 转换延时（默认 14 ms；低于 8 按 8 处理，同参考库）──

    void    setConversionDelay(uint8_t cd = 14);
    uint8_t getConversionDelay() const { return _conversionDelay; }

    // ============================================================
    //  偏移修正
    // ============================================================

    void    setTemperatureOffset_mC(int32_t offset_mC);    // 默认 0
    int32_t getTemperatureOffset_mC() const { return _tempOffset_mC; }
    void    setHumidityOffset_x100(int32_t offset_x100);   // 默认 0
    int32_t getHumidityOffset_x100() const { return _humOffset_x100; }

    // ============================================================
    //  CONFIG 寄存器
    // ============================================================

    void     setConfigRegister(uint16_t bitmask);   // 原值写入（bit1-0 强制写 0）
    uint16_t getConfigRegister();                   // 读失败返回 0
    void     setConfig(const Config& cfg);          // 更新 _cfg 并写入
    const Config& getConfig() const { return _cfg; }

    void  softReset();                   // 置 bit15 → 芯片复位回默认值
    void  setI2CClockStretch(bool on = false);
    bool  getI2CClockStretch();
    void  setHeaterOn(bool on = false);  // ⚠ 用户自行控制加热时序
    bool  getHeater();
    void  setMeasurementMode(bool both = true);
    bool  getMeasurementMode();
    bool  getVCCstatus();                // 1 = VCC > 2.8V
    void  setTemperatureResolution(uint8_t res = 0);   // 1 = 11bit, 0 = 14bit
    uint8_t getTemperatureResolution();
    void  setHumidityResolution(uint8_t res = 0);      // 2 = 8bit, 1 = 11bit, 0 = 14bit
    uint8_t getHumidityResolution();
    void  setVCCenable(bool enable = true);
    bool  getVCCenable();

    // ============================================================
    //  报警（寄存器 0x03，仅上限报警，无下限）
    // ============================================================

    bool  setAlertTriggerMode(uint8_t mode = 0);   // 0..3，非法返回 false
    uint8_t getAlertTriggerMode();
    bool  getAlertPendingStatus();
    bool  getAlertHumidityStatus();
    bool  getAlertTemperatureStatus();
    /**
     * @brief 设置报警上限（温度 m°C，湿度 %RH×100）
     * @note  取值按寄存器分辨率截断；T 范围 [-40,125]°C，RH [0,100]%
     */
    bool  setAlertLevels_mC(int32_t t_mC, int32_t rh_x100);
    int32_t getAlertLevelTemperature_mC();   // 读取截断后的实际值
    int32_t getAlertLevelHumidity_x100();

    // ============================================================
    //  电压（含义不明，参考库 best guess: V = raw × 5V / 32768）
    // ============================================================

    uint16_t getVoltageRaw();
    int32_t  getVoltage_mV();

    // ============================================================
    //  设备信息
    // ============================================================

    uint16_t getManufacturer();    // 期望 0x5959
    uint16_t getVersionID();       // 测试返回 0x8305

private:

    inter_i2c_bus* _bus;    // 转换触发原语用（见文件头说明）
    inter_i2c_dev  _dev;
    uint8_t        _addr;

    Config   _cfg;
    int32_t  _temperature_mC  = 0;
    int32_t  _humidity_x100   = 0;
    uint32_t _lastReadTick    = 0;          // 0 = 尚未读过
    uint8_t  _conversionDelay = 14;
    int32_t  _tempOffset_mC   = 0;
    int32_t  _humOffset_x100  = 0;
    int      _error = ERR_OK;

    // ── 寄存器构建 / 掩码操作 ──

    uint16_t buildConfigReg() const;
    void     _setConfigMask(uint16_t mask);
    void     _clrConfigMask(uint16_t mask);

    // ── 两阶段读取（转换触发 + 延时 + 读数据） ──

    bool _triggerConversion(uint8_t reg);    // 仅写寄存器地址（无数据）
    bool _readConverted(uint8_t reg, uint64_t* data, uint8_t len);

    // ── 定点换算（参考库公式，int64 中间量防溢出） ──

    static int32_t rawToTemp_mC(uint16_t raw);    // raw×165000/65535 − 40000
    static int32_t rawToHum_x100(uint16_t raw);   // raw×10000/65535
};

#endif
