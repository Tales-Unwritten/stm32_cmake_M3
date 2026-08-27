#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// MCP9808 数字温度传感器（I2C）
//
// 来源库 : MCP9808_RT v0.4.2（Rob Tillaart，2020-05-03）
// URL    : https://github.com/RobTillaart/MCP9808_RT
//
// 移植说明:
//  - 禁止 float/double：温度以 m°C（×1000）定点返回
//    LSB = 0.0625°C = 62.5 m°C；换算同参考库 readFloat()
//  - 温度/阈值寄存器均为 12 位幅值 + bit12 符号位
//    阈值写入按 0.25°C 取整（同参考库 writeFloat: round(f×4)×4）
//  - 芯片无软件复位命令（数据手册无 RST 位）：上电即复位，
//    init() 重写 CONFIG 解除关断模式并应用配置
//  - 无动态内存 / STL / 异常，C++17，Cortex-M0+
// ============================================================

class MCP9808
{
public:

    // ── I2C 地址（A2 A1 A0 引脚决定，最多 8 个传感器）──

    enum Addr : uint8_t
    {
        ADDR_0x18 = 0x18, ADDR_0x19 = 0x19,
        ADDR_0x1A = 0x1A, ADDR_0x1B = 0x1B,
        ADDR_0x1C = 0x1C, ADDR_0x1D = 0x1D,
        ADDR_0x1E = 0x1E, ADDR_0x1F = 0x1F,
    };

    // ── 寄存器地址 ──

    enum Reg : uint8_t
    {
        REG_RFU    = 0x00,   // 保留寄存器 (R)
        REG_CONFIG = 0x01,   // 配置寄存器 (R/W, reset=0x0000)
        REG_TUPPER = 0x02,   // 上限报警阈值 (R/W)
        REG_TLOWER = 0x03,   // 下限报警阈值 (R/W)
        REG_TCRIT  = 0x04,   // 临界报警阈值 (R/W)
        REG_TA     = 0x05,   // 温度寄存器 (R)
        REG_MID    = 0x06,   // 制造商 ID (R, 0x0054)
        REG_DID    = 0x07,   // 设备 ID/版本 (R, 0x0400)
        REG_RES    = 0x08,   // 分辨率寄存器 (R/W, 低 2 位有效)
    };

    // ── CONFIG 寄存器位掩码 ──

    static constexpr uint16_t CFG_THYSTERESIS = 0x0600;   // bit12-11 迟滞 0/1.5/3/6°C
    static constexpr uint16_t CFG_SHUTDOWN    = 0x0100;   // bit8  关断模式（省电）
    static constexpr uint16_t CFG_CRIT_LOCK   = 0x0080;   // bit7  锁定 TCRIT 寄存器
    static constexpr uint16_t CFG_WIN_LOCK    = 0x0040;   // bit6  锁定 TUPPER/TLOWER
    static constexpr uint16_t CFG_INT_CLEAR   = 0x0020;   // bit5  中断标志清除（写 1）
    static constexpr uint16_t CFG_ALERT_STATUS= 0x0010;   // bit4  报警状态（只读）
    static constexpr uint16_t CFG_ALERT_CTRL  = 0x0008;   // bit3  ALERT 引脚控制
    static constexpr uint16_t CFG_ALERT_SELECT= 0x0004;   // bit2  报警选择
    static constexpr uint16_t CFG_ALERT_POLAR = 0x0002;   // bit1  报警极性
    static constexpr uint16_t CFG_ALERT_MODE  = 0x0001;   // bit0  报警模式

    // ── 迟滞 / 分辨率 ──

    enum class Hysteresis : uint8_t { HYS_0C = 0, HYS_1_5C = 1, HYS_3C = 2, HYS_6C = 3 };
    enum class Resolution : uint8_t
    {
        RES_0_5C = 0,      // 0.5°C   ，30 ms
        RES_0_25C = 1,     // 0.25°C  ，65 ms
        RES_0_125C = 2,    // 0.125°C ，130 ms
        RES_0_0625C = 3,   // 0.0625°C，250 ms
    };

    // ── 温度状态（getTemperature_mC() 读取时更新）──

    enum Status : uint8_t
    {
        STA_OK           = 0,   // 正常
        STA_BELOW_TLOWER = 1,   // 低于 TLOWER
        STA_ABOVE_TUPPER = 2,   // 高于 TUPPER
        STA_ABOVE_TCRIT  = 3,   // 高于 TCRIT
    };

    // ============================================================
    //  Config 结构体
    // ============================================================

    struct Config
    {
        Hysteresis hysteresis  = Hysteresis::HYS_0C;
        bool       shutdown    = false;    // 关断模式（读温度前必须解除）
        bool       critLock    = false;    // 锁定 TCRIT（防误写）
        bool       winLock     = false;    // 锁定 TUPPER/TLOWER（防误写）
        bool       alertCtrl   = false;    // ALERT 引脚作为中断输出
        bool       alertSel    = false;    // 0 = TUPPER/TLOWER 触发, 1 = TCRIT 触发
        bool       alertPolar  = false;    // 0 = 低有效, 1 = 高有效
        bool       alertMode   = false;    // 0 = 比较器模式, 1 = 中断模式
        Resolution resolution  = Resolution::RES_0_0625C;   // 默认最高精度
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit MCP9808(inter_i2c_bus* bus, uint8_t addr = ADDR_0x18);

    MCP9808(const MCP9808&) = delete;
    MCP9808& operator=(const MCP9808&) = delete;

    ~MCP9808();

    /**
     * @brief 上电初始化：写入 CONFIG + 分辨率寄存器
     * @note  MCP9808 无软件复位命令，上电本身即复位；
     *        写 CONFIG（shutdown=0）可解除关断模式，等效"上电复位 + 配置"
     */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    /** @brief I2C 探测（仅发地址检查 ACK） */
    [[nodiscard]] bool isConnected();

    /** @brief 最近一次寄存器操作错误码（0 = 无错误；读取前自动清零） */
    [[nodiscard]] int getLastError();

    // ============================================================
    //  测量
    // ============================================================

    /** @brief 读取温度（m°C ×1000；返回前更新状态标志） */
    int32_t getTemperature_mC();

    /**
     * @brief 温度状态（需先调用 getTemperature_mC()）
     * @return 0=正常, 1=低于 TLOWER, 2=高于 TUPPER, 3=高于 TCRIT
     */
    uint8_t getStatus() const { return _status; }

    // ============================================================
    //  偏移
    // ============================================================

    void    setOffset_mC(int32_t offset_mC);    // 默认 0
    int32_t getOffset_mC() const { return _offset_mC; }

    // ============================================================
    //  CONFIG 寄存器
    // ============================================================

    void     setConfigRegister(uint16_t configuration);   // 原值写入（不更新 _cfg）
    uint16_t getConfigRegister();
    void     setConfig(const Config& cfg);                // 更新 _cfg 并写入
    const Config& getConfig() const { return _cfg; }

    // ============================================================
    //  报警阈值（m°C ×1000；写入按 0.25°C 取整，同参考库）
    // ============================================================

    void    setTupper_mC(int32_t t_mC);
    int32_t getTupper_mC();
    void    setTlower_mC(int32_t t_mC);
    int32_t getTlower_mC();
    void    setTcritical_mC(int32_t t_mC);
    int32_t getTcritical_mC();

    // ============================================================
    //  分辨率
    // ============================================================

    void       setResolution(Resolution res);
    Resolution getResolution();

    // ============================================================
    //  设备信息
    // ============================================================

    uint16_t getManufacturerID();    // 期望 0x0054
    uint8_t  getDeviceID();          // 期望 0x04
    uint8_t  getRevision();          // 版本号
    uint16_t getRFU();               // 保留寄存器（调试用）

private:

    inter_i2c_dev _dev;
    Config        _cfg;
    int32_t       _offset_mC = 0;
    uint8_t       _status    = 0;

    // ── I2C 读写（MCP9808 全部 16-bit 寄存器）──

    void     writeRaw(uint8_t reg, uint16_t data);
    uint16_t readRaw(uint8_t reg);

    // ── 寄存器构建 ──

    uint16_t buildConfigReg() const;

    // ── 转换 ──

    static int32_t  raw12To_mC(uint16_t raw);         // 12 位 + 符号位 → m°C
    static uint16_t mCToThresholdReg(int32_t t_mC);   // m°C → 阈值寄存器值
};

#endif
