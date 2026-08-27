#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_i2c_bus.hpp"
#include <cstdint>

// ============================================================
// BH1750FVI 数字光照度传感器（I2C，GY-30 模块）
//
// 来源库 : BH1750FVI_RT v0.3.4（Rob Tillaart）
// URL    : https://github.com/RobTillaart/BH1750FVI_RT
//
// 移植说明:
//  - 禁止 float/double：光照度以 0.01 lx 定点返回（×100）
//    基础换算 raw/1.2 = raw×5/6 [lx]，与参考库 getRaw() 一致
//  - BH1750 无寄存器地址，命令为单字节。inter_i2c_dev 的 freedom_write
//    会额外发送一个尾随字节：如 powerOff(0x01) 后跟 0x00(=powerOn) 会使
//    关断失效，故本类命令/读数据直接用 inter_i2c_bus 原语实现
//    （读数据同参考库：直接 addr+R 读 2 字节，不写任何命令字节）
//  - HIGH2 模式输出为实际照度 2 倍，需 ÷2（数据手册 P11）
//  - 灵敏度因子 MT（changeTiming，默认 69）参与照度换算
//  - setAngle() 未移植：需要 cos()（Lambert 定律），Cortex-M0+ 无 FPU
//  - 无动态内存 / STL / 异常，C++17，Cortex-M0+
// ============================================================

class BH1750FVI
{
public:

    // ── I2C 地址（ADD 引脚：0 = 0x23, 1 = 0x5C）──

    enum Addr : uint8_t
    {
        ADDR_0x23 = 0x23,
        ADDR_0x5C = 0x5C,
    };

    // ── 命令（单字节，无寄存器地址）──

    enum Cmd : uint8_t
    {
        CMD_POWER_ON   = 0x00,
        CMD_POWER_OFF  = 0x01,
        CMD_RESET      = 0x07,     // 重置测量寄存器 + 上电模式
        CMD_CONT_HIGH  = 0x10,     // 连续 高分辨率 1 lx（120 ms）
        CMD_CONT_HIGH2 = 0x11,     // 连续 高分辨率2 0.5 lx（120 ms）
        CMD_CONT_LOW   = 0x13,     // 连续 低分辨率 4 lx（16 ms）
        CMD_ONCE_HIGH  = 0x20,     // 单次 高分辨率（120 ms 后读数）
        CMD_ONCE_HIGH2 = 0x21,     // 单次 高分辨率2（120 ms 后读数）
        CMD_ONCE_LOW   = 0x23,     // 单次 低分辨率（16 ms 后读数）
    };

    // ── 模式索引（也用于 isReady 超时表 {16, 120, 120} ms）──

    enum Mode : uint8_t
    {
        MODE_LOW  = 0,
        MODE_HIGH = 1,
        MODE_HIGH2 = 2,
    };

    // ── 测量时间因子（默认 69，见 changeTiming）──

    static constexpr uint8_t REFERENCE_TIME = 0x45;

    // ── 错误码 ──

    enum ErrCode : int
    {
        ERR_OK           = 0,
        ERR_WIRE_REQUEST = -10,    // 读数据失败（返回上次数据，同参考库）
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit BH1750FVI(inter_i2c_bus* bus, uint8_t addr = ADDR_0x23);

    BH1750FVI(const BH1750FVI&) = delete;
    BH1750FVI& operator=(const BH1750FVI&) = delete;

    ~BH1750FVI();

    /** @brief 初始化：复位内部状态 + 连接检查（同参考库 begin()） */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool isConnected();
    uint8_t getAddress() const { return _address; }
    /** @brief 最近一次操作错误码（0 = 无错误；读取后自动清零） */
    [[nodiscard]] int getLastError();

    // ============================================================
    //  测量
    // ============================================================

    /** @brief 未修正照度（0.01 lx：raw×250/3，无 MT / HIGH2 / 修正因子） */
    int32_t getRawLux_x100();
    /** @brief 修正后照度（0.01 lx：MT、温度、波长、HIGH2） */
    int32_t getLux_x100();

    // ── 电源 / 复位 ──

    void powerOn();
    void powerOff();
    void reset();   // 0x07：重置测量寄存器 + 上电模式

    // ── 模式设置（连续 / 单次）──

    void setContHighRes();
    void setContHigh2Res();
    void setContLowRes();
    void setOnceHighRes();
    void setOnceHigh2Res();
    void setOnceLowRes();

    uint8_t getMode() const { return _mode; }

    /**
     * @brief 测量是否完成（仅单次模式有意义）
     * @note  等待时间 = 模式基准时间 × MT/69：LOW 16 ms，HIGH/HIGH2 120 ms
     */
    bool isReady();

    // ============================================================
    //  测量时间 / 灵敏度
    // ============================================================

    /**
     * @brief 修改测量时间因子 MT（31..254，默认 69）
     * @note  更改后需重新发起测量命令才生效（数据手册 P5/P11）
     */
    void changeTiming(uint8_t time = REFERENCE_TIME);
    uint8_t getSensitivityFactor() const { return _sensitivityFactor; }
    /** @brief 修正系数 MT/69 ×10000（参考库 getCorrectionFactor 定点版） */
    int32_t getCorrectionFactor_x10000() const;

    // ============================================================
    //  修正因子（参考库 setTemperature / setWaveLength 定点版）
    // ============================================================

    void setTemperature(int temp = 20);   // 温度补偿 ≈1%/20°C
    int getTemperature() const { return _temperature; }
    void setWaveLength(int waveLength = 580);   // 波长补偿（分线段性近似）
    int getWaveLength() const { return _waveLength; }
    // 注: 参考库 setAngle() 需 cos()（Lambert 定律入射角补偿），
    //     Cortex-M0+ 无 FPU 未移植

private:

    inter_i2c_bus* _bus;    // 命令 / 读数据原语（见文件头说明）
    inter_i2c_dev  _dev;    // 仅用于 ping()/isConnected()
    uint8_t        _address;

    uint16_t _data = 0;                  // 最近一次读回值（失败时返回）
    int      _error = ERR_OK;
    uint8_t  _sensitivityFactor = REFERENCE_TIME;
    uint8_t  _mode = MODE_HIGH;
    uint32_t _requestTick = 0;
    int      _temperature = 20;
    int      _waveLength  = 580;

    // ── I2C 原语 ──

    uint16_t readData();                 // 失败返回上次数据（同参考库）
    bool     command(uint8_t value);

    // ── 定点换算 / 修正因子 ──

    static int32_t rawToLux_x100(uint16_t raw);        // raw × 250 / 3
    static int32_t tempFactor_x10000(int temp);        // 10000 − (temp−20)×5
    static int32_t waveLengthFactor_x10000(int wl);    // 1/tmp ×10000
};

#endif
