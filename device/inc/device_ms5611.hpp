#pragma once

#ifdef __cplusplus

#include <cstdint>
#include "inter_spi_bus.hpp"

// ============================================================
//  MS5611 高精度气压/温度传感器（SPI）驱动
// ============================================================
//  来源库 : MS5611_SPI v0.4.3（Rob Tillaart，2022-01-18）
//    URL : https://github.com/RobTillaart/MS5611_SPI
//
//  移植说明（对照参考库 0.4.3）：
//    - 构造收 spi_port&；CS 控制约定同其他 SPI 器件（见下）
//    - 禁止 float/double：全部换算定点整数。
//      * 温度：内部 0.01°C（TEMP 中间量），对外 m°C（×10）
//      * 压力：Pascal（int32）
//      * 校准系数 C1..C6 预缩放为 int64 定点常量（见 device_ms5611.cpp
//        顶部公式推导注释，含逐行对应参考库 float 版本）
//    - 仅实现 mathMode=0（数据手册公式）。参考库 mathMode=1 是
//      appnote 的 2 倍因子修正版（C1/C2 ×2、C3/C4 ×2），未移植
//    - 裁剪：软件 SPI、setSPIspeed（速度由 spi_port prescaler 决定）、
//      usesHWSPI、海拔计算 getAltitude/getSeaLevelPressure
//      （依赖 float pow，与定点化要求冲突，未移植）
//    - 转换等待用 delay_us() 忙等（参考库 micros 循环 + yield）；
//      时间戳用 get_tick() 代替 millis()
//    - 无浮点、无堆分配、无 STL/异常，C++17，Cortex-M0+
//    - ⚠️ int64 运算在 Cortex-M0+ 由编译器软模拟，速度较慢但
//      量级（≤ 10 次/秒）无压力
//
//  SPI 模式：Mode 0（CPOL=0, CPHA=0）——参考库使用 SPI_MODE0，
//  数据手册：命令与数据在 SCK 上升沿锁存、下降沿输出。最高 20 MHz。
//
//  协议（数据手册 P9/P10）：
//    复位 0x1E（等 3 ms）；PROM 读 0xA0+reg×2（reg 0..7，2 字节）；
//    转换命令 0x40+OSR（D1 压力）/ 0x50+OSR（D2 温度），
//    OSR 偏移 = (OSR-8)×2：0x40/42/44/46 与 0x50/52/54/56；
//    转换完成轮询：发 0x00 后读 3 字节（参考库按最大转换时间
//    延时后直接读，0x00 帧即读取命令，见 .cpp readADC 注释）
// ============================================================

class MS5611_SPI
{
public:

    // ── 过采样率（转换时间见 .cpp convert()，参考库 osr_t）──

    enum OSR : uint8_t
    {
        OSR_ULTRA_HIGH = 12,   // 约 10 ms
        OSR_HIGH       = 11,   // 约  5 ms
        OSR_STANDARD   = 10,   // 约  3 ms
        OSR_LOW        = 9,    // 约  2 ms
        OSR_ULTRA_LOW  = 8,    // 约  1 ms（默认，参考库向后兼容）
    };

    // ── 错误码（与参考库一致）─────────────────────────────

    enum Err : int
    {
        ERR_OK       = 0,      // 成功
        ERR_2        = 2,      // 兼容 I2C 版占位
        ERR_ADC      = -10,    // ADC 读数异常（0 或 0xFFFF）
        ERR_PROM     = -11,    // PROM 读数异常（0 或 0xFFFF）
        ERR_NOT_READ = -999,   // 尚未读取
    };

    // ============================================================
    //  构造
    // ============================================================

    /**
     * @param spi 共享 SPI 总线（CS 约定见头注释）
     */
    explicit MS5611_SPI(spi_bus &spi);

    MS5611_SPI(const MS5611_SPI&) = delete;
    MS5611_SPI& operator=(const MS5611_SPI&) = delete;

    /** @brief 初始化：复位 + 读 PROM 校准系数（固定 mathMode=0） */
    bool begin();

    /** @brief 探测：执行一次完整读取，成功返回 true */
    bool isConnected();

    /** @brief 复位 + 读 PROM；返回 false 表示 PROM 系数全 0 */
    bool reset();

    // ============================================================
    //  测量
    // ============================================================

    /**
     * @brief 完成一次 D1+D2 转换与定点换算
     * @param bits OSR 8..12（OSR_* 枚举值），越界自动钳位
     * @return ERR_OK / ERR_ADC / ERR_PROM
     */
    int read(uint8_t bits);

    /** @brief 使用当前过采样率的 read() 包装 */
    int read() { return read(_samplingRate); }

    /** @brief 设置过采样率（OSR_*） */
    void setOversampling(OSR rate) { _samplingRate = static_cast<uint8_t>(rate); }
    OSR  getOversampling() const   { return static_cast<OSR>(_samplingRate); }

    // ============================================================
    //  测量结果（read() 成功之后有效）
    // ============================================================

    /** @brief 温度 m°C（内部 0.01°C ×10；含温度偏移） */
    int32_t getTemperature_mC() const;

    /** @brief 压力 Pa（int32；含压力偏移） */
    int32_t getPressure_Pa() const;

    /** @brief 压力 mBar（Pa/100，整数截断；参考库 getPressure 的定点版） */
    int32_t getPressure_mBar() const { return _pressure_Pa / 100; }

    // ============================================================
    //  偏移校准（参考库 setPressureOffset/setTemperatureOffset 定点版）
    // ============================================================

    /** @brief 压力偏移 Pa（用于对本地气象站校准） */
    void    setPressureOffset_Pa(int32_t offset) { _pressureOffset_Pa = offset; }
    int32_t getPressureOffset_Pa() const         { return _pressureOffset_Pa; }

    /** @brief 温度偏移 m°C（补偿自发热，参考库 README） */
    void    setTemperatureOffset_mC(int32_t offset) { _temperatureOffset_mC = offset; }
    int32_t getTemperatureOffset_mC() const         { return _temperatureOffset_mC; }

    // ============================================================
    //  状态 / 调试
    // ============================================================

    /** @brief 最近一次操作结果码（ERR_*） */
    int      getLastResult() const { return _result; }

    /** @brief 最近一次成功读取时刻（get_tick） */
    uint32_t lastRead() const      { return _lastRead; }

    /** @brief 7 个 PROM 寄存器移位异或合并的器件 ID（参考库） */
    uint32_t getDeviceID() const   { return _deviceID; }

    /** @brief 二阶温度补偿开关（默认开） */
    void setCompensation(bool flag) { _compensation = flag; }
    bool getCompensation() const    { return _compensation; }

    // ── PROM 读取（实验/开发用，参考库同款，实时读芯片）──

    /** @brief 制造商 ID（PROM[0]） */
    uint16_t getManufacturer() { return readProm(0); }

    /** @brief 序列号（PROM[7] 高 12 位） */
    uint16_t getSerialCode()   { return static_cast<uint16_t>(readProm(7) >> 4); }

    /** @brief CRC（PROM[7] 低 4 位） */
    uint16_t getCRC()          { return static_cast<uint16_t>(readProm(7) & 0x0F); }

    /** @brief 读指定 PROM 寄存器（0..7，7 = CRC） */
    uint16_t getProm(uint8_t index) { return readProm(index); }

private:

    // ── 命令（数据手册 P10）──────────────────────────────

    static constexpr uint8_t CMD_RESET      = 0x1E;
    static constexpr uint8_t CMD_READ_PROM  = 0xA0;   // + reg×2
    static constexpr uint8_t CMD_CONVERT_D1 = 0x40;   // + OSR 偏移
    static constexpr uint8_t CMD_CONVERT_D2 = 0x50;   // + OSR 偏移
    static constexpr uint8_t CMD_READ_ADC   = 0x00;

    // ── 低层 ─────────────────────────────────────────────

    void     command(uint8_t cmd);                 // CS 低 → 1 字节 → CS 高
    uint32_t readADC();                            // 0x00 帧 + 3 字节应答
    uint16_t readProm(uint8_t reg);                // 0xA0+reg×2 + 2 字节
    void     convert(uint8_t addr, uint8_t bits);  // 发转换命令 + 延时
    void     initConstants();                      // PROM → 定点系数

    spi_bus &_spi;
    uint8_t  _samplingRate;        // OSR_*（8..12）
    int32_t  _temperature_x100;    // 温度，0.01°C 单位（内部中间量）
    int32_t  _pressure_Pa;         // 压力 Pa
    int32_t  _pressureOffset_Pa;   // 压力偏移 Pa
    int32_t  _temperatureOffset_mC;// 温度偏移 m°C
    int      _result;              // 最近结果码
    uint16_t _prom[8];             // 原始 PROM 系数（C[0] 制造商…C[7] CRC）
    int64_t  _c1s, _c2s;           // 定点系数：C1×2^15, C2×2^16
    int64_t  _c3s, _c4s;           // 定点系数：C3（÷2^8）, C4（÷2^7）
    int64_t  _c5s, _c6s;           // 定点系数：C5×2^8,  C6（÷2^23）
    uint32_t _lastRead;            // get_tick 时间戳
    uint32_t _deviceID;            // PROM 移位异或合并 ID
    bool     _compensation;        // 二阶补偿开关
};

#endif /* __cplusplus */
