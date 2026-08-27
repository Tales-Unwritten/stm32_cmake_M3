#pragma once

#ifdef __cplusplus

#include <cstdint>
#include "inter_spi_bus.hpp"

// ============================================================
//  MAX31855 K 型热电偶温度计（SPI）驱动
// ============================================================
//  来源库 : MAX31855_RT v0.6.2（Rob Tillaart，2014-01-01）
//    URL : https://github.com/RobTillaart/MAX31855_RT
//
//  移植说明（对照参考库 0.6.2）：
//    - 仅保留硬件 SPI 路径；构造收 spi_port&（软件 CS 由用户配置进
//      spi_port，或共享总线时由用户在 spi_port 外部自行控制——
//      器件每次传输内部都会 cs_select()/cs_deselect()，若 spi_port
//      未配置 CS（has_cs()==false）这两个调用为空操作）
//    - 禁止 float/double：温度用 m°C 定点（0.25°C/LSB → ×250 精确，
//      冷端 0.0625°C/LSB → ×125/2，截断误差 ≤ 0.5 m°C）
//    - Seebeck 系数（µV/°C）×1000 定点存储；非 K 型修正公式
//      Vout = K_TC·ΔT，T' = Vout/SC + 冷端 + 偏移 全部整数换算
//    - 裁剪：软件 SPI、setSPIspeed（速度由 spi_port 的 prescaler
//      决定）、lastRead 用 get_tick() 代替 millis()
//    - 无浮点、无堆分配、无 STL/异常，C++17，Cortex-M0+
//
//  SPI 模式：Mode 0（CPOL=0, CPHA=0）——参考库使用 SPI_MODE0；
//  数据手册串行接口：SO 在 SCK 下降沿输出、上升沿采样，
//  Mode 0 与 Mode 3 均兼容（本驱动按参考库用 Mode 0）。
//  注意：读到全 1（0xFFFFFFFF）说明 MISO 无上拉/未连接，
//  read() 返回 STATUS_NO_COMMUNICATION。
// ============================================================

class MAX31855
{
public:

    // ── 状态常量（read() 返回值）──────────────────────────

    static constexpr uint8_t STATUS_OK                = 0x00;   // 正常
    static constexpr uint8_t STATUS_OPEN_CIRCUIT      = 0x01;   // bit0: 热电偶开路
    static constexpr uint8_t STATUS_SHORT_TO_GND      = 0x02;   // bit1: 短路到 GND
    static constexpr uint8_t STATUS_SHORT_TO_VCC      = 0x04;   // bit2: 短路到 VCC
    static constexpr uint8_t STATUS_ERROR             = 0x07;   // 任一故障位
    static constexpr uint8_t STATUS_NOREAD            = 0x80;   // 尚未读
    static constexpr uint8_t STATUS_NO_COMMUNICATION  = 0x81;   // 全 1 帧（MISO 上拉缺失）

    // ── 无效温度（参考库 -999，m°C 定点）─────────────────

    static constexpr int32_t NO_TEMPERATURE_mC = -999000;

    // ── Seebeck 系数（µV/°C × 1000，数据手册 P8）──────────
    // 仅 K 型（默认）返回温度直读；设置其他系数时用修正公式换算

    static constexpr uint32_t E_TC =  76373;   // 76.373 µV/°C
    static constexpr uint32_t J_TC =  57953;   // 57.953 µV/°C
    static constexpr uint32_t K_TC =  41276;   // 41.276 µV/°C（默认）
    static constexpr uint32_t N_TC =  36256;   // 36.256 µV/°C
    static constexpr uint32_t R_TC =  10506;   // 10.506 µV/°C
    static constexpr uint32_t S_TC =   9587;   //  9.587 µV/°C
    static constexpr uint32_t T_TC =  52180;   // 52.180 µV/°C

    // ============================================================
    //  构造
    // ============================================================

    /**
     * @param spi 共享 SPI 总线。单设备时把本器件的 CS 配置进 spi_port
     *            （cs_port/cs_pin 非 0）；多设备共用总线时由用户在
     *            外部控制 CS，spi_port 的 cs_select/cs_deselect 为空操作
     */
    explicit MAX31855(spi_bus &spi);

    MAX31855(const MAX31855&) = delete;
    MAX31855& operator=(const MAX31855&) = delete;

    /** @brief 复位内部状态（温度/状态缓存、偏移、Seebeck 系数） */
    void begin();

    /**
     * @brief 读取一次 32 位帧并更新温度缓存
     * @return 状态字节（STATUS_*，0 = 正常）
     * @note  故障位仅影响状态，冷端/温度数据仍有效（参考库 0.4.0 起行为）
     */
    uint8_t read();

    // ============================================================
    //  测量结果（read() 之后有效）
    // ============================================================

    /** @brief 冷端（芯片内部）温度 m°C */
    int32_t getInternal_mC() const { return _internal_mC; }

    /**
     * @brief 热电偶温度 m°C（K 型 = 直读 + 偏移；
     *        其他 Seebeck 系数 = 修正公式 + 偏移）
     */
    int32_t getTemperature_mC();

    uint8_t getStatus() const { return _status; }

    // ── 故障查询（等价于比较状态位）───────────────────────

    bool openCircuit()     { return _status == STATUS_OPEN_CIRCUIT; }
    bool shortToGND()      { return _status == STATUS_SHORT_TO_GND; }
    bool shortToVCC()      { return _status == STATUS_SHORT_TO_VCC; }
    bool genericError()    { return _status == STATUS_ERROR; }
    bool noRead()          { return _status == STATUS_NOREAD; }
    bool noCommunication() { return _status == STATUS_NO_COMMUNICATION; }

    // ============================================================
    //  校准
    // ============================================================

    /** @brief 热电偶偏移校准（m°C，参考库 setOffset 的 °C ×1000） */
    void setOffset_mC(int32_t t)        { _offset_mC = t; }
    int32_t getOffset_mC() const        { return _offset_mC; }

    /**
     * @brief 设置 Seebeck 系数（µV/°C × 1000），可用上方 E_TC..T_TC
     * @param sc_x1000 如 41276 = 41.276 µV/°C；K 型用 K_TC 时走直读路径
     */
    void setSeebeckCoefficient_x1000(uint32_t sc_x1000) { _seebeck_x1000 = sc_x1000; }
    uint32_t getSeebeckCoefficient_x1000() const        { return _seebeck_x1000; }

    // ============================================================
    //  调试
    // ============================================================

    /** @brief 最近一次原始 32 位帧 */
    uint32_t getRawData() const { return _rawData; }

    /** @brief 最近一次成功读取的时刻（get_tick） */
    uint32_t lastRead() const   { return _lastTimeRead; }

private:

    uint32_t _read();          // 低层：CS 低 → 4 字节全双工读 → CS 高

    spi_bus &_spi;
    uint8_t  _status;
    int32_t  _internal_mC;     // 冷端温度 m°C
    int32_t  _temperature_mC;  // 热电偶温度 m°C（直读，未含偏移/修正）
    int32_t  _offset_mC;       // 偏移校准 m°C
    uint32_t _seebeck_x1000;   // Seebeck 系数 ×1000
    uint32_t _lastTimeRead;    // get_tick
    uint32_t _rawData;         // 原始帧
};

#endif /* __cplusplus */
