#pragma once

#ifdef __cplusplus

#include <cstdint>
#include "inter_spi_bus.hpp"

// ============================================================
//  MCP3204 / MCP3208 12 位 SPI ADC 驱动
// ============================================================
//  来源库 : MCP_ADC v0.5.2（Rob Tillaart，2019-10-24）
//    URL : https://github.com/RobTillaart/MCP_ADC
//
//  移植说明（对照参考库 0.5.2）：
//    - 仅移植 MCP3204/MCP3208（12 位），10 位 MCP300x 与
//      单通道 MCP3201 未移植；两个芯片协议完全相同，仅通道数
//      不同，用构造参数 channels = 4/8 区分
//    - 构造收 spi_port&；CS 控制约定同其他 SPI 器件：把本器件
//      CS 配进 spi_port，或共享总线时用户在外部控制
//      （spi_port 未配 CS 时内部 cs_select/cs_deselect 为空操作）
//    - 禁止 float/double：电压 uV 定点 = raw × Vref_uV / 4096，
//      int64 中间量。参考库 0.5.2 已移除 setMaxVoltage（旧版 API），
//      本驱动以 setMaxVoltage_mV 重新提供，默认 Vref = 3300 mV
//    - 裁剪：软件 SPI、setSPIspeed（速度由 spi_port prescaler 决定）、
//      usesHWSPI、MCP300x/MCP3201/3202 派生类
//    - 无浮点、无堆分配、无 STL/异常，C++17，Cortex-M0+
//
//  SPI 模式：Mode 0（CPOL=0, CPHA=0）——参考库使用 SPI_MODE0，
//  数据手册 6.0：兼容 Mode 0,0 与 1,1。参考库默认 1 MHz（数据
//  手册标称值，实测 4 MHz 可工作）。
//
//  命令帧（MCP3204/3208 数据手册 fig 6.1，24 时钟 = 3 字节）：
//    byte0: 0x04(起始位) | 0x02(单端/差分) | 0x01(D2=通道 bit2)
//    byte1: D1 D0（通道 bit1..0）置于 bit7..6
//    应答：byte1 低 4 位 + byte2 8 位 = 12 位数据
// ============================================================

class MCP3208
{
public:

    // ============================================================
    //  构造
    // ============================================================

    /**
     * @param spi      共享 SPI 总线（CS 约定见头注释）
     * @param channels 通道数：8 = MCP3208（默认），4 = MCP3204
     */
    explicit MCP3208(spi_bus &spi, uint8_t channels = 8);

    MCP3208(const MCP3208&) = delete;
    MCP3208& operator=(const MCP3208&) = delete;

    /**
     * @brief 初始化。参考库 begin() 用一次 CS 脉冲强制通信
     *        （上电后首次转换可能损坏，见数据手册），这里做一次
     *        哑读等效
     */
    void init();

    // ============================================================
    //  基本信息
    // ============================================================

    uint8_t  channels() const { return _channels; }   // 4 或 8
    uint16_t maxValue() const { return 4095; }        // 12 位

    // ============================================================
    //  测量 API（返回原始 12 位值 0..4095）
    // ============================================================

    /** @brief 单端读取：通道对 GND，channel = 0.._channels-1 */
    int16_t read(uint8_t channel);

    /**
     * @brief 差分读取：channel 选择输入对（MCP3208 有 4 对：
     *        CH0/CH1 = IN0-IN1 / IN1-IN0，CH2/CH3 = IN2-IN3 / IN3-IN2，
     *        CH4/CH5 = IN4-IN5 / IN5-IN4，CH6/CH7 = IN6-IN7 / IN7-IN6）
     * @note  返回 0..4095 原始值（差分结果实为有符号，参考库同样
     *        返回无符号原始值，见 deltaRead 的处理）
     */
    int16_t differentialRead(uint8_t channel);

    /**
     * @brief 软件差分增强读取（参考库 delta 模式），可返回负值：
     *        读 (ch&~1) 与 (ch|1) 两个差分对并求差；
     *        若第一读数已为正则省去第二次读取（参考库优化）
     */
    int16_t deltaRead(uint8_t channel);

    /** @brief 批量单端读取（同一事务中逐通道读，参考库 readMultiple） */
    void readMultiple(const uint8_t channels[], uint8_t numChannels, int16_t readings[]);

    // ============================================================
    //  电压换算（定点 uV）
    // ============================================================

    /** @brief 原始值 → 电压 uV：raw × Vref_uV / 4096（int64 中间量） */
    int32_t rawToVoltage_uV(int16_t raw) const;

    /** @brief 单端读取并换算为电压 uV（等价 read() + rawToVoltage_uV） */
    int32_t read_uV(uint8_t channel);

    /**
     * @brief 设置参考电压（满量程电压），默认 3300 mV
     * @note  对应参考库旧版 setMaxVoltage（0.5.2 已移除）
     */
    void setMaxVoltage_mV(uint32_t mV) { _vref_uV = static_cast<uint64_t>(mV) * 1000; }
    uint32_t getMaxVoltage_mV() const  { return static_cast<uint32_t>(_vref_uV / 1000); }

    // ============================================================
    //  调试
    // ============================================================

    /** @brief 累计读取次数（读后清零，参考库 count()） */
    uint32_t count();

private:

    int16_t readADC(uint8_t channel, bool single);

    spi_bus &_spi;
    uint8_t  _channels;
    uint64_t _vref_uV;    // 参考电压 µV（setMaxVoltage_mV 写入）
    uint32_t _count;      // 读取次数计数
};

#endif /* __cplusplus */
