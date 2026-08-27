#pragma once

#ifdef __cplusplus

#include <cstdint>
#include "inter_io_ctrl.hpp"
#include "inter_spi_bus.hpp"

// ============================================================
//  MCP4921 / MCP4922 12 位 SPI DAC 驱动
// ============================================================
//  来源库 : MCP_DAC v0.5.4（Rob Tillaart，2021-02-03）
//    URL : https://github.com/RobTillaart/MCP_DAC
//
//  移植说明（对照参考库 0.5.4）：
//    - 仅移植 MCP4921（单通道）/ MCP4922（双通道）12 位芯片，
//      8/10 位 MCP48xx/49xx 系列未移植；两者协议相同，用构造参数
//      channels = 1/2 区分
//    - 构造收 spi_port&；CS 控制约定同其他 SPI 器件（见下）
//    - 禁止 float/double：setPercentage 用 %×100 定点
//      （value = perc × maxValue / 10000），输出电压 mV 定点
//      = value × maxVoltage_mV / maxValue（int64 中间量）。
//      参考库 0.5.4 已移除 setMaxVoltage（旧版 API），本驱动以
//      setMaxVoltage_mV 重新提供，默认满量程 = Vref = 3300 mV
//      （注意 gain=2x 时满量程 = 2×Vref，需用户按实际配置设置）
//    - 裁剪：软件 SPI、setSPIspeed（速度由 spi_port prescaler 决定）、
//      usesHWSPI、MCP48xx/49xx 其他系列派生类
//    - lastValue 缓存存写入后的钳位值（参考库存未钳位原值，此处
//      修正为钳位值，避免 lastValue 超过 maxValue）
//    - 无浮点、无堆分配、无 STL/异常，C++17，Cortex-M0+
//
//  SPI 模式：Mode 0（CPOL=0, CPHA=0）——参考库使用 SPI_MODE0，
//  数据手册 4.1：SCK 空闲低电平、上升沿锁存输入。最高 20 MHz。
//
//  16 位数据帧（数据手册 4.1.1，先发高字节）：
//    bit15  A/B（0=通道 A，1=通道 B）
//    bit14  BUF（1=输入缓冲使能）
//    bit13  GA（1=增益 1x，0=增益 2x）
//    bit12  SHDN（1=正常，0=关断）
//    bit11..0  12 位数据
// ============================================================

class MCP4921
{
public:

    // ============================================================
    //  构造
    // ============================================================

    /**
     * @param spi      共享 SPI 总线（CS 约定见头注释）
     * @param channels 通道数：1 = MCP4921（默认），2 = MCP4922
     */
    explicit MCP4921(spi_bus &spi, uint8_t channels = 1);

    MCP4921(const MCP4921&) = delete;
    MCP4921& operator=(const MCP4921&) = delete;

    /** @brief 复位内部状态（增益 1x、输出 0、非缓冲、正常工作） */
    void reset();

    // ============================================================
    //  基本信息
    // ============================================================

    uint8_t  channels() const { return _channels; }   // 1 或 2
    uint16_t maxValue() const { return 4095; }        // 12 位

    // ============================================================
    //  增益
    // ============================================================

    /**
     * @brief 设置增益 1x 或 2x
     * @note  参考库默认 1x（输出最低，最安全）；数据手册复位默认 2x
     */
    bool setGain(uint8_t gain = 1);
    uint8_t getGain() const { return _gain; }

    // ============================================================
    //  输出 API
    // ============================================================

    /**
     * @brief 写入 DAC 输出（value 超 maxValue 时钳位）
     * @param channel 0 = A，1 = B（MCP4922）
     * @return false = 通道越界
     */
    bool write(uint16_t value, uint8_t channel = 0);

    /** @brief 最近写入的输出值（钳位后，见头注释） */
    uint16_t lastValue(uint8_t channel = 0) const;

    /** @brief 快速写入通道 A（固定 BUF=0、GA=1x、SHDN=1，参考库） */
    void fastWriteA(uint16_t value);

    /** @brief 快速写入通道 B（固定 BUF=0、GA=1x、SHDN=1，参考库） */
    void fastWriteB(uint16_t value);

    /** @brief 输出值 +1（到 maxValue 为止） */
    bool increment(uint8_t channel = 0);
    /** @brief 输出值 -1（到 0 为止） */
    bool decrement(uint8_t channel = 0);

    /**
     * @brief 按百分比设置输出（%×100 定点，0..10000 = 0.00%..100.00%）
     * @return false = 通道越界
     */
    bool setPercentage_x100(uint16_t percentage, uint8_t channel = 0);

    /** @brief 当前输出百分比（%×100） */
    uint16_t getPercentage_x100(uint8_t channel = 0) const;

    // ============================================================
    //  电压换算（定点 mV）
    // ============================================================

    /**
     * @brief 设置满量程电压 mV（默认 3300 = Vref 3.3 V）
     * @note  对应参考库旧版 setMaxVoltage；gain=2x 时满量程为
     *        2×Vref，请按实际输出范围设置
     */
    void setMaxVoltage_mV(uint32_t mV) { _maxVoltage_mV = mV; }
    uint32_t getMaxVoltage_mV() const  { return _maxVoltage_mV; }

    /** @brief 当前输出电压 mV：value × maxVoltage_mV / maxValue */
    int32_t getVoltage_mV(uint8_t channel = 0) const;

    // ============================================================
    //  缓冲 / 关断
    // ============================================================

    /** @brief Vref 输入缓冲（MCP49xx 系列特性，参考库默认关闭） */
    void setBufferedMode(bool mode = false) { _buffered = mode; }
    bool getBufferedMode() const            { return _buffered; }

    /**
     * @brief 关断输出（SHDN=0 帧），下一次 write() 自动唤醒
     * @note  isActive() 反映库内状态：参考库中 write() 不清
     *        除关断标志（芯片实际已唤醒），本移植保留该行为
     */
    void shutDown();
    bool isActive() const { return _active; }

    // ============================================================
    //  LDAC 锁存引脚（可选，不设置则无操作）
    // ============================================================

    /** @brief 绑定 LDAC 引脚（初始化输出高，触发时拉低 1µs 再拉高） */
    void setLatchPin(io_ctrl *pin);
    void triggerLatch();

private:

    void transfer(uint16_t data);   // 低层：CS 低 → 高字节 → 低字节 → CS 高

    spi_bus &_spi;
    io_ctrl  *_latchPin;
    uint8_t   _channels;
    uint16_t  _value[2];            // 输出值缓存（钳位后）
    uint8_t   _gain;                // 1 或 2
    bool      _buffered;
    bool      _active;
    uint32_t  _maxVoltage_mV;
};

#endif /* __cplusplus */
