#pragma once

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include <cstdint>

// ============================================================
//  DS2401 1-Wire 唯一序列号芯片驱动
// ============================================================
//  来源：Rob Tillaart 的 Arduino 库 "DS2401"
//  版本：0.1.3（2026-01-02）
//    URL：https://github.com/RobTillaart/DS2401
//
//  移植说明（对照参考库 0.1.3）：
//    - 参考库依赖 OneWire 库（Arduino）；本项目无 OneWire，按任务
//      要求用 io_ctrl + delay_us 私有实现标准速度 1-Wire 底层时序
//      （复位/读时隙/写时隙，与 DS18B20 同族）
//    - 参考库为 header-only（begin/getUID/compareUID）；本驱动改为
//      .hpp/.cpp 结构：init() + readSerial(uint64_t&)，一次读出
//      64 位 ROM 并做 CRC8 校验，失败返回 false
//    - 64 位 ROM 布局（MSB 在前）：[家族码 0x01][48 位序列号][CRC8]
//    - 0x33 读 ROM 命令只适用于总线上仅挂 1 个器件
//    - CRC8：Dallas/Maxim 多项式 x8+x5+x4+1（0x31，LSB 先移 0x8C）
//    - 引脚配置为开漏输出 + 内部上拉；外部仍需 4.7kΩ 上拉电阻
//    - 时序用 delay.hpp 的 delay_us 产生，其精度依赖主频宏校准
//      （见 delay.hpp 头注释），标准速度 1-Wire 容差 ±几 µs，
//      若换主频需重新校准 delay_us
//    - 无浮点、无堆分配、无异常
//
//  1-Wire 时序（标准速度，µs）：
//    复位：拉低 480 → 释放 → 70µs 采样存在脉冲（从机低 60~240µs）
//          → 补满复位周期（共 480µs）
//    写 1：拉低 6 → 释放 → 补满时隙（共 60µs）
//    写 0：拉低 60 → 释放 → 恢复 2µs
//    读：  拉低 6 → 释放 → 9µs 采样（tRDV < 15µs）→ 补满时隙（共 60µs）
// ============================================================

class DS2401
{
public:

    static constexpr uint8_t  FAMILY_DS2401 = 0x01;   // 家族码
    static constexpr uint8_t  ROM_LEN       = 8;      // ROM 字节数
    static constexpr uint8_t  CMD_READ_ROM  = 0x33;   // 读 ROM（仅单器件总线）

    /** @brief 绑定 1-Wire 数据线（外部需 4.7kΩ 上拉电阻） */
    explicit DS2401(io_ctrl& pin);

    /** @brief 数据线初始化为开漏输出 + 内部上拉，并释放总线 */
    void init();

    /**
     * @brief 复位 + 发 0x33 读 ROM + 读 8 字节 + CRC8 校验
     * @param serial  输出 64 位序列号（MSB 在前：家族码在最高字节）
     * @return true = 成功；false = 无器件/CRC 错误
     */
    bool readSerial(uint64_t& serial);

    /** @brief 校验 8 字节 ROM：crc8(data, 7) == data[7] */
    static bool checkCRC(const uint8_t rom[ROM_LEN]);

    /** @brief Dallas/Maxim CRC8（公开，供调试或其它 1-Wire 器件复用） */
    static uint8_t crc8(const uint8_t* data, uint8_t len);

private:

    io_ctrl& _pin;

    // ── 1-Wire 底层 ──
    bool     _reset();              // 复位 + 存在检测
    void     _writeBit(bool bit);
    bool     _readBit();
    void     _writeByte(uint8_t byte);
    uint8_t  _readByte();
};

#endif
