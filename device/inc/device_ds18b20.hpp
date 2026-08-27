#pragma once
//
//    FILE: device_ds18b20.hpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.2.7（参考库 DS18B20_RT）
//    DATE: 2017-07-25
// PURPOSE: DS18B20 单总线（1-Wire）温度传感器
//     URL: https://github.com/RobTillaart/DS18B20_RT
//
//  移植说明（Arduino → STM32 项目风格）：
//   - 参考库依赖 OneWire 底层（复位 / 读写位 / ROM 搜索 / CRC8），
//     本移植用 io_ctrl + delay_us 在类内部实现了完整的 1-Wire 时序，
//     不引入共享文件（与 DS2401 的 1-Wire 底层互不影响，各自私有）
//   - 总线引脚开漏 + 上拉（mode_out_od + pullup），
//     输入/输出模式切换用 io_ctrl::reinit()（对应参考库 DIRECT_MODE_*）
//   - 禁止浮点：温度返回 m°C（int32_t，×1000）
//     定点换算：raw(12bit) × 0.0625°C = raw × 125 / 2（m°C）
//   - 错误码按 m°C 刻度 ×1000：-127000 未连接 / -128000 CRC 错 /
//     -129000 上电复位 85°C 假值 / -130000 数据线对地短路
//   - setOffset() 单位 m°C；参考库 getTempF() 为浮点，未移植
//   - 参考库默认 9 位分辨率（只做整数运算），setResolution() 可改 9..12
//   - 时序参数（µs）取自 OneWire 标准库，原样移植：
//     复位 480 / 70 / 410，写 1 = 拉低 6 + 释放 60，写 0 = 拉低 60 + 释放 60，
//     读时隙 = 拉低 6 + 释放 9 + 采样 + 60
//     ★ 真机需按实测调参 ★：delay_us 的校准精度与每次 reinit() 的
//     HAL 开销（约 1µs）会平移时隙边沿，需示波器实测微调
//   - 建议外部 4.7kΩ 上拉（内部上拉约 30kΩ，长线或挂多器件时偏弱）

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include <cstdint>

// ── 温度范围（m°C，×1000）────────────────────────────
// 超出保证范围的测量值按未连接处理（参考库 DS18B20_MINIMUM = -55）
constexpr int32_t DS18B20_MINIMUM = -55000;
constexpr int32_t DS18B20_MAXIMUM = 125000;   // 保留备用（参考库同款注释）

// ── 错误码（m°C 刻度，数值 = 参考库错误码 × 1000）────
constexpr int32_t DEVICE_DISCONNECTED = -127000;   // 总线上无器件
constexpr int32_t DEVICE_CRC_ERROR    = -128000;   // 暂存器 CRC 校验失败
constexpr int32_t DEVICE_POR_ERROR    = -129000;   // 上电复位假值 85°C
constexpr int32_t DEVICE_GND_ERROR    = -130000;   // 数据线对地短路（寄生供电接法 Vdd 须接 GND）

// ── 配置掩码（setConfig，参考库同值）──────────────────
constexpr uint8_t DS18B20_CLEAR = 0x00;   // 快速模式：只读温度寄存器
constexpr uint8_t DS18B20_CRC   = 0x01;   // 慢速模式：读全部寄存器 + CRC 校验

class DeviceDS18B20
{
public:
    /**
     * @brief 构造并初始化 1-Wire 总线引脚（开漏输出 + 上拉，空闲高电平）
     * @param onewire_pin 1-Wire 数据引脚（io_ctrl 引用）
     * @param resolution  默认分辨率 9..12（参考库默认 9）
     */
    explicit DeviceDS18B20(io_ctrl &onewire_pin, uint8_t resolution = 9);

    DeviceDS18B20(const DeviceDS18B20 &)            = delete;
    DeviceDS18B20 &operator=(const DeviceDS18B20 &) = delete;

    // ── 连接管理 ────────────────────────────────────────

    /** @brief 搜索总线上的器件地址并设置分辨率；成功返回 true */
    bool begin(uint8_t retries = 3);

    /** @brief 用 ROM 搜索 + CRC 校验探测器件（每次调用都会重搜总线） */
    bool isConnected(uint8_t retries = 3);

    /** @brief 拷贝已找到的 8 字节 ROM 地址到 buf；未找到返回 false */
    bool getAddress(uint8_t *buf);

    // ── 温度转换（参考库同名流程）──────────────────────

    /** @brief 广播启动温度转换（Skip ROM，12 位最慢约 750ms） */
    void requestTemperatures(void);

    /**
     * @brief 查询转换是否完成（读一个位）
     * @note  仅寄生供电接法有效（转换期间器件拉低总线）；
     *        外部供电接法下恒为 true，需自行延时等待
     *        （参考库同样行为）
     */
    bool isConversionComplete(void);

    /**
     * @brief 读取温度，返回 m°C（int32_t，×1000）；错误返回负错误码
     * @param checkConnect true 时先做连接检测（耗时较长，参考库默认）
     */
    int32_t getTempCmC(bool checkConnect = true);

    // ── 偏移 / 分辨率 / 配置（参考库同款 API）──────────

    void     setOffset(int32_t offset = 0);   // 单位 m°C
    int32_t  getOffset(void);                 // 单位 m°C

    bool     setResolution(uint8_t resolution = 9);   // 写暂存器配置字节
    uint8_t  getResolution(void);             // 返回缓存值（参考库行为）

    void     setConfig(uint8_t config);       // DS18B20_CLEAR / DS18B20_CRC
    uint8_t  getConfig(void);

private:
    // ── 1-Wire 底层（类内部私有实现，参考 OneWire 库时序）──

    void     _busDriveLow(void);              // reinit 输出开漏 + 拉低
    void     _busRelease(void);               // reinit 输入 + 上拉（释放总线）
    bool     _reset(void);                    // 复位脉冲，true = 检测到存在脉冲
    void     _writeBit(uint8_t v);            // 60µs 时隙
    uint8_t  _readBit(void);                  // 读时隙（6µs 拉低 + 9µs 采样）
    void     _writeByte(uint8_t v);           // LSB 先出
    uint8_t  _readByte(void);                 // LSB 先入
    void     _select(const uint8_t *rom);     // Match ROM 0x55
    void     _skip(void);                     // Skip ROM 0xCC
    bool     _search(uint8_t *newAddr);       // Search ROM 0xF0（多器件寻址）
    void     _resetSearch(void);              // 重置搜索状态机
    static uint8_t _crc8(const uint8_t *data, uint8_t len);   // 多项式 0x8C

    void     _readScratchPad(uint8_t *scratchPad, uint8_t fields);
    void     _setResolution(void);

    // ── 状态 ────────────────────────────────────────────

    io_ctrl &_pin;
    uint8_t  _deviceAddress[8];   // 已找到的器件 ROM 地址
    bool     _addressFound;
    uint8_t  _resolution;         // 9..12（缓存）
    uint8_t  _config;             // DS18B20_CLEAR / DS18B20_CRC
    int32_t  _offset;             // 温度偏移（m°C）

    // ROM 搜索状态机（参考 OneWire 库 search()）
    uint8_t  _lastDiscrepancy;
    uint8_t  _lastFamilyDiscrepancy;
    bool     _lastDeviceFlag;
    uint8_t  _lastAddress[8];
};

#endif /* __cplusplus */
