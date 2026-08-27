#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// Honeywell ABP2 压力传感器（24 位原始值，I2C）
//
// 移植自 Rob Tillaart I2C_ABP2 v0.1.0
//   URL: https://github.com/RobTillaart/I2C_ABP2
//
// 移植说明：
//  - I2C 地址由硬件引脚决定（本驱动默认 0x28，请按实际接线修改）
//  - 命令帧 0xAA 0x00 0x00 用 inter_i2c_dev::freedom_write(0xAA, 0, 2)
//    实现（寄存器字节即命令首字节）
//  - 读 7 字节（状态 1 + 压力 24 位 + 温度 24 位）是"无寄存器指针"
//    的纯读操作，inter_i2c_dev 不支持 → 直接操作 inter_i2c_bus
//  - 无 FPU：压力 Pa、温度 m°C 定点，int64 中间量防溢出
//  - 输出类型固定按 10%..90% of 2^24 处理（原库同；型号不同时
//    需按数据手册调整 minCnt/maxCnt 常量）
//  - 异步接口与原库一致：request() 后需等待转换完成再 read()，
//    转换时间由数据手册/具体型号决定（本驱动不内置延时）
//  - 错误码：read()/request() 返回详细错误码；getLastError()
//    返回总线错误（0=OK 1=NACK，对齐 inter_i2c_dev 语义）
// ============================================================

class I2C_ABP2
{
public:
    // ── I2C 地址（由传感器硬件引脚决定） ──────────────────

    enum Addr : uint8_t
    {
        ADDR_0x28 = 0x28,   // 常用地址
        ADDR_0x38 = 0x38,
        ADDR_0x48 = 0x48,
        ADDR_0x58 = 0x58,
    };

    // ── 错误码（语义对齐原库） ─────────────────────────────

    enum Err : int16_t
    {
        ERR_OK            = 0,
        ERR_REQUEST       = -100,   // 命令帧发送失败
        ERR_READ          = -101,   // 数据读取失败
    };

    // ============================================================
    //  构造
    // ============================================================

    /**
     * @param addr         I2C 地址（引脚决定）
     * @param minPressure_Pa 量程下限 Pa（仅换算用，不写入芯片）
     * @param maxPressure_Pa 量程上限 Pa
     */
    explicit I2C_ABP2(inter_i2c_bus* bus, uint8_t addr = ADDR_0x28,
                      int32_t minPressure_Pa = 0,
                      int32_t maxPressure_Pa = 100000);

    I2C_ABP2(const I2C_ABP2&) = delete;
    I2C_ABP2& operator=(const I2C_ABP2&) = delete;

    /** @brief 初始化：只做 I2C 探测（量程在构造时已配置） */
    void init();

    /** @brief 运行时修改压力量程（Pa） */
    void setPressureRange(int32_t minPressure_Pa, int32_t maxPressure_Pa);

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool isConnected();
    [[nodiscard]] uint8_t getLastError();

    // ============================================================
    //  异步接口
    // ============================================================

    /** @brief 发送测量命令帧（0xAA 0x00 0x00），返回 Err */
    int request();

    /**
     * @brief 读取 7 字节数据（状态 + 压力 + 温度）并换算
     * @note  应在 request() 且转换完成后调用；读失败不更新结果
     */
    int read();

    /** @brief 最近一次成功读的时间戳（get_tick） */
    uint32_t lastRead() const { return _lastRead; }

    // ============================================================
    //  结果（最近一次成功 read()）
    // ============================================================

    /** @brief 状态字节（bit7-6 = 设备状态，见数据手册） */
    uint8_t getState() const { return _state; }

    /** @brief 压力 Pa */
    int32_t getPressure_Pa() const { return _pressure_Pa; }

    /** @brief 温度 m°C（×1000） */
    int32_t getTemperature_mC() const { return _temperature_mC; }

    /** @brief 压力原始 24 位计数（调试用） */
    uint32_t getRawPressure() const { return _rawP; }

    /** @brief 温度原始 24 位计数（调试用） */
    uint32_t getRawTemperature() const { return _rawT; }

private:
    inter_i2c_dev _dev;
    inter_i2c_bus* _bus;     // 无寄存器指针的纯读需要直接操作总线
    uint8_t _addr;
    uint8_t _err;            // 总线错误标志（0=OK 1=NACK，镜像 lastError 语义）

    int32_t _minPa;
    int32_t _maxPa;

    uint32_t _lastRead = 0;      // get_tick
    uint8_t  _state = 0;
    int32_t  _pressure_Pa = 0;
    int32_t  _temperature_mC = 0;
    uint32_t _rawP = 0;
    uint32_t _rawT = 0;
};

#endif
