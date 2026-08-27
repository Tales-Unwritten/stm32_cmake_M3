#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// Honeywell MPRLS 微压传感器（24 位原始值，I2C，地址 0x18）
//
// 移植自 Rob Tillaart I2C_MPRLS v0.1.1
//   URL: https://github.com/RobTillaart/MPRLS
//        https://github.com/RobTillaart/pressure
//
// 移植说明：
//  - I2C 地址 0x18 为数据手册默认值（型号变体可选 0x08/0x28/0x38，
//    原库头注释中的 0x30 与数据手册不符，未采用）
//  - 测量命令 0xAA 0x00 0x00（数据手册 6.6.1；任务描述中的 0xF0 未见于
//    数据手册与参考库，本驱动以数据手册为准）
//  - 状态/数据读（1 字节 / 4 字节）为"无寄存器指针"的纯读操作，
//    inter_i2c_dev 不支持 → 直接操作 inter_i2c_bus
//  - 无 FPU：压力 Pa 定点，int64 中间量防溢出
//  - 传输函数 A/B/C 的 24 位计数边界采用原库常量：
//      A 10%..90%  → 1677722..15099494（span 13421772）
//      B 2.5%..22.5% → 419430..3774874（span 3355444）
//      C 20%..80%   → 3355444..13421772（span 10066330）
//  - 原库 conversionReady() 在 BUSY 置位时返回 true（疑似语义反转 bug），
//    本驱动按数据手册语义实现：conversionReady() = BUSY 清零
//  - 错误码语义对齐原库（OK=1）
// ============================================================

class I2C_MPRLS
{
public:
    // ── I2C 地址（数据手册：默认 0x18） ───────────────────

    enum Addr : uint8_t
    {
        ADDR_0x08 = 0x08,
        ADDR_0x18 = 0x18,   // 默认
        ADDR_0x28 = 0x28,
        ADDR_0x38 = 0x38,
    };

    // ── 错误码（语义对齐原库） ─────────────────────────────

    enum Err : int16_t
    {
        ERR_OK       = 1,
        ERR_INIT     = 0,
        ERR_READ     = -1,
        ERR_WRITE    = -2,
        ERR_CONNECT  = -3,
    };

    // ── 状态位（数据手册 Table 11） ───────────────────────

    static constexpr uint8_t STATUS_POWER   = 0x40;   // 电源模式
    static constexpr uint8_t STATUS_BUSY    = 0x20;   // 转换中
    static constexpr uint8_t STATUS_MEMTEST = 0x04;   // 存储器自检失败
    static constexpr uint8_t STATUS_MATH    = 0x01;   // 数学运算饱和

    // ── 传输函数（输出占 2^24 计数的比例） ────────────────

    enum TransferFunction : uint8_t
    {
        TF_A = 0,   // 10%..90%（默认）
        TF_B = 1,   // 2.5%..22.5%
        TF_C = 2,   // 20%..80%
    };

    // ============================================================
    //  构造
    // ============================================================

    /**
     * @param addr          I2C 地址
     * @param minPressure_Pa 量程下限 Pa（仅换算用，不写入芯片）
     * @param maxPressure_Pa 量程上限 Pa
     */
    explicit I2C_MPRLS(inter_i2c_bus* bus, uint8_t addr = ADDR_0x18,
                       int32_t minPressure_Pa = 0,
                       int32_t maxPressure_Pa = 25000);

    I2C_MPRLS(const I2C_MPRLS&) = delete;
    I2C_MPRLS& operator=(const I2C_MPRLS&) = delete;

    /** @brief 初始化：只做 I2C 探测（量程/传输函数在构造或 setter 配置） */
    void init();

    /** @brief 运行时修改压力量程（Pa） */
    void setPressureRange(int32_t minPressure_Pa, int32_t maxPressure_Pa);

    /** @brief 设置传输函数（A/B/C，默认 A） */
    void setTransferFunction(TransferFunction tf) { _transferFunction = tf; }
    TransferFunction getTransferFunction() const { return _transferFunction; }

    /** @brief 复位统计量 */
    void reset();

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

    /** @brief 读 1 字节状态并判断是否忙（BUSY 位置位） */
    bool isBusy();

    /** @brief 状态字节（isBusy() 刚读到的值） */
    uint8_t getState() const { return _state; }

    /** @brief 转换是否完成（= !isBusy()；按数据手册语义实现） */
    bool conversionReady();

    /** @brief 读 4 字节（状态 + 24 位压力）并换算，返回 Err */
    int getData();

    // ============================================================
    //  阻塞接口
    // ============================================================

    /**
     * @brief request + 5 ms 转换等待 + getData
     * @note  5 ms 为数据手册给出的保守转换时间
     */
    int read();

    // ============================================================
    //  结果（最近一次成功 getData()）
    // ============================================================

    /** @brief 压力 Pa */
    int32_t getPressure_Pa() const { return _pressure_Pa; }

    /** @brief 压力原始 24 位计数（调试/自换算用） */
    int32_t getRawCount() const { return _rpc; }

    /** @brief 最近一次成功读的时间戳（HAL_GetTick） */
    uint32_t lastRead() const { return _lastRead; }

    /** @brief 自上次 reset() 以来的错误次数 */
    uint16_t getErrorCount() const { return _errorCount; }

private:
    // ── 无寄存器指针的纯读（长度 1/4 字节） ───────────────
    bool _readRaw(uint8_t len, uint64_t* out);

    inter_i2c_dev _dev;
    inter_i2c_bus* _bus;
    uint8_t _addr;
    uint8_t _err;            // 总线错误标志（0=OK 1=NACK）

    int32_t  _minPa;
    int32_t  _maxPa;
    TransferFunction _transferFunction = TF_A;

    int32_t  _pressure_Pa = 0;
    int32_t  _rpc = 0;           // 24 位原始压力计数
    uint8_t  _state = 0x00;      // 最近状态字节（读前为 0，对齐原库 NONE）
    int16_t  _error = ERR_INIT;
    uint16_t _errorCount = 0;
    uint32_t _lastRead = 0;
};

#endif
