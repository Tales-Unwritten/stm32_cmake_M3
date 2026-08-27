#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// Honeywell ASDX 压力传感器（14 位原始值 + 2 位状态，I2C）
//
// 移植自 Rob Tillaart I2C_ASDX v0.4.3
//   URL: https://github.com/RobTillaart/I2C_ASDX
//
// 移植说明：
//  - ASDX 无命令帧：上电后持续输出，主机直接读 2 字节
//    （状态 2 位 + 压力 14 位）→ 无寄存器指针的纯读，
//    inter_i2c_dev 不支持 → 直接操作 inter_i2c_bus
//  - 无 FPU：压力 Pa 定点，int64 中间量防溢出
//  - 量程由构造参数 psi 决定（1/5/15/30/60/100，见数据手册型号表），
//    psi 非法时量程为 0、读数恒为 0
//  - 换算公式定点化：P = (raw - 1638) × maxPa / 13108
//    （原库：mBar = (raw-1638) × maxMBar × 7.6289289e-5，
//      7.6289289e-5 = 1/13108；maxPa = psi × 6894.75729）
//  - 未移植多种单位换算（ATM/Dynes/InchHg 等），保留 Pa/mBar/psi
//  - 错误码语义对齐原库（OK=1）
// ============================================================

class I2C_ASDX
{
public:
    // ── I2C 地址（依型号，见数据手册） ────────────────────

    enum Addr : uint8_t
    {
        ADDR_0x28 = 0x28,   // 30 psi 典型
        ADDR_0x38 = 0x38,   // 60 psi 典型
        ADDR_0x58 = 0x58,   // 100 psi 典型
    };

    // ── 错误码（语义对齐原库） ─────────────────────────────

    enum Err : int16_t
    {
        ERR_OK       = 1,   // 成功
        ERR_INIT     = 0,   // 未初始化
        ERR_READ     = -1,  // I2C 读失败
        ERR_STATUS   = -2,  // 状态位 0xC000 置位（传感器故障）
        ERR_CONNECT  = -3,  // 连接失败
    };

    // ============================================================
    //  构造
    // ============================================================

    /**
     * @param addr I2C 地址
     * @param psi  量程 psi（1/5/15/30/60/100；非法值 → 量程 0）
     */
    explicit I2C_ASDX(inter_i2c_bus* bus, uint8_t addr = ADDR_0x28,
                      uint8_t psi = 30);

    I2C_ASDX(const I2C_ASDX&) = delete;
    I2C_ASDX& operator=(const I2C_ASDX&) = delete;

    /** @brief 初始化：只做 I2C 探测 */
    void init();

    /** @brief 复位统计量（错误计数/时间戳/压力清零） */
    void reset();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool isConnected();
    [[nodiscard]] uint8_t getLastError();

    // ============================================================
    //  读取
    // ============================================================

    /** @brief 读 2 字节（状态 + 压力）并换算，返回 Err */
    int read();

    // ============================================================
    //  结果（最近一次成功 read()）
    // ============================================================

    /** @brief 压力 Pa */
    int32_t getPressure_Pa() const { return _pressure_Pa; }

    /** @brief 压力 mBar（Pa/100） */
    int32_t getPressure_mBar() const { return _pressure_Pa / 100; }

    /** @brief 压力 psi（Pa × 100000 / 689475729） */
    int32_t getPressure_psi() const;

    /** @brief 原始 14 位压力计数（调试/自换算用） */
    int32_t getRawCount() const { return _rpc; }

    /** @brief 最近一次状态（Err） */
    int16_t getState() const { return _state; }

    /** @brief 自上次 reset() 以来的错误次数 */
    uint16_t getErrorCount() const { return _errorCount; }

    /** @brief 最近一次成功读的时间戳（get_tick） */
    uint32_t lastRead() const { return _lastRead; }

private:
    inter_i2c_dev _dev;
    inter_i2c_bus* _bus;     // 无寄存器指针的纯读需要直接操作总线
    uint8_t _addr;
    uint8_t _err;            // 总线错误标志（0=OK 1=NACK）

    int32_t _maxPa;          // 满量程压力 Pa（psi × 6894.75729 定点）
    int32_t _pressure_Pa;    // 最近一次成功读的压力 Pa
    int32_t _rpc;            // 原始 14 位计数（调试用）

    int16_t  _state = ERR_INIT;
    uint16_t _errorCount = 0;
    uint32_t _lastRead = 0;
};

#endif
