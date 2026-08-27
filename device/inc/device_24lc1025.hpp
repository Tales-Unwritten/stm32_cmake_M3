#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_i2c_bus.hpp"
#include <cstdint>

// ============================================================
// 24LC1025 1Mbit I2C EEPROM（128KB，分两个 64KB 块）
//
// 来源库 : I2C_24LC1025 v0.3.4（Rob Tillaart）
// URL    : https://github.com/RobTillaart/I2C_24LC1025
//
// 移植说明:
//  - 无 float/STL/动态内存/异常；软件 I2C（GPIO 模拟），C++17
//  - 17 位内存地址空间：低 16 位放在两个地址字节（高字节在前），
//    bit16（块选择 B0）放在器件地址 bit2（7 位地址 0x04）
//    ——与参考库 _beginTransmission() 一致
//  - 器件地址 = 0x50 | (A1<<1) | A0 | (B0<<2)；A2 引脚必须接 VCC
//    （数据手册要求，悬空/接 GND 不工作）；默认 0x50
//  - 芯片分两个 64KB 块：地址指针在块内回绕（0xFFFF→0x0000），
//    不会自动跨块，因此读/写都要在 0x10000 边界切分
//  - 页大小 128 字节；页写自动分页；写周期用 ACK 轮询等待
//  - inter_i2c_dev 的 freedom_read/write 只能表达单字节寄存器
//    地址且 ≤8 字节，无法承载 16 位内存地址与 >8 字节块传输，
//    故与 device_eeprom 一致，直接使用 inter_i2c_bus 原语
// ============================================================

class device_24lc1025
{
public:
    // ── I2C 地址（A0/A1 引脚；A2 必须接 VCC）──

    enum Addr : uint8_t
    {
        ADDR_0x50 = 0x50,   // A1=0 A0=0（默认）
        ADDR_0x51 = 0x51,   // A1=0 A0=1
        ADDR_0x52 = 0x52,   // A1=1 A0=0
        ADDR_0x53 = 0x53,   // A1=1 A0=1
    };

    // ── 器件参数 ──

    static constexpr uint32_t DEVICE_SIZE = 131072;   // 128 KB
    static constexpr uint16_t PAGE_SIZE    = 128;     // 页大小（字节）
    static constexpr uint32_t BLOCK_SIZE   = 65536;   // 单块 64 KB

    // ── 错误码 ──

    enum ErrCode : int
    {
        ERR_OK      = 0,
        ERR_ADDR    = -10,   // 地址超界 / 参数非法
        ERR_I2C     = -11,   // I2C 传输失败（NACK / 写周期超时）
        ERR_CONNECT = -12,   // 探测无应答
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit device_24lc1025(inter_i2c_bus* bus, uint8_t addr = ADDR_0x50);

    device_24lc1025(const device_24lc1025&)            = delete;
    device_24lc1025& operator=(const device_24lc1025&) = delete;

    ~device_24lc1025();

    /** @brief 延迟初始化：连接检查（可重复调用，幂等） */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool is_connected();
    /** @brief 最近一次操作错误码（0 = 无错误；读取后自动清零） */
    [[nodiscard]] int get_last_error();
    uint8_t get_address() const { return _addr; }

    // ============================================================
    //  写（返回 0 = OK，否则错误码）
    // ============================================================

    /// 写单字节，自动跨页切分
    int write_byte(uint32_t mem_addr, uint8_t value);
    /// 写块，自动跨页/跨块切分，每页后 ACK 轮询等写周期
    int write_block(uint32_t mem_addr, const uint8_t* buffer, uint32_t length);
    /// 填充常量值（循环写同一字节）
    int set_block(uint32_t mem_addr, uint8_t value, uint32_t length);
    /// 写前比较：内容相同则不写（省一次写周期），返回 0 = 相同或写成功
    int update_byte(uint32_t mem_addr, uint8_t value);

    // ============================================================
    //  读
    // ============================================================

    /// 读单字节（失败/超界返回 0xFF，查 get_last_error）
    uint8_t read_byte(uint32_t mem_addr);
    /// 读块，自动跨块切分；返回实际读到的字节数（应等于 length）
    uint32_t read_block(uint32_t mem_addr, uint8_t* buffer, uint32_t length);

    // ============================================================
    //  写 + 回读校验
    // ============================================================

    bool write_byte_verify(uint32_t mem_addr, uint8_t value);
    bool write_block_verify(uint32_t mem_addr, const uint8_t* buffer, uint32_t length);
    bool set_block_verify(uint32_t mem_addr, uint8_t value, uint32_t length);

    // ============================================================
    //  写周期等待（ACK 轮询超时上限）
    // ============================================================

    void    set_write_timeout_ms(uint8_t ms);   // 默认 10 ms（覆盖所有型号 tWR）
    uint8_t get_write_timeout_ms() const { return _write_timeout_ms; }

private:
    inter_i2c_bus* _bus;
    uint8_t        _addr;
    uint8_t        _write_timeout_ms = 10;
    int            _error = ERR_OK;

    /// 计算含块选择位（bit2）的 7 位器件地址
    uint8_t _device_addr(uint32_t mem_addr) const;

    /// 发送"伪写"设置地址指针（器件地址+高字节+低字节），
    /// 调用后总线停在等待数据状态；失败返回 false（调用方负责 STOP）
    bool _begin_write(uint32_t mem_addr);

    /// ACK 轮询：等待内部写周期结束
    bool _wait_write_cycle();

    /// 单页（且单块）内写，len ≤ PAGE_SIZE；返回错误码
    int _write_chunk(uint32_t addr, const uint8_t* buf, uint16_t len);

    /// 单块内读（连续读），自动处理 ACK/NACK 结束
    uint32_t _read_chunk(uint32_t addr, uint8_t* buf, uint32_t len);
};

#endif
