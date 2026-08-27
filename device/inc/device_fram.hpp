#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_i2c_bus.hpp"
#include <cstdint>

// ============================================================
// FM24C / MB85RC 系列 I2C FRAM（铁电存储器）
//
// 来源库 : FRAM_I2C v0.8.4（Rob Tillaart）
// URL    : https://github.com/RobTillaart/FRAM_I2C
//
// 移植说明:
//  - 无 float/STL/动态内存/异常；软件 I2C（GPIO 模拟），C++17
//  - 仅移植 FRAM 基类（16 位内存地址，覆盖 MB85RC64T/64V/128A/256V、
//    FM24C64/128/256 等 ≤64KB 型号）；以下子模块未移植：
//      · FRAM32（17 位地址 MB85RC512T/1MT，块位在器件地址 bit0）
//      · FRAM11 / FRAM9（页地址型 MB85RC16 / MB85RC04）
//      · FRAM_RINGBUFFER 环形缓冲子模块（与驱动无关的应用层）
//  - FRAM 无写周期延迟、无页限制、写入寿命无限（参考库特性），
//    可任意长度连续写；块操作按 64 字节分块，避免单次事务
//    长时间占用总线（inter_i2c_bus::lock 期间关中断）
//  - update_byte 写前比较（读-比-写），内容相同则不写
//  - 器件 ID（厂商/密度/产品）：从机地址 0x7C 读 3 字节
//  - sleep/wakeup：0x7C/0x86 专用命令序列（见数据手册 P12）
//  - inter_i2c_dev 的 freedom_read/write 无法表达 16 位内存地址
//    与 >8 字节块传输，故与 device_eeprom 一致，直接使用
//    inter_i2c_bus 原语
// ============================================================

class device_fram
{
public:
    // ── I2C 地址（A2/A1/A0 引脚）──

    enum Addr : uint8_t
    {
        ADDR_0x50 = 0x50,   // A2=0 A1=0 A0=0（默认）
        ADDR_0x51 = 0x51,
        ADDR_0x52 = 0x52,
        ADDR_0x53 = 0x53,
        ADDR_0x54 = 0x54,
        ADDR_0x55 = 0x55,
        ADDR_0x56 = 0x56,
        ADDR_0x57 = 0x57,
    };

    // ── 器件参数 ──

    /// 块操作分块长度（参考库受 Wire 缓冲限制用 24，软件 I2C 无此
    /// 限制，取 64 平衡总线占用时间与事务次数）
    static constexpr uint16_t BLOCK_CHUNK = 64;

    // ── 错误码（同参考库）──

    enum ErrCode : int
    {
        ERR_OK      = 0,
        ERR_ADDR    = -10,   // 地址超界 / 参数非法
        ERR_I2C     = -11,   // I2C 传输失败
        ERR_CONNECT = -12,   // 探测无应答
        ERR_REQUEST = -13,   // 读回字节数不符（参考库保留，本移植不使用）
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit device_fram(inter_i2c_bus* bus, uint8_t addr = ADDR_0x50);

    device_fram(const device_fram&)            = delete;
    device_fram& operator=(const device_fram&) = delete;

    ~device_fram();

    /** @brief 延迟初始化：连接检查 + 尝试读取器件 ID 尺寸（可重复调用） */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool is_connected();
    /** @brief 最近一次操作错误码（0 = 无错误；读取后自动清零） */
    [[nodiscard]] int get_last_error();
    uint8_t get_address() const { return _address; }

    // ============================================================
    //  基本读写（返回 0 = OK，否则错误码）
    // ============================================================

    /// 写单字节
    int write_byte(uint16_t mem_addr, uint8_t value);
    /// 写块（任意长度，内部按 64 字节分块）
    int write_block(uint16_t mem_addr, const uint8_t* buf, uint16_t length);
    /// 读单字节（失败返回 0，查 get_last_error）
    int read_byte(uint16_t mem_addr, uint8_t& data);
    /// 读块（任意长度，内部按 64 字节分块）
    int read_block(uint16_t mem_addr, uint8_t* buf, uint16_t length);
    /// 写前比较：相同则不写（省一次写寿命/总线时间），返回 0 = 相同或写成功
    int update_byte(uint16_t mem_addr, uint8_t value);

    // ============================================================
    //  类型读写（主机小端序存储，与参考库一致）
    // ============================================================

    void     write8(uint16_t mem_addr, uint8_t  value);
    void     write16(uint16_t mem_addr, uint16_t value);
    void     write32(uint16_t mem_addr, uint32_t value);
    void     write64(uint16_t mem_addr, uint64_t value);
    uint8_t  read8(uint16_t mem_addr);
    uint16_t read16(uint16_t mem_addr);
    uint32_t read32(uint16_t mem_addr);
    uint64_t read64(uint16_t mem_addr);

    /// 全片填充（需先有有效尺寸：init 成功或 set_size_bytes 设置过）
    uint32_t clear(uint8_t value = 0);

    // ============================================================
    //  器件 ID / 尺寸（FRAM 从机地址 0x7C 特殊事务）
    // ============================================================

    /// 厂商 ID（Fujitsu = 0x0A，Cypress/Infineon = 0x04；失败返回 0）
    uint16_t get_manufacturer_id();
    /// 产品 ID（失败返回 0）
    uint16_t get_product_id();
    /// 由密度码推导尺寸（KB；失败返回 0）
    uint16_t get_size();
    /// 当前尺寸（字节）
    uint32_t get_size_bytes() const { return _size_bytes; }
    /// 手动指定尺寸（get_size 失败时使用，如无器件 ID 的兼容片）
    void     set_size_bytes(uint32_t value) { _size_bytes = value; }

    // ============================================================
    //  睡眠 / 唤醒（MB85RC 系列；FM24C 系列不支持）
    // ============================================================

    bool sleep();
    /// @param time_recover_us 恢复时间（数据手册 trec ≤ 400µs）
    bool wakeup(uint32_t time_recover_us = 400);

private:
    // 器件 ID 从机地址与睡眠命令（数据手册：S 0xF8 A 地址 A S 86 A P）
    static constexpr uint8_t FRAM_SLAVE_ID  = 0x7C;   // 器件 ID 从机（7 位）
    static constexpr uint8_t FRAM_SLEEP_CMD = 0x86;   // 睡眠命令字节

    inter_i2c_bus* _bus;
    uint8_t        _address;
    uint32_t       _size_bytes = 0;
    int            _error = ERR_OK;

    /// 器件 ID 事务：返回 24 位原始值（失败返回 0xFFFFFFFF）
    uint32_t _get_meta_data();

    /// 单块写（len ≤ BLOCK_CHUNK），返回 true 成功
    bool _write_block(uint16_t mem_addr, const uint8_t* obj, uint16_t size);

    /// 单块读（len ≤ BLOCK_CHUNK），返回 true 成功
    bool _read_block(uint16_t mem_addr, uint8_t* obj, uint16_t size);
};

#endif
