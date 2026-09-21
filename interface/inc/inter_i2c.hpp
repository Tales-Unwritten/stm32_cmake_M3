#pragma once

#ifdef __cplusplus

#include <cstdint>

/**
 * @brief I2C 总线抽象接口（软/硬 I2C 统一入口）
 *
 * 软件 I2C（inter_i2c_bus）与硬件 I2C（i2c_hw_port）实现同一接口，
 * 设备驱动只需依赖本基类，即可透明切换软/硬 I2C 总线，
 * 保证「接口功能一致」（与 inter_spi_bus.hpp 的 spi_bus 同款做法）。
 *
 * 公共方法（设备驱动依赖的最小集合）：
 *   init() / deinit()                     生命周期
 *   start() / stop()                      起止条件
 *   write_byte() / read_byte()            单字节收发
 *   wait_ack() / write_ack()              应答位
 *   read_bytes()                          批量接收（末字节 NACK + STOP）
 *   lock() / unlock() / is_busy()         互斥
 *   bus_recovery()                        总线死锁恢复
 *
 * 平台特有扩展保留在各自实现类中，不进入公共接口：
 *   i2c_hw_port::i2c_restart() / i2c_write_reg() / i2c_read_reg() / periph()
 *   inter_i2c_bus::scan() / _delay()
 *
 * @note 调用方负责在设备操作前完成 init()；wait_ack() 返回 true 表示收到应答。
 */
class i2c_bus
{
public:
    virtual ~i2c_bus() = default;

    // ── 生命周期 ──────────────────────────────────────────
    virtual void init()   = 0;
    virtual void deinit() = 0;

    // ── 总线控制 ──────────────────────────────────────────
    virtual void    start() = 0;
    virtual void    stop()  = 0;
    virtual void    write_byte(uint8_t data) = 0;
    virtual uint8_t read_byte() = 0;
    virtual bool    wait_ack(uint16_t timeout = 1000) = 0;
    virtual void    write_ack(uint8_t ack) = 0; // 0=ACK, 1=NACK

    // ── 批量接收 ──────────────────────────────────────────
    /**
     * @brief 连续接收 len 个字节，并结束本次读传输（末字节 NACK + STOP）
     *
     * 默认实现用 read_byte() + write_ack() 逐字节完成 —— 软 I2C 直接可用，
     * 因此 inter_i2c_bus 不需要覆写。
     *
     * 硬件 I2C（i2c_hw_port）必须覆写：STM32F1 的接收应答位必须在「清 ADDR
     * 之前」配置，且末字节要先 STOP 再读 DR，无法用「先 read_byte 再 write_ack」
     * 这种逐字节原语表达，否则多字节读会错位（详见 inter_i2c_hw.cpp）。
     *
     * @param buf 接收缓冲区（不可为 nullptr）
     * @param len 字节数（>= 1）
     * @note  本方法内含 STOP，调用方不要再调用 stop()。
     */
    virtual void read_bytes(uint8_t *buf, uint16_t len)
    {
        if (buf == nullptr || len == 0U) return;

        for (uint16_t i = 0; i < len; i++)
        {
            buf[i] = read_byte();
            write_ack((i == (uint16_t)(len - 1U)) ? 1U : 0U);
        }
        stop();
    }

    // ── 互斥 ──────────────────────────────────────────────
    virtual void    lock()   = 0;
    virtual void    unlock() = 0;
    virtual uint8_t is_busy() const = 0;

    // ── 总线恢复（从机卡死时发 9 个时钟 + STOP） ────────────
    virtual void bus_recovery() = 0;
};

#endif /* __cplusplus */
