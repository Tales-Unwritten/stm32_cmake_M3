#pragma once
// ============================================================
// @platform GD32F4xx
//   移植到新 MCU 时，本 .hpp 文件需替换：
//     - [PORT] #include "gd32f4xx.h" → 目标 SDK 头文件
//     - [PORT] enum class i2c_hw_id（按目标芯片调整枚举项）
// ============================================================

#ifdef __cplusplus

#include <cstdint>
#include "inter_io_ctrl.hpp"
#include "gd32f4xx.h"       // [PORT] 替换为目标 SDK 头文件

// ============================================================
//  平台中性类型
// ============================================================

enum class i2c_hw_id : uint8_t { i2c0, i2c1, i2c2 };

// ============================================================
//  配置结构体（POD）
// ============================================================

struct I2cHwConfig {
    i2c_hw_id periph;      ///< 外设 ID
    GPIO_TypeDef* scl_port;  ///< SCL GPIO 端口
    pin_enum_t    scl_pin;   ///< SCL 引脚掩码
    GPIO_TypeDef* sda_port;  ///< SDA GPIO 端口
    pin_enum_t    sda_pin;   ///< SDA 引脚掩码
    afio_enum_t   af = afio_enum_t::RM_I2C1_DISABLE;  ///< STM32F1 AFIO 重映射选项
    uint32_t  clock_speed; ///< 时钟速度: 100000 (标准) 或 400000 (快速)
};

// ============================================================
//  硬件 I2C 总线（兼容 inter_i2c_bus 接口）
// ============================================================

/**
 * @brief 硬件 I2C 总线端口
 *
 * 提供与 inter_i2c_bus 相同风格的总线级操作接口。
 * inter_i2c_dev 可以通过组合此对象来使用硬件 I2C。
 *
 * 总线级使用示例：
 *   i2c_hw_port bus({i2c_hw_id::i2c0, GPIOB, pin6,
 *                     GPIOB, pin7, afio_enum_t::RM_I2C1_DISABLE, 400000});
 *   bus.init();
 *   bus.start();
 *   bus.write_byte(0xA0);     // 设备地址 (写)
 *   bus.wait_ack();
 *   bus.write_byte(0x00);     // 寄存器地址
 *   bus.wait_ack();
 *   bus.start();              // 重复 START
 *   bus.write_byte(0xA1);     // 设备地址 (读)
 *   bus.wait_ack();
 *   uint8_t data = bus.read_byte();
 *   bus.write_ack(1);         // NACK
 *   bus.stop();
 *
 * 设备级使用示例（封装推荐）：
 *   bus.i2c_write_reg(0x50, 0x00, tx_data, 2);
 *   bus.i2c_read_reg(0x50, 0x00, rx_data, 2);
 */
class i2c_hw_port {
public:
    explicit i2c_hw_port(const I2cHwConfig &cfg);
    ~i2c_hw_port();

    i2c_hw_port(const i2c_hw_port &) = delete;
    i2c_hw_port &operator=(const i2c_hw_port &) = delete;

    // ── 生命周期 ──────────────────────────────────────────
    void init();
    void deinit();
    [[nodiscard]] bool is_initialized() const noexcept { return _initialized; }

    // 总线控制（兼容 inter_i2c_bus 接口） ──────────────

    void    start();        ///< 起始条件（等待总线空闲）
    void    i2c_restart();      ///< 重复起始条件（不等待空闲，总线已占有）
    void    stop();
    void    write_byte(uint8_t data);
    uint8_t read_byte();
    bool    wait_ack(uint16_t timeout = 1000);
    void    write_ack(uint8_t ack);    // 0=ACK, 1=NACK

    // ── 互斥 ──────────────────────────────────────────────
    void    lock();
    void    unlock();
    [[nodiscard]] uint8_t is_busy() const { return _busy; }

    // ── 设备级便利方法（基于总线操作实现） ───────────────

    /** @brief 写设备寄存器：START→ADDR(W)→REG→DATA...→STOP */
    void i2c_write_reg(uint8_t dev_addr_7bit, uint8_t reg,
                       const uint8_t *data, uint16_t len);

    /** @brief 读设备寄存器：START→ADDR(W)→REG→RESTART→ADDR(R)→DATA...→STOP */
    void i2c_read_reg(uint8_t dev_addr_7bit, uint8_t reg,
                      uint8_t *data, uint16_t len);

    // ── 查询 ──────────────────────────────────────────────
    [[nodiscard]] i2c_hw_id periph() const noexcept { return _cfg.periph; }

private:
    void _enable_clock();
    void _wait_flag(i2c_flag_enum flag);

    I2cHwConfig    _cfg;
    io_ctrl        _scl;
    io_ctrl        _sda;
    bool           _initialized;
    uint32_t       _periph;
    volatile uint8_t _busy;
};

#endif /* __cplusplus */
