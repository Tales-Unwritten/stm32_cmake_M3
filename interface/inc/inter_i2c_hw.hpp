#pragma once
// ============================================================
// @platform STM32F1xx（当前平台）
//   移植到新 MCU 时，本 .hpp 文件需替换：
//     - [PORT] #include "inter_io_ctrl.hpp" → 目标平台 GPIO 封装
//     - [PORT] enum class i2c_hw_id（按目标芯片调整枚举项）
//     - [PORT] 私有 _periph 的类型（目标平台 I2C 外设类型）
// ============================================================

#ifdef __cplusplus

#include <cstdint>
#include "inter_i2c.hpp"
#include "inter_io_ctrl.hpp"

// ============================================================
//  平台中性类型
// ============================================================

/** @brief 硬件 I2C 外设 ID（STM32F103 仅有 I2C1 / I2C2） */
enum class i2c_hw_id : uint8_t { i2c1, i2c2 };

// ============================================================
//  配置结构体（POD）
// ============================================================

struct I2cHwConfig {
    i2c_hw_id periph;      ///< 外设 ID
    GPIO_TypeDef* scl_port;  ///< SCL GPIO 端口
    pin_enum_t    scl_pin;   ///< SCL 引脚
    GPIO_TypeDef* sda_port;  ///< SDA GPIO 端口
    pin_enum_t    sda_pin;   ///< SDA 引脚
    afio_enum_t   af = afio_enum_t::RM_I2C1_DISABLE;  ///< I2C1 重映射选项（I2C2 固定 PB10/PB11，填 NONE）
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
 *   i2c_hw_port bus({i2c_hw_id::i2c1, GPIOB, pin6,
 *                     GPIOB, pin7, afio_enum_t::RM_I2C1_DISABLE, 400000});
 *   bus.init();
 *   bus.start();
 *   bus.write_byte(0xA0);     // 设备地址 (写)
 *   bus.wait_ack();
 *   bus.write_byte(0x00);     // 寄存器地址
 *   bus.i2c_restart();        // 重复 START
 *   bus.write_byte(0xA1);     // 设备地址 (读)
 *   bus.wait_ack();
 *   uint8_t rx[4];
 *   bus.read_bytes(rx, 4);    // 批量接收（内含末字节 NACK + STOP）
 *
 * 注意：wait_ack() 只等 ADDR 置位，**不在此处清除 ADDR**。
 *   写方向由 write_byte()/stop() 补清；读方向由 read_bytes() 在清 ADDR
 *   的同一临界区内按 len 配好 POS/ACK（这正是多字节接收能对齐的原因）。
 *
 * 单字节读仍可沿用旧写法（ACK 位在清 ADDR 前已就位）：
 *   bus.wait_ack();
 *   bus.write_ack(1);         // NACK
 *   bus.stop();               // 同时补清 ADDR + 发 STOP
 *   uint8_t data = bus.read_byte();
 *
 * 设备级使用示例（封装推荐，内部已按字节数走正确时序）：
 *   bus.i2c_write_reg(0x50, 0x00, tx_data, 2);
 *   bus.i2c_read_reg(0x50, 0x00, rx_data, 2);
 */
class i2c_hw_port : public i2c_bus {
public:
    explicit i2c_hw_port(const I2cHwConfig &cfg);
    ~i2c_hw_port();

    i2c_hw_port(const i2c_hw_port &) = delete;
    i2c_hw_port &operator=(const i2c_hw_port &) = delete;

    // ── 生命周期 ──────────────────────────────────────────
    void init() override;
    void deinit() override;
    [[nodiscard]] bool is_initialized() const noexcept { return _initialized; }

    // 总线控制（兼容 inter_i2c_bus 接口） ──────────────

    void    start() override;        ///< 起始条件（等待总线空闲）
    void    i2c_restart();      ///< 重复起始条件（不等待空闲，总线已占有）
    void    stop() override;
    void    write_byte(uint8_t data) override;
    uint8_t read_byte() override;
    bool    wait_ack(uint16_t timeout = 1000) override;
    void    write_ack(uint8_t ack) override;    // 0=ACK, 1=NACK

    /**
     * @brief 批量接收 len 个字节（F1 EV7 时序）+ 末字节 NACK + STOP
     * @note  覆写 i2c_bus 的默认逐字节实现：F1 硬件无法用
     *        「先 read_byte 再 write_ack」表达接收应答时序。
     *        进入前应是「读地址已发、wait_ack() 已收到 ADDR」的状态。
     */
    void    read_bytes(uint8_t *buf, uint16_t len) override;

    // ── 互斥 ──────────────────────────────────────────────
    void    lock() override;
    void    unlock() override;
    [[nodiscard]] uint8_t is_busy() const override { return _busy; }

    // ── 总线恢复 ──────────────────────────────────────────
    /** @brief 从机卡死恢复：关外设 → 手动 9 时钟 + STOP → 重配外设 */
    void bus_recovery() override;

    // ── 设备级便利方法（基于总线操作实现） ───────────────

    /** @brief 写设备寄存器：START→ADDR(W)→REG→DATA...→STOP */
    void i2c_write_reg(uint8_t dev_addr_7bit, uint8_t reg,
                       const uint8_t *data, uint16_t len);

    /** @brief 读设备寄存器：START→ADDR(W)→REG→RESTART→ADDR(R)→DATA...→STOP */
    void i2c_read_reg(uint8_t dev_addr_7bit, uint8_t reg,
                      uint8_t *data, uint16_t len);

    // ── 查询 ──────────────────────────────────────
    [[nodiscard]] i2c_hw_id periph() const noexcept { return _cfg.periph; }

private:
    void _enable_clock();
    void _disable_clock();
    void _configure_peripheral();   ///< 外设配置（init 与总线恢复共用）

    // 状态标志等待：超时（循环次数）返回 false，避免死等卡死
    bool _wait_sb(uint16_t timeout);      ///< 起始条件已发出 (SB)
    bool _wait_txe(uint16_t timeout);     ///< 发送数据寄存器空 (TXE)
    bool _wait_rxne(uint16_t timeout);    ///< 接收数据寄存器非空 (RXNE)
    bool _wait_btf(uint16_t timeout);     ///< 字节传输完成 (BTF)，含 NACK 检测
    void _wait_bus_idle(uint16_t timeout); ///< 等待总线释放 (BUSY=0)

    I2cHwConfig    _cfg;
    io_ctrl        _scl;
    io_ctrl        _sda;
    bool           _initialized;
    bool           _addr_phase;   ///< true = 下一个 write_byte 是地址字节（F1 需直接写 DR，不等 TxE）
    bool           _last_addr;    ///< true = 最近写入的是地址字节（wait_ack 等 ADDR；否则等 BTF）
    bool           _addr_pending; ///< true = ADDR 已置位但尚未清除（读/写/STOP 时按场景补清）
    I2C_TypeDef   *_periph;
    volatile uint8_t _busy;
};

#endif /* __cplusplus */
