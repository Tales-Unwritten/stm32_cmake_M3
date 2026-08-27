#pragma once

#ifdef __cplusplus
#include "stdint.h"
// #include <cstdint>
#include "inter_io_ctrl.hpp"
#include "inter_spi_bus.hpp"
#include "delay.h"

/**
 * @brief 软件 SPI 总线（GPIO 位操作模拟，阻塞全双工）
 *
 * 架构对齐 inter_i2c_bus（总线模式）：
 *  - SCK/MOSI/MISO 为共享总线，由本对象独占管理（含互斥锁）
 *  - CS 双模式：内部单 CS（Config 指定，自动 select/deselect）
 *              或外部 CS（cs_port=nullptr，由设备驱动层自行控制，支持多设备共享总线）
 *  - 零堆分配：全部 io_ctrl 成员对象
 *
 * 参考开源实现：
 *  - Rob Tillaart SWSPI（Arduino）：mode/bitOrder 语义、位循环思路
 *  - 项目 inter_i2c_bus：io_ctrl 组合、lock/unlock 互斥、delay_us 速率控制
 */
struct SoftSpiConfig
{
    // ── 共享总线引脚 ──
    GPIO_TypeDef* sck_port;        // SCK 端口，如 GPIOA
    pin_enum_t    sck_pin;         // SCK 引脚掩码，如 pin5
    GPIO_TypeDef* mosi_port;       // MOSI 端口
    pin_enum_t    mosi_pin;        // MOSI 引脚掩码
    GPIO_TypeDef* miso_port;       // MISO 端口（输入）
    pin_enum_t    miso_pin;        // MISO 引脚掩码

    // ── 内部片选（nullptr/pin_none = 外部 CS 模式） ──
    GPIO_TypeDef* cs_port        = nullptr;  // CS 端口（nullptr = 不使用内部 CS）
    pin_enum_t    cs_pin         = pin_none; // CS 引脚掩码（pin_none = 不使用）
    uint32_t cs_active_level = 0;  // CS 有效电平: RESET(0,低有效) / SET(1,高有效)

    // ── 时序 ──
    uint8_t  mode           = 0;   // SPI_MODE0~3（CPOL = mode>>1, CPHA = mode&1）
    uint8_t  bit_order      = 0;   // 0 = MSB first, 1 = LSB first
    uint8_t  delay_us       = 0;   // 半周期延时（0 = 最快速度，io_ctrl 开销决定速率）
};

/**
 * @brief 软件 SPI 总线（阻塞式全双工，基础资源层）
 *
 * 使用示例（单设备，内部 CS，MODE0，MSB first）：
 *   static soft_spi_bus flash({
 *       GPIOA, pin5, GPIOA, pin7, GPIOA, pin6,
 *       GPIOA, pin4, RESET, 0, 0, 0
 *   });
 *   flash.init();
 *   flash.cs_select();
 *   flash.transfer(tx, rx, len);
 *   flash.cs_deselect();
 *
 * 使用示例（多设备共享总线，外部 CS）：
 *   // cs_port = nullptr；每设备各自持有 io_ctrl 片选：
 *   //   dev_a.cs_select();  bus.transfer(...);  dev_a.cs_deselect();
 */
class soft_spi_bus : public spi_bus
{
public:
    explicit soft_spi_bus(const SoftSpiConfig &cfg);
    ~soft_spi_bus();

    soft_spi_bus(const soft_spi_bus &) = delete;
    soft_spi_bus &operator=(const soft_spi_bus &) = delete;

    // ── 生命周期 ──────────────────────────────────────────

    void init() override;                       // SCK/MOSI=推挽输出, MISO=输入, CS=输出(可选)
    void deinit() override;                     // 全部恢复模拟输入，可再次 init()
    [[nodiscard]] bool is_initialized() const noexcept { return _initialized; }

    // ── 数据传输（阻塞、全双工） ──────────────────────────

    /**
     * @brief 单字节全双工传输（发送 data，返回接收值）
     */
    uint8_t transfer_byte(uint8_t data) override;

    /**
     * @brief 多字节全双工传输（字节间 CS 不断开）
     * @param tx  发送缓冲区（nullptr = 发送 0xFF 占位）
     * @param rx  接收缓冲区（nullptr = 丢弃接收数据）
     * @param len 传输字节数
     * @note  tx/rx 可指向同一缓冲区（原地交换）
     */
    void transfer(const uint8_t *tx, uint8_t *rx, uint16_t len) override;

    // ── 片选（仅内部 CS 模式有效） ────────────────────────

    void cs_select() override;                  // 按 cs_active_level 拉有效电平
    void cs_deselect() override;                // 恢复无效电平

    // ── 运行时配置 ────────────────────────────────────────

    void set_mode(uint8_t mode);       // 0..3（运行时切换 CPOL/CPHA）
    void set_bit_order(uint8_t order); // 0=MSB first, 1=LSB first
    void set_speed(uint8_t delay_us);  // 半周期延时（0 = 最快）

    // ── 互斥（对齐 inter_i2c_bus，多任务/ISR 安全） ────────

    void lock();                       // IRQ-safe spinlock
    void unlock();
    [[nodiscard]] bool is_busy() const { return _busy != 0; }

private:
    void     _delay();                 // delay_us(_delay_us)；0 为空操作
    uint8_t  _transfer_byte_core(uint8_t data);   // 8 位位循环核心

    io_ctrl _sck;
    io_ctrl _mosi;
    io_ctrl _miso;
    io_ctrl _cs;                       // 内部 CS（_has_cs=false 时未初始化）

    uint8_t _mode;                     // SPI_MODE0~3
    uint8_t _bit_order;                // 0=MSB, 1=LSB
    uint8_t _delay_us;
    bool    _cpol;                     // 缓存：SCK 空闲电平（1=高, 0=低）
    bool    _cpha;                     // 缓存：采样沿（0=第一沿, 1=第二沿）
    bool    _cs_active_level;          // 缓存：CS 有效电平（1=高有效, 0=低有效）
    bool    _has_cs;
    bool    _initialized;
    volatile uint8_t _busy;
};

#endif /* __cplusplus */
