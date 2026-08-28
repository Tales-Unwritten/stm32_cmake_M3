#pragma once
// ============================================================
// @platform STM32F1xx
//   基于 STM32 HAL 的硬件 SPI 主机端口
// ============================================================

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include "inter_spi_bus.hpp"
#include <cstdint>

// ============================================================
//  平台中性类型
// ============================================================

/** @brief SPI 外设 ID */
enum class spi_id : uint8_t
{
    spi1,
    spi2,
    spi3,
};

// ============================================================
//  配置结构体（POD）
// ============================================================

/**
 * @brief SPI 主机端口配置（STM32F1 HAL）
 *
 *   prescaler:      SPI_BAUDRATEPRESCALER_2 / _4 / _8 ...
 *   clock_polarity: SPI_POLARITY_LOW / SPI_POLARITY_HIGH
 *   clock_phase:    SPI_PHASE_1EDGE / SPI_PHASE_2EDGE
 *   first_bit:      SPI_FIRSTBIT_MSB / SPI_FIRSTBIT_LSB
 *   data_size:      SPI_DATASIZE_8BIT / SPI_DATASIZE_16BIT
 *   cs_active_level: RESET（低有效）或 SET（高有效）
 */
struct SpiPortConfig
{
    spi_id periph;                      ///< 外设 ID
    GPIO_TypeDef *sck_port;             ///< SCK GPIO 端口
    pin_enum_t sck_pin;                 ///< SCK 引脚掩码
    GPIO_TypeDef *mosi_port;            ///< MOSI GPIO 端口
    pin_enum_t mosi_pin;                ///< MOSI 引脚掩码
    GPIO_TypeDef *miso_port;            ///< MISO GPIO 端口
    pin_enum_t miso_pin;                ///< MISO 引脚掩码
    GPIO_TypeDef *cs_port;              ///< CS GPIO 端口（nullptr = 不使用软件 CS）
    pin_enum_t cs_pin;                  ///< CS 引脚掩码（pin_none = 不使用软件 CS）
    uint32_t cs_active_level;           ///< CS 有效电平: GPIO_PIN_SET 或 GPIO_PIN_RESET
    afio_enum_t af = afio_enum_t::NONE; ///< STM32F1 AFIO 重映射选项
    uint32_t prescaler;                 ///< SPI_BAUDRATEPRESCALER_x
    uint32_t clock_polarity;            ///< SPI_POLARITY_LOW / HIGH
    uint32_t clock_phase;               ///< SPI_PHASE_1EDGE / 2EDGE
    uint32_t first_bit;                 ///< SPI_FIRSTBIT_MSB / LSB
    uint16_t data_size;                 ///< SPI_DATASIZE_8BIT / 16BIT
};

// ============================================================
//  SPI 主机端口（阻塞模式，基础资源层）
// ============================================================

/**
 * @brief SPI 主机端口（阻塞式全双工传输）
 *
 * 使用示例（单设备，自动 CS 控制）：
 *   static spi_port flash({
 *       spi_id::spi1, GPIOA, pin5, GPIOA, pin7,
 *       GPIOA, pin6, GPIOA, pin4, GPIO_PIN_RESET,
 *       afio_enum_t::NONE, SPI_BAUDRATEPRESCALER_4,
 *       SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_FIRSTBIT_MSB, SPI_DATASIZE_8BIT
 *   });
 *   flash.init();
 *   flash.transfer(tx_buf, rx_buf, 16);
 *
 * 使用示例（多设备共享总线，手动 CS）：
 *   bus.cs_select();
 *   bus.transfer(tx, rx, len);
 *   bus.cs_deselect();
 */
class spi_port : public spi_bus
{
  public:
    explicit spi_port(const SpiPortConfig &cfg);
    ~spi_port();

    spi_port(const spi_port &) = delete;
    spi_port &operator=(const spi_port &) = delete;

    // ── 生命周期 ──────────────────────────────────────────
    void init() override;
    void deinit() override;
    [[nodiscard]] bool is_initialized() const noexcept
    {
        return _initialized;
    }

    // ── 数据传输（阻塞、全双工） ──────────────────────────

    /**
     * @brief 全双工传输
     * @param tx  发送缓冲区（nullptr = 发送 0xFF 占位）
     * @param rx  接收缓冲区（nullptr = 丢弃接收数据）
     * @param len 传输字节数
     * @note  tx 和 rx 可指向同一缓冲区（原地交换）
     */
    void transfer(const uint8_t *tx, uint8_t *rx, uint16_t len) override;

    /** @brief 单字节全双工传输（发送 data，返回接收值） */
    uint8_t transfer_byte(uint8_t data) override;

    // ── 片选控制 ──────────────────────────────────────────

    /** @brief 拉低/拉高 CS（取决于 cs_active_level 配置） */
    void cs_select() override;

    /** @brief 释放 CS（恢复到无效电平） */
    void cs_deselect() override;

    // ── 查询 ──────────────────────────────────────────────

    [[nodiscard]] spi_id periph() const noexcept
    {
        return _cfg.periph;
    }
    [[nodiscard]] bool has_cs() const noexcept
    {
        return _has_cs;
    }

  private:
    void _enable_clock();
    void _disable_clock();

    SpiPortConfig _cfg;
    io_ctrl _sck;
    io_ctrl _mosi;
    io_ctrl _miso;
    io_ctrl _cs; // 软件 CS（_has_cs=false 时未初始化）
    bool _has_cs;
    bool _initialized;
    SPI_HandleTypeDef _hspi{}; // STM32 HAL SPI 句柄
};

#endif /* __cplusplus */
