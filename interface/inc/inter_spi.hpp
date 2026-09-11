#pragma once
// ============================================================
// @platform STM32F1xx
//   基于 STM32 HAL 的硬件 SPI 主机端口
// ============================================================

#ifdef __cplusplus

#include "inter_dma.hpp" // DMA 通道（可选：enable_dma() 后才占用）
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
 * @brief 硬件 SPI 配置结构体（适用于 STM32F1 系列）
 *
 * 该结构体封装了初始化一个硬件 SPI 外设所需的全部参数，包括引脚定义、
 * 时钟配置、数据格式、片选控制等。用户填充此结构体后，可传递给 SPI 初始化函数。
 */
struct SpiPortConfig
{
    spi_id periph;                      ///< 外设 ID，指定使用哪个 SPI 外设（如 spi1、spi2,spi3）
                                        ///< 可选值：通常是枚举类型，例如 SPI1、SPI2 等

    GPIO_TypeDef *sck_port;             ///< SCK 时钟引脚所在的 GPIO 端口（如 GPIOA、GPIOB）
    pin_enum_t sck_pin;                 ///< SCK 引脚编号（如 GPIO_PIN_5、GPIO_PIN_13）
                                        ///< 注意：不同 SPI 外设的引脚有固定映射，需参考数据手册选择正确的端口和引脚

    GPIO_TypeDef *mosi_port;            ///< MOSI（主机输出从机输入）引脚所在的 GPIO 端口
    pin_enum_t mosi_pin;                ///< MOSI 引脚编号

    GPIO_TypeDef *miso_port;            ///< MISO（主机输入从机输出）引脚所在的 GPIO 端口
    pin_enum_t miso_pin;                ///< MISO 引脚编号

    GPIO_TypeDef *cs_port;              ///< CS（片选）引脚所在的 GPIO 端口，若为 nullptr 表示不使用软件控制的 CS
                                        ///< 通常硬件 SPI 的 NSS 引脚可由硬件管理，但也可用普通 GPIO 软件控制片选
    pin_enum_t cs_pin;                  ///< CS 引脚编号，若 cs_port 为 nullptr 则忽略此值；若使用软件 CS，则此引脚有效

    polarity cs_active_level = active_low; ///< CS 有效电平（选中从机时的电平）
                                        ///< 可选值：active_low（低电平有效）或 active_high（高电平有效）
                                        ///< 注意：大多数 SPI 从机使用低电平有效（即 CS 拉低时选中）

    afio_enum_t af = afio_enum_t::NONE; ///< AFIO 重映射选项，用于将 SPI 引脚重映射到其他位置（仅 STM32F1 系列需要）
                                        ///< 默认值为 NONE（不重映射），当默认引脚被占用或需要特殊布局时设置相应重映射值
                                        ///< 可选值：如 AFIO_NONE、AFIO_SPI1_REMAP、AFIO_SPI2_REMAP 等

    uint32_t prescaler;                 ///< SPI 时钟预分频值，决定 SCK 频率 = PCLK / 预分频系数
                                        ///< 可选值：SPI_BAUDRATEPRESCALER_2、_4、_8、_16、_32、_64、_128、_256
                                        ///< 注意：需要根据从机支持的最大时钟频率合理选择，避免通信失败

    uint32_t clock_polarity;            ///< 时钟极性（CPOL），定义 SCK 空闲时的电平
                                        ///< 可选值：SPI_POLARITY_LOW（空闲低）或 SPI_POLARITY_HIGH（空闲高）

    uint32_t clock_phase;               ///< 时钟相位（CPHA），定义数据采样发生在第几个时钟边沿
                                        ///< 可选值：SPI_PHASE_1EDGE（第一边沿采样）或 SPI_PHASE_2EDGE（第二边沿采样）
                                        ///< 与 clock_polarity 组合成四种 SPI 模式（Mode 0~3）

    uint32_t first_bit;                 ///< 数据位传输顺序
                                        ///< 可选值：SPI_FIRSTBIT_MSB（高位先发）或 SPI_FIRSTBIT_LSB（低位先发）

    uint16_t data_size;                 ///< 数据帧大小，通常为 8 位或 16 位
                                        ///< 可选值：SPI_DATASIZE_8BIT 或 SPI_DATASIZE_16BIT
                                        ///< 注意：某些 SPI 外设还支持其他数据宽度，具体参考芯片手册
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
 *       GPIOA, pin6, GPIOA, pin4, active_low,
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

    // ── DMA 块传输（可选；需先 enable_dma()） ──────────────
    //
    //  F1 的 SPI DMA 请求映射（硬件固定）：
    //    SPI1: RX=DMA1_Ch2 TX=DMA1_Ch3      SPI2: RX=DMA1_Ch4 TX=DMA1_Ch5
    //    SPI3: F103 无此 SPI → enable_dma() 返回 false
    //  硬件冲突提醒：SPI1 的 Ch2/Ch3 与 USART3 共用；SPI2 的 Ch4/Ch5 与 USART1/I2C2 共用。
    //
    //  典型用法（W25Qxx 读一整帧：命令+地址+数据一次搬完，CS 由调用方管）：
    //    uint8_t tx[4 + N] = {0x03, addr>>16, addr>>8, addr, /* 后面全 0xFF */};
    //    uint8_t rx[4 + N];
    //    spi.cs_select();
    //    spi.transfer_dma(tx, rx, sizeof(tx));  // 数据在 rx + 4
    //    spi.cs_deselect();

    /** @brief 开通本 SPI 的 DMA 收发通道（幂等）
     *  @return true=可用；false=该 SPI 无 DMA 映射或通道号越界 */
    bool enable_dma();

    /** @brief DMA 是否已开通 */
    [[nodiscard]] bool dma_enabled() const noexcept
    {
        return _dma_tx != nullptr && _dma_rx != nullptr;
    }

    /** @brief 全双工块传输（DMA，阻塞到 HAL 状态回 READY）
     *  @return true=完成  false=未开 DMA/参数错/超时 */
    bool transfer_dma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms = 200);

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
    void _release_dma(); // 析构两个 DMA 通道并让出资源

    SpiPortConfig _cfg;
    io_ctrl _sck;
    io_ctrl _mosi;
    io_ctrl _miso;
    io_ctrl _cs; // 软件 CS（_has_cs=false 时未初始化）
    bool _has_cs;
    bool _initialized;
    SPI_HandleTypeDef _hspi{}; // STM32 HAL SPI 句柄

    // ── DMA 资源（enable_dma() 后才有值；对齐存储 + placement new，零堆分配） ──
    struct
    {
        alignas(dma_channel) uint8_t rx[sizeof(dma_channel)];
        alignas(dma_channel) uint8_t tx[sizeof(dma_channel)];
    } _dma_storage;
    dma_channel *_dma_rx;
    dma_channel *_dma_tx;
};

#endif /* __cplusplus */
