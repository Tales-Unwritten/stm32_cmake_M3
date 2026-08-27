#pragma once

#ifdef __cplusplus

#include <cstdint>
#include "inter_spi_bus.hpp"

// ════════════════════════════════════════════════════════════
//  W25Q128 命令定义
// ════════════════════════════════════════════════════════════
#define W25Q_CMD_WRITE_ENABLE  0x06
#define W25Q_CMD_WRITE_DISABLE 0x04
#define W25Q_CMD_READ_STATUS1  0x05
#define W25Q_CMD_READ_DATA     0x03
#define W25Q_CMD_PAGE_PROGRAM  0x02
#define W25Q_CMD_SECTOR_ERASE  0x20
#define W25Q_CMD_CHIP_ERASE    0xC7
#define W25Q_CMD_JEDEC_ID      0x9F

// ════════════════════════════════════════════════════════════
//  W25Q128 全功能驱动
// ════════════════════════════════════════════════════════════

/**
 * @brief W25Q128 SPI NOR Flash 驱动
 *
 * 使用 spi_port 进行通信。写入前自动调用 write_enable + wait_busy。
 *
 * GPIOA: CS=PA4, CLK=PA5, MISO=PA6, MOSI=PA7, SPI1, AF5, Mode 3
 * （占位配置，用户按实物修改引脚）
 */
class w25q128 {
public:
    explicit w25q128(spi_bus &spi);
    void init();

    // ── 识别 ──────────────────────────────────────────
    uint32_t jedec_id();         ///< 预期 0xEF7018
    uint8_t  read_status();      ///< 状态寄存器 1

    // ── 控制 ──────────────────────────────────────────
    void write_enable();
    void write_disable();
    void wait_busy();            ///< 轮询 BUSY 位直到空闲

    // ── 数据读写 ──────────────────────────────────────
    void read(uint32_t addr, uint8_t *buf, uint16_t len);
    void write(uint32_t addr, const uint8_t *buf, uint16_t len);  ///< 自动擦除+跨页写入

    // ── 擦除 ──────────────────────────────────────────
    void sector_erase(uint32_t addr);  ///< 4KB 扇区擦除
    void chip_erase();                 ///< 全片擦除（~80s）

private:
    void     _cs_low();
    void     _cs_high();
    uint8_t  _transfer(uint8_t data);
    void     _write_page(uint32_t addr, const uint8_t *buf, uint16_t len);

    spi_bus &_spi;
    bool      _init;
};

#endif
