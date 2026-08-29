#pragma once
#include "inter_spi_bus.hpp"
// ════════════════════════════════════════════════════════════
//  W25Q128 命令定义
// ════════════════════════════════════════════════════════════
#define W25QXX_CMD_WRITE_ENABLE  0x06
#define W25QXX_CMD_WRITE_DISABLE 0x04
#define W25QXX_CMD_READ_STATUS1  0x05
#define W25QXX_CMD_READ_DATA     0x03
#define W25QXX_CMD_PAGE_PROGRAM  0x02
#define W25QXX_CMD_SECTOR_ERASE  0x20
#define W25QXX_CMD_CHIP_ERASE    0xC7
#define W25QXX_CMD_JEDEC_ID      0x9F

class w25qxx
{
  public:
    explicit w25qxx(spi_bus &spi);

    void init();
    uint32_t jedec_id();
    uint8_t read_status();

    void write_enable();
    void write_disable();
    void wait_busy();

    void read(uint32_t addr, uint8_t *buf, uint16_t len);
    void write(uint32_t addr, const uint8_t *buf, uint16_t len);

    void sector_rease(uint32_t addr);
    void chip_rease();

  private:
    void _cs_low();
    void _cs_high();
    uint8_t _transfer(uint8_t data);
    void _write_page(uint32_t addr, const uint8_t *buf, uint16_t len);

    spi_bus &_spi;
    bool _init;
};
