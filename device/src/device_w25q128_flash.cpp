#include "device_w25q128_flash.hpp"

// ── W25Q128 命令 ──────────────────────────────────────────
#define W25Q_CMD_JEDEC_ID  0x9F

// ── SPI 配置（占位配置，用户按实物修改） ──────────────────
// SPI1, GPIOA: CS=PA4, CLK=PA5, MISO=PA6, MOSI=PA7, AF5
// Mode 3 (CPOL=1, CPHA=1), prescaler=2, MSB, 8-bit
// 注：GD32F4 的 SPI1 在 PA5/PA6/PA7 上复用功能为 AF5（占位，按实物核对）

w25q128_flash::w25q128_flash()
    : _spi(SpiPortConfig{
          spi_id::spi1,
          GPIOA, pin5,   // SCK
          GPIOA, pin7,   // MOSI
          GPIOA, pin6,   // MISO
          GPIOA, pin4,   // CS
          RESET,                          // CS 低有效
          afio_enum_t::NONE,
          SPI_BAUDRATEPRESCALER_2,
          SPI_POLARITY_HIGH,
          SPI_PHASE_2EDGE,                // Mode 3
          SPI_FIRSTBIT_MSB,
          SPI_DATASIZE_8BIT
      })
    , _initialized(false)
{
}

void w25q128_flash::init()
{
    _spi.init();
    _initialized = true;
}

uint32_t w25q128_flash::read_jedec_id()
{
    uint32_t id = 0;

    _spi.cs_select();
    _spi.transfer_byte(W25Q_CMD_JEDEC_ID);
    id  = (uint32_t)_spi.transfer_byte(0xFF) << 16;
    id |= (uint32_t)_spi.transfer_byte(0xFF) << 8;
    id |= (uint32_t)_spi.transfer_byte(0xFF);
    _spi.cs_deselect();

    return id;
}
