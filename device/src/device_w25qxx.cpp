#include "device_w25qxx.hpp"

#include "stm32f1xx_hal.h" // HAL_GetTick（wait_busy 超时兜底）

w25qxx::w25qxx(spi_bus &spi) : _spi(spi), _init(false)
{
}
void w25qxx::init()
{
    _init = true;
}

void w25qxx::_cs_low()
{
    _spi.cs_select(); // cs_select() 按配置拉低 CS（低有效）→ 选中器件
}
void w25qxx::_cs_high()
{
    _spi.cs_deselect(); // cs_deselect() 拉高 CS → 释放器件
}

uint8_t w25qxx::_transfer(uint8_t data)
{
    return _spi.transfer_byte(data);
}

uint32_t w25qxx::jedec_id()
{
    uint32_t id = 0;
    _cs_low();
    _transfer(W25QXX_CMD_JEDEC_ID);       // 命令字节返回值无效，丢弃
    id = (uint32_t)_transfer(0xFF) << 16; // Manufacturer ID, 期望 0xEF
    id |= (uint32_t)_transfer(0xFF) << 8; // Memory Type, 期望 0x40
    id |= (uint32_t)_transfer(0xFF);      // Capacity,   期望 0x17 (W25Q64)
    _cs_high();
    return id;
}

uint8_t w25qxx::read_status()
{
    uint8_t status = 0;
    _cs_low();
    _transfer(W25QXX_CMD_READ_STATUS1);
    status = _transfer(0xFF);
    _cs_high();
    return status;
}

void w25qxx::write_enable()
{
    _cs_low();
    _transfer(W25QXX_CMD_WRITE_ENABLE);
    _cs_high();
}

void w25qxx::write_disable()
{
    _cs_low();
    _transfer(W25QXX_CMD_WRITE_DISABLE);
    _cs_high();
}

void w25qxx::wait_busy()
{
    // 带超时兜底：器件不在/损坏时不能死等（片擦最慢，W25Q64 手册上限约 100s，
    // 这里给 30s；超时后直接返回，由调用方的后续读写自行暴露失败）
    const uint32_t t0 = HAL_GetTick();
    while (read_status() & 0x01)
    {
        if ((uint32_t)(HAL_GetTick() - t0) > 30000U)
            return;
    }
}

void w25qxx::read(uint32_t addr, uint8_t *buf, uint16_t len)
{
    _cs_low();
    _transfer(W25QXX_CMD_READ_DATA);
    _transfer((uint8_t)(addr >> 16)); // 24 位地址：高→低（左移会截断成 0，地址 >255 全错）
    _transfer((uint8_t)(addr >> 8));
    _transfer((uint8_t)(addr));

    for (uint16_t i = 0; i < len; i++)
    {

        buf[i] = _transfer(0xFF);
    }

    _cs_high();
}

void w25qxx::write(uint32_t addr, const uint8_t *buf, uint16_t len)
{

    while (len > 0)
    {

        uint32_t page_end = (addr & 0xFFFFFF00) + 256;
        uint16_t chunk = (uint16_t)(page_end - addr);
        if (chunk > len)
        {
            chunk = len;
        }
        _write_page(addr, buf, chunk);
        addr += chunk;
        buf += chunk;
        len -= chunk;
    }
}

void w25qxx::_write_page(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    _cs_low();
    _transfer(W25QXX_CMD_PAGE_PROGRAM);
    _transfer((uint8_t)(addr >> 16));
    _transfer((uint8_t)(addr >> 8));
    _transfer((uint8_t)(addr));
    for (uint16_t i = 0; i < len; i++)
    {
        _transfer(buf[i]);
    }
    _cs_high();
    wait_busy();
}

void w25qxx::sector_rease(uint32_t addr)
{
    _cs_low();
    _transfer(W25QXX_CMD_SECTOR_ERASE);
    _transfer((uint8_t)(addr >> 16));
    _transfer((uint8_t)(addr >> 8));
    _transfer((uint8_t)(addr));

    _cs_high();
    wait_busy();
}

void w25qxx::chip_rease()
{
    _cs_low();
    _transfer(W25QXX_CMD_CHIP_ERASE);

    _cs_high();
    wait_busy();
}
