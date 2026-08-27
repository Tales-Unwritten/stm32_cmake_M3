#include "device_w25q128.hpp"

w25q128::w25q128(spi_bus &spi) : _spi(spi), _init(false) {}

void w25q128::init()
{
    _init = true;
}

// ════════════════════════════════════════════════════════════
//  CS 控制
// ════════════════════════════════════════════════════════════

void w25q128::_cs_low()  { _spi.cs_select(); }
void w25q128::_cs_high() { _spi.cs_deselect(); }

uint8_t w25q128::_transfer(uint8_t data)
{
    return _spi.transfer_byte(data);
}

// ════════════════════════════════════════════════════════════
//  识别
// ════════════════════════════════════════════════════════════

uint32_t w25q128::jedec_id()
{
    uint32_t id = 0;
    _cs_low();
    _transfer(W25Q_CMD_JEDEC_ID);
    id  = (uint32_t)_transfer(0xFF) << 16;
    id |= (uint32_t)_transfer(0xFF) << 8;
    id |= (uint32_t)_transfer(0xFF);
    _cs_high();
    return id;
}

uint8_t w25q128::read_status()
{
    uint8_t status;
    _cs_low();
    _transfer(W25Q_CMD_READ_STATUS1);
    status = _transfer(0xFF);
    _cs_high();
    return status;
}

// ════════════════════════════════════════════════════════════
//  控制
// ════════════════════════════════════════════════════════════

void w25q128::write_enable()
{
    _cs_low();
    _transfer(W25Q_CMD_WRITE_ENABLE);
    _cs_high();
}

void w25q128::write_disable()
{
    _cs_low();
    _transfer(W25Q_CMD_WRITE_DISABLE);
    _cs_high();
}

void w25q128::wait_busy()
{
    while (read_status() & 0x01);  // BUSY = bit 0
}

// ════════════════════════════════════════════════════════════
//  数据读取
// ════════════════════════════════════════════════════════════

void w25q128::read(uint32_t addr, uint8_t *buf, uint16_t len)
{
    _cs_low();
    _transfer(W25Q_CMD_READ_DATA);
    _transfer((uint8_t)(addr >> 16));
    _transfer((uint8_t)(addr >> 8));
    _transfer((uint8_t)(addr));
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = _transfer(0xFF);
    }
    _cs_high();
}

// ════════════════════════════════════════════════════════════
//  数据写入（自动跨页）
// ════════════════════════════════════════════════════════════

void w25q128::_write_page(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    write_enable();

    _cs_low();
    _transfer(W25Q_CMD_PAGE_PROGRAM);
    _transfer((uint8_t)(addr >> 16));
    _transfer((uint8_t)(addr >> 8));
    _transfer((uint8_t)(addr));
    for (uint16_t i = 0; i < len; i++) {
        _transfer(buf[i]);
    }
    _cs_high();

    wait_busy();
}

void w25q128::write(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    while (len > 0) {
        uint32_t page_end = (addr & 0xFFFFFF00) + 256;  // 下一页边界
        uint16_t chunk = (uint16_t)(page_end - addr);
        if (chunk > len) chunk = len;

        _write_page(addr, buf, chunk);

        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
}

// ════════════════════════════════════════════════════════════
//  擦除
// ════════════════════════════════════════════════════════════

void w25q128::sector_erase(uint32_t addr)
{
    write_enable();

    _cs_low();
    _transfer(W25Q_CMD_SECTOR_ERASE);
    _transfer((uint8_t)(addr >> 16));
    _transfer((uint8_t)(addr >> 8));
    _transfer((uint8_t)(addr));
    _cs_high();

    wait_busy();  // 擦除约 45ms
}

void w25q128::chip_erase()
{
    write_enable();

    _cs_low();
    _transfer(W25Q_CMD_CHIP_ERASE);
    _cs_high();

    wait_busy();  // 全片擦除约 80s
}
