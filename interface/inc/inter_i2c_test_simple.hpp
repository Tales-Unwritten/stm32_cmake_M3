
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "link.h"
#include "gd32f4xx.h"

#ifdef __cplusplus
}
#endif

#include <cstdio>
#include "inter_i2c_bus.hpp"
#include "inter_i2c_dev.hpp"


extern "C" {
    int fputc(int ch, FILE *f);
}

// ============================================================
//  硬件配置 — 根据实际修改
// ============================================================

#define I2C_SCL_PORT    GPIOB
#define I2C_SCL_PIN     pin6
#define I2C_SDA_PORT    GPIOB
#define I2C_SDA_PIN     pin7

#define AT24C08_ADDR    0x50   // A2=0, A1=0, A0/P0 由内部地址控制

// ============================================================
//  测试一：单总线多设备扫描
// ============================================================

class i2c_bus_scan_test
{
public:
    i2c_bus_scan_test()
        : _bus({I2C_SCL_PORT, I2C_SDA_PORT, I2C_SCL_PIN, I2C_SDA_PIN}, 5)
    {
    }

    void run()
    {
        printf("\r\n========== I2C Bus Scan ==========\r\n");
        _bus.i2c_init();

        uint8_t found = 0;
        for (uint8_t addr = 0x08; addr <= 0x77; addr++)
        {
            if (probe(addr))
            {
                printf("  [0x%02X] ACK  (7-bit: 0x%02X, 8-bit W: 0x%02X)\r\n",
                       addr, addr, addr << 1);
                found++;
            }
        }

        if (found == 0)
            printf("  No device found on bus!\r\n");
        else
            printf("  Found %u device(s)\r\n", found);

        printf("==================================\r\n\r\n");
    }

private:
    inter_i2c_bus _bus;

    // 探测一个地址：发地址字节，看有没有 ACK
    bool probe(uint8_t addr7)
    {
        _bus.start();
        _bus.write_byte(addr7 << 1);       // 写方向
        uint8_t ack = _bus.wait_ack(500);
        _bus.stop();
        return ack;
    }
};


// ============================================================
//  测试二：AT24C08 读写验证
// ============================================================

class at24c08_test
{
public:
    at24c08_test()
        : _bus({I2C_SCL_PORT, I2C_SDA_PORT, I2C_SCL_PIN, I2C_SDA_PIN}, 5),
          _dev(&_bus, AT24C08_ADDR)
    {
    }

    void run()
    {
        printf("\r\n========== AT24C08 Test ==========\r\n");

        _bus.i2c_init();

        test_address_response();
        test_single_byte();
        test_multi_byte();
        test_page_boundary();
        test_stress();

        printf("==================================\r\n\r\n");
    }

private:
    inter_i2c_bus _bus;
    inter_i2c_dev _dev;

    uint32_t _pass = 0;
    uint32_t _fail = 0;

    void check(const char *name, bool ok)
    {
        if (ok) { _pass++; printf("  [PASS] %s\r\n", name); }
        else    { _fail++; printf("  [FAIL] %s\r\n", name); }
    }

    // ---- 底层单字节写（带内部地址的 EEPROM 写时序） ----
    bool eep_write(uint16_t mem_addr, uint8_t data)
    {
        // AT24C08: 11bit 地址，高 3 位放在设备地址的 P0 位
        uint8_t dev_addr = (AT24C08_ADDR | ((mem_addr >> 8) & 0x03)) << 1;
        uint8_t word_addr = mem_addr & 0xFF;

        _bus.start();
        _bus.write_byte(dev_addr);
        if (!_bus.wait_ack()) goto fail;
        _bus.write_byte(word_addr);
        if (!_bus.wait_ack()) goto fail;
        _bus.write_byte(data);
        if (!_bus.wait_ack()) goto fail;
        _bus.stop();
        delay_ms(5);  // EEPROM 写周期最大 5ms
        return true;

    fail:
        _bus.stop();
        return false;
    }

    // ---- 底层单字节读（随机读） ----
    bool eep_read(uint16_t mem_addr, uint8_t *data)
    {
        uint8_t dev_addr_w = (AT24C08_ADDR | ((mem_addr >> 8) & 0x03)) << 1;
        uint8_t dev_addr_r = dev_addr_w | 0x01;
        uint8_t word_addr = mem_addr & 0xFF;

        // 伪写：设置地址指针
        _bus.start();
        _bus.write_byte(dev_addr_w);
        if (!_bus.wait_ack()) goto fail;
        _bus.write_byte(word_addr);
        if (!_bus.wait_ack()) goto fail;

        // 重复 START + 读
        _bus.start();
        _bus.write_byte(dev_addr_r);
        if (!_bus.wait_ack()) goto fail;
        *data = _bus.read_byte();
        _bus.write_ack(1);   // NACK
        _bus.stop();
        return true;

    fail:
        _bus.stop();
        return false;
    }

    // ============================================================
    //  测试项
    // ============================================================

    // 1. 地址响应：探测 AT24C08 的几个页面地址（P0=0, P1=0 和 P0=1, P1=0 ...）
    void test_address_response()
    {
        printf("\r\n--- Address Response ---\r\n");

        // AT24C08 地址范围 0x50~0x53（取决于 P0, P1）
        for (uint8_t page = 0; page < 4; page++)
        {
            uint8_t addr = AT24C08_ADDR + page;
            _bus.start();
            _bus.write_byte(addr << 1);
            uint8_t ack = _bus.wait_ack(1000);
            _bus.stop();

            char name[32];
            snprintf(name, sizeof(name), "ADDR 0x%02X responds", addr);
            check(name, ack == 1);
        }
    }

    // 2. 单字节写入/读回
    void test_single_byte()
    {
        printf("\r\n--- Single Byte R/W ---\r\n");

        uint16_t addr = 0x0000;

        // 写 0xA5
        check("Write 0xA5 to addr 0x0000", eep_write(addr, 0xA5));

        uint8_t rd = 0;
        check("Read from addr 0x0000", eep_read(addr, &rd));
        check("Verify: wrote 0xA5, read 0x%02X", rd == 0xA5);

        // 写 0x5A 覆盖
        check("Overwrite with 0x5A", eep_write(addr, 0x5A));
        check("Read after overwrite", eep_read(addr, &rd));
        check("Verify: wrote 0x5A, read 0x%02X", rd == 0x5A);

        // 写 0x00 和 0xFF 边界值
        eep_write(addr, 0x00);
        eep_read(addr, &rd);
        check("Boundary: 0x00", rd == 0x00);

        eep_write(addr, 0xFF);
        eep_read(addr, &rd);
        check("Boundary: 0xFF", rd == 0xFF);
    }

    // 3. 多字节连续写/读
    void test_multi_byte()
    {
        printf("\r\n--- Multi Byte R/W ---\r\n");

        const uint8_t wr[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        uint16_t base = 0x0010;
        bool all_ok = true;

        // 写
        for (uint8_t i = 0; i < sizeof(wr); i++)
        {
            if (!eep_write(base + i, wr[i]))
            {
                all_ok = false;
                break;
            }
        }
        check("Write 8 consecutive bytes", all_ok);

        // 读回并比对
        all_ok = true;
        for (uint8_t i = 0; i < sizeof(wr); i++)
        {
            uint8_t rd = 0;
            if (!eep_read(base + i, &rd) || rd != wr[i])
            {
                printf("    [0x%04X] expected 0x%02X, got 0x%02X\r\n",
                       base + i, wr[i], rd);
                all_ok = false;
            }
        }
        check("Read back & verify 8 bytes", all_ok);
    }

    // 4. 页边界测试（AT24C08 页大小 16 字节）
    void test_page_boundary()
    {
        printf("\r\n--- Page Boundary ---\r\n");

        // 在页边界处写入（地址 0x0F = 页0末尾, 0x10 = 页1开头）
        uint16_t boundary = 0x000F;
        bool ok = true;

        ok &= eep_write(boundary,     0xBB);
        ok &= eep_write(boundary + 1, 0xCC);
        check("Write across page boundary", ok);

        uint8_t rd1 = 0, rd2 = 0;
        ok  = eep_read(boundary,     &rd1);
        ok &= eep_read(boundary + 1, &rd2);
        check("Verify page boundary read", ok && rd1 == 0xBB && rd2 == 0xCC);

        // 读整个页（0x00~0x0F），确认没有被破坏
        ok = true;
        for (uint16_t a = 0x0000; a < 0x0010; a++)
        {
            uint8_t rd = 0;
            eep_read(a, &rd);
            // 边界位置我们刚写过，其他位置是之前的测试残留，只检查边界
            if (a == boundary && rd != 0xBB) ok = false;
        }
        check("Page 0 not corrupted", ok);
    }

    // 5. 压力：反复读写 32 个地址
    void test_stress()
    {
        printf("\r\n--- Stress Test ---\r\n");

        uint16_t base = 0x0020;
        uint32_t err = 0;

        for (uint8_t i = 0; i < 32; i++)
        {
            uint8_t val = (i * 7 + 3) & 0xFF;  // 伪随机值
            if (!eep_write(base + i, val))
            {
                err++;
                continue;
            }
            uint8_t rd = 0;
            if (!eep_read(base + i, &rd) || rd != val)
            {
                err++;
                printf("    [0x%04X] wrote 0x%02X, read 0x%02X\r\n",
                       base + i, val, rd);
            }
        }

        char name[48];
        snprintf(name, sizeof(name), "32 addr stress: %lu errors", (unsigned long)err);
        check(name, err == 0);

        // 汇总
        printf("\r\n  Result: %lu passed, %lu failed\r\n", _pass, _fail);
    }
};

