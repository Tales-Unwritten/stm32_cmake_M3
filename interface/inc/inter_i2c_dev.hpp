#pragma once

#ifdef __cplusplus

#include "inter_i2c_bus.hpp"

class inter_i2c_dev
{
public:
    inter_i2c_dev(inter_i2c_bus *bus, uint8_t addr);

    void     write_16bit(uint8_t reg, uint16_t data);
    uint16_t read_16bit(uint8_t reg);

    void     freedom_write(uint8_t reg, uint64_t data, uint8_t length);
    bool     freedom_read(uint8_t reg, uint64_t *data, uint8_t length);

    /**
     * @brief 设备探测：仅发地址字节检查 ACK，不读写寄存器
     * @return true = 设备在线；失败时可用 lastError() 查原因
     */
    bool ping();

    /**
     * @brief SMBus 通用软复位：向保留地址 0x00 发 0x06 命令，
     *        支持 SMBus 软复位的设备会复位（如 EEPROM 卡死解锁）
     * @note  总线级 9 脉冲恢复请用 inter_i2c_bus::bus_recovery()
     */
    void softReset();

    /**
     * @brief 最近一次总线操作的错误码（0 = 无错误）
     * @note  每次操作前自动清零；地址 NACK / 数据 NACK 记为 1
     */
    uint8_t lastError();

private:
    inter_i2c_bus *_bus;
    uint8_t _addr;
    uint8_t _err = 0;   // 0 = OK, 1 = NACK / 总线错误
};

#endif /* __cplusplus */
