#pragma once

#ifdef __cplusplus
#include "stdint.h"
#include "inter_i2c.hpp"
#include "inter_io_ctrl.hpp"
#include "delay.h"

/**
 * @brief  I2C 总线引脚配置（STM32F1 平台）
 */
typedef struct
{
    GPIO_TypeDef* PORT_SCL;   // SCL 所在端口，如 GPIOA
    GPIO_TypeDef* PORT_SDA;   // SDA 所在端口，如 GPIOB
    pin_enum_t SCL;        // SCL 引脚掩码，如 pin6
    pin_enum_t SDA;        // SDA 引脚掩码，如 pin7
} I2C_Bus_Info_t;

class inter_i2c_bus : public i2c_bus
{
public:
    inter_i2c_bus(const I2C_Bus_Info_t &cfg, uint8_t delay_us);

    // 禁止拷贝
    inter_i2c_bus(const inter_i2c_bus &) = delete;
    inter_i2c_bus &operator=(const inter_i2c_bus &) = delete;

    void init() override;

    // 总线控制:
    void start() override;
    void stop() override;

    uint8_t scan(uint8_t *found,uint8_t max_count, uint8_t start_addr,uint8_t end_addr);

    void write_byte(uint8_t ByteValue) override;
    uint8_t read_byte() override;

    bool wait_ack(uint16_t timeout = 1000) override;
    void write_ack(uint8_t AckValue) override;

    // 互斥控制
    void lock() override;
    void unlock() override;
    uint8_t is_busy() const override;

    // 总线恢复
    void bus_recovery() override;

    void _delay();
    void deinit() override;

private:
    void _write_scl(uint8_t BitValue);
    void _write_sda(uint8_t BitValue);
    polarity _read_sda();

private:
    io_ctrl _scl;          // SCL 引脚对象
    io_ctrl _sda;          // SDA 引脚对象
    uint8_t _delay_val;
    volatile uint8_t _busy;
};

#endif /* __cplusplus */
