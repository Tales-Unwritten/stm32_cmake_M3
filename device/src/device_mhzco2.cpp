#include "device_mhzco2.hpp"

#include <cstring>
#include "systick.h"   // get_tick()

// ============================================================
//  MH-Z19 系列 CO2 传感器 UART 驱动
//  来源：Rob Tillaart 的 Arduino 库 "MHZCO2" v0.2.4
//    URL：https://github.com/RobTillaart/MHZCO2
//  移植说明见 device_mhzco2.hpp 头注释
// ============================================================

// ============================================================
//  构造 / 复位
// ============================================================

MHZCO2::MHZCO2(usart_port& uart)
    : _uart(uart)
    , _timeout(1000)
    , _range(0)
    , _co2(0)
    , _temp_mC(0)
    , _accuracy(0)
    , _minCO2(0)
    , _maxCO2(0)
    , _lastMeasurement(0)
{
}

void MHZCO2::reset()
{
    _range           = 0;
    _co2             = 0;
    _temp_mC         = 0;
    _accuracy        = 0;
    _minCO2          = 0;
    _maxCO2          = 0;
    _lastMeasurement = 0;
}

// ============================================================
//  测量
// ============================================================

int MHZCO2::measure()
{
    uint8_t answer[FRAME_LEN];

    _send9(CMD_MEASURE, 0, 0, 0, 0);

    int rv = _receive(answer);

    // 参考库：CRC 错也保留部分数据（数据一致性由调用方判断）
    if ((rv == ERR_OK) || (rv == ERR_CRC))
    {
        _co2      = (uint16_t)((answer[2] << 8) | answer[3]);
        _temp_mC  = (int32_t)(answer[4] - 40) * 1000;   // °C → m°C
        _accuracy = answer[5];
        _minCO2   = (uint16_t)((answer[6] << 8) | answer[7]);
        if (_co2 > _maxCO2)
        {
            _maxCO2 = _co2;
        }
    }
    if (rv == ERR_OK)
    {
        _lastMeasurement = get_tick();
    }
    return rv;
}

// ============================================================
//  校准 / 量程
// ============================================================

void MHZCO2::calibrateZero()
{
    _send9(CMD_CAL_ZERO, 0, 0, 0, 0);
}

void MHZCO2::calibrateSpan(uint16_t span)
{
    _send9(CMD_CAL_SPAN, (uint8_t)(span >> 8), (uint8_t)(span & 0xFF), 0, 0);
}

void MHZCO2::calibrateAuto(bool mode)
{
    _send9(CMD_ABC_MODE, mode ? 0xA0 : 0x00, 0, 0, 0);
}

void MHZCO2::setRange(uint16_t ppm)
{
    _range = ppm;
    // 量程值放在 d6/d7（与参考库 setPPM 一致）
    _send9(CMD_SET_RANGE, 0, 0, (uint8_t)(ppm >> 8), (uint8_t)(ppm & 0xFF));
}

void MHZCO2::setTimeOut(uint16_t timeout)
{
    _timeout = timeout;
}

// ============================================================
//  私有：组帧发送 / 读响应
// ============================================================

void MHZCO2::_send9(uint8_t cmd, uint8_t d3, uint8_t d4, uint8_t d6, uint8_t d7)
{
    uint8_t frame[FRAME_LEN] = {0xFF, 0x01, cmd, d3, d4, 0x00, d6, d7, 0x00};
    frame[8] = _checksum(frame);
    _uart.send_data(frame, FRAME_LEN);   // 阻塞发送
}

int MHZCO2::_receive(uint8_t answer[FRAME_LEN])
{
    uart_buffer_t *buf = _uart.buffer();
    uint32_t start = get_tick();

    // 轮询 RX 中断缓冲，凑满 9 字节
    while (buf->rx_len < FRAME_LEN)
    {
        if ((_timeout > 0) && (get_tick() - start > _timeout))
        {
            // 丢弃残留字节，避免污染下一次响应
            buf->rx_len  = 0;
            buf->rx_flag = 0;
            return ERR_TIMEOUT;
        }
    }

    memcpy(answer, buf->rx_buf, FRAME_LEN);
    buf->rx_len  = 0;
    buf->rx_flag = 0;

    return (answer[8] == _checksum(answer)) ? ERR_OK : ERR_CRC;
}

uint8_t MHZCO2::_checksum(const uint8_t arr[FRAME_LEN])
{
    uint8_t sum = 0;
    for (uint8_t i = 1; i < 8; i++)
    {
        sum += arr[i];
    }
    return (uint8_t)(0xFF - sum) + 1;
}
