#pragma once

#ifdef __cplusplus

#include "inter_usart.hpp"
#include <cstdint>

// ============================================================
//  MH-Z19 系列 CO2 传感器 UART 驱动
// ============================================================
//  来源：Rob Tillaart 的 Arduino 库 "MHZCO2"
//  版本：0.2.4（2026-04-22）
//    URL：https://github.com/RobTillaart/MHZCO2
//
//  协议（MH-Z19 全系通用，9 字节帧）：
//    命令帧：0xFF 0x01 CMD d3 d4 d5 d6 d7 CHK
//    响应帧：0xFF 0x01 CMD CO2_HI CO2_LO T ACC MIN_HI MIN_LO CHK
//      CO2 = byte2<<8 | byte3（ppm）
//      T   = byte4 - 40（°C，偏移 40）
//      ACC = byte5（精度等级）
//      MIN = byte6<<8 | byte7（最低 CO2 记录）
//    校验和 CHK = 0xFF - sum(byte1..byte7) + 1（命令与响应同规则）
//
//  移植说明（对照参考库 0.2.4）：
//    - 串口：参考库注入 Stream*（硬件/软件串口皆可）；本项目绑定
//      usart_port&（RX 为中断缓冲 + send_data 阻塞发送）
//    - 读取方式：参考库逐字节 available()/read()；本驱动改为
//      "发送命令 + 轮询 buffer()->rx_len >= 9 一次取走"，消费方式与
//      项目 Modbus_FillData 一致；超时/取走响应后清 rx_len/rx_flag，
//      防止残留字节污染下一次响应
//    - 无浮点：温度以 m°C 返回（参考库返回 °C 整数）
//    - 时间源：get_tick()（替代参考库 millis()）
//    - 裁剪：uptime()、参考库 setPPM()/getPPM()（量程命令 0x99）
//      更名为 setRange()/getRange()；派生类 MHZ1311A/MHZ19/
//      MHZ19B~E（无协议差异，仅标识型号）去掉
//    - 无堆分配、无异常
// ============================================================

class MHZCO2
{
public:

    // ── 错误码（与参考库一致）─────────────────────────────

    enum ErrCode : int8_t
    {
        ERR_OK      = 0,     // 成功
        ERR_TIMEOUT = -10,   // 等待响应超时
        ERR_CRC     = -11,   // 校验和不匹配
    };

    // ── 命令字 ────────────────────────────────────────────

    enum Cmd : uint8_t
    {
        CMD_MEASURE   = 0x86,   // 读 CO2 浓度
        CMD_CAL_ZERO  = 0x87,   // 零点校准
        CMD_CAL_SPAN  = 0x88,   // 跨度校准
        CMD_ABC_MODE  = 0x79,   // ABC 自动校准开关
        CMD_SET_RANGE = 0x99,   // 量程设置（2000/5000/10000 ppm）
    };

    static constexpr uint8_t FRAME_LEN = 9;   // 命令/响应帧长

    /** @brief 绑定串口（如 rs485_uart / serial_uart1）；串口由外部 init() */
    explicit MHZCO2(usart_port& uart);

    /** @brief 复位内部管理数据（不操作传感器） */
    void reset();

    /**
     * @brief 发送 0x86 读命令并等待 9 字节响应
     * @return ERR_OK / ERR_TIMEOUT / ERR_CRC
     * @note  ERR_CRC 时结果字段也已按参考库更新（由调用方判断是否可用）
     */
    int measure();

    // ── 测量结果（measure() 成功后有效）───────────────────

    uint16_t getCO2()            const { return _co2; }        // CO2 浓度 ppm
    int32_t  getTemperature_mC() const { return _temp_mC; }    // 温度 m°C（可负）
    uint8_t  getAccuracy()       const { return _accuracy; }   // 精度等级
    uint16_t getMinCO2()         const { return _minCO2; }     // 最低 CO2 记录
    uint16_t getMaxCO2()         const { return _maxCO2; }     // 自 reset() 以来最高 CO2

    /** @brief 最近一次成功测量时刻（get_tick，ms） */
    uint32_t lastMeasurement()   const { return _lastMeasurement; }

    // ── 校准（⚠️ 慎用，请先读数据手册）────────────────────

    void calibrateZero();                  // 0x87 零点校准
    void calibrateSpan(uint16_t span);     // 0x88 跨度校准（span ppm）
    void calibrateAuto(bool mode = true);  // 0x79 ABC 自动校准

    // ── 量程（0x99；仅部分型号支持，其它值行为未知）────────

    void     setRange(uint16_t ppm);
    uint16_t getRange() const { return _range; }

    // ── 超时 ──────────────────────────────────────────────

    /** @brief 响应超时 ms；0 = 不检测（可能长时间阻塞） */
    void     setTimeOut(uint16_t timeout = 1000);
    uint16_t getTimeOut() const { return _timeout; }

private:

    usart_port& _uart;

    uint16_t _timeout;       // 响应超时 ms
    uint16_t _range;         // 量程设置（0 = 未设置）
    uint16_t _co2;           // ppm
    int32_t  _temp_mC;       // m°C
    uint8_t  _accuracy;      // 精度等级
    uint16_t _minCO2;        // 最低 CO2
    uint16_t _maxCO2;        // 最高 CO2
    uint32_t _lastMeasurement; // 最近成功测量时刻

    /** @brief 组帧并阻塞发送：{0xFF,0x01,cmd,d3,d4,0x00,d6,d7,CHK} */
    void _send9(uint8_t cmd, uint8_t d3, uint8_t d4, uint8_t d6, uint8_t d7);

    /** @brief 轮询 RX 缓冲取 9 字节，校验和校验；超时/取走后清缓冲 */
    int _receive(uint8_t answer[FRAME_LEN]);

    static uint8_t _checksum(const uint8_t arr[FRAME_LEN]);
};

#endif
