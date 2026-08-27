#pragma once
//
//    FILE: device_dhtnew.hpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.5.5（参考库 DHTNEW）
//    DATE: 2023-12-30
// PURPOSE: DHT11 / DHT22（AM2302 兼容）温湿度传感器（单总线）
//     URL: https://github.com/RobTillaart/DHTNEW
//
//  移植说明（Arduino → STM32 项目风格）：
//   - 仅移植 DHT11 / DHT22 及自动识别（type 0 = 未知 / 11 / 22；
//     23 映射为 22，参考库同款），Si7021（type 70）未移植
//   - 禁止浮点，定点换算：
//       湿度：×10（0.1%RH，int32_t）
//             DHT22 = 16 位原始值直接返回（×10 定点）；
//             DHT11 = bits[0]×10 + bits[1]（0.1%RH）
//       温度：m°C（×1000，int32_t）
//             DHT22 = 16 位原始值 × 100（含符号-幅值 / 补码两种负温表示）；
//             DHT11 = bits[2]×1000 + bits[3]×100
//   - 参考库用 millis() 限制最小读取间隔（DHT11 ≥ 1s，DHT22 ≥ 2s）并记录
//     lastRead；本工程无毫秒时钟，read() 不做限速，调用方必须自行保证
//     读取间隔，否则传感器无响应
//   - 参考库首次读前有 while(millis()<1000) 上电等待，本移植移除，
//     上电后需给传感器 1~2s 稳定时间（调用方保证）
//   - 时序（µs，参考库原样移植）：唤醒拉低 DHT11 = 19800（18ms×1.1）/
//     DHT22 = 1100（1ms×1.1）；响应超时 DHT22 = 50 / DHT11 = 15000；
//     位超时 A/B/C/D = 90；0/1 判据阈值 = 50（26~28µs = 0，70µs = 1）
//   - 本移植无 micros()，超时与位宽测量用 delay_us(1) 轮询计数实现，
//     计数与 µs 的比例取决于 delay_us(1) 的实测精度
//     ★ 真机需按实测调参 ★：若 delay_us(1) 实际时长偏离 1µs，
//     需等比修正 DHTLIB_BIT_THRESHOLD 与各超时值
//   - 采样 40 位期间关中断（参考库 noInterrupts() 同款，本移植用
//     __disable_irq()，可通过 setDisableIRQ(false) 关闭）；
//     响应等待期保持中断开启（参考库在整个读取期间关中断，本移植
//     缩短关中断窗口，避免 DHT11 最长 15ms 的关中断时间）

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include <cstdint>

// ── read() 返回值（与参考库数值一致）────────────────────
enum DHTErr : int8_t
{
    DHTLIB_OK                     = 0,
    DHTLIB_ERROR_CHECKSUM         = -1,   // 校验和错误
    DHTLIB_ERROR_TIMEOUT_A        = -2,   // 响应低电平超时
    DHTLIB_ERROR_BIT_SHIFT        = -3,   // 湿度最高位错误（位偏移）
    DHTLIB_ERROR_SENSOR_NOT_READY = -4,   // 传感器未就绪
    DHTLIB_ERROR_TIMEOUT_C        = -5,   // 位起始低电平超时
    DHTLIB_ERROR_TIMEOUT_D        = -6,   // 位高电平结束超时
    DHTLIB_ERROR_TIMEOUT_B        = -7,   // 响应高电平超时
};

// ── 出错时写入测量值的无效标记（按各自刻度）────────────
constexpr int32_t DHTLIB_INVALID_HUMIDITY    = -9990;    // -999.0%（×10）
constexpr int32_t DHTLIB_INVALID_TEMPERATURE = -999000;  // -999.0°C（m°C）

// ── 0/1 判据阈值（µs，数据手册 26~28µs=0 / 70µs=1）──────
//  ★ 真机需按实测调参 ★：本移植用 delay_us(1) 轮询计数测量位宽，
//  若 delay_us(1) 实测时长 ≠ 1µs，需等比修正此值
constexpr uint8_t DHTLIB_BIT_THRESHOLD = 50;

class DeviceDHTNEW
{
public:
    /**
     * @brief 构造并初始化数据引脚（推挽输出，总线空闲高电平）
     * @param data_pin DHT 数据引脚（io_ctrl 引用）
     */
    explicit DeviceDHTNEW(io_ctrl &data_pin);

    DeviceDHTNEW(const DeviceDHTNEW &)            = delete;
    DeviceDHTNEW &operator=(const DeviceDHTNEW &) = delete;

    // ── 类型 ────────────────────────────────────────────

    /** @brief 复位所有内部状态（参考库 reset()） */
    void reset();

    /**
     * @brief 传感器类型：0 = 未知（自动识别）/ 11 = DHT11 / 22 = DHT22
     * @note  类型未知时先执行一次 read() 自动识别（参考库行为）
     */
    uint8_t getType();
    void    setType(uint8_t type = 0);

    // ── 核心读取 ────────────────────────────────────────

    /**
     * @brief 启动一次读取，返回 DHTErr 错误码
     * @note  类型未知时按 DHT22 → DHT11 顺序自动识别重试
     *        调用方须保证两次读取间隔（DHT11 ≥ 1s，DHT22 ≥ 2s）
     */
    int8_t read();

    /** @brief 湿度，×10（0.1%RH）；出错时返回 DHTLIB_INVALID_HUMIDITY */
    int32_t getHumidityX10() { return _humidity10; }

    /** @brief 温度，m°C（×1000）；出错时返回 DHTLIB_INVALID_TEMPERATURE */
    int32_t getTemperaturemC() { return _temperaturemC; }

    // ── 偏移（定点）─────────────────────────────────────

    /** @brief 湿度偏移，单位 0.1%RH（加偏移后限幅 0..100.0%） */
    void    setHumidityOffset(int32_t offset) { _humidityOffset = offset; }
    int32_t getHumidityOffset() { return _humidityOffset; }

    /** @brief 温度偏移，单位 m°C（加 -273150 即开尔文） */
    void    setTemperatureOffset(int32_t offset) { _temperatureOffset = offset; }
    int32_t getTemperatureOffset() { return _temperatureOffset; }

    // ── 中断 / 错误处理 ─────────────────────────────────

    /** @brief 采样期间是否关中断（默认 true；关中断窗口约 4ms） */
    bool getDisableIRQ() { return _disableIRQ; }
    void setDisableIRQ(bool b) { _disableIRQ = b; }

    /** @brief 出错时是否保留上次有效测量值（默认 false = 写入无效值） */
    bool getSuppressError() { return _suppressError; }
    void setSuppressError(bool b) { _suppressError = b; }

    // ── 低功耗 ──────────────────────────────────────────

    /** @brief 上电（数据脚拉高 + 空读一次同步，参考库行为） */
    void powerUp();

    /** @brief 掉电（数据脚拉低；参考库低功耗用法，仅适用于数据线供电接法） */
    void powerDown();

private:
    int8_t _read(void);          // 读 40 位 + 定点换算 + 校验
    int8_t _readSensor(void);    // 唤醒 + 响应检测 + 40 位采样
    bool   _waitFor(uint8_t state, uint32_t timeout);   // true = 超时
    void   _pinOut(void);        // reinit 推挽输出
    void   _pinIn(void);         // reinit 输入 + 上拉

    io_ctrl &_pin;
    uint8_t  _type;              // 0 / 11 / 22
    uint32_t _wakeupDelay;       // µs
    int32_t  _humidity10;        // 湿度 ×10（0.1%RH）
    int32_t  _temperaturemC;     // 温度 m°C
    int32_t  _humidityOffset;    // 0.1%RH
    int32_t  _temperatureOffset; // m°C
    bool     _disableIRQ;
    bool     _suppressError;
    uint8_t  _bits[5];           // 40 位接收缓冲
};

#endif /* __cplusplus */
