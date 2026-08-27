#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// SHT4x 温湿度传感器（I2C，SHT40/SHT41/SHT43/SHT45）
//
// 来源库 : SHT4x v0.1.2（Samuel Cuerrier Auclair / Rob Tillaart，
//         2025-08-11，SHT31 库的 SHT4x 分支）
// URL    : https://github.com/RobTillaart/SHT4x
//
// 移植说明:
//  - SHT4x 无寄存器地址，命令为单字节（无 16 位命令）
//    → inter_i2c_dev::freedom_write(cmd, 0x00, 1) 发命令：尾随的 0x00
//      为未定义命令，SHT4x 忽略（数据手册）；freedom_write 不检查数据
//      字节 ACK，即使被 NACK 也无害
//    → inter_i2c_dev::freedom_read(0x00, ...) 读数据（同 SHT31 移植）
//  - 禁止 float/double：温度 m°C（×1000）、湿度 %RH×100 定点
//  - 参考库的 SHT40/SHT41/SHT43/SHT45 派生类仅为构造地址糖，未移植
//  - 加热保护（默认开启）：加热测量命令之间有冷却间隔限制
//  - 无动态内存 / STL / 异常，C++17，Cortex-M0+
// ============================================================

class SHT4x
{
public:

    // ── I2C 地址 ──

    enum Addr : uint8_t
    {
        ADDR_0x44 = 0x44,   // SHT40/41/43/45
        ADDR_0x45 = 0x45,   // SHT40/43
        ADDR_0x46 = 0x46,   // SHT40
    };

    // ── 测量命令（单字节）──

    enum MeasType : uint8_t
    {
        MEAS_SLOW        = 0xFD,   // 高精度，typ 8.2 ms
        MEAS_MEDIUM      = 0xF6,   // 中精度，typ 4.5 ms
        MEAS_FAST        = 0xE0,   // 低精度，typ 1.7 ms
        MEAS_LONG_HIGH   = 0x39,   // 加热 1 s 高功率 + 测量
        MEAS_SHORT_HIGH  = 0x32,   // 加热 0.1 s 高功率 + 测量
        MEAS_LONG_MED    = 0x2F,   // 加热 1 s 中功率 + 测量
        MEAS_SHORT_MED   = 0x24,   // 加热 0.1 s 中功率 + 测量
        MEAS_LONG_LOW    = 0x1E,   // 加热 1 s 低功率 + 测量
        MEAS_SHORT_LOW   = 0x15,   // 加热 0.1 s 低功率 + 测量
    };

    // ── 其它命令 ──

    enum Cmd : uint8_t
    {
        CMD_SOFT_RESET = 0x94,   // 软复位（约 1 ms）
        CMD_GET_SERIAL = 0x89,   // 读序列号（32bit + 2×CRC）
    };

    // ── 错误码（同参考库）──

    enum ErrCode : uint8_t
    {
        ERR_OK              = 0x00,
        ERR_WRITECMD        = 0x81,   // 命令发送失败
        ERR_READBYTES       = 0x82,   // 数据读取失败
        ERR_HEATER_OFF      = 0x83,   // 关闭加热器失败（本移植不产生，保留兼容）
        ERR_NOT_CONNECT     = 0x84,   // 设备无应答
        ERR_CRC_TEMP        = 0x85,   // 温度 CRC 错误
        ERR_CRC_HUM         = 0x86,   // 湿度 CRC 错误
        ERR_CRC_STATUS      = 0x87,   // 状态 CRC 错误（本移植不产生，保留兼容）
        ERR_HEATER_COOLDOWN = 0x88,   // 加热冷却间隔未到
        ERR_HEATER_ON       = 0x89,   // 打开加热器失败（本移植不产生，保留兼容）
        ERR_SERIAL_CRC      = 0x8A,   // 序列号 CRC 错误
        ERR_INVALID_ADDRESS = 0x8B,   // 地址不在 0x44..0x46
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit SHT4x(inter_i2c_bus* bus, uint8_t addr = ADDR_0x44);

    SHT4x(const SHT4x&) = delete;
    SHT4x& operator=(const SHT4x&) = delete;

    ~SHT4x();

    /** @brief 初始化：地址校验（0x44..0x46）+ 软复位 */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    [[nodiscard]] bool isConnected();
    uint8_t getAddress() const { return _address; }
    /** @brief 最近一次操作错误码（0 = 无错误；读取后自动清零） */
    [[nodiscard]] int getLastError();

    // ============================================================
    //  测量
    // ============================================================

    /**
     * @brief 阻塞式测量（发起命令 + 按类型延时 + 读数据）
     * @param type     测量类型（默认高精度；加热命令受冷却保护限制）
     * @param crcCheck 是否 CRC 校验
     */
    bool read(MeasType type = MEAS_SLOW, bool crcCheck = true);

    // ── 异步接口 ──
    bool requestData(MeasType type = MEAS_SLOW);
    bool dataReady();
    bool readData(bool crcCheck = true);

    /** @brief 软复位（fast = true 时不等待复位完成） */
    bool reset(bool fast = false);

    // ============================================================
    //  加热保护（默认开启）
    // ============================================================

    void setHeatProtection(bool enable);   // 关闭后不再限制加热命令频率
    /** @brief 距上次加热测量是否已过冷却间隔 */
    bool heatingReady();

    // ============================================================
    //  数据（定点换算）
    // ============================================================

    /** @brief 温度 m°C ×1000：raw×175000/65535 − 45000 */
    int32_t getTemperature_mC() const;
    /** @brief 湿度 %RH×100：raw×12500/65535 − 600，限幅 0..10000 */
    int32_t getHumidity_x100()  const;
    /** @brief 原始数据（调试 / 高效传输用） */
    uint16_t getRawTemperature() const { return _rawTemperature; }
    uint16_t getRawHumidity()    const { return _rawHumidity; }
    /** @brief 最近一次读取时刻（HAL_GetTick） */
    uint32_t lastRead() const { return _lastReadTick; }

    // ============================================================
    //  其它
    // ============================================================

    /**
     * @brief 读 32 位序列号
     * @param crcCheck false = 跳过 CRC 校验
     */
    bool getSerialNumber(uint32_t& serial, bool crcCheck = true);

private:

    inter_i2c_dev _dev;
    uint8_t  _address;
    uint16_t _delay              = 0;    // 当前测量命令的等待时间 ms
    uint32_t _lastReadTick       = 0;
    uint32_t _lastRequestTick    = 0;
    uint16_t _heatInterval       = 0;    // 加热冷却间隔 ms
    uint32_t _lastHeatRequestTick = 0;
    uint16_t _rawHumidity        = 0;
    uint16_t _rawTemperature     = 0;
    uint8_t  _error = ERR_OK;
    bool     _heatProtection     = true;

    // ── 测量类型参数表 ──

    void setDelay(MeasType type);        // 数据手册表 5
    void setHeatInterval(MeasType type); // 10% 占空比：长加热 10 s / 短加热 1 s
    static bool isHeatCmd(MeasType type);

    // ── I2C 原语 ──

    bool writeCommand(uint8_t cmd);           // freedom_write(cmd, 0x00, 1)
    bool readBytes(uint8_t n, uint8_t* buf);  // freedom_read(0x00, ...) 占位

    // ── CRC8（poly 0x31, init 0xFF）──

    static uint8_t crc8(const uint8_t* data, uint8_t len);
};

#endif
