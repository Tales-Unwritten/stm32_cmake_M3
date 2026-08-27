#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
// SHT31 温湿度传感器（I2C）
//
// 来源库 : SHT31 v0.5.3（Rob Tillaart，2019-02-08）
// URL    : https://github.com/RobTillaart/SHT31
//
// 移植说明:
//  - SHT3x 无寄存器地址，全部为 16 位命令字（MSB 先发）
//    → inter_i2c_dev::freedom_write(高字节, 低字节, 1) 发命令
//    → inter_i2c_dev::freedom_read(0x00, ...) 读数据：
//      0x00 为不完整命令，SHT3x 忽略后直接返回测量数据
//      （参考库 requestFrom 读数据前同样不写任何字节）
//  - 禁止 float/double：温度 m°C（×1000）、湿度 %RH×100 定点
//  - CRC8 校验保留（poly 0x31，参考库原样移植）；fast 模式跳过校验
//  - 加热器定时 / 冷却保护 / 异步接口用 HAL_GetTick() 计时
//  - 无动态内存 / STL / 异常，C++17，Cortex-M0+
// ============================================================

class SHT31
{
public:

    // ── I2C 地址 ──

    enum Addr : uint8_t
    {
        ADDR_0x44 = 0x44,
        ADDR_0x45 = 0x45,
    };

    // ── 命令字（16 位，无寄存器地址）──

    enum Cmd : uint16_t
    {
        CMD_READ_STATUS   = 0xF32D,   // 读状态寄存器（16bit + CRC）
        CMD_CLEAR_STATUS  = 0x3041,   // 清状态位
        CMD_SOFT_RESET    = 0x30A2,   // 软复位
        CMD_HARD_RESET    = 0x0006,   // 硬复位（拉低供电）
        CMD_MEASURE_FAST  = 0x2416,   // 单次测量（时钟延展），约 4 ms 可读
        CMD_MEASURE_SLOW  = 0x2400,   // 单次测量（无延展），约 15 ms 可读
        CMD_HEAT_ON       = 0x306D,   // 加热器开
        CMD_HEAT_OFF      = 0x3066,   // 加热器关
        CMD_GET_SERIAL    = 0x3682,   // 读序列号（32bit + 2×CRC）
    };

    // ── 状态寄存器位 ──

    static constexpr uint16_t STATUS_ALERT_PENDING    = 1 << 15;   // 报警挂起
    static constexpr uint16_t STATUS_HEATER_ON        = 1 << 13;   // 加热器状态
    static constexpr uint16_t STATUS_HUM_TRACK_ALERT  = 1 << 11;   // 湿度跟踪报警
    static constexpr uint16_t STATUS_TEMP_TRACK_ALERT = 1 << 10;   // 温度跟踪报警
    static constexpr uint16_t STATUS_SYSTEM_RESET     = 1 << 4;    // 复位检测
    static constexpr uint16_t STATUS_COMMAND_STATUS   = 1 << 1;    // 命令状态
    static constexpr uint16_t STATUS_WRITE_CRC_STATUS = 1 << 0;    // 写 CRC 状态

    // ── 错误码（同参考库）──

    enum ErrCode : uint8_t
    {
        ERR_OK              = 0x00,
        ERR_WRITECMD        = 0x81,   // 命令发送失败
        ERR_READBYTES       = 0x82,   // 数据读取失败
        ERR_HEATER_OFF      = 0x83,   // 关闭加热器失败
        ERR_NOT_CONNECT     = 0x84,   // 设备无应答
        ERR_CRC_TEMP        = 0x85,   // 温度 CRC 错误
        ERR_CRC_HUM         = 0x86,   // 湿度 CRC 错误
        ERR_CRC_STATUS      = 0x87,   // 状态 CRC 错误
        ERR_HEATER_COOLDOWN = 0x88,   // 加热器冷却中（3 分钟）
        ERR_HEATER_ON       = 0x89,   // 打开加热器失败
        ERR_SERIAL_CRC      = 0x8A,   // 序列号 CRC 错误
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit SHT31(inter_i2c_bus* bus, uint8_t addr = ADDR_0x44);

    SHT31(const SHT31&) = delete;
    SHT31& operator=(const SHT31&) = delete;

    ~SHT31();

    /** @brief 初始化：地址校验（0x44/0x45）+ 软复位 */
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
     * @brief 阻塞式测量（发起命令 + 延时 + 读数据）
     * @param fast true  = CMD_MEASURE_FAST（约 4 ms，跳过 CRC）
     *             false = CMD_MEASURE_SLOW（约 15 ms，含 CRC 校验）
     */
    bool read(bool fast = true);

    // ── 异步接口 ──
    bool requestData();      // 发起慢速测量
    bool dataReady();        // 距 requestData ≥ 15 ms
    bool readData(bool fast = true);

    // ============================================================
    //  状态 / 复位
    // ============================================================

    /** @brief 读状态寄存器（16 bit + CRC），失败返回 0xFFFF */
    uint16_t readStatus();
    bool clearStatus();      // 清报警/复位标志位
    bool reset(bool hard = false);   // 软复位 0x30A2 / 硬复位 0x0006

    // ============================================================
    //  加热器（连续加热 ≤ 3 分钟，冷却 ≥ 3 分钟，参考库限制）
    // ============================================================

    void setHeatTimeout(uint8_t seconds);   // 加热时长上限（>180 按 180）
    uint8_t getHeatTimeout() const { return _heatTimeout; }
    bool heatOn();
    bool heatOff();
    /** @brief 加热是否仍在进行；超时自动关闭 */
    bool isHeaterOn();

    // ============================================================
    //  数据（定点换算）
    // ============================================================

    /** @brief 温度 m°C ×1000：raw×175000/65535 − 45000 */
    int32_t getTemperature_mC() const;
    /** @brief 湿度 %RH×100：raw×10000/65535 */
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
     * @param fast true = 跳过 CRC 校验
     */
    bool getSerialNumber(uint32_t& serial, bool fast = true);

private:

    inter_i2c_dev _dev;
    uint8_t  _address;
    uint8_t  _heatTimeout     = 0;
    uint32_t _lastReadTick    = 0;
    uint32_t _lastRequestTick = 0;
    uint32_t _heaterStartTick = 0;
    uint32_t _heaterStopTick  = 0;
    bool     _heaterOn        = false;
    uint16_t _rawHumidity     = 0;
    uint16_t _rawTemperature  = 0;
    uint8_t  _error = ERR_OK;

    // ── I2C 原语 ──

    bool writeCmd(uint16_t cmd);          // 16 位命令 = freedom_write(hi, lo, 1)
    bool readBytes(uint8_t n, uint8_t* buf);   // freedom_read(0x00, ...) 占位

    // ── CRC8（poly 0x31, init 0xFF）──

    static uint8_t crc8(const uint8_t* data, uint8_t len);
};

#endif
