#pragma once
//
//    FILE: device_hx711.hpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.6.4（参考库 HX711）
//    DATE: 2019-09-04
// PURPOSE: HX711 24 位 ADC 称重传感器（DOUT / PD_SCK 两线）
//     URL: https://github.com/RobTillaart/HX711
//
//  移植说明（Arduino → STM32 项目风格）：
//   - 参考库 API 超集（median / medavg / runavg 模式、scale 标定、
//     rate 引脚、价格等）未移植，仅保留核心子集：
//     read / read_average / tare / get_value / get_tare /
//     set_gain / power_down / power_up / wait_ready 系列
//   - 引脚由 uint8_t 改为 io_ctrl& 引用；DOUT 输入（上拉，参考库
//     INPUT_PULLUP 同款），PD_SCK 推挽输出，空闲低电平
//   - 禁止浮点：read() 直接返回 24 位有符号原始值（int32_t），
//     不做 scale 换算（参考库用 float 表示，本移植无 scale 概念）
//   - 时序（µs，数据手册 + 参考库 fastProcessor 分支）：
//     PD_SCK 高/低电平各 ≥ 1µs（T2 ≥ 0.2µs，快速处理器加 1µs 延时），
//     掉电 PD_SCK 高电平 ≥ 60µs（移植取 64µs）
//   - PD_SCK 脉冲数决定通道/增益：25 = CH_A ×128（默认）、
//     26 = CH_B ×32、27 = CH_A ×64（数据手册 Table 3）
//   - 首次 read() 最长阻塞 ~400ms（器件上电/通道切换后，数据手册）；
//     阻塞等待期中断保持开启，仅 24 位移位期间关中断
//   - 参考库 wait_ready_timeout() 依赖 millis()，未移植（无毫秒时钟），
//     可用 wait_ready_retry() 代替
//   - 器件工作电压 2.6~5.5V，注意与 MCU 电平匹配（G070 为 3.3V，
//     5V 供电的 HX711 模块需要电平转换或确认模块已做处理）

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include <cstdint>

class DeviceHX711
{
public:
    // ── 通道 / 增益（PD_SCK 脉冲数：25 / 26 / 27）────────

    static constexpr uint8_t CHANNEL_A_GAIN_128 = 128;   // 默认，25 脉冲
    static constexpr uint8_t CHANNEL_A_GAIN_64  = 64;    // 27 脉冲
    static constexpr uint8_t CHANNEL_B_GAIN_32  = 32;    // 26 脉冲

    /**
     * @brief 构造并初始化引脚
     * @param dataPin  DOUT 数据脚（io_ctrl 引用，输入 + 上拉）
     * @param clockPin PD_SCK 时钟脚（io_ctrl 引用，推挽输出，空闲低）
     */
    explicit DeviceHX711(io_ctrl &dataPin, io_ctrl &clockPin);

    DeviceHX711(const DeviceHX711 &)            = delete;
    DeviceHX711 &operator=(const DeviceHX711 &) = delete;

    // ── 初始化 / 复位 ───────────────────────────────────

    /** @brief doReset = true 时执行 reset()（参考库 begin 语义） */
    void begin(bool doReset = true);

    /** @brief 掉电再上电 + 复位内部状态 + 强制一次读（参考库 reset） */
    void reset();

    // ── 就绪查询 ────────────────────────────────────────

    /** @brief 数据是否就绪（DOUT 为低）；就绪前 read() 会阻塞等待 */
    bool isReady();

    /** @brief 阻塞等待就绪，每 ms 毫秒轮询一次 */
    void waitReady(uint32_t ms = 0);

    /** @brief 最多重试 retries 次，每次间隔 ms 毫秒 */
    bool waitReadyRetry(uint8_t retries = 3, uint32_t ms = 0);

    // ── 读取 ────────────────────────────────────────────

    /**
     * @brief 读取 24 位原始值（有符号 int32_t，符号扩展）
     * @note  阻塞直到 DOUT 拉低；首次读取最长 ~400ms；
     *        读取末尾自动补足第 25/26/27 个脉冲（按 _gain 选择下一轮通道）
     */
    int32_t read();

    /** @brief 多次读取取平均（times < 1 时按 1 处理） */
    int32_t readAverage(uint8_t times = 10);

    /** @brief 平均读数 - 零偏（去皮后重量原始值） */
    int32_t getValue(uint8_t times = 1);

    // ── 去皮（tare）────────────────────────────────────

    /** @brief 以当前载荷平均值为零偏（去皮） */
    void tare(uint8_t times = 10);

    /** @brief 当前零偏的相反数（scale = 1 时即去皮重量） */
    int32_t getTare();

    /** @brief 是否已去皮（零偏 ≠ 0） */
    bool tareSet();

    // ── 增益 / 通道 ─────────────────────────────────────

    /**
     * @brief 设置增益/通道；非法值返回 false
     * @param forced true 时即使已相同也强制重设
     * @note  切换后新通道从下一次读开始生效（器件最长 400ms 生效）
     */
    bool setGain(uint8_t gain = CHANNEL_A_GAIN_128, bool forced = false);
    uint8_t getGain();

    // ── 零偏 ────────────────────────────────────────────

    void    setOffset(int32_t offset = 0);
    int32_t getOffset();

    // ── 电源管理 ────────────────────────────────────────

    /** @brief 掉电：PD_SCK 拉高 ≥ 60µs */
    void powerDown();

    /** @brief 上电：PD_SCK 拉低 */
    void powerUp();

private:
    uint8_t _shiftIn(void);   // MSB 先入，8 位（参考库 fastProcessor 版）

    io_ctrl &_data;
    io_ctrl &_clock;
    int32_t  _offset;         // 零偏（原始计数）
    uint8_t  _gain;           // 128 / 64 / 32
};

#endif /* __cplusplus */
