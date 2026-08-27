#include "device_bh1750.hpp"
#include "systick.h"

// ============================================================
//  构造
// ============================================================

BH1750FVI::BH1750FVI(inter_i2c_bus* bus, uint8_t addr)
    : _bus(bus)
    , _dev(bus, addr)
    , _address(addr)
{
}

BH1750FVI::~BH1750FVI()
{
}

// ============================================================
//  init：复位内部状态 + 连接检查（同参考库 begin()）
// ============================================================

void BH1750FVI::init()
{
    _data              = 0;
    _error             = ERR_OK;
    _sensitivityFactor = REFERENCE_TIME;
    _mode              = MODE_HIGH;
    (void)isConnected();
}

// ============================================================
//  连接 / 错误
// ============================================================

bool BH1750FVI::isConnected()
{
    return _dev.ping();
}

int BH1750FVI::getLastError()
{
    int e = _error;
    _error = ERR_OK;
    return e;
}

// ============================================================
//  I2C 原语
//
//  BH1750 命令为单字节且无寄存器指针。若用 freedom_write(cmd, 0x00, 1)，
//  尾随的 0x00 会被芯片当作 powerOn 命令，使 powerOff()/reset() 失效；
//  读数同理（参考库 requestFrom 不写任何字节）。
//  因此命令与读数直接用 inter_i2c_bus 原语实现，与参考库 Wire 流程一致。
// ============================================================

bool BH1750FVI::command(uint8_t value)
{
    bool ok = false;

    _bus->lock();
    _bus->start();
    _bus->write_byte(_address << 1);
    if (_bus->wait_ack())
    {
        _bus->write_byte(value);
        if (_bus->wait_ack()) ok = true;
    }
    _bus->stop();
    _bus->unlock();

    _error = ok ? ERR_OK : ERR_WIRE_REQUEST;
    return ok;
}

uint16_t BH1750FVI::readData()
{
    uint16_t data = _data;   // 失败返回上次数据（同参考库）
    bool ok = false;

    _bus->lock();
    _bus->start();
    _bus->write_byte((_address << 1) | 0x01);
    if (_bus->wait_ack())
    {
        uint8_t msb = _bus->read_byte();
        _bus->write_ack(0);   // ACK

        uint8_t lsb = _bus->read_byte();
        _bus->write_ack(1);   // NACK

        data = static_cast<uint16_t>((msb << 8) | lsb);
        ok   = true;
    }
    _bus->stop();
    _bus->unlock();

    if (ok)
    {
        _data  = data;
        _error = ERR_OK;
    }
    else
    {
        _error = ERR_WIRE_REQUEST;
    }
    return data;
}

// ============================================================
//  定点换算
//
//  BH1750 原始值 = 实际照度 × 1.2（MT=69 时）
//  → lx = raw / 1.2 = raw × 5/6
//  → lx×100 = raw × 500/6 = raw × 250/3
// ============================================================

int32_t BH1750FVI::rawToLux_x100(uint16_t raw)
{
    return static_cast<int32_t>((static_cast<int64_t>(raw) * 250) / 3);
}

int32_t BH1750FVI::getRawLux_x100()
{
    return rawToLux_x100(readData());
}

// ============================================================
//  修正照度（同参考库 getLux() 计算顺序，全部乘法可交换）
//
//  ① raw/1.2 基础照度
//  ② 灵敏度因子：×69/MT（changeTiming 修改 MT 后生效）
//  ③ 温度补偿：×(1 − (T−20)×0.0005)
//  ④ 波长补偿：×1/tmp（tmp 为分线段性近似的相对灵敏度）
//  ⑤ HIGH2 模式输出为实际照度 2 倍：÷2
// ============================================================

int32_t BH1750FVI::getLux_x100()
{
    int64_t lux = rawToLux_x100(readData());

    // ② 灵敏度因子（合并系数：250×69/3 = 5750）
    lux = (lux * REFERENCE_TIME) / _sensitivityFactor;

    // ③ 温度补偿（默认 20°C 不修正）
    if (_temperature != 20)
    {
        lux = (lux * tempFactor_x10000(_temperature)) / 10000;
    }

    // ④ 波长补偿（默认 580 nm 不修正）
    if (_waveLength != 580)
    {
        lux = (lux * waveLengthFactor_x10000(_waveLength)) / 10000;
    }

    // ⑤ HIGH2 模式 ÷2
    if (_mode == MODE_HIGH2)
    {
        lux /= 2;
    }

    return static_cast<int32_t>(lux);
}

// ============================================================
//  电源 / 复位
// ============================================================

void BH1750FVI::powerOn()  { (void)command(CMD_POWER_ON); }
void BH1750FVI::powerOff() { (void)command(CMD_POWER_OFF); }
void BH1750FVI::reset()    { (void)command(CMD_RESET); }

// ============================================================
//  模式设置
// ============================================================

void BH1750FVI::setContHighRes()  { _mode = MODE_HIGH;  (void)command(CMD_CONT_HIGH);  _requestTick = get_tick(); }
void BH1750FVI::setContHigh2Res() { _mode = MODE_HIGH2; (void)command(CMD_CONT_HIGH2); _requestTick = get_tick(); }
void BH1750FVI::setContLowRes()   { _mode = MODE_LOW;   (void)command(CMD_CONT_LOW);   _requestTick = get_tick(); }
void BH1750FVI::setOnceHighRes()  { _mode = MODE_HIGH;  (void)command(CMD_ONCE_HIGH);  _requestTick = get_tick(); }
void BH1750FVI::setOnceHigh2Res() { _mode = MODE_HIGH2; (void)command(CMD_ONCE_HIGH2); _requestTick = get_tick(); }
void BH1750FVI::setOnceLowRes()   { _mode = MODE_LOW;   (void)command(CMD_ONCE_LOW);   _requestTick = get_tick(); }

// ============================================================
//  测量完成判断（数据手册 P2/P11 最大测量时间 × MT/69）
// ============================================================

bool BH1750FVI::isReady()
{
    static const uint8_t timeout[3] = { 16, 120, 120 };   // LOW / HIGH / HIGH2
    uint32_t t = (static_cast<uint32_t>(timeout[_mode]) * _sensitivityFactor) / REFERENCE_TIME;
    return (get_tick() - _requestTick) > t;
}

// ============================================================
//  测量时间 / 灵敏度
// ============================================================

void BH1750FVI::changeTiming(uint8_t time)
{
    if (time < 31)  time = 31;
    if (time > 254) time = 254;
    _sensitivityFactor = time;

    // 数据手册 P5 指令表：先写 MT 高 3 位，再写 MT 低 5 位
    (void)command(static_cast<uint8_t>(0x40 | (time >> 5)));
    (void)command(static_cast<uint8_t>(0x60 | (time & 0x1F)));
}

int32_t BH1750FVI::getCorrectionFactor_x10000() const
{
    // f = MT / 69 → ×10000
    return static_cast<int32_t>((static_cast<int64_t>(_sensitivityFactor) * 10000) / REFERENCE_TIME);
}

// ============================================================
//  温度补偿（参考库: 1 − (temp−20)×0.0005）
//  效果约 3%/60°C ≈ 1%/20°C（数据手册 P3 图 7）
// ============================================================

void BH1750FVI::setTemperature(int temp)
{
    _temperature = temp;
}

int32_t BH1750FVI::tempFactor_x10000(int temp)
{
    return 10000 - (temp - 20) * 5;
}

// ============================================================
//  波长补偿（参考库 setWaveLength 分线段性近似，×10000 定点）
//
//  tmp 为相对光谱灵敏度（580 nm 时 = 1.0），补偿因子 = 1/tmp：
//    <440:  0.01 + (wl−400)×0.09/40
//    <510:  0.10 + (wl−440)×0.80/70
//    <545:  0.90 − (wl−510)×0.10/35
//    <580:  0.80 + (wl−545)×0.20/35
//    <700:  1.00 − (wl−580)×0.93/120
//    <715:  0.07 − (wl−700)×0.07/15
//    ≥715:  0.01
// ============================================================

void BH1750FVI::setWaveLength(int waveLength)
{
    if (waveLength < 400) waveLength = 400;
    if (waveLength > 715) waveLength = 715;
    _waveLength = waveLength;
}

int32_t BH1750FVI::waveLengthFactor_x10000(int wl)
{
    int32_t tmp10000;
    if      (wl < 440) tmp10000 = 100 + (wl - 400) * 45 / 2;        // 斜率 22.5/10000/nm
    else if (wl < 510) tmp10000 = 1000 + (wl - 440) * 800 / 7;      // 斜率 ≈114.29
    else if (wl < 545) tmp10000 = 9000 - (wl - 510) * 1000 / 35;    // 斜率 ≈−28.57
    else if (wl < 580) tmp10000 = 8000 + (wl - 545) * 2000 / 35;    // 斜率 ≈57.14
    else if (wl < 700) tmp10000 = 10000 - (wl - 580) * 155 / 2;     // 斜率 −77.5
    else if (wl < 715) tmp10000 = 700 - (wl - 700) * 140 / 3;       // 斜率 ≈−46.67
    else               tmp10000 = 100;                              // 0.01

    if (tmp10000 < 1) tmp10000 = 1;   // 防除零（wl=714 时 tmp≈0.0046）
    return static_cast<int32_t>(100000000LL / tmp10000);   // 1/tmp ×10000
}

//  -- END OF FILE --
