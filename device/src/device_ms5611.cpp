#include "device_ms5611.hpp"
#include "systick.h"   // get_tick() / delay_ms() / delay_us()

// ============================================================
//  MS5611 高精度气压/温度传感器（SPI）
//  来源：Rob Tillaart "MS5611_SPI" v0.4.3
//    URL：https://github.com/RobTillaart/MS5611_SPI
//  移植说明见 device_ms5611.hpp 头注释
//
//  ────────────────────────────────────────────────────────────
//  定点换算公式推导（对照参考库 read() 的 float 版本，逐行翻译）
//
//  参考库（mathMode=0，数据手册 P7/P8）：
//    dT    = D2 - C[5]                      C[5] = C5×256
//    TEMP  = 2000 + dT×C[6]                 C[6] = C6/2^23，TEMP 单位 0.01°C
//    OFF   = C[2] + dT×C[4]                 C[2] = C2×2^16，C[4] = C4/2^7
//    SENS  = C[1] + dT×C[3]                 C[1] = C1×2^15，C[3] = C3/2^8
//    二阶补偿（TEMP < 2000 时）：
//      T2    = dT²×2^-31
//      t     = (TEMP-2000)²
//      OFF2  = 2.5×t，SENS2 = 1.25×t
//      TEMP < -1500 时：t' = (TEMP+1500)²
//        OFF2 += 7×t'，SENS2 += 5.5×t'
//      TEMP -= T2；OFF -= OFF2；SENS -= SENS2
//    压力：P = (D1×SENS×2^-21 - OFF)×2^-15   [Pa]
//
//  定点化（全部 int64 中间量，系数预缩放，除法用 2 的幂）：
//    dT   = D2 - C5×256          （int64）
//    TEMP = 2000 + dT×C6 / 2^23  （0.01°C）
//    OFF  = C2×2^16 + dT×C4 / 128
//    SENS = C1×2^15 + dT×C3 / 256
//    T2   = dT×dT / 2^31
//    t    = (TEMP-2000)²；OFF2 = 5t/2；SENS2 = 5t/4
//    t'   = (TEMP+1500)²；OFF2 += 7t'；SENS2 += 11t'/2
//    P    = (D1×SENS / 2^21 - OFF) / 2^15    [Pa]
//
//  溢出核算（最坏：PROM = 0xFFFF、ADC = 0xFFFFFF）：
//    dT×C6 ≈ 1.1e12，dT² ≈ 2.8e14，OFF ≈ 1.3e10，SENS ≈ 6.4e9，
//    D1×SENS ≈ 1.1e17（int64 上限 9.2e18）——全部安全；
//    P 结果 ≤ ~1.2e6 Pa，int32 安全。
//
//  截断说明：参考库浮点保留小数位，本版每步整数除法向零截断，
//  单步误差 ≤ 1 LSB（OFF/SENS 尺度），折合最终压力 < 0.1 Pa，
//  可忽略。TEMP 存储为 int32（0.01°C），对外 ×10 = m°C。
//  ────────────────────────────────────────────────────────────
// ============================================================

// ============================================================
//  构造 / 生命周期
// ============================================================

MS5611_SPI::MS5611_SPI(spi_bus &spi)
    : _spi(spi)
    , _samplingRate(OSR_ULTRA_LOW)
    , _temperature_x100(ERR_NOT_READ)
    , _pressure_Pa(ERR_NOT_READ)
    , _pressureOffset_Pa(0)
    , _temperatureOffset_mC(0)
    , _result(ERR_NOT_READ)
    , _prom{0}
    , _c1s(0), _c2s(0), _c3s(0), _c4s(0), _c5s(0), _c6s(0)
    , _lastRead(0)
    , _deviceID(0)
    , _compensation(true)
{
}

bool MS5611_SPI::begin()
{
    return reset();
}

bool MS5611_SPI::isConnected()
{
    return (read() == ERR_OK);
}

bool MS5611_SPI::reset()
{
    command(CMD_RESET);
    delay_ms(3);   // 参考库 micros 循环等 3 ms（首次 PROM 读取曾因等待不足丢失）

    _deviceID = 0;

    // 读 7 个校准字（C[0] 制造商…C[6] TEMPSENS；C[7] CRC 不参与）
    bool romOk = true;
    for (uint8_t reg = 0; reg < 7; reg++)
    {
        uint16_t tmp = readProm(reg);
        _prom[reg] = tmp;
        // 器件 ID：7 个 PROM 寄存器移位异或合并（参考库）
        _deviceID <<= 4;
        _deviceID ^= tmp;
        if (reg > 0)
        {
            romOk = romOk && (tmp != 0);
        }
    }

    initConstants();
    return romOk;
}

// ============================================================
//  测量（定点换算，推导见文件头注释）
// ============================================================

int MS5611_SPI::read(uint8_t bits)
{
    // 参考库：SPI 版没有 endTransmission 清错机制，必须在开头
    // 清掉 NOT_READ，否则首调用的错误检查会卡死后续流程
    _result = ERR_OK;

    convert(CMD_CONVERT_D1, bits);
    if (_result) return _result;
    uint32_t d1 = readADC();
    if (_result) return _result;

    convert(CMD_CONVERT_D2, bits);
    if (_result) return _result;
    uint32_t d2 = readADC();
    if (_result) return _result;

    // ── 一阶换算（数据手册 P7；参考库逐行对应）──
    int64_t dT  = static_cast<int64_t>(d2) - _c5s;            // D2 - C5×256
    int64_t temp = 2000 + (dT * _c6s) / 8388608LL;            // 0.01°C：dT×C6/2^23
    int64_t off  = _c2s + (dT * _c4s) / 128LL;                // C2×2^16 + dT×C4/2^7
    int64_t sens = _c1s + (dT * _c3s) / 256LL;                // C1×2^15 + dT×C3/2^8

    // ── 二阶补偿（数据手册 P8；TEMP 单位 0.01°C）──
    if (_compensation && (temp < 2000))
    {
        int64_t t2 = (dT * dT) / 2147483648LL;                // dT²/2^31
        int64_t t  = (temp - 2000) * (temp - 2000);
        int64_t off2  = (5 * t) / 2;                          // 2.5×t
        int64_t sens2 = (5 * t) / 4;                          // 1.25×t
        if (temp < -1500)
        {
            int64_t t2b = (temp + 1500) * (temp + 1500);
            off2  += 7 * t2b;                                 // +7×t'
            sens2 += (11 * t2b) / 2;                          // +5.5×t'
        }
        temp -= t2;
        off  -= off2;
        sens -= sens2;
    }

    // ── 压力（Pa）：(D1×SENS/2^21 - OFF)/2^15 ──
    int64_t p = (static_cast<int64_t>(d1) * sens / 2097152LL - off) / 32768LL;

    _temperature_x100 = static_cast<int32_t>(temp);
    _pressure_Pa      = static_cast<int32_t>(p);
    _lastRead         = get_tick();
    return ERR_OK;
}

// ============================================================
//  测量结果 / 偏移
// ============================================================

int32_t MS5611_SPI::getTemperature_mC() const
{
    // 0.01°C × 10 = m°C（参考库 getTemperature = TEMP×0.01）
    return _temperature_x100 * 10 + _temperatureOffset_mC;
}

int32_t MS5611_SPI::getPressure_Pa() const
{
    return _pressure_Pa + _pressureOffset_Pa;
}

// ============================================================
//  低层
// ============================================================

void MS5611_SPI::command(uint8_t cmd)
{
    _spi.cs_select();
    _spi.transfer_byte(cmd);
    _spi.cs_deselect();
}

uint32_t MS5611_SPI::readADC()
{
    uint32_t value = 0;

    _spi.cs_select();
    // 0x00 即 ADC 读取命令；首字节应答丢弃（参考库行为），
    // 连续 3 字节应答拼成 24 位读数
    _spi.transfer_byte(CMD_READ_ADC);
    value  = _spi.transfer_byte(0x00);
    value <<= 8;
    value += _spi.transfer_byte(0x00);
    value <<= 8;
    value += _spi.transfer_byte(0x00);
    _spi.cs_deselect();

    if ((value == 0) || (value == 0xFFFF))
    {
        _result = ERR_ADC;
        return value;
    }
    _result = ERR_OK;
    return value;
}

uint16_t MS5611_SPI::readProm(uint8_t reg)
{
    if (reg > 7) return 0;   // 最后一个 EEPROM 寄存器是 CRC（P13 数据手册）

    uint16_t value = 0;

    _spi.cs_select();
    _spi.transfer_byte(static_cast<uint8_t>(CMD_READ_PROM + reg * 2));
    value  = _spi.transfer_byte(0x00);
    value <<= 8;
    value += _spi.transfer_byte(0x00);
    _spi.cs_deselect();

    if ((value == 0) || (value == 0xFFFF))
    {
        _result = ERR_PROM;
        return value;
    }
    _result = ERR_OK;
    return value;
}

void MS5611_SPI::convert(uint8_t addr, uint8_t bits)
{
    uint8_t index = bits;
    if (index < 8)      index = 8;
    else if (index > 12) index = 12;
    index -= 8;

    // OSR 偏移 ×2：D1 = 0x40/42/44/46，D2 = 0x50/52/54/56
    command(static_cast<uint8_t>(addr + index * 2));

    // 最大转换时间（数据手册 P3 MAX 列，向上取整）
    static const uint16_t waitUs[5] = { 600, 1200, 2300, 4600, 9100 };
    delay_us(waitUs[index]);
}

void MS5611_SPI::initConstants()
{
    // 参考库 initConstants() 预缩放系数（mathMode=0），再乘 PROM 值：
    //   C[1] = 32768×C1（×2^15）   C[2] = 65536×C2（×2^16）
    //   C[3] = C3/2^8              C[4] = C4/2^7
    //   C[5] = 256×C5（×2^8）      C[6] = C6/2^23
    _c1s = static_cast<int64_t>(_prom[1]) * 32768LL;
    _c2s = static_cast<int64_t>(_prom[2]) * 65536LL;
    _c3s = static_cast<int64_t>(_prom[3]);
    _c4s = static_cast<int64_t>(_prom[4]);
    _c5s = static_cast<int64_t>(_prom[5]) * 256LL;
    _c6s = static_cast<int64_t>(_prom[6]);
}
