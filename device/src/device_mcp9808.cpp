#include "device_mcp9808.hpp"

// ============================================================
//  构造
// ============================================================

MCP9808::MCP9808(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
{
}

MCP9808::~MCP9808()
{
}

// ============================================================
//  init：上电初始化
//
//  注：MCP9808 无软件复位命令，上电本身即复位（数据手册）。
//  写 CONFIG 寄存器（shutdown=0）可解除关断模式，等效
//  "上电复位 + 重新配置"；随后写入分辨率。
// ============================================================

void MCP9808::init()
{
    writeRaw(REG_CONFIG, buildConfigReg());
    writeRaw(REG_RES, static_cast<uint16_t>(_cfg.resolution));
}

// ============================================================
//  连接 / 错误
// ============================================================

bool MCP9808::isConnected()
{
    return _dev.ping();
}

int MCP9808::getLastError()
{
    return _dev.lastError();
}

// ============================================================
//  温度
// ============================================================

int32_t MCP9808::getTemperature_mC()
{
    uint16_t raw = readRaw(REG_TA);
    _status = static_cast<uint8_t>((raw >> 13) & 0x07);   // bit15-13 状态字段
    return raw12To_mC(raw) + _offset_mC;
}

// ============================================================
//  12 位幅值 + bit12 符号位 → m°C（温度/阈值寄存器共用）
//
//  正数: T = raw12 × 0.0625°C = raw12 × 62.5 m°C = raw12 × 125/2
//  负数: T = raw12 × 0.0625°C − 256°C（同参考库 val×0.0625 − 256）
// ============================================================

int32_t MCP9808::raw12To_mC(uint16_t raw)
{
    uint16_t mag = raw & 0x0FFF;
    int32_t  t   = (static_cast<int32_t>(mag) * 125) / 2;
    if (raw & 0x1000)
    {
        t -= 256000;
    }
    return t;
}

// ============================================================
//  m°C → 阈值寄存器值（同参考库 writeFloat: round(f×4)×4 + 符号位）
//
//  f×4 = t_mC / 250（0.25°C 单位，四舍五入到整 0.25°C）
//  再 ×4 → 0.0625°C 单位；负数置 bit12
// ============================================================

uint16_t MCP9808::mCToThresholdReg(int32_t t_mC)
{
    bool negative = (t_mC < 0);
    if (negative) t_mC = -t_mC;
    uint16_t reg = static_cast<uint16_t>(((t_mC + 125) / 250) * 4);
    if (negative) reg |= 0x1000;
    return reg;
}

// ============================================================
//  偏移
// ============================================================

void MCP9808::setOffset_mC(int32_t offset_mC)
{
    _offset_mC = offset_mC;
}

// ============================================================
//  CONFIG 寄存器
// ============================================================

void MCP9808::setConfigRegister(uint16_t configuration)
{
    writeRaw(REG_CONFIG, configuration);
}

uint16_t MCP9808::getConfigRegister()
{
    return readRaw(REG_CONFIG);
}

void MCP9808::setConfig(const Config& cfg)
{
    _cfg = cfg;
    writeRaw(REG_CONFIG, buildConfigReg());
    writeRaw(REG_RES, static_cast<uint16_t>(_cfg.resolution));
}

uint16_t MCP9808::buildConfigReg() const
{
    uint16_t r = 0;
    r |= static_cast<uint16_t>(_cfg.hysteresis) << 11;
    if (_cfg.shutdown)   r |= CFG_SHUTDOWN;
    if (_cfg.critLock)   r |= CFG_CRIT_LOCK;
    if (_cfg.winLock)    r |= CFG_WIN_LOCK;
    if (_cfg.alertCtrl)  r |= CFG_ALERT_CTRL;
    if (_cfg.alertSel)   r |= CFG_ALERT_SELECT;
    if (_cfg.alertPolar) r |= CFG_ALERT_POLAR;
    if (_cfg.alertMode)  r |= CFG_ALERT_MODE;
    return r;
}

// ============================================================
//  报警阈值
// ============================================================

void MCP9808::setTupper_mC(int32_t t_mC)    { writeRaw(REG_TUPPER, mCToThresholdReg(t_mC)); }
int32_t MCP9808::getTupper_mC()             { return raw12To_mC(readRaw(REG_TUPPER)); }
void MCP9808::setTlower_mC(int32_t t_mC)    { writeRaw(REG_TLOWER, mCToThresholdReg(t_mC)); }
int32_t MCP9808::getTlower_mC()             { return raw12To_mC(readRaw(REG_TLOWER)); }
void MCP9808::setTcritical_mC(int32_t t_mC) { writeRaw(REG_TCRIT,  mCToThresholdReg(t_mC)); }
int32_t MCP9808::getTcritical_mC()          { return raw12To_mC(readRaw(REG_TCRIT)); }

// ============================================================
//  分辨率
// ============================================================

void MCP9808::setResolution(Resolution res)
{
    _cfg.resolution = res;
    // 分辨率寄存器低 2 位有效，高字节保留位写 0
    writeRaw(REG_RES, static_cast<uint16_t>(res));
}

MCP9808::Resolution MCP9808::getResolution()
{
    return static_cast<Resolution>(readRaw(REG_RES) & 0x03);
}

// ============================================================
//  设备信息
// ============================================================

uint16_t MCP9808::getManufacturerID() { return readRaw(REG_MID); }
uint8_t  MCP9808::getDeviceID()       { return static_cast<uint8_t>(readRaw(REG_DID) >> 8); }
uint8_t  MCP9808::getRevision()       { return static_cast<uint8_t>(readRaw(REG_DID) & 0xFF); }
uint16_t MCP9808::getRFU()            { return readRaw(REG_RFU); }

// ============================================================
//  I2C 读写（全部 16-bit 寄存器）
//  reg > 0x08 的地址只读（MID/DID），禁止写（同参考库 p.16 检查）
// ============================================================

void MCP9808::writeRaw(uint8_t reg, uint16_t data)
{
    if (reg > REG_RES) return;
    _dev.write_16bit(reg, data);
}

uint16_t MCP9808::readRaw(uint8_t reg)
{
    if (reg > REG_RES) return 0;
    return _dev.read_16bit(reg);
}

//  -- END OF FILE --
