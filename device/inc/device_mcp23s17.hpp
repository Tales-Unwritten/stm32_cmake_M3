#pragma once

#ifdef __cplusplus

#include <cstdint>
#include "inter_spi_bus.hpp"

// ============================================================
//  MCP23S17 SPI 16 位 IO 扩展器驱动（双 8 位端口 A/B）
// ============================================================
//  来源：Rob Tillaart 的 Arduino 库 "MCP23S17" v0.8.2（2021-12-30）
//    URL：https://github.com/RobTillaart/MCP23S17
//  （寄存器定义与 I2C 版 MCP23017_RT 共用，见 MCP23x17_registers.h）
//
//  移植说明（对照参考库 0.8.2）：
//    - 与已移植 I2C 版 device_mcp23017 的 API 对齐：
//      pinMode1/write1/read1、pinMode8/write8/read8、
//      pinMode16/write16/read16、setPullup/getPullup、
//      setPolarity/getPolarity、enableInterrupt/disableInterrupt、
//      init(→begin)/isConnected/getLastError
//    - 器件地址 3 位（A2 A1 A0），控制字节 = 0x40 | (addr<<1) | RW：
//      写 0x40|addr<<1，读 0x41|addr<<1（地址在构造时左移 1 位保存）
//    - SPI 版特有：getAddress、enableHardwareAddress /
//      disableHardwareAddress（IOCON.HAEN，bit3，多片共用 CS 时必须
//      使能硬件地址，见数据手册 3.3.2）
//    - 寄存器映射固定 BANK=0；begin() 读 IOCON 清 SEQOP 并顺带清
//      BANK 位（16 位 API 依赖地址自增；参考库只清 SEQOP，此处
//      额外保证 BANK=0，I2C 版 MCP23017 同样显式写 IOCON=0x00）
//    - 裁剪：reverse16ByteOrder（默认位序：高字节 = Port A）、
//      setSPIspeed（速度由 spi_port prescaler 决定）、软件 SPI、
//      usesHWSPI、中断 8/16 位批量接口、中断极性/镜像、
//      enableControlRegister/disableControlRegister 保留
//    - isConnected()：SPI 版无法可靠探测（参考库同样直接返回 true）
//    - SPI 错误（ERR_SPI）不可检测，仅保留枚举与 I2C 版对齐
//    - ⚠️ REV D 芯片 GPA7/GPB7 作输入时行为异常，详见参考库 README
//    - 无浮点、无堆分配、无 STL/异常，C++17，Cortex-M0+
//
//  SPI 模式：Mode 0（CPOL=0, CPHA=0）——参考库使用 SPI_MODE0，
//  数据手册：数据在 SCK 上升沿锁存，最高 10 MHz（参考库默认 8 MHz）。
// ============================================================

class MCP23S17
{
public:

    // ── 寄存器地址（BANK=0 模式）─────────────────────────

    enum Reg : uint8_t
    {
        REG_DDR_A     = 0x00,   // 数据方向 A（1=输入，0=输出）
        REG_DDR_B     = 0x01,   // 数据方向 B
        REG_POL_A     = 0x02,   // 输入极性 A（1=反相）
        REG_POL_B     = 0x03,   // 输入极性 B
        REG_GPINTEN_A = 0x04,   // 中断使能 A
        REG_GPINTEN_B = 0x05,   // 中断使能 B
        REG_DEFVAL_A  = 0x06,   // 中断比较默认值 A
        REG_DEFVAL_B  = 0x07,   // 中断比较默认值 B
        REG_INTCON_A  = 0x08,   // 中断控制 A（1=与 DEFVAL 比较，0=与上次值比较）
        REG_INTCON_B  = 0x09,   // 中断控制 B
        REG_IOCON     = 0x0A,   // IO 控制寄存器（两种 BANK 模式下地址相同）
        REG_PUR_A     = 0x0C,   // 内部上拉 A
        REG_PUR_B     = 0x0D,   // 内部上拉 B
        REG_INTF_A    = 0x0E,   // 中断标志 A（只读）
        REG_INTF_B    = 0x0F,   // 中断标志 B（只读）
        REG_INTCAP_A  = 0x10,   // 中断捕获 A（只读）
        REG_INTCAP_B  = 0x11,   // 中断捕获 B（只读）
        REG_GPIO_A    = 0x12,   // 端口 A 数据（写 = 输出锁存，读 = 引脚电平）
        REG_GPIO_B    = 0x13,   // 端口 B 数据
        REG_OLAT_A    = 0x14,   // 输出锁存 A
        REG_OLAT_B    = 0x15,   // 输出锁存 B
    };

    // ── IOCON 寄存器位定义 ───────────────────────────────

    static constexpr uint8_t IOCON_BANK   = 0x80;   // 1 = banked 寄存器映射
    static constexpr uint8_t IOCON_MIRROR = 0x40;   // 1 = INTA/INTB 合并
    static constexpr uint8_t IOCON_SEQOP  = 0x20;   // 1 = 关闭地址自增
    static constexpr uint8_t IOCON_DISSLW = 0x10;   // 1 = 关闭 SDA 压摆率控制
    static constexpr uint8_t IOCON_HAEN   = 0x08;   // 1 = 使能硬件地址（SPI 版）
    static constexpr uint8_t IOCON_ODR    = 0x04;   // 1 = INT 开漏输出
    static constexpr uint8_t IOCON_INTPOL = 0x02;   // 1 = INT 高有效

    // ── 控制字节（数据手册 3.1）：0x40 | (addr<<1) | RW ──

    static constexpr uint8_t OPCODE_WRITE = 0x40;   // 写控制字节基地址
    static constexpr uint8_t OPCODE_READ  = 0x41;   // 读控制字节基地址

    // ── 引脚模式（取值对应 Arduino INPUT/OUTPUT/INPUT_PULLUP）──

    enum class Mode : uint8_t
    {
        Input = 0, Output = 1, InputPullup = 2,
    };

    // ── 中断触发模式 ─────────────────────────────────────

    enum class IntMode : uint8_t
    {
        Rising = 1, Falling = 2, Change = 3,
    };

    // ── 错误码（与参考库一致）─────────────────────────────

    enum ErrCode : uint8_t
    {
        ERR_NONE     = 0x00,
        ERR_PIN      = 0x81,   // 引脚号越界（> 15）
        ERR_SPI      = 0x82,   // SPI 总线错误（SPI 版不可检测，保留对齐）
        ERR_VALUE    = 0x83,   // 非法参数（如模式）
        ERR_PORT     = 0x84,   // 端口号越界（> 1）
        ERR_REGISTER = 0xFF,   // 非法寄存器地址
    };

    // ============================================================
    //  构造
    // ============================================================

    /**
     * @param spi     共享 SPI 总线（CS 约定见头注释）
     * @param address 器件硬件地址 A2 A1 A0（0..7，默认 0）。
     *                默认 IOCON.HAEN=0 时芯片忽略地址位；多片共用
     *                CS 时需先 enableHardwareAddress() 使能地址
     */
    explicit MCP23S17(spi_bus &spi, uint8_t address = 0x00);

    MCP23S17(const MCP23S17&) = delete;
    MCP23S17& operator=(const MCP23S17&) = delete;

    /**
     * @brief 初始化（对齐 I2C 版 init()）：清 SEQOP/BANK 保证
     *        16 位 API 可用；可选全部 16 脚使能内部上拉
     */
    bool begin(bool pullup = true);

    // ============================================================
    //  连接 / 错误
    // ============================================================

    /** @brief SPI 版无法可靠探测，恒返回 true（参考库行为） */
    [[nodiscard]] bool isConnected();

    /** @brief 构造设置的器件地址（0..7） */
    uint8_t getAddress() const { return static_cast<uint8_t>(_address >> 1); }

    /** @brief 最近一次操作错误码（读取后自动清零） */
    [[nodiscard]] uint8_t getLastError();

    // ============================================================
    //  单引脚 API（pin = 0..15）
    // ============================================================

    bool    pinMode1(uint8_t pin, Mode mode);
    bool    write1(uint8_t pin, uint8_t value);   // value: 0/1
    uint8_t read1(uint8_t pin);                   // 返回 0/1；失败返回 0xFF

    bool setPolarity(uint8_t pin, bool reversed); // 输入极性反相
    bool getPolarity(uint8_t pin, bool& reversed);
    bool setPullup(uint8_t pin, bool pullup);
    bool getPullup(uint8_t pin, bool& pullup);

    // ============================================================
    //  8 引脚（端口）API（port = 0..1）
    // ============================================================

    bool    pinMode8(uint8_t port, uint8_t mask); // mask 位 1 = 输入
    bool    write8(uint8_t port, uint8_t value);
    uint8_t read8(uint8_t port);                  // 失败返回 0xFF

    // ============================================================
    //  16 引脚 API（高 8 位 = Port A，低 8 位 = Port B）
    // ============================================================

    bool     pinMode16(uint16_t mask);
    bool     write16(uint16_t value);
    uint16_t read16();                            // 失败返回 0（查 getLastError）

    // ============================================================
    //  中断（单引脚；INTA/INTB 引脚需外部接线并读取标志）
    // ============================================================

    bool enableInterrupt(uint8_t pin, IntMode mode);
    bool disableInterrupt(uint8_t pin);

    // ============================================================
    //  IOCON 控制（SPI 版特有）
    // ============================================================

    /** @brief 置位 IOCON 位（mask 见 IOCON_* 常量） */
    bool enableControlRegister(uint8_t mask);

    /** @brief 清 IOCON 位（mask 见 IOCON_* 常量） */
    bool disableControlRegister(uint8_t mask);

    /** @brief 使能硬件地址（IOCON.HAEN=1）——多片共用 CS 时使用 */
    bool enableHardwareAddress()  { return enableControlRegister(IOCON_HAEN); }

    /** @brief 禁用硬件地址（IOCON.HAEN=0，复位默认，地址位被忽略） */
    bool disableHardwareAddress() { return disableControlRegister(IOCON_HAEN); }

private:

    // ── 低层寄存器访问（参考库 writeReg/readReg/writeReg16/readReg16）──
    // 单字节：控制字节 + 寄存器地址 + [数据]；16 位：地址自增连读/写两字节
    // （依赖 begin() 已清 SEQOP）

    bool     writeReg(uint8_t reg, uint8_t value);
    uint8_t  readReg(uint8_t reg);
    bool     writeReg16(uint8_t reg, uint16_t value);
    uint16_t readReg16(uint8_t reg);

    spi_bus &_spi;
    uint8_t  _address;          // 器件地址（已左移 1 位，与控制字节直接或）
    uint8_t  _error;

    static constexpr uint8_t INVALID_READ = 0xFF;
};

#endif /* __cplusplus */
