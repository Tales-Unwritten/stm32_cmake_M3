#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include <cstdint>

// ============================================================
//  MCP23017 I2C 16 位 IO 扩展器驱动（双 8 位端口 A/B）
// ============================================================
//  来源：Rob Tillaart 的 Arduino 库 "MCP23017_RT"
//  版本：0.9.3（2019-10-12）
//    URL：https://github.com/RobTillaart/MCP23017_RT
//
//  移植说明（对照参考库 0.9.3）：
//    - 寄存器映射固定 BANK=0（非 banked）模式，与参考库
//      MCP23x17_registers.h 一致；init() 显式写 IOCON=0x00，保证
//      BANK=0 且 SEQOP=0（地址指针自增，16 位 API 依赖）。
//      IOCON 在两种 BANK 模式下地址都是 0x0A，该写入始终有效
//    - 16 位 API 位序与参考库默认（reverse16ByteOrder=false）一致：
//      高 8 位 = Port A（pin 8..15），低 8 位 = Port B（pin 0..7）；
//      8 位寄存器走 freedom_*，16 位寄存器走 write_16bit/read_16bit
//      （MSB 在前，与参考库 writeReg16/readReg16 相同）
//    - 引脚模式用枚举（Input/Output/InputPullup），避开参考库
//      "不要用 0/1 表示模式"的陷阱；取值与 Arduino 常量一致
//    - 裁剪：reverse16ByteOrder、8/16 位中断批量接口、中断标志/捕获
//      寄存器、setInterruptPolarity/mirrorInterrupts、
//      enable/disableControlRegister、setAddress/setWire
//    - ⚠️ REV D 芯片 GPA7/GPB7 作输入时行为异常，详见参考库 README
//    - 无浮点、无堆分配，I2C 访问仅依赖 inter_i2c_dev
// ============================================================

class MCP23017
{
public:

    // ── I2C 地址（A2 A1 A0 决定）─────────────────────────

    enum Addr : uint8_t
    {
        ADDR_0x20 = 0x20, ADDR_0x21 = 0x21,
        ADDR_0x22 = 0x22, ADDR_0x23 = 0x23,
        ADDR_0x24 = 0x24, ADDR_0x25 = 0x25,
        ADDR_0x26 = 0x26, ADDR_0x27 = 0x27,
    };

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
    static constexpr uint8_t IOCON_ODR    = 0x04;   // 1 = INT 开漏输出
    static constexpr uint8_t IOCON_INTPOL = 0x02;   // 1 = INT 高有效

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
        ERR_I2C      = 0x82,   // I2C 总线错误
        ERR_VALUE    = 0x83,   // 非法参数（如模式）
        ERR_PORT     = 0x84,   // 端口号越界（> 1）
        ERR_REGISTER = 0xFF,   // 非法寄存器地址
    };

    // ============================================================
    //  Config 结构体（用户可定制）
    // ============================================================

    struct Config
    {
        bool pullups = true;    // init() 时给全部 16 个引脚使能内部上拉
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit MCP23017(inter_i2c_bus* bus, uint8_t addr = ADDR_0x20);
    explicit MCP23017(inter_i2c_bus* bus, uint8_t addr, const Config& cfg);

    MCP23017(const MCP23017&) = delete;
    MCP23017& operator=(const MCP23017&) = delete;

    /** @brief 初始化：写 IOCON 强制 BANK=0/SEQOP=0，可选全部上拉 */
    void init();

    // ============================================================
    //  连接 / 错误
    // ============================================================

    /** @brief I2C 地址探测 */
    [[nodiscard]] bool isConnected();

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

private:

    inter_i2c_dev _dev;
    Config        _cfg;
    uint8_t       _error;

    static constexpr uint8_t INVALID_READ = 0xFF;

    // ── 低层寄存器访问 ────────────────────────────────────
    // 8 位寄存器走 freedom_*；16 位寄存器（A+B 连续地址）走
    // write_16bit / read_16bit（MSB 在前，与参考库 writeReg16 /
    // readReg16 默认位序一致）

    bool     writeReg(uint8_t reg, uint8_t value);
    uint8_t  readReg(uint8_t reg);
    bool     writeReg16(uint8_t reg, uint16_t value);
    uint16_t readReg16(uint8_t reg);
};

#endif
