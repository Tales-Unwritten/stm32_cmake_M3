#pragma once

#ifdef __cplusplus

#include "inter_i2c_dev.hpp"
#include "inter_i2c_bus.hpp"
#include <cstdint>

// ============================================================
//  PCF8574 I2C 8 位 IO 扩展器驱动（准双向口）
// ============================================================
//  来源：Rob Tillaart 的 Arduino 库 "PCF8574"
//  版本：0.4.5（2013-02-02 起）
//    URL：https://github.com/RobTillaart/PCF8574
//
//  移植说明（对照参考库 0.4.5）：
//    - PCF8574 没有寄存器地址空间：写 = 直接发 1 字节输出值，
//      读 = 直接收 1 字节端口状态。inter_i2c_dev 的 freedom_* 接口
//      总是先发寄存器字节，无法表达该协议（会把寄存器字节当作输出
//      数据写进端口），因此 read8/write8 直接使用 inter_i2c_bus
//      原语（与 inter_i2c_dev.cpp 相同的时序），ping/isConnected
//      仍走 _dev
//    - 准双向口：输出 1 时引脚呈高阻（可被外部拉低），读引脚前
//      必须先 write(pin, 1)；输出采用读-改-写（基于本地 _dataOut
//      缓存，与参考库一致）
//    - 参考库 begin(value) 对应本驱动 Config{initialValue} + init()
//    - 裁剪：writeArray/readArray、readButton8/readButton、
//      toggleMask/shift/rotate/reverse、select/selectN/selectNone/
//      selectAll、setAddress/getAddress
//    - 无浮点、无堆分配
// ============================================================

class PCF8574
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

    // 注意：PCF8574 无寄存器地址空间，故不提供 enum Reg

    // ── 错误码（与参考库一致）─────────────────────────────

    enum ErrCode : uint8_t
    {
        ERR_NONE = 0x00,
        ERR_PIN  = 0x81,   // 引脚号越界（> 7）
        ERR_I2C  = 0x82,   // I2C 总线错误
    };

    // ============================================================
    //  Config 结构体（用户可定制）
    // ============================================================

    struct Config
    {
        uint8_t initialValue = 0xFF;   // 初始输出值（复位后端口 = 0xFF 全 1）
    };

    // ============================================================
    //  构造
    // ============================================================

    explicit PCF8574(inter_i2c_bus* bus, uint8_t addr = ADDR_0x20);
    explicit PCF8574(inter_i2c_bus* bus, uint8_t addr, const Config& cfg);

    PCF8574(const PCF8574&) = delete;
    PCF8574& operator=(const PCF8574&) = delete;

    /** @brief 初始化：输出初始值（对应参考库 begin(value)） */
    void init();

    /** @brief I2C 地址探测 */
    [[nodiscard]] bool isConnected();

    // ============================================================
    //  读取
    // ============================================================

    uint8_t read8();                // 读 8 位端口；失败返回上次缓存值
    uint8_t read(uint8_t pin);      // 读单个引脚（0/1）；读前该引脚需已写 1

    /** @brief 最近一次读入值（不触发 I2C 读） */
    uint8_t value() const { return _dataIn; }

    // ============================================================
    //  写入（读-改-写，基于 _dataOut 缓存）
    // ============================================================

    void write8(uint8_t value);     // 直接输出 8 位
    void write(uint8_t pin, uint8_t value);   // 改单个引脚（0/1）
    void toggle(uint8_t pin);       // 翻转单个引脚输出

    /** @brief 最近一次输出值（不触发 I2C 写） */
    uint8_t valueOut() const { return _dataOut; }

    // ============================================================
    //  错误
    // ============================================================

    /** @brief 最近一次操作错误码（读取后自动清零） */
    [[nodiscard]] uint8_t getLastError();

private:

    inter_i2c_bus *_bus;      // 裸字节传输用（见移植说明）
    inter_i2c_dev  _dev;
    uint8_t        _addr;
    uint8_t        _dataIn;   // 最近一次读入值
    uint8_t        _dataOut;  // 最近一次输出值
    uint8_t        _err;      // 错误码
    Config         _cfg;
};

#endif
