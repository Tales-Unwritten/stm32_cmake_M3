#pragma once

#ifdef __cplusplus

#include <cstdint>
#include "inter_i2c_bus.hpp"
#include "inter_io_ctrl.hpp"

// ============================================================================
// AT24Cxx 系列 EEPROM 器件地址参考手册
// ============================================================================
// 本文档详细说明 AT24Cxx 各型号的 I2C 地址结构、地址引脚用法和寻址差异。
// 内容基于各型号数据手册整理。
// ============================================================================


// ============================================================================
// 一、地址结构基础
// ============================================================================
//
// AT24Cxx 的 I2C 器件地址由 8 位组成（最后 1 位为 R/W 方向位）：
//
//     Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0
//     ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
//     │  1  │  0  │  1  │  0  │ H2  │ H1  │ H0  │ R/W │
//     └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
//      固定值(0xA)        ↑     ↑     ↑
//                    容量小的型号 → 这里是 A2/A1/A0 引脚（芯片选择）
//                    容量大的型号 → 这里是 P2/P1/P0 内部位（页选择）
//
// • Bit7~Bit4：固定为 1010（0xA），所有型号一致
// • Bit3~Bit1：由硬件引脚电平或内部页地址决定（见各型号说明）
// • Bit0：0 = 写操作，1 = 读操作
//
// 关键规律：
//   容量越大的型号，外部地址引脚越少 —— 因为高位地址位被"借"去寻址更大的存储空间了。
//   24C04/08/16 就是这种情况。而 24C32/64 改用双字节寻址，地址引脚全部回归。


// ============================================================================
// 二、各型号详细说明
// ============================================================================


// ---------------------------------------------------------------------------
// 2.1  24C02（2Kbit / 256 字节）
// ---------------------------------------------------------------------------
// 地址引脚：A2、A1、A0 全部有效，可挂载 8 片
// 地址字节：1 0 1 0 A2 A1 A0 R/W
//
//   A2  A1  A0 | 写地址   读地址
//   -----------|-----------------
//   0   0   0  | 0xA0     0xA1
//   0   0   1  | 0xA2     0xA3
//   0   1   0  | 0xA4     0xA5
//   0   1   1  | 0xA6     0xA7
//   1   0   0  | 0xA8     0xA9
//   1   0   1  | 0xAA     0xAB
//   1   1   0  | 0xAC     0xAD
//   1   1   1  | 0xAE     0xAF


// ---------------------------------------------------------------------------
// 2.2  24C04（4Kbit / 512 字节）
// ---------------------------------------------------------------------------
// 地址引脚：A2、A1 有效，A0 无效（被 P0 页地址位取代）
// 地址字节：1 0 1 0 A2 A1 P0 R/W
//
//   ⚠ A0 引脚悬空（NC），接了也不影响地址。P0 是存储地址的第 9 位（选择高/低 256 字节页）。
//
//   A2  A1  P0 | 写地址   读地址   访问范围
//   -----------|--------------------------
//   0   0   0  | 0xA0     0xA1     地址 0~255
//   0   0   1  | 0xA2     0xA3     地址 256~511
//   0   1   0  | 0xA4     0xA5     地址 0~255
//   0   1   1  | 0xA6     0xA7     地址 256~511
//   1   0   0  | 0xA8     0xA9     地址 0~255
//   1   0   1  | 0xAA     0xAB     地址 256~511
//   1   1   0  | 0xAC     0xAD     地址 0~255
//   1   1   1  | 0xAE     0xAF     地址 256~511
//
// 同一片 24C04 会响应两个地址（P0=0 和 P0=1），分别对应低页和高页。最多挂 4 片。


// ---------------------------------------------------------------------------
// 2.3  24C08（8Kbit / 1024 字节）
// ---------------------------------------------------------------------------
// 地址引脚：A2 有效，A1、A0 无效（被 P1、P0 页地址位取代）
// 地址字节：1 0 1 0 A2 P1 P0 R/W
//
//   ⚠ A0、A1 引脚悬空（NC）。P1、P0 是存储地址的第 10、9 位（选择 4 个 256 字节页）。
//
//   A2  P1  P0 | 写地址   读地址   访问范围
//   -----------|--------------------------
//   0   0   0  | 0xA0     0xA1     地址 0~255
//   0   0   1  | 0xA2     0xA3     地址 256~511
//   0   1   0  | 0xA4     0xA5     地址 512~767
//   0   1   1  | 0xA6     0xA7     地址 768~1023
//   1   0   0  | 0xA8     0xA9     地址 0~255
//   1   0   1  | 0xAA     0xAB     地址 256~511
//   1   1   0  | 0xAC     0xAD     地址 512~767
//   1   1   1  | 0xAE     0xAF     地址 768~1023
//
// 同一片 24C08 会响应 4 个地址（P1P0 = 00/01/10/11）。最多挂 2 片。


// ---------------------------------------------------------------------------
// 2.4  24C16（16Kbit / 2048 字节）
// ---------------------------------------------------------------------------
// 地址引脚：A2、A1、A0 全部无效（被 P2、P1、P0 页地址位取代）
// 地址字节：1 0 1 0 P2 P1 P0 R/W
//
//   ⚠ A0、A1、A2 引脚全部悬空（NC），没有任何可配置的地址引脚。只能挂 1 片！
//
//   P2  P1  P0 | 写地址   读地址   访问范围
//   -----------|--------------------------
//   0   0   0  | 0xA0     0xA1     地址 0~255
//   0   0   1  | 0xA2     0xA3     地址 256~511
//   0   1   0  | 0xA4     0xA5     地址 512~767
//   0   1   1  | 0xA6     0xA7     地址 768~1023
//   1   0   0  | 0xA8     0xA9     地址 1024~1279
//   1   0   1  | 0xAA     0xAB     地址 1280~1535
//   1   1   0  | 0xAC     0xAD     地址 1536~1791
//   1   1   1  | 0xAE     0xAF     地址 1792~2047
//
// 同一片 24C16 会响应 8 个地址，分别对应 8 个 256 字节页。总线上只能存在 1 片。


// ---------------------------------------------------------------------------
// 2.5  24C32（32Kbit / 4096 字节）
// ---------------------------------------------------------------------------
// 地址引脚：A2、A1、A0 全部有效（和 24C02 一样！）
// 地址字节：1 0 1 0 A2 A1 A0 R/W
//
//   💡 24C32/64 容量变大后改用双字节地址寻址，不再需要"借用"地址位，
//      A2/A1/A0 恢复为纯粹的芯片选择功能。
//
//   A2  A1  A0 | 写地址   读地址
//   -----------|-----------------
//   0   0   0  | 0xA0     0xA1
//   0   0   1  | 0xA2     0xA3
//   0   1   0  | 0xA4     0xA5
//   0   1   1  | 0xA6     0xA7
//   1   0   0  | 0xA8     0xA9
//   1   0   1  | 0xAA     0xAB
//   1   1   0  | 0xAC     0xAD
//   1   1   1  | 0xAE     0xAF
//
// 最多挂 8 片。地址表与 24C02 完全相同，区别每片容量大了 16 倍（4KB vs 256B）。


// ---------------------------------------------------------------------------
// 2.6  24C64（64Kbit / 8192 字节）
// ---------------------------------------------------------------------------
// 地址引脚：A2、A1、A0 全部有效（和 24C02/32 一样）
// 地址字节：1 0 1 0 A2 A1 A0 R/W
//
//   A2  A1  A0 | 写地址   读地址
//   -----------|-----------------
//   0   0   0  | 0xA0     0xA1
//   0   0   1  | 0xA2     0xA3
//   0   1   0  | 0xA4     0xA5
//   0   1   1  | 0xA6     0xA7
//   1   0   0  | 0xA8     0xA9
//   1   0   1  | 0xAA     0xAB
//   1   1   0  | 0xAC     0xAD
//   1   1   1  | 0xAE     0xAF
//
// 最多挂 8 片。地址表同 24C02/32，单芯片容量 8KB。


// ============================================================================
// 三、总览对比
// ============================================================================
//
//  型号     容量     有效引脚    最多挂载  响应地址数  地址计算公式
//  ------  --------  ----------  --------  ----------  -------------------------
//  24C02   256B      A2 A1 A0    8 片      1           0xA0 | (A2<<3)|(A1<<2)|(A0<<1)
//  24C04   512B      A2 A1       4 片      2           0xA0 | (A2<<3)|(A1<<2)  (P0 自动)
//  24C08   1KB       A2          2 片      4           0xA0 | (A2<<3)           (P1P0 自动)
//  24C16   2KB       无          1 片      8           固定 0xA0                (P2P1P0 自动)
//  24C32   4KB       A2 A1 A0    8 片      1           0xA0 | (A2<<3)|(A1<<2)|(A0<<1)
//  24C64   8KB       A2 A1 A0    8 片      1           0xA0 | (A2<<3)|(A1<<2)|(A0<<1)
//
// 一句话记忆：
//    24C04/08/16 —— 容量小的反而地址引脚少，因为高位被借走做页寻址了。
//    24C32/64   —— 改用双字节地址寻址，地址引脚全部回归，又能挂 8 片。


// ============================================================================
// 四、实际使用确认步骤
// ============================================================================
//
// 1. 检查硬件引脚接线
//    • 接 GND         → 该位 = 0
//    • 接 VCC         → 该位 = 1
//    • 悬空（NC）     → 默认为 0（但不建议悬空，可能不稳定）
//
// 2. 按对应型号的公式计算器件地址
//    数据手册典型应用（第 11 页）：U1 的 A0=A1=A2=GND → 地址为 0xA0
//
// 3. 确认总线地址不冲突


// ============================================================================
// 五、常见踩坑提醒
// ============================================================================
//
// • 24C04/08/16 的"无效"引脚必须悬空（NC）—— 接了也白接，那个位已被内部占用做页地址。
// • 24C16 只能挂 1 片：3 个地址位全被内部占用，A0/A1/A2 全部无效。
// • 24C32/64 恢复了 3 个地址引脚，和 24C02 一样能挂 8 片。
// • 总线上多个器件地址不能冲突，通过引脚电平分配不同地址。
// • WP 引脚不影响 I2C 地址：接 VCC 写保护（只读），接 GND 允许读写。


// ============================================================================
// 驱动代码
// ============================================================================


// ============================================================================
// 型号枚举
// ============================================================================

enum class eeprom_model : uint8_t
{
    AT24C02,        //   256B, page=8,   单字节地址, A2/A1/A0 全有效
    AT24C04,        //   512B, page=16,  单字节地址+P0, A2/A1 有效
    AT24C08,        //  1024B, page=16,  单字节地址+P1P0, A2 有效
    AT24C16,        //  2048B, page=16,  单字节地址+P2P1P0, 全部无效
    AT24C32,        //  4096B, page=32,  双字节地址, A2/A1/A0 全有效
    AT24C64,        //  8192B, page=32,  双字节地址, A2/A1/A0 全有效
    UNKNOWN,        // 自动检测失败时使用，此时仅 probe() 有效
};


// ============================================================================
// 配置结构体
// ============================================================================

struct eeprom_config
{
    eeprom_model model = eeprom_model::AT24C02;

    // 硬件引脚接线（接 GND=0, 接 VCC=1）
    uint8_t a2 : 1;
    uint8_t a1 : 1;
    uint8_t a0 : 1;

    // 写周期最大等待时间（毫秒），默认 10ms 覆盖所有型号的 tWR
    // 实际用 ACK 轮询，此值为超时上限
    uint8_t write_timeout_ms = 10;

    // WP 引脚（可选，nullptr 则 wp_enable/disable 为空操作）
    io_ctrl *wp_pin = nullptr;
};


// ============================================================================
// AT24Cxx 统一驱动类
// ============================================================================

class device_eeprom
{
public:
    // ================================================================
    //  构造
    // ================================================================

    explicit device_eeprom(inter_i2c_bus &bus, const eeprom_config &cfg);

    device_eeprom(const device_eeprom &)            = delete;
    device_eeprom &operator=(const device_eeprom &) = delete;

    // 延迟初始化：构造后调用，可重复调用
    void init();

    // ================================================================
    //  L1 — 硬件协议层
    // ================================================================

    /// 从指定地址读取 len 字节到 buf，返回 true 成功
    bool read(uint16_t mem_addr, uint8_t *buf, uint16_t len);

    /// 从指定地址写入 len 字节，自动处理跨页切分
    bool write(uint16_t mem_addr, const uint8_t *buf, uint16_t len);

    /// 单字节读
    bool read_byte(uint16_t mem_addr, uint8_t &data);

    /// 单字节写
    bool write_byte(uint16_t mem_addr, uint8_t data);

    /// 填充常量值（循环写入同一个字节）
    bool fill_byte(uint16_t start, uint8_t value, uint16_t len);

    /// ACK 探测：器件是否存在
    bool probe();

    /// 全片手动复位：发送 9 个时钟脉冲 + STOP（从 I2C 死锁恢复）
    void bus_recovery();

    // ── 属性查询 ──────────────────────────────────────────────

    uint16_t       capacity()   const { return _capacity; }
    uint8_t        page_size()  const { return _page_size; }
    uint8_t        i2c_addr()   const { return _i2c_base; }
    eeprom_model   model()      const { return _cfg.model; }
    bool           is_page_addressed() const { return _page_addressed; }
    bool           is_dual_addr() const { return _dual_addr; }

    // ================================================================
    //  L2 — 应用便利层
    // ================================================================

    /// 全片 dump 到缓冲区（调用方提供 >= capacity() 字节的 buf）
    bool dump(uint8_t *buf);

    /// 写入后自动回读校验（逐字节比对）
    bool write_verified(uint16_t addr, const uint8_t *buf, uint16_t len);

    /// 单字节版 update_block：读旧值，相同跳过写（省 EEPROM 写寿命）
    bool update_byte(uint16_t mem_addr, uint8_t data);

    /// 按页粒度更新：读旧数据逐字节比较，页内有变化才整页重写
    /// （吸收 Rob Tillaart I2C_EEPROM 的 updateByte/updateBlock 思想，
    ///  但按页批量写，避免参考库逐字节写的 n×5ms 写周期）
    /// 返回实际写入的字节数；0 = 全部相同无需写；失败返回失败前已写字节数
    uint16_t update_block(uint16_t mem_addr, const uint8_t *buf, uint16_t len);

    /// 区域空片检查（全 0xFF 返回 true）
    bool is_blank(uint16_t addr, uint16_t len);

    /// 连续读取（利用硬件 auto-increment，比多次 read 快）
    bool read_sequential(uint16_t start_addr, uint8_t *buf, uint16_t len);

    /// 写保护：拉高 WP 引脚（防误写）
    void wp_enable();

    /// 写保护：拉低 WP 引脚（允许写入）
    void wp_disable();

    /// 自动检测型号（非破坏性），返回 true 表示检测成功
    /// 算法：保存地址 0 的原值，用写入回读测试容量边界，最后恢复原值
    bool detect_model(eeprom_model &detected);

    // ================================================================
    //  L3 — 结构化存取（模板，在头文件实现）
    // ================================================================

    /// 结构化配置块头
    /// 格式：[magic:2B 0xA5E5] [version:1B] [data:sizeof(T)] [crc16:2B]
    static constexpr uint16_t CONFIG_MAGIC   = 0xA5E5;
    static constexpr uint8_t  CONFIG_HEADER  = 5;  // magic(2) + version(1) + data_len_hint(2)
    static constexpr uint8_t  CONFIG_CRC_LEN = 2;

    /// 计算存储配置所需总字节数
    template<typename T>
    static constexpr uint16_t config_storage_size()
    {
        return CONFIG_HEADER + sizeof(T) + CONFIG_CRC_LEN;
    }

    /// 读取结构化配置（带 magic + CRC16 校验）
    template<typename T>
    bool read_config(uint16_t base_addr, T &data, uint8_t expected_version = 1);

    /// 写入结构化配置（自动添加 magic + version + CRC16）
    template<typename T>
    bool write_config(uint16_t base_addr, const T &data, uint8_t version = 1);

    /// 校验配置块完整性（仅校验不读取数据）
    template<typename T>
    bool verify_config(uint16_t base_addr, uint8_t expected_version = 1);

private:
    // ── 内部参数表 ──────────────────────────────────────────

    struct _model_info {
        uint16_t capacity;
        uint8_t  page_size;
        bool     dual_addr;       // 是否双字节内存地址
        bool     page_addressed;  // 24C04/08/16：页位在 I2C 器件地址中
        uint8_t  page_bits;       // 占用 I2C 地址位的页位数（0/1/2/3）
    };

    static const _model_info _model_table[];

    // ── 数据成员 ────────────────────────────────────────────

    inter_i2c_bus  &_bus;
    eeprom_config   _cfg;
    uint8_t         _i2c_base;      // 基础 7-bit 器件地址（含引脚配置，不含页位）
    uint16_t        _capacity;
    uint8_t         _page_size;
    bool            _dual_addr;
    bool            _page_addressed;
    uint8_t         _page_bits;
    bool            _initialized;

    // ── L1 内部方法 ────────────────────────────────────────

    /// 发送内存地址并等待 ACK（处理页地址型号的 I2C 地址重组）
    /// 调用后总线保持在：已发完地址字节，等待数据的状态
    /// 返回 false 表示通信失败，调用方负责发 STOP
    bool _begin_write(uint16_t mem_addr);

    /// 发送"伪写"设置地址指针，然后重复 START 进入读模式
    /// 调用后总线保持在：已进入读模式，等待 read_byte
    /// 返回 false 表示通信失败，调用方负责发 STOP
    bool _begin_read(uint16_t mem_addr);

    /// ACK 轮询：等待内部写入周期完成
    bool _wait_write_cycle();

    /// 计算包含页位的 I2C 器件地址
    uint8_t _page_device_addr(uint16_t mem_addr) const;

    /// 计算页内偏移（对于页寻址型号，即 mem_addr % 256）
    uint8_t _page_offset(uint16_t mem_addr) const;

    // ── L2 内部方法 ────────────────────────────────────────

    /// 内部跨页写入循环
    bool _write_paged(uint16_t addr, const uint8_t *buf, uint16_t len);

    // ── L3 内部静态方法 ────────────────────────────────────

    /// CRC-16-CCITT (polynomial 0x1021, initial 0xFFFF)
    static uint16_t _crc16(const uint8_t *data, uint16_t len);
    static uint16_t _crc16_update(uint16_t crc, uint8_t byte);
};


// ============================================================================
//  L3 模板实现（必须在头文件）
// ============================================================================

template<typename T>
bool device_eeprom::read_config(uint16_t base_addr, T &data, uint8_t expected_version)
{
    constexpr uint16_t total = config_storage_size<T>();
    uint8_t buf[total];

    // 读取整个配置块
    if (!read(base_addr, buf, total))
        return false;

    // 校验 magic
    uint16_t magic = (buf[0] << 8) | buf[1];
    if (magic != CONFIG_MAGIC)
        return false;

    // 校验版本
    if (buf[2] != expected_version)
        return false;

    // 校验数据长度
    uint16_t stored_len = (buf[3] << 8) | buf[4];
    if (stored_len != sizeof(T))
        return false;

    // 校验 CRC16（覆盖 magic + version + length + data）
    uint16_t crc_stored  = (buf[5 + sizeof(T)] << 8) | buf[5 + sizeof(T) + 1];
    uint16_t crc_calc    = _crc16(buf, 5 + sizeof(T));

    if (crc_stored != crc_calc)
        return false;

    // 提取数据（用 memcpy 避免对齐问题）
    for (uint16_t i = 0; i < sizeof(T); i++)
        reinterpret_cast<uint8_t *>(&data)[i] = buf[5 + i];

    return true;
}


template<typename T>
bool device_eeprom::write_config(uint16_t base_addr, const T &data, uint8_t version)
{
    constexpr uint16_t total = config_storage_size<T>();
    uint8_t buf[total];

    // 构建 header
    buf[0] = (CONFIG_MAGIC >> 8) & 0xFF;
    buf[1] = CONFIG_MAGIC & 0xFF;
    buf[2] = version;
    buf[3] = (sizeof(T) >> 8) & 0xFF;
    buf[4] = sizeof(T) & 0xFF;

    // 拷贝数据
    for (uint16_t i = 0; i < sizeof(T); i++)
        buf[5 + i] = reinterpret_cast<const uint8_t *>(&data)[i];

    // 计算并追加 CRC16
    uint16_t crc = _crc16(buf, 5 + sizeof(T));
    buf[5 + sizeof(T)]     = (crc >> 8) & 0xFF;
    buf[5 + sizeof(T) + 1] = crc & 0xFF;

    // 安全写入（自动回读校验）
    return write_verified(base_addr, buf, total);
}


template<typename T>
bool device_eeprom::verify_config(uint16_t base_addr, uint8_t expected_version)
{
    constexpr uint16_t total = config_storage_size<T>();
    uint8_t buf[total];

    if (!read(base_addr, buf, total))
        return false;

    // 校验 magic
    uint16_t magic = (buf[0] << 8) | buf[1];
    if (magic != CONFIG_MAGIC)
        return false;

    // 校验版本
    if (buf[2] != expected_version)
        return false;

    // 校验数据长度
    uint16_t stored_len = (buf[3] << 8) | buf[4];
    if (stored_len != sizeof(T))
        return false;

    // 校验 CRC16
    uint16_t crc_stored = (buf[5 + sizeof(T)] << 8) | buf[5 + sizeof(T) + 1];
    uint16_t crc_calc   = _crc16(buf, 5 + sizeof(T));

    return crc_stored == crc_calc;
}


#endif /* __cplusplus */
