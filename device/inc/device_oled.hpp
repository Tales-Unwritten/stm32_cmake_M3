#pragma once

#include <cstdint>
#include "inter_i2c_dev.hpp"

/**
 * @brief SSD1306 OLED 显示驱动（128×64 单色，I2C 接口）
 *
 * 通信依赖 inter_i2c_dev（软/硬 I2C 均可）。类本身不管理总线生命周期，
 * 调用者负责构造好 inter_i2c_dev 并保证其生存期长于 Oled 对象。
 *
 * 典型用法：
 * @code
 *   inter_i2c_bus bus(cfg, 5);   bus.init();
 *   inter_i2c_dev dev(&bus, 0x3C);   // 7 位地址，内部左移 1 位
 *   Oled oled(dev);
 *   oled.init();
 *   oled.print(0, 0, "Hello", Oled::Font::F8x16);
 * @endcode
 */
class Oled
{
public:
    /** 字体（与原 C 版 oled_font_t 语义一致） */
    enum class Font : uint8_t
    {
        F6x8  = 0,   ///< 6×8  ASCII，每行最多 21 字符
        F8x16 = 1,   ///< 8×16 ASCII，每行最多 16 字符
    };

    /** SSD1306 8 位从机地址（SA0 拉高 → 7 位 0x3C << 1 = 0x78） */
    static constexpr uint8_t kAddr8Bit = 0x78;

    explicit Oled(inter_i2c_dev &dev) noexcept;
    ~Oled() = default;

    Oled(const Oled &)            = delete;
    Oled &operator=(const Oled &) = delete;
    Oled(Oled &&)                 = delete;
    Oled &operator=(Oled &&)      = delete;

    // ── 生命周期 ──────────────────────────────────────────
    /** @brief 上电初始化序列（须在绘制前调用一次） */
    void init();

    // ── 电源 / 清屏 ───────────────────────────────────────
    void clear();
    void displayOn();
    void displayOff();

    // ── 文字渲染 ──────────────────────────────────────────
    /**
     * @brief 显示以 '\0' 结尾的 ASCII 字符串
     * @param x  起始字符格：F6x8 每格 6px，F8x16 每格 8px
     * @param y  起始逻辑行：F6x8 为 0–7（页），F8x16 为 0–3（双页）
     */
    void print(uint8_t x, uint8_t y, const char *str, Font font);

    /** @brief 显示有符号整数 (-2147483648 ~ 2147483647) */
    void printInt(uint8_t x, uint8_t y, int32_t num, Font font);

    /** @brief 显示无符号整数 (0 ~ 4294967295) */
    void printUint(uint8_t x, uint8_t y, uint32_t num, Font font);

    // ── 图形 ──────────────────────────────────────────────
    /**
     * @brief 绘制 16×16 图元（汉字/图标）
     * @param glyphIndex 索引至 F16x16[] 中的图元（每个图元 32 字节）
     */
    void drawGlyph(uint8_t x, uint8_t y, uint8_t glyphIndex);

    /**
     * @brief 绘制位图
     * @param x       起始列 (0–127)
     * @param y       起始页 (0–7)
     * @param width   像素列数
     * @param height  页数（每页 8 像素）
     * @param bitmap  行优先排列的位图数据
     */
    void drawBitmap(uint8_t x, uint8_t y,
                    uint8_t width, uint8_t height,
                    const uint8_t *bitmap);

private:
    // SSD1306 控制字节
    static constexpr uint8_t kCtrlCmd  = 0x00;   ///< 后续字节为命令
    static constexpr uint8_t kCtrlData = 0x40;   ///< 后续字节为 GDDRAM 数据

    // ── 底层原语 ─────────────────────────────────────────
    void writeCmd(uint8_t cmd);
    /** @brief 批量写 GDDRAM 数据，内部按 8 字节分块（freedom_write 上限） */
    void writeData(const uint8_t *data, uint16_t len);
    /** @brief 将 GDDRAM 写光标设置到 (col, page) */
    void setCursor(uint8_t col, uint8_t page);

    // 内部绘图辅助
    void drawAscii8x16(uint8_t x, uint8_t y, const char *str);
    void drawAscii6x8 (uint8_t x, uint8_t y, const char *str);

    inter_i2c_dev &_dev;   ///< 外部 I2C 设备对象（不拥有）
};