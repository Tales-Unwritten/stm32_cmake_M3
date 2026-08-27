#pragma once
// ============================================================
// @platform GD32F4xx
// ============================================================

#ifdef __cplusplus

#include <cstdint>

/* ════ Flash 容量配置（必须与链接脚本 FLASH LENGTH 一致） ════
 *   1024 = 1MB（单 bank，12 扇区）
 *   2048 = 2MB（双 bank，24 扇区，当前板）
 *   3072 = 3MB（双 bank + 尾部 4×256KB，28 扇区）
 * 换芯片密度时修改此宏，并同步 gd32f4xxXx_flash.ld 的 LENGTH。 */
#ifndef FLASH_CAPACITY_KB
#define FLASH_CAPACITY_KB 2048
#endif

/**
 * @brief 内部 Flash 安全读写
 *
 * 安全机制：所有写入/擦除操作限制在 init() 指定的安全区域内。
 * 超出区域的操作直接返回 false，防止误擦固件。
 *
 * 使用示例：
 *   // 使用 Flash 尾部 16KB 作为数据存储区
 *   flash_port::init(0x08020000, 0x20000);  // 安全区：0x08020000~0x0803FFFF（整 128KB 扇区）
 *
 *   flash_port::erase(0x08020000);           // 擦除整个扇区（须整扇区在安全区内）
 *   flash_port::write(0x08020000, 0xA5);     // 写 byte
 *   uint8_t val = flash_port::read_byte(0x08020000);
 */
class flash_port {
public:
    // ── 生命周期 ──────────────────────────────────────────

    /** @brief 初始化 Flash 控制器并设置安全区域
     *  @param safe_start  允许操作的起始地址
     *  @param safe_size   安全区域大小（字节）
     */
    static void init(uint32_t safe_start, uint32_t safe_size);

    /** @brief 锁定 Flash（操作完成后调用） */
    static void lock();

    // ── 擦除 ──────────────────────────────────────────────

    /** @brief 擦除扇区（仅当 addr 在安全区域内）
     *  @param addr 扇区内任意地址
     *  @return 成功返回 true
     */
    static bool erase(uint32_t addr);

    // ── 写入 ──────────────────────────────────────────────

    /** @brief 写 32-bit 字 */
    static bool write_word(uint32_t addr, uint32_t data);

    /** @brief 写 byte */
    static bool write_byte(uint32_t addr, uint8_t data);

    /** @brief 批量写 byte 数组（最多一页 256 字节） */
    static bool write_bytes(uint32_t addr, const uint8_t *data, uint16_t len);

    // ── 读取（直接内存访问，无需解锁） ────────────────────

    static uint8_t  read_byte(uint32_t addr);
    static uint32_t read_word(uint32_t addr);
    static void     read_bytes(uint32_t addr, uint8_t *buf, uint16_t len);

    // ── 查询 ──────────────────────────────────────────────

    static uint32_t safe_start() { return _safe_start; }
    static uint32_t safe_end()   { return _safe_end; }
    static uint32_t safe_size()  { return _safe_end - _safe_start; }

private:
    static bool _in_range(uint32_t addr, uint32_t len);
    static bool _wait_busy();

    static uint32_t _safe_start;
    static uint32_t _safe_end;

public:
    /* ── 调试诊断（最近一次失败原因；无失败时均为 0） ── */
    static uint32_t diag_fail_addr;   /* 失败地址 */
    static uint8_t  diag_fail_stage;  /* 0=无 1=查表 2=安全区 3=擦除 4=等待 */
};

#endif
