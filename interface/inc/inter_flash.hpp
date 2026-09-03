#pragma once
// ============================================================
// @platform STM32F103xx（STM32F10xxx / Cortex-M3，基于 STM32F1xx HAL 库）
// ============================================================
#ifdef __cplusplus
#include <cstdint>

/* ════ Flash 容量配置（必须与实际芯片 & 链接脚本 FLASH LENGTH 一致） ════
 *   FLASH_CAPACITY_KB : 芯片总 Flash 容量，单位 KB，仅用于本驱动自身的
 *                        地址范围合法性检查（见 .cpp 里的 page_base_of()）。
 *
 *   页大小（擦除粒度）不需要在这里配置：HAL 库会根据你工程里定义的芯片型号宏
 *   （例如 STM32F103xB / STM32F103xE，通常由 IDE 工程设置或编译器 -D 选项传入）
 *   自动在 stm32f1xx_hal_flash_ex.h 中定义好 FLASH_PAGE_SIZE 常量（1KB 或 2KB）。
 *   本驱动直接使用该宏，见 .cpp。
 *
 *   换芯片型号时：
 *     1. 改这里的 FLASH_CAPACITY_KB
 *     2. 确认工程里的芯片型号宏（STM32F103xx 系列）改对了，FLASH_PAGE_SIZE
 *        会跟着自动变，不需要手动同步
 *     3. 同步链接脚本 FLASH LENGTH
 *   默认值对应最常见的 STM32F103C8T6 / RCT6（中容量，128KB，1KB 页）。 */
#ifndef FLASH_CAPACITY_KB
#define FLASH_CAPACITY_KB 128
#endif

/**
 * @brief 内部 Flash 安全读写（STM32F103 / STM32F1xx HAL 库版本）
 *
 * 安全机制：所有写入/擦除操作限制在 init() 指定的安全区域内。
 * 超出区域的操作直接返回 false，防止误擦固件。
 *
 * 与 GD32F4xx 版本的关键实现差异（公开 API 未变）：
 *   1. 编程粒度：F103 FPEC 只支持 16-bit half-word 编程（GD32F4xx 为 32-bit word），
 *      write_byte() 的读-改-写对齐单位相应从 4 字节改为 2 字节。
 *   2. 擦除粒度：F103 同一芯片内所有页大小一致（1KB 或 2KB，由密度决定），不存在
 *      GD32F4xx 那种混合扇区大小，因此不需要扇区查找表，直接按 HAL 提供的
 *      FLASH_PAGE_SIZE 整除即可定位页首地址。
 *   3. 无 SN 位域编码：F103 擦除时通过 FLASH_EraseInitTypeDef 直接指定页首地址，
 *      没有 GD32F4xx "扇区号 → SN 位域" 的非连续编码步骤。
 *   4. 使用 HAL 库（HAL_FLASH_Program / HAL_FLASHEx_Erase）：这些函数内部已经包含
 *      忙等待和状态判断，不需要像 GD32 版本那样自己实现 _wait_busy()。
 *
 * 使用示例（假设芯片为 128KB 中容量，1KB 页）：
 *   flash_port::init(0x0801FC00, 0x400);     // 安全区：0x0801FC00~0x0801FFFF（整 1KB 页）
 *   flash_port::erase(0x0801FC00);           // 擦除整个页（须整页在安全区内）
 *   flash_port::write_byte(0x0801FC00, 0xA5);
 *   uint8_t val = flash_port::read_byte(0x0801FC00);
 *
 * ⚠ 平台约束（PM0075 §2.3.3，实测一致）：
 *   F103 半字(16-bit)编程前，FPEC 会校验目标半字处于擦除态(0xFFFF)；
 *   非擦除态则跳过编程并置 PGERR。因此：
 *     · 写入/改写前必须先整页擦除（无字节级擦除，擦除粒度=整页）；
 *     · 同一 2B 半字在每次页擦除周期内只能成功编程一次；
 *     · 不要对一个已擦除周期内写过的地址重复 write_byte/write_bytes。
 *   write_bytes() 内部会合并相邻字节、每个半字只编程一次。
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
    /** @brief 擦除页（仅当 addr 在安全区域内）
     *  @param addr 页内任意地址
     *  @return 成功返回 true
     */
    static bool erase(uint32_t addr);
    // ── 写入 ──────────────────────────────────────────────
    /** @brief 写 32-bit 字（内部拆成两次 16-bit half-word 编程）
     *  @note addr 必须 4 字节对齐；非对齐返回 false 且 diag_fail_stage=6
     *        （非对齐在 F103 上会让 FPEC 卡死，见 .cpp 注释） */
    static bool write_word(uint32_t addr, uint32_t data);
    /** @brief 写 byte（目标半字须处于擦除态；每擦除周期每 2B 半字只可编程一次） */
    static bool write_byte(uint32_t addr, uint8_t data);
    /** @brief 批量写 byte 数组（自动合并相邻字节为半字、每半字只编程一次） */
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
    static bool _write_halfword(uint32_t addr, uint16_t data);
    static uint32_t _safe_start;
    static uint32_t _safe_end;
public:
    /* ── 调试诊断（最近一次失败原因；无失败时均为 0） ── */
    static uint32_t diag_fail_addr;   /* 失败地址 */
    static uint8_t  diag_fail_stage;  /* 0=无 1=地址超出芯片Flash范围 2=超出安全区
                                          3=擦除命令失败(HAL) 4=擦除等待超时
                                          5=编程失败(HAL) 6=write_word地址未对齐(需4B) */
};
#endif