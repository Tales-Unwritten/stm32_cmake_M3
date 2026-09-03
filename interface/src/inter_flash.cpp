// ============================================================
// @platform STM32F103xx（从 GD32F4xx 版本移植，使用 HAL 库）
// ============================================================

#include "inter_flash.hpp"

// STM32F1xx HAL 库头文件
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash_ex.h"

uint32_t flash_port::_safe_start = 0;
uint32_t flash_port::_safe_end = 0;
uint32_t flash_port::diag_fail_addr = 0;
uint8_t flash_port::diag_fail_stage = 0;

// ════════════════════════════════════════════════════════════
//  STM32F10xxx Flash 页布局
//    与 GD32F4xx 不同：同一芯片内所有页大小一致（1KB 或 2KB，由密度决定），
//    不存在混合扇区大小，因此不需要 GD32 版本那种"扇区查找表 + sector_to_sn()
//    位域编码"，直接按 FLASH_PAGE_SIZE 整除即可定位页首地址。
// ════════════════════════════════════════════════════════════
static constexpr uint32_t kFlashBase = 0x08000000u;
static constexpr uint32_t kPageSize = FLASH_PAGE_SIZE; // 由 HAL 库头文件根据芯片型号定义
static constexpr uint32_t kFlashTotalBytes = FLASH_CAPACITY_KB * 1024u;

// 静态检查：页大小必须是 1024 或 2048（STM32F1 系列仅这两种情况）
static_assert(kPageSize == 1024u || kPageSize == 2048u,
              "FLASH_PAGE_SIZE 仅支持 1024（小/中容量）或 2048（大容量/互联型/XL-density）");
static_assert((FLASH_CAPACITY_KB * 1024u) % kPageSize == 0, "FLASH_CAPACITY_KB 必须是 FLASH_PAGE_SIZE 的整数倍");

/**
 * @brief 计算 addr 所在页的页首地址
 * @return true  = addr 落在本芯片 Flash 地址范围内，page_base 有效
 *         false = addr 超出芯片总容量范围
 * @note 暂不支持 XL-density（768KB~1MB）芯片的双 Bank：这类芯片在 0x08080000
 *       处还有第二个独立 FPEC（KEYR2/SR2/CR2/AR2，寄存器基址偏移 +0x40），
 *       跨过该边界的地址需要切换到 Bank2 的寄存器组，此实现仅覆盖单 Bank
 *       （容量 ≤512KB）场景，与常见的 STM32F103C8/RB/RC/RE/RG 等主流型号一致。
 */
static bool page_base_of(uint32_t addr, uint32_t &page_base)
{
    if (addr < kFlashBase || addr >= kFlashBase + kFlashTotalBytes)
        return false;
    uint32_t offset = addr - kFlashBase;
    page_base = kFlashBase + (offset / kPageSize) * kPageSize;
    return true;
}

void flash_port::init(uint32_t safe_start, uint32_t safe_size)
{
    _safe_start = safe_start;
    _safe_end = safe_start + safe_size;
    HAL_FLASH_Unlock();
}

void flash_port::lock()
{
    HAL_FLASH_Lock();
}

bool flash_port::_in_range(uint32_t addr, uint32_t len)
{
    // 防整数溢出：先检查 len 是否超出剩余空间
    if (len > (_safe_end - _safe_start))
        return false;
    return (addr >= _safe_start) && ((addr + len) <= _safe_end);
}

// ════════════════════════════════════════════════════════════
//  擦除
// ════════════════════════════════════════════════════════════

bool flash_port::erase(uint32_t addr)
{
    // 页擦除会清除整页，需确保整页都在安全区内
    uint32_t page_base;
    if (!page_base_of(addr, page_base))
    {
        diag_fail_addr = addr;
        diag_fail_stage = 1;
        return false;
    }
    if (page_base < _safe_start || (page_base + kPageSize) > _safe_end)
    {
        diag_fail_addr = addr;
        diag_fail_stage = 2;
        return false;
    }

    // 清除可能存在的错误标志（HAL 库中的宏）
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

    // 使用 HAL 库的页擦除函数
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = page_base;
    erase_init.NbPages = 1;

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK)
    {
        diag_fail_addr = addr;
        // 区分超时和其他错误
        diag_fail_stage = (status == HAL_TIMEOUT) ? 4 : 3;
        return false;
    }
    return true;
}

// ════════════════════════════════════════════════════════════
//  写入
// ════════════════════════════════════════════════════════════

bool flash_port::_write_halfword(uint32_t addr, uint16_t data)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, data);
    return (status == HAL_OK);
}

bool flash_port::write_word(uint32_t addr, uint32_t data)
{
    if (!_in_range(addr, 4))
        return false; // 安全检查

    // ── 4 字节对齐保护（为什么必须加）： ─────────────────────
    //  1. F103 FPEC 只接受 16-bit 半字编程，且目标地址必须 2 字节对齐；
    //     非半字长的写入会触发总线错误（PM0075 §2.3.3）。
    //  2. HAL 的 WORD 编程 = 对 addr 与 addr+2 两个半字串行编程，
    //     所以 write_word 的语义要求 addr 本身 4 字节对齐。
    //  3. 若调用方传入非 4 字节对齐地址（典型：奇数地址），第一个半字将
    //     落在奇地址上——实测该操作会让 FPEC 进入 BSY=1 卡死状态，
    //     随后 CPU 任何 Flash 取指都被永久阻塞 → 整机死机，
    //     连 HAL 错误码都得不到（比返回 false 严重得多）。
    //  因此这里必须在进 HAL 前拦截，把“死机级风险”降级为
    //  “可预期的 false + diag_fail_stage=6”。
    if (addr & 0x3u)
    {
        diag_fail_addr = addr;
        diag_fail_stage = 6; // 6 = 地址未对齐（write_word 需 4 字节对齐）
        return false;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

    // HAL 库的 FLASH_TYPEPROGRAM_WORD 会内部拆成两个半字编程，并自动处理等待
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data);
    if (status != HAL_OK)
    {
        diag_fail_addr = addr;
        diag_fail_stage = (status == HAL_TIMEOUT) ? 4 : 5; // 5 表示编程失败
        return false;
    }
    return true;
}

bool flash_port::write_byte(uint32_t addr, uint8_t data)
{
    if (!_in_range(addr, 1))
        return false;

    // 平台约束（PM0075 §2.3.3）：F103 FPEC 半字编程前会校验目标半字是否处于
    // 擦除态(0xFFFF)，非擦除态则跳过编程并置 PGERR。因此同一 2B 半字在一次
    // 页擦除周期内只能成功编程一次；要改写必须先整页擦除。
    // 注意：F103 编程粒度是 16-bit half-word（GD32F4xx 是 32-bit word），
    // 所以读-改-写的对齐单位从 4 字节改成了 2 字节。
    uint32_t hw_addr = addr & ~0x1u;
    uint32_t shift = (addr & 0x1u) * 8u;
    uint16_t old_hw = *(volatile uint16_t *)hw_addr;
    uint16_t old_byte = (uint16_t)((old_hw >> shift) & 0xFFu);
    uint16_t new_byte = data;

    // 如果新值试图将 old_byte 中的某个 0 bit 改为 1，Flash 无法做到
    if ((old_byte & new_byte) != new_byte)
        return false;

    uint16_t new_hw = (uint16_t)((old_hw & ~(0xFFu << shift)) | (new_byte << shift));
    return _write_halfword(hw_addr, new_hw);
}

bool flash_port::write_bytes(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (!_in_range(addr, len))
        return false;

    // F103 FPEC：目标半字必须处于擦除态才能编程（PM0075 §2.3.3），
    // 同一 2B 半字每次页擦除周期只能编程一次。因此不能逐字节调用
    // write_byte（相邻字节 = 对同一半字编程两次，第二次必被硬件跳过）。
    // 这里把缓冲区相邻字节合成半字，每个半字只编程一次。
    // 约定：调用前必须先整页擦除；孤立的头/尾单字节按伙伴字节=0xFF
    // （擦除态）合成半字。
    uint32_t a = addr;
    const uint8_t *d = data;
    uint16_t n = len;

    // 奇数起始：头 1 字节与擦除态高字节合成半字
    if (a & 0x1u)
    {
        uint16_t hw = (uint16_t)(0xFF00u | d[0]);
        if (!_write_halfword(a & ~0x1u, hw))
            return false;
        a++;
        d++;
        n--;
    }
    // 成对字节：每次合成一个完整半字并编程一次
    while (n >= 2)
    {
        uint16_t hw = (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
        if (!_write_halfword(a, hw))
            return false;
        a += 2;
        d += 2;
        n -= 2;
    }
    // 奇数长度：尾部 1 字节（此时 a 必为偶地址）与擦除态高字节合成
    if (n == 1)
    {
        uint16_t hw = (uint16_t)(d[0] | 0xFF00u);
        if (!_write_halfword(a, hw))
            return false;
    }
    return true;
}

// ════════════════════════════════════════════════════════════
//  读取（直接内存访问）
// ════════════════════════════════════════════════════════════

uint8_t flash_port::read_byte(uint32_t addr)
{
    return *(volatile uint8_t *)addr;
}

uint32_t flash_port::read_word(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

void flash_port::read_bytes(uint32_t addr, uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        buf[i] = *(volatile uint8_t *)(addr + i);
    }
}
