// ============================================================
// @platform GD32F4xx（当前平台）
// ============================================================

#include "inter_flash.hpp"

#include <array>

// 必须先包含 gd32f4xx.h：其 extern "C" 块包裹 libopt 下所有外设声明，
// 否则 gd32f4xx_fmc.h 的 fmc_* 函数在 C++ 下会被名字修饰导致链接失败
#include "gd32f4xx.h"
#include "gd32f4xx_fmc.h"

uint32_t flash_port::_safe_start = 0;
uint32_t flash_port::_safe_end = 0;
uint32_t flash_port::diag_fail_addr = 0;
uint8_t flash_port::diag_fail_stage = 0;

// ════════════════════════════════════════════════════════════
//  GD32F4xx 扇区布局（板上实测确认，勿用统一公式替代）
//    每个 1MB bank：4×16KB + 1×64KB + 7×128KB
//    3MB 芯片：双 bank 之后追加 4×256KB
//  容量由 FLASH_CAPACITY_KB（inter_flash.hpp）决定，编译期生成表
// ════════════════════════════════════════════════════════════
struct FmcSectorInfo
{
    uint32_t base;
    uint32_t size;
    uint8_t sector;
};

static constexpr uint32_t kFlashBase = 0x08000000u;
static constexpr uint32_t kBankCount = FLASH_CAPACITY_KB / 1024u;                         // 1 / 2 / 3
static constexpr uint32_t kBankSectors = 12;                                              // 4×16K + 1×64K + 7×128K
static constexpr uint32_t kSectorCount = (kBankCount >= 3) ? (2 * kBankSectors + 4)       // 3MB：24 + 4×256K
                                                           : (kBankCount * kBankSectors); // 1MB：12；2MB：24

static constexpr std::array<FmcSectorInfo, kSectorCount> make_sector_table()
{
    std::array<FmcSectorInfo, kSectorCount> t{};
    uint32_t base = kFlashBase;
    uint32_t i = 0;
    for (uint32_t b = 0; b < kBankCount && b < 2; b++)
    {
        for (uint32_t s = 0; s < 4; s++)
        {
            t[i] = {base, 16u * 1024, (uint8_t)i};
            i++;
            base += 16u * 1024;
        }
        t[i] = {base, 64u * 1024, (uint8_t)i};
        i++;
        base += 64u * 1024;
        for (uint32_t s = 0; s < 7; s++)
        {
            t[i] = {base, 128u * 1024, (uint8_t)i};
            i++;
            base += 128u * 1024;
        }
    }
    if (kBankCount >= 3)
    {
        for (uint32_t s = 0; s < 4; s++)
        {
            t[i] = {base, 256u * 1024, (uint8_t)i};
            i++;
            base += 256u * 1024;
        }
    }
    return t;
}

static constexpr std::array<FmcSectorInfo, kSectorCount> kSectors = make_sector_table();

static_assert(FLASH_CAPACITY_KB == 1024 || FLASH_CAPACITY_KB == 2048 || FLASH_CAPACITY_KB == 3072,
              "FLASH_CAPACITY_KB 仅支持 1024 / 2048 / 3072");

static constexpr uint32_t table_total()
{
    uint32_t sum = 0;
    for (const auto &s : kSectors)
        sum += s.size;
    return sum;
}
static_assert(table_total() == FLASH_CAPACITY_KB * 1024u, "扇区表总大小与 FLASH_CAPACITY_KB 不一致");

static const FmcSectorInfo *find_sector(uint32_t addr)
{
    for (const auto &s : kSectors)
    {
        if (addr >= s.base && addr < s.base + s.size)
            return &s;
    }
    return nullptr;
}

/**
 * @brief 扇区号 → FMC SN 位域编码
 * GD32F4xx 的 SN 编码非连续：bank0 = 0~11，bank1 = 16~27（扇区号+4，12~15 保留）。
 * 依据 gd32f4xx_fmc.h：CTL_SECTOR_NUMBER_12 = CTL_SN(16) ... CTL_SECTOR_NUMBER_23 = CTL_SN(27)
 */
static uint32_t sector_to_sn(uint8_t sector)
{
    return (sector >= 12u) ? (uint32_t)(sector + 4u) : (uint32_t)sector;
}

void flash_port::init(uint32_t safe_start, uint32_t safe_size)
{
    _safe_start = safe_start;
    _safe_end = safe_start + safe_size;
    fmc_unlock();
}

void flash_port::lock()
{
    fmc_lock();
}

bool flash_port::_in_range(uint32_t addr, uint32_t len)
{
    // 防整数溢出：先检查 len 是否超出剩余空间
    if (len > (_safe_end - _safe_start))
        return false;
    return (addr >= _safe_start) && ((addr + len) <= _safe_end);
}

bool flash_port::_wait_busy()
{
    uint32_t timeout = 0xFFFF;
    while (fmc_flag_get(FMC_FLAG_BUSY))
    {
        if (--timeout == 0)
        {
            fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
            return false;
        }
    }
    return true;
}

// ════════════════════════════════════════════════════════════
//  擦除
// ════════════════════════════════════════════════════════════

bool flash_port::erase(uint32_t addr)
{
    // 扇区擦除会清除整个扇区，需确保整个扇区在安全区内
    // 扇区边界查表（每 1MB bank：4×16K+64K+7×128K；3MB 尾部 4×256K），
    // 避免按统一 128KB 计算导致 bank1 起始 16K/64K 扇区被误判
    const FmcSectorInfo *sec = find_sector(addr);
    if (sec == nullptr)
    {
        diag_fail_addr = addr;
        diag_fail_stage = 1;
        return false;
    }
    if (sec->base < _safe_start || (sec->base + sec->size) > _safe_end)
    {
        diag_fail_addr = addr;
        diag_fail_stage = 2;
        return false;
    }

    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);

    // 必须用扇区擦除（fmc_sector_erase）：fmc_page_erase 仅擦 4KB 页，
    // 与扇区表（16K/64K/128K）粒度不匹配，会导致同扇区其余部分残留旧数据。
    // 入参须经 sector_to_sn() 编码（bank1 扇区号+4）进 SN 位域，
    // 传裸扇区号会映射到错误扇区（如 9 -> SN=1 误擦 boot 区，12 -> 保留值擦除失败）
    if (fmc_sector_erase(CTL_SN(sector_to_sn(sec->sector))) != FMC_READY)
    {
        diag_fail_addr = addr;
        diag_fail_stage = 3;
        return false;
    }
    if (!_wait_busy())
    {
        diag_fail_addr = addr;
        diag_fail_stage = 4;
        return false;
    }
    return true;
}

// ════════════════════════════════════════════════════════════
//  写入
// ════════════════════════════════════════════════════════════

bool flash_port::write_word(uint32_t addr, uint32_t data)
{
    if (!_in_range(addr, 4))
        return false; // 安全检查

    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);

    if (fmc_word_program(addr, data) != FMC_READY)
        return false;
    return _wait_busy();
}

bool flash_port::write_byte(uint32_t addr, uint8_t data)
{
    if (!_in_range(addr, 1))
        return false;

    // Flash 只能将 bit 从 1 编程为 0，不能从 0 变为 1（需先擦除）
    uint32_t word_addr = addr & ~0x3;
    uint32_t shift = (addr & 0x3) * 8;
    uint32_t old_word = *(volatile uint32_t *)word_addr;
    uint32_t old_byte = (old_word >> shift) & 0xFF;
    uint32_t new_byte = (uint32_t)data;

    // 如果新值试图将 old_byte 中的某个 0 bit 改为 1，Flash 无法做到
    if ((old_byte & new_byte) != new_byte)
        return false;

    uint32_t new_word = (old_word & ~(0xFFUL << shift)) | (new_byte << shift);
    return write_word(word_addr, new_word);
}

bool flash_port::write_bytes(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (!_in_range(addr, len))
        return false;

    for (uint16_t i = 0; i < len; i++)
    {
        if (!write_byte(addr + i, data[i]))
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
