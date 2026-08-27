/**
 * @file    iap_param.cpp
 * @brief   参数区双扇区日志读写（Param0 / Param1）
 *
 * 存储布局：
 *   每扇区 128KB，槽大小 512B（iap_param_t）→ 每扇区 256 槽，顺序写入（seq 递增）
 *   当前扇区写满或下一槽为脏数据时：擦除另一扇区，从槽 0 重新写
 *
 * 可靠性：
 *   - 读取：扫描两扇区所有已写槽，取「seq 最大且 CRC16 有效」的记录
 *   - 掉电安全：擦除瞬间断电时，最新有效记录仍在未被擦除的扇区中
 *   - 半写记录：CRC16 校验失败被跳过，自动回退到上一条有效记录
 *   - 擦除频率：256 槽 / 每次升级约 6~8 次写 ≈ 每 30~40 次升级才擦除 1 次
 *
 * 调用方职责：
 *   - 使用前必须完成 flash_port::init（安全区至少覆盖 Param0/Param1 两扇区）
 *   - 擦写期间自行处理中断（ISR 取指安全）；boot 与 app 均可调用
 */

#include "iap_param.hpp"
#include "iap_conf.hpp"
#include "iap_crc.hpp"
#include "inter_flash.hpp"
#include "inter_wdt.hpp"

#include <cstddef>
#include <cstring>

/* ══════════════════════════════════════════════════════════
 *  槽布局
 * ══════════════════════════════════════════════════════════ */

#define IAP_PARAM_SLOTS  (IAP_PARAM_SIZE / IAP_PARAM_RECORD_SIZE)  /* 256 */

static const uint32_t s_sectors[2] = { IAP_PARAM0_ADDR, IAP_PARAM1_ADDR };

/** 记录有效性：magic + CRC16（覆盖 crc16 字段之前全部字节） */
static bool iap_param_valid(const iap_param_t *rec)
{
    if (rec->magic != IAP_PARAM_MAGIC)
        return false;
    uint16_t crc = Iap_Crc16((const uint8_t *)rec, offsetof(iap_param_t, crc16));
    return crc == rec->crc16;
}

/**
 * @brief 扫描两扇区，定位「seq 最大且有效」的记录
 * @return true=找到（out/sector_idx/slot_idx 输出）
 */
static bool iap_find_latest(iap_param_t *out, uint32_t *sector_idx, uint32_t *slot_idx)
{
    iap_param_t best;
    bool        found   = false;
    uint32_t    best_seq = 0;

    for (uint32_t s = 0; s < 2; s++)
    {
        const uint32_t base = s_sectors[s];
        for (uint32_t i = 0; i < IAP_PARAM_SLOTS; i++)
        {
            const uint32_t slot_addr = base + i * IAP_PARAM_RECORD_SIZE;

            /* 未写槽（全 0xFF）：读 magic 字即可跳过，避免无谓扫描 */
            if (flash_port::read_word(slot_addr) == 0xFFFFFFFFu)
                continue;

            iap_param_t rec;
            flash_port::read_bytes(slot_addr, (uint8_t *)&rec, sizeof(rec));
            if (iap_param_valid(&rec) && rec.seq > best_seq)
            {
                best     = rec;
                best_seq = rec.seq;
                found    = true;
                if (sector_idx) *sector_idx = s;
                if (slot_idx)   *slot_idx   = i;
            }
        }
    }

    if (found)
        memcpy(out, &best, sizeof(best));
    return found;
}

/* ══════════════════════════════════════════════════════════
 *  公共接口
 * ══════════════════════════════════════════════════════════ */

bool Iap_Param_Read(iap_param_t *out)
{
    if (out == nullptr)
        return false;
    return iap_find_latest(out, nullptr, nullptr);
}

bool Iap_Param_Write(const iap_param_t *rec_in)
{
    if (rec_in == nullptr)
        return false;

    iap_param_t rec = *rec_in;

    iap_param_t latest;
    uint32_t    latest_sector = 0;
    uint32_t    latest_slot   = 0;
    const bool  has_latest    = iap_find_latest(&latest, &latest_sector, &latest_slot);

    rec.seq   = has_latest ? (latest.seq + 1u) : 1u;
    rec.crc16 = Iap_Crc16((const uint8_t *)&rec, offsetof(iap_param_t, crc16));

    uint32_t target_sector;
    uint32_t target_slot;

    if (!has_latest)
    {
        /* 无有效记录（空扇区/全脏）：擦扇区 0，从槽 0 写 */
        target_sector = 0;
        target_slot   = 0;
        wdt_port::iwdg_feed();   /* 擦除 1-2s，先喂狗防 IWDG 超时 */
        if (!flash_port::erase(s_sectors[0]))
            return false;
    }
    else if ((latest_slot + 1u) < IAP_PARAM_SLOTS &&
             flash_port::read_word(s_sectors[latest_sector] + (latest_slot + 1u) * IAP_PARAM_RECORD_SIZE) == 0xFFFFFFFFu)
    {
        /* 最新扇区的下一槽干净：顺序写 */
        target_sector = latest_sector;
        target_slot   = latest_slot + 1u;
    }
    else
    {
        /* 当前扇区写满或下一槽为脏数据（掉电残留）：擦另一扇区，从槽 0 写 */
        target_sector = 1u - latest_sector;
        target_slot   = 0;
        wdt_port::iwdg_feed();   /* 擦除 1-2s，先喂狗防 IWDG 超时 */
        if (!flash_port::erase(s_sectors[target_sector]))
            return false;
    }

    const uint32_t addr = s_sectors[target_sector] + target_slot * IAP_PARAM_RECORD_SIZE;
    if (!flash_port::write_bytes(addr, (const uint8_t *)&rec, sizeof(rec)))
        return false;

    /* 写后验证：读回并校验 */
    iap_param_t check;
    flash_port::read_bytes(addr, (uint8_t *)&check, sizeof(check));
    return iap_param_valid(&check) && check.seq == rec.seq;
}
