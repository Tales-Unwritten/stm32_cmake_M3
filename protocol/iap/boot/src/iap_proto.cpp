/**
 * @file    iap_proto.cpp
 * @brief   IAP 升级协议实现（boot 侧，USART0 通道）
 *
 * 帧格式：0xA5 0x5A | CMD(1) | LEN(2 LE) | PAYLOAD[LEN] | CRC16(2 LE, 覆盖整帧)
 * 传输语义：逐包停等 + 包序号去重；任何坏帧静默丢弃（主机超时重传）
 *
 * 时序安全设计：
 *   - START：擦除整个目标分区（关中断；主机停等应答，无数据流丢失风险）
 *   - DATA ：写 256B（关中断 ~ms 级；主机停等 ACK）
 *   - 关键状态变更均落参数区（双扇区日志），断电可恢复
 */

#include "iap_boot.hpp"
#include "iap_conf.hpp"
#include "iap_crc.hpp"
#include "iap_image.hpp"
#include "iap_proto.hpp"
#include "inter_flash.hpp"
#include "inter_wdt.hpp"

#include "gd32f4xx.h"
#include "systick.h"

#include <cstring>

/* 帧头：0xA5 0x5A CMD LEN_LO LEN_HI */
#define FRAME_HDR_LEN   5u
#define FRAME_CRC_LEN   2u
#define RESP_EXTRA_MAX  15u   /* QUERY 附加负载上限 */
#define IAP_FRAME_BUF_MAX 512u /* 单帧最大长度（与 usart rx 缓冲一致） */

static void Proto_HandleCmd(uint8_t cmd, const uint8_t *payload, uint16_t plen);

/* ══════════════════════════════════════════════════════════
 *  发送应答帧：[status(1)][next_seq(2)][extra...]
 * ══════════════════════════════════════════════════════════ */

static void Proto_Send(uint8_t cmd, uint8_t status, uint16_t next_seq, const uint8_t *extra, uint8_t extra_len)
{
    if (extra_len > RESP_EXTRA_MAX)
        extra_len = RESP_EXTRA_MAX;

    const uint16_t plen = (uint16_t)(3u + extra_len);
    uint8_t frame[FRAME_HDR_LEN + 3u + RESP_EXTRA_MAX + FRAME_CRC_LEN];

    frame[0] = IAP_FRAME_HEAD0;
    frame[1] = IAP_FRAME_HEAD1;
    frame[2] = cmd;
    frame[3] = (uint8_t)(plen & 0xFFu);
    frame[4] = (uint8_t)(plen >> 8);

    frame[FRAME_HDR_LEN + 0] = status;
    frame[FRAME_HDR_LEN + 1] = (uint8_t)(next_seq & 0xFFu);
    frame[FRAME_HDR_LEN + 2] = (uint8_t)(next_seq >> 8);

    if (extra != nullptr && extra_len > 0)
        memcpy(frame + FRAME_HDR_LEN + 3, extra, extra_len);

    const uint16_t crc = Iap_Crc16(frame, FRAME_HDR_LEN + plen);
    frame[FRAME_HDR_LEN + plen] = (uint8_t)(crc & 0xFFu);
    frame[FRAME_HDR_LEN + plen + 1] = (uint8_t)(crc >> 8);

    if (iap_port_count > 0)
        iap_ports[0].send(frame, FRAME_HDR_LEN + plen + FRAME_CRC_LEN);
}

/* ══════════════════════════════════════════════════════════
 *  分区擦除（按扇区表；与 iap_conf.hpp 分区定义联动，编译期校验）
 * ══════════════════════════════════════════════════════════ */

static_assert(IAP_APP_A_SIZE == 4u * 128u * 1024u, "AppA sector layout mismatch (expect 4x128K)");
static_assert(IAP_APP_B_SIZE == 3u * 128u * 1024u + 4u * 16u * 1024u + 64u * 1024u,
              "AppB sector layout mismatch (expect 3x128K + 4x16K + 64K)");

static bool Erase_Partition(uint8_t slot)
{
    const uint32_t base = Iap_SlotBase(slot);

    if (slot == IAP_SLOT_A)
    {
        /* AppA：4 x 128K @ 0x08020000 */
        for (uint32_t i = 0; i < 4; i++)
        {
            if (!flash_port::erase(base + i * 128u * 1024u))
                return false;
            wdt_port::iwdg_feed();   /* 单扇区擦除 1-2s，间歇喂狗防 IWDG 超时 */
        }
    }
    else
    {
        /* AppB：3 x 128K @ 0x080A0000 + 4 x 16K @ 0x08100000 + 1 x 64K @ 0x08110000 */
        for (uint32_t i = 0; i < 3; i++)
        {
            if (!flash_port::erase(base + i * 128u * 1024u))
                return false;
            wdt_port::iwdg_feed();
        }
        for (uint32_t i = 0; i < 4; i++)
        {
            if (!flash_port::erase(0x08100000u + i * 16u * 1024u))
                return false;
        }
        if (!flash_port::erase(0x08110000u))
            return false;
        wdt_port::iwdg_feed();
    }
    return true;
}

/* ══════════════════════════════════════════════════════════
 *  镜像头一致性（包 0 写入后校验，防错槽/错包）
 * ══════════════════════════════════════════════════════════ */

static bool Check_Header_Consistent(uint32_t base)
{
    iap_image_header_t hdr;
    flash_port::read_bytes(base, (uint8_t *)&hdr, sizeof(hdr));

    if (hdr.magic != IAP_IMAGE_MAGIC)
        return false;
    if (hdr.platform != IAP_PLATFORM_ID)
        return false;
    if (hdr.slot != g_iap_boot.param.pending_slot)
        return false;
    if (hdr.image_len != g_iap_boot.param.image_len)
        return false;
    if (hdr.crc32 != g_iap_boot.param.image_crc32)
        return false;
    return true;
}

/* ══════════════════════════════════════════════════════════
 *  命令分发
 * ══════════════════════════════════════════════════════════ */

static void Proto_HandleCmd(uint8_t cmd, const uint8_t *payload, uint16_t plen)
{
    iap_param_t &p = g_iap_boot.param;

    switch (cmd)
    {
    /* ── QUERY：状态 + 版本 ── */
    case IAP_CMD_QUERY: {
        uint8_t extra[RESP_EXTRA_MAX];
        extra[0] = (uint8_t)p.state;
        extra[1] = p.active_slot;
        extra[2] = p.pending_slot;

        uint32_t boot_ver = IAP_VERSION_PACK();
        uint32_t act_ver = 0u;
        uint32_t oth_ver = 0u;

        /* 读两分区镜像头版本（无需依赖 active 合法性） */
        {
            iap_image_header_t ha, hb;
            flash_port::read_bytes(IAP_APP_A_ADDR, (uint8_t *)&ha, sizeof(ha));
            flash_port::read_bytes(IAP_APP_B_ADDR, (uint8_t *)&hb, sizeof(hb));
            act_ver = (ha.magic == IAP_IMAGE_MAGIC) ? ha.version : 0u;
            oth_ver = (hb.magic == IAP_IMAGE_MAGIC) ? hb.version : 0u;
            if (p.active_slot == IAP_SLOT_B)
            {
                const uint32_t t = act_ver;
                act_ver = oth_ver;
                oth_ver = t;
            }
        }
        memcpy(extra + 3, &boot_ver, 4);
        memcpy(extra + 7, &act_ver, 4);
        memcpy(extra + 11, &oth_ver, 4);

        Proto_Send(cmd, IAP_ACK_OK, g_iap_boot.next_seq, extra, 15);
        break;
    }

    /* ── START：版本 + 长度 + CRC32 + 目标槽 ── */
    case IAP_CMD_START: {
        if (plen != 13)
        {
            Proto_Send(cmd, IAP_ACK_ERR_PARAM, 0, nullptr, 0);
            break;
        }

        uint32_t version;
        uint32_t image_len;
        uint32_t crc32;
        uint8_t slot;
        memcpy(&version, payload, 4);
        memcpy(&image_len, payload + 4, 4);
        memcpy(&crc32, payload + 8, 4);
        slot = payload[12];

        const uint32_t max_size = (slot == IAP_SLOT_A) ? IAP_APP_A_SIZE : IAP_APP_B_SIZE;

        if (slot != IAP_SLOT_A && slot != IAP_SLOT_B)
        {
            Proto_Send(cmd, IAP_ACK_ERR_PARAM, 0, nullptr, 0);
            break;
        }
        if (slot == p.active_slot) /* 禁止升级活动分区（回滚底线） */
        {
            Proto_Send(cmd, IAP_ACK_ERR_SLOT, 0, nullptr, 0);
            break;
        }
        if (image_len < IAP_IMAGE_HEADER_SIZE || image_len > max_size || (image_len & 3u) != 0u)
        {
            Proto_Send(cmd, IAP_ACK_ERR_PARAM, 0, nullptr, 0);
            break;
        }

        /* 擦除整个目标分区（关中断；主机停等应答，无数据流） */
        __disable_irq();
        const bool erased = Erase_Partition(slot);
        __enable_irq();
        if (!erased)
        {
            Proto_Send(cmd, IAP_ACK_ERR_BUSY, 0, nullptr, 0);
            break;
        }

        /* 记录传输状态（覆盖上次半包） */
        p.state = IAP_STATE_TRANSFERRING;
        p.pending_slot = slot;
        p.image_len = image_len;
        p.image_crc32 = crc32;
        p.image_version = version;
        p.attempt_count = 0;
        p.success_count = 0;
        p.boot_ok = 0;
        p.attempt_slot = 0;
        (void)Iap_Param_Write(&p);

        g_iap_boot.target_base = Iap_SlotBase(slot);
        g_iap_boot.next_seq = 0;
        g_iap_boot.verified = false;

        Proto_Send(cmd, IAP_ACK_OK, g_iap_boot.next_seq, nullptr, 0);
        break;
    }

    /* ── DATA：包序号 + 数据 ── */
    case IAP_CMD_DATA: {
        if (plen < 2)
        {
            Proto_Send(cmd, IAP_ACK_ERR_PARAM, g_iap_boot.next_seq, nullptr, 0);
            break;
        }
        if (p.state != IAP_STATE_TRANSFERRING)
        {
            Proto_Send(cmd, IAP_ACK_ERR_STATE, g_iap_boot.next_seq, nullptr, 0);
            break;
        }

        const uint16_t seq = (uint16_t)(payload[0] | ((uint16_t)payload[1] << 8));
        const uint16_t dlen = plen - 2;

        /* 重复包：ACK 当前进度（主机跳过） */
        if (seq < g_iap_boot.next_seq)
        {
            Proto_Send(cmd, IAP_ACK_OK, g_iap_boot.next_seq, nullptr, 0);
            break;
        }
        /* 乱序包：NAK */
        if (seq > g_iap_boot.next_seq)
        {
            Proto_Send(cmd, IAP_ACK_ERR_PARAM, g_iap_boot.next_seq, nullptr, 0);
            break;
        }

        /* 包长必须与镜像位置严格匹配（末包可短，中间包必须 256B）。
         * 字节流 = 头部(64B) + 固件，包 0 起始于分区基址（含头部），
         * 按 image_len 连续分片：remain = image_len - seq*256 */
        const uint32_t remain = p.image_len - (uint32_t)seq * IAP_DATA_PAYLOAD_MAX;
        const uint16_t expect = (remain >= IAP_DATA_PAYLOAD_MAX) ? (uint16_t)IAP_DATA_PAYLOAD_MAX : (uint16_t)remain;
        if (dlen != expect)
        {
            Proto_Send(cmd, IAP_ACK_ERR_PARAM, g_iap_boot.next_seq, nullptr, 0);
            break;
        }

        /* 写入 flash（关中断 ~ms 级；主机停等 ACK） */
        const uint32_t addr = g_iap_boot.target_base + (uint32_t)seq * IAP_DATA_PAYLOAD_MAX;
        __disable_irq();
        const bool ok = flash_port::write_bytes(addr, payload + 2, dlen);
        __enable_irq();
        if (!ok)
        {
            Proto_Send(cmd, IAP_ACK_ERR_BUSY, g_iap_boot.next_seq, nullptr, 0);
            break;
        }

        /* 包 0：校验镜像头与 START 记录一致（防错槽/错包） */
        if (seq == 0u && !Check_Header_Consistent(g_iap_boot.target_base))
        {
            p.state = IAP_STATE_IDLE;
            p.pending_slot = 0;
            (void)Iap_Param_Write(&p);
            Proto_Send(cmd, IAP_ACK_ERR_SLOT, 0, nullptr, 0);
            break;
        }

        g_iap_boot.next_seq++;
        g_iap_boot.verified = false; /* 新数据写入后须重新 END 校验 */
        Proto_Send(cmd, IAP_ACK_OK, g_iap_boot.next_seq, nullptr, 0);
        break;
    }

    /* ── END：全量 CRC32 校验 ── */
    case IAP_CMD_END: {
        if (p.state != IAP_STATE_TRANSFERRING)
        {
            Proto_Send(cmd, IAP_ACK_ERR_STATE, g_iap_boot.next_seq, nullptr, 0);
            break;
        }

        uint32_t crc = 0xFFFFFFFFu;
        uint32_t remain = p.image_len - IAP_IMAGE_HEADER_SIZE;
        uint32_t addr = g_iap_boot.target_base + IAP_IMAGE_HEADER_SIZE;
        uint8_t buf[256];

        while (remain > 0)
        {
            const uint32_t chunk = (remain > sizeof(buf)) ? sizeof(buf) : remain;
            flash_port::read_bytes(addr, buf, (uint16_t)chunk);
            crc = Iap_Crc32_Step(crc, buf, chunk);
            addr += chunk;
            remain -= chunk;
        }

        if ((crc ^ 0xFFFFFFFFu) != p.image_crc32)
        {
            /* 校验失败：放弃本次传输（需重新 START） */
            p.state = IAP_STATE_IDLE;
            p.pending_slot = 0;
            (void)Iap_Param_Write(&p);
            Proto_Send(cmd, IAP_ACK_ERR_CRC, 0, nullptr, 0);
            break;
        }

        g_iap_boot.verified = true;
        Proto_Send(cmd, IAP_ACK_OK, g_iap_boot.next_seq, nullptr, 0);
        break;
    }

    /* ── ACTIVATE：切换 active 并复位 ── */
    case IAP_CMD_ACTIVATE: {
        if (p.state != IAP_STATE_TRANSFERRING || !g_iap_boot.verified)
        {
            Proto_Send(cmd, IAP_ACK_ERR_STATE, g_iap_boot.next_seq, nullptr, 0);
            break;
        }

        /* 切 active 到新槽，进入确认窗口（回滚=改标志，旧固件完好）
         * attempt_slot 指向新槽：boot 首启判定 attempt_slot==active 成立，
         * 否则会被误判为"参数不一致"而清除确认窗口（回滚保护失效） */
        p.state        = IAP_STATE_PENDING_ACTIVATE;
        p.active_slot  = p.pending_slot;
        p.attempt_count = 0;
        p.success_count = 0;
        p.boot_ok       = 0;
        p.attempt_slot  = p.pending_slot;
        (void)Iap_Param_Write(&p);

        Proto_Send(cmd, IAP_ACK_OK, g_iap_boot.next_seq, nullptr, 0);
        NVIC_SystemReset();
        break;
    }

    /* ── ABORT：放弃升级，跳 active app ── */
    case IAP_CMD_ABORT: {
        if (p.state == IAP_STATE_TRANSFERRING || p.state == IAP_STATE_REQ_UPGRADE)
        {
            p.state = IAP_STATE_IDLE;
            p.pending_slot = 0;
            (void)Iap_Param_Write(&p);
        }
        Proto_Send(cmd, IAP_ACK_OK, g_iap_boot.next_seq, nullptr, 0);
        g_iap_boot.upgrade_mode = false;
        break;
    }

    /* ── REBOOT：复位 ── */
    case IAP_CMD_REBOOT: {
        Proto_Send(cmd, IAP_ACK_OK, g_iap_boot.next_seq, nullptr, 0);
        NVIC_SystemReset();
        break;
    }

    default:
        /* 未知命令：静默忽略 */
        break;
    }
}

/* ══════════════════════════════════════════════════════════
 *  公共接口
 * ══════════════════════════════════════════════════════════ */

void Iap_Proto_Init(void)
{
    g_iap_boot.next_seq = 0;
    g_iap_boot.verified = false;
}

void Iap_Proto_Task(void)
{
    if (iap_port_count == 0)
        return;

    uart_buffer_t *buf = iap_ports[0].buffer;

    /* 长度驱动判帧：不依赖 IDLE 标志。
     * 原因：USB CDC 虚拟串口（DAP-Link）每 64B 一个 USB 帧，帧间间隙
     * ~1ms 远大于 1 字节时间（86us @115200），USART IDLE 会在 64B 边界
     * 误触发，导致长帧被拆成多段"半帧"全部校验失败；
     * 改为解析帧头 LEN 后等待 rx_len 收满再处理。 */
    if (buf->rx_len < FRAME_HDR_LEN)
        return;

    const uint8_t *f = buf->rx_buf;
    if (f[0] != IAP_FRAME_HEAD0 || f[1] != IAP_FRAME_HEAD1)
    {
        /* 帧头不匹配：丢弃（主机停等重发） */
        buf->rx_len  = 0;
        buf->rx_flag = 0;
        return;
    }

    const uint16_t plen = (uint16_t)(f[3] | ((uint16_t)f[4] << 8));
    const uint16_t need = FRAME_HDR_LEN + plen + FRAME_CRC_LEN;

    if (need > IAP_FRAME_BUF_MAX)
    {
        /* 超长帧：丢弃 */
        buf->rx_len  = 0;
        buf->rx_flag = 0;
        return;
    }
    if (buf->rx_len < need)
    {
        /* 帧未收满（USB 拆包中）：等待后续字节 */
        return;
    }

    /* 帧完整：CRC 校验 + 分发 */
    bool handled = false;
    const uint16_t crc = Iap_Crc16(f, FRAME_HDR_LEN + plen);
    if (crc == (uint16_t)(f[FRAME_HDR_LEN + plen] |
                          ((uint16_t)f[FRAME_HDR_LEN + plen + 1] << 8)))
    {
        Proto_HandleCmd(f[2], f + FRAME_HDR_LEN, plen);
        handled = true;
    }

    buf->rx_len  = 0;
    buf->rx_flag = 0;

    if (handled)
        g_iap_boot.mode_start_tick = get_tick();   /* 刷新超时 */
}
