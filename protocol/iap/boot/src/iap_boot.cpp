/**
 * @file    iap_boot.cpp
 * @brief   Boot 主流程：初始化 / 参数状态机（确认·回滚）/ 分区校验 / 跳转
 *
 * 启动流程：
 *   1. 读参数区（无有效记录则初始化默认 active=A）
 *   2. PENDING_ACTIVATE 状态机：连续 3 次成功上报 -> 固化；连续 2 次失败 -> 回滚
 *   3. 升级入口：PC13 按键（低电平防抖）或 app 请求标志（REQ_UPGRADE）
 *   4. 校验 active 分区（magic/platform/slot/len + 全量 CRC32）：
 *      - 有效 -> 跳转；无效 -> 试另一分区（自救）；双坏 -> 死等升级
 *   5. 跳转前在 PENDING 下记录 attempt；使能 IWDG 兜底（跳转即死也能复位回 boot）
 */

#include "iap_boot.hpp"
#include "iap_conf.hpp"
#include "iap_image.hpp"
#include "iap_crc.hpp"
#include "iap_proto.hpp"

#include "inter_usart.hpp"
#include "inter_io_ctrl.hpp"
#include "inter_flash.hpp"
#include "inter_wdt.hpp"

#include "systick.h"
#include "gd32f4xx.h"
#include "gd32f4xx_fwdgt.h"

#include <cstring>

/* ── boot 硬件实例 ────────────────────────────────────────
 * USART0 @ PA9(TX)/PA10(RX)，115200-8N1（与 app 侧软串口同引脚，
 * 每次复位后重新初始化，互不干扰） */
static usart_port s_boot_uart({
    USART0, GPIOA, pin9, GPIOA, pin10,
    afio_enum_t::NONE, IAP_BAUDRATE, 512
});

/* PC13 按键：上拉输入，低电平=按下 */
static io_ctrl s_key(GPIOC, pin13);

/* ── 端口表（对齐 modbus_ports[] / string_ports[] 模式） ── */
static void Boot_UartSend(uint8_t *b, uint16_t l) { (void)s_boot_uart.send_data(b, l); }

const iap_port_t iap_ports[] = {
    { s_boot_uart.buffer(), Boot_UartSend },
};
const uint8_t iap_port_count = sizeof(iap_ports) / sizeof(iap_ports[0]);

iap_boot_ctx_t g_iap_boot;

/* ══════════════════════════════════════════════════════════
 *  槽位工具
 * ══════════════════════════════════════════════════════════ */

uint32_t Iap_SlotBase(uint8_t slot)
{
    return (slot == IAP_SLOT_A) ? IAP_APP_A_ADDR : IAP_APP_B_ADDR;
}

uint8_t Iap_OtherSlot(uint8_t slot)
{
    return (slot == IAP_SLOT_A) ? IAP_SLOT_B : IAP_SLOT_A;
}

/* ══════════════════════════════════════════════════════════
 *  初始化
 * ══════════════════════════════════════════════════════════ */

static void Boot_Init(void)
{
    systick_config();
    s_boot_uart.init();
    s_key.init(mode_input, pullup);

    /* flash 安全区：AppA + AppB + Param0 + Param1（不含 boot 自身扇区，防误擦） */
    flash_port::init(IAP_APP_A_ADDR, (IAP_PARAM1_ADDR + IAP_PARAM_SIZE) - IAP_APP_A_ADDR);

    /* IWDG 生命周期：跳转 app 前使能的 IWDG 在系统复位后仍运行（硬件特性），
     * boot 启动即喂狗刷新，后续升级模式/擦除长耗时路径内持续喂狗，
     * 避免 boot 自身被 IWDG 打断；跳转 app 后由 app 接管喂狗职责 */
    wdt_port::iwdg_feed();
}

/** 按键防抖：50ms 采样窗口，低电平占比 >=80% 判定按下 */
static bool Key_Pressed(void)
{
    uint8_t low = 0;
    for (uint8_t i = 0; i < 50; i++)
    {
        if (s_key.read() == Low)
            low++;
        delay_ms(1);
    }
    return low >= 40;
}

/* ══════════════════════════════════════════════════════════
 *  镜像校验
 * ══════════════════════════════════════════════════════════ */

static bool Check_Image(uint32_t base, uint8_t slot, uint32_t max_size)
{
    iap_image_header_t hdr;
    flash_port::read_bytes(base, (uint8_t *)&hdr, sizeof(hdr));

    if (hdr.magic != IAP_IMAGE_MAGIC)          return false;
    if (hdr.platform != IAP_PLATFORM_ID)       return false;
    if (hdr.slot != slot)                      return false;
    if (hdr.image_len < IAP_IMAGE_HEADER_SIZE) return false;
    if (hdr.image_len > max_size)              return false;
    if ((hdr.image_len & 3u) != 0u)            return false;

    /* 全量 CRC32（增量计算，分块读 flash） */
    uint32_t crc    = 0xFFFFFFFFu;
    uint32_t remain = hdr.image_len - IAP_IMAGE_HEADER_SIZE;
    uint32_t addr   = base + IAP_IMAGE_HEADER_SIZE;
    uint8_t  buf[256];

    while (remain > 0)
    {
        const uint32_t chunk = (remain > sizeof(buf)) ? sizeof(buf) : remain;
        flash_port::read_bytes(addr, buf, (uint16_t)chunk);
        crc = Iap_Crc32_Step(crc, buf, chunk);
        addr  += chunk;
        remain -= chunk;
    }
    return (crc ^ 0xFFFFFFFFu) == hdr.crc32;
}

/** 读分区镜像头版本（无有效镜像返回 0） */
static uint32_t Image_Version(uint32_t base)
{
    iap_image_header_t hdr;
    flash_port::read_bytes(base, (uint8_t *)&hdr, sizeof(hdr));
    return (hdr.magic == IAP_IMAGE_MAGIC) ? hdr.version : 0u;
}

/* ══════════════════════════════════════════════════════════
 *  参数默认值
 * ══════════════════════════════════════════════════════════ */

static void Param_InitDefault(iap_param_t *p)
{
    memset(p, 0, sizeof(*p));
    p->magic       = IAP_PARAM_MAGIC;
    p->state       = IAP_STATE_IDLE;
    p->active_slot = IAP_SLOT_A;
}

/* ══════════════════════════════════════════════════════════
 *  PENDING 状态机：固化 / 回滚
 * ══════════════════════════════════════════════════════════ */

static void Boot_ProcessPending(void)
{
    iap_param_t &p = g_iap_boot.param;

    if (p.state != IAP_STATE_PENDING_ACTIVATE)
        return;

    if (p.pending_slot == p.active_slot && p.attempt_slot == p.active_slot)
    {
        if (p.boot_ok == 1u)
        {
            /* 上次启动成功上报：累计成功次数，达到阈值则固化 */
            p.success_count++;
            p.boot_ok = 0;
            if (p.success_count >= IAP_CONFIRM_THRESHOLD)
            {
                p.state        = IAP_STATE_IDLE;
                p.pending_slot = 0;
                p.attempt_slot = 0;
                p.attempt_count = 0;
                p.success_count = 0;
            }
            (void)Iap_Param_Write(&p);
        }
        else
        {
            /* 上次启动未上报（跳转即死/崩溃/断电）：累计失败次数，达到阈值回滚 */
            p.success_count = 0;
            p.attempt_count++;
            if (p.attempt_count >= IAP_ROLLBACK_THRESHOLD)
            {
                /* 回滚：active 切回另一槽——旧固件全程未被擦除，回滚即改标志 */
                p.active_slot  = Iap_OtherSlot(p.active_slot);
                p.state        = IAP_STATE_IDLE;
                p.pending_slot = 0;
                p.attempt_slot = 0;
                p.attempt_count = 0;
                p.success_count = 0;
            }
            (void)Iap_Param_Write(&p);
        }
    }
    else
    {
        /* 参数不一致（异常路径）：保守清除 pending，active 保持原值 */
        p.state        = IAP_STATE_IDLE;
        p.pending_slot = 0;
        p.attempt_slot = 0;
        p.attempt_count = 0;
        p.success_count = 0;
        p.boot_ok       = 0;
        (void)Iap_Param_Write(&p);
    }
}

/* ══════════════════════════════════════════════════════════
 *  升级模式：协议循环 + 超时退出
 * ══════════════════════════════════════════════════════════ */

static void Boot_UpgradeMode(void)
{
    g_iap_boot.upgrade_mode      = true;
    g_iap_boot.mode_start_tick   = get_tick();
    Iap_Proto_Init();

    while (g_iap_boot.upgrade_mode)
    {
        wdt_port::iwdg_feed();
        Iap_Proto_Task();

        /* 超时：无有效命令 -> 丢弃半包/清除升级请求 -> 跳 active app */
        if (get_tick() - g_iap_boot.mode_start_tick > IAP_CMD_TIMEOUT_MS)
        {
            if (g_iap_boot.param.state == IAP_STATE_TRANSFERRING ||
                g_iap_boot.param.state == IAP_STATE_REQ_UPGRADE)
            {
                g_iap_boot.param.state       = IAP_STATE_IDLE;
                g_iap_boot.param.pending_slot = 0;
                (void)Iap_Param_Write(&g_iap_boot.param);
            }
            g_iap_boot.upgrade_mode = false;
        }
    }
}

/* ══════════════════════════════════════════════════════════
 *  正常启动：校验 active -> 自救 -> 跳转
 * ══════════════════════════════════════════════════════════ */

static void Boot_StartActive(void)
{
    uint8_t slot = g_iap_boot.param.active_slot;

    if (!Check_Image(Iap_SlotBase(slot), slot, IAP_APP_A_SIZE))
    {
        /* active 分区无效：试另一槽（自救） */
        const uint8_t other = Iap_OtherSlot(slot);
        if (Check_Image(Iap_SlotBase(other), other, IAP_APP_A_SIZE))
        {
            g_iap_boot.param.active_slot = other;
            g_iap_boot.param.state       = IAP_STATE_IDLE;
            (void)Iap_Param_Write(&g_iap_boot.param);
            slot = other;
        }
        else
        {
            /* 双分区均无效：进入升级模式等待主机（无超时，防变砖） */
            Iap_Proto_Init();
            for (;;)
            {
                wdt_port::iwdg_feed();
                Iap_Proto_Task();
            }
        }
    }

    /* PENDING 下记录本次启动尝试（供下次启动判定） */
    if (g_iap_boot.param.state == IAP_STATE_PENDING_ACTIVATE)
    {
        g_iap_boot.param.attempt_slot = slot;
        g_iap_boot.param.boot_ok      = 0;
        (void)Iap_Param_Write(&g_iap_boot.param);
    }

    Iap_JumpToApp(Iap_SlotBase(slot) + IAP_IMAGE_HEADER_SIZE);
}

/* ══════════════════════════════════════════════════════════
 *  跳转 app
 * ══════════════════════════════════════════════════════════ */

void Iap_JumpToApp(uint32_t app_base)
{
    __disable_irq();

    /* 停 SysTick */
    SysTick->CTRL = 0u;

    /* 关闭所有 NVIC 中断（含挂起） */
    for (uint32_t i = 0; i < 8; i++)
        NVIC->ICER[i] = 0xFFFFFFFFu;

    /* IWDG 兜底：跳转即死/死循环也能复位回 boot，触发 PENDING 回滚（~6.5s） */
    wdt_port::iwdg_init(FWDGT_PSC_DIV64, 4095);
    wdt_port::iwdg_feed();

    /* 取 app 向量表：MSP + Reset_Handler */
    const uint32_t msp = *(volatile uint32_t *)app_base;
    const uint32_t pc  = *(volatile uint32_t *)(app_base + 4u);

    __set_MSP(msp);
    ((void (*)(void))pc)();

    for (;;)
    {
    }
}

/* ══════════════════════════════════════════════════════════
 *  主入口
 * ══════════════════════════════════════════════════════════ */

void Iap_Boot_Run(void)
{
    Boot_Init();

    /* 1. 读参数（无有效记录则初始化默认并写回） */
    g_iap_boot.has_param = Iap_Param_Read(&g_iap_boot.param);
    if (!g_iap_boot.has_param)
    {
        Param_InitDefault(&g_iap_boot.param);
        (void)Iap_Param_Write(&g_iap_boot.param);
        g_iap_boot.has_param = true;
    }

    /* 2. PENDING 确认 / 回滚判定 */
    Boot_ProcessPending();

    /* 3. 升级入口：按键 或 app 请求标志 */
    if (Key_Pressed() || g_iap_boot.param.state == IAP_STATE_REQ_UPGRADE)
    {
        Boot_UpgradeMode();
    }

    /* 4. 正常启动（校验 -> 跳转） */
    Boot_StartActive();
}
