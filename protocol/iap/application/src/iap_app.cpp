/**
 * @file    iap_app.cpp
 * @brief   App 侧 IAP 实现：VTOR / 喂狗 / boot_ok 上报 / 升级请求
 */

#include "iap_app.hpp"
#include "iap_conf.hpp"
#include "iap_image.hpp"
#include "iap_param.hpp"
#include "inter_flash.hpp"
#include "inter_wdt.hpp"

#include "gd32f4xx.h"

#include <cstdio>

void Iap_AppInit(void)
{
    /* 1. 向量表重定位到固件入口（分区基址 + 64B 镜像头；编译期宏 APP_SLOT_A/B 决定）：
     *    必须在任何中断使能之前（main() 第一行调用），否则中断会落入 boot 向量表 */
    SCB->VTOR = IAP_APP_ADDR + IAP_IMAGE_HEADER_SIZE;

    /* 2. flash 安全区：仅参数区（app 无权擦写固件区，防呆） */
    flash_port::init(IAP_PARAM0_ADDR, (IAP_PARAM1_ADDR + IAP_PARAM_SIZE) - IAP_PARAM0_ADDR);

    /* 3. 立即喂狗：boot 跳转前已使能 IWDG（~6.5s），防止启动流程超时复位 */
    wdt_port::iwdg_feed();
}

void Iap_AppFeedWdt(void)
{
    wdt_port::iwdg_feed();
}

static bool s_boot_ok_reported = false;

void Iap_AppReportBootOk(void)
{
    if (s_boot_ok_reported)
        return;
    s_boot_ok_reported = true;

    /* 仅当处于确认窗口（PENDING）且本槽=active 且未上报时写 boot_ok */
    iap_param_t p;
    if (!Iap_Param_Read(&p))
        return;
    if (p.state != IAP_STATE_PENDING_ACTIVATE)
        return;
    if (p.active_slot != IAP_APP_SLOT)
        return;
    if (p.boot_ok != 0)
        return;

    p.boot_ok = 1;
    (void)Iap_Param_Write(&p);
}

void Iap_RequestUpgrade(void)
{
    /* 写升级请求标志（幂等）后复位；boot 检测到 REQ_UPGRADE 进入升级模式。
     * 参数区无有效记录时跳过写入直接复位（boot 会重建参数并正常启动）。 */
    iap_param_t p;
    if (Iap_Param_Read(&p))
    {
        p.state = IAP_STATE_REQ_UPGRADE;
        (void)Iap_Param_Write(&p);
    }
    NVIC_SystemReset();
}

void Iap_AppGetStatus(char *buf, uint16_t size)
{
    if (buf == nullptr || size == 0)
        return;
    buf[0] = '\0';

    iap_param_t p;
    if (!Iap_Param_Read(&p))
    {
        snprintf(buf, size, "IAP: no param record\r\n");
        return;
    }

    /* 读 active 分区镜像头版本 */
    uint32_t ver = 0u;
    if (p.active_slot == IAP_SLOT_A || p.active_slot == IAP_SLOT_B)
    {
        const uint32_t base = (p.active_slot == IAP_SLOT_A) ? IAP_APP_A_ADDR : IAP_APP_B_ADDR;
        iap_image_header_t hdr;
        flash_port::read_bytes(base, (uint8_t *)&hdr, sizeof(hdr));
        if (hdr.magic == IAP_IMAGE_MAGIC)
            ver = hdr.version;
    }

    if (p.state == IAP_STATE_PENDING_ACTIVATE)
    {
        /* 激活确认窗口：新固件已激活，等待连续成功固化 */
        snprintf(buf, size,
                 "IAP: NEW firmware on slot %c v%u.%u.%u, confirming (%u/3)\r\n",
                 p.active_slot,
                 (unsigned)(ver >> 16), (unsigned)((ver >> 8) & 0xFF), (unsigned)(ver & 0xFF),
                 (unsigned)(p.success_count + 1u));
    }
    else
    {
        /* 正常运行：当前生效固件（固化后即显示 confirmed） */
        snprintf(buf, size,
                 "IAP: active slot %c v%u.%u.%u (confirmed)\r\n",
                 p.active_slot,
                 (unsigned)(ver >> 16), (unsigned)((ver >> 8) & 0xFF), (unsigned)(ver & 0xFF));
    }
}
