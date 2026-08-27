#pragma once
/**
 * @file    iap_boot.hpp
 * @brief   Boot 主流程 / 状态机 / 跳转接口
 */

#include "iap_param.hpp"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief boot 运行上下文（全局单例） */
typedef struct {
    iap_param_t param;          /* 当前参数（启动读入，变更后写回） */
    bool        has_param;      /* 参数区是否有有效记录 */
    bool        upgrade_mode;   /* 1=升级模式中 */
    uint32_t    target_base;    /* 升级目标分区基址 */
    uint16_t    next_seq;       /* 期望的下一包序号 */
    bool        verified;       /* 1=END 全量校验已通过（ACTIVATE 前置） */
    uint32_t    mode_start_tick; /* 升级模式进入时刻（超时计时） */
} iap_boot_ctx_t;

extern iap_boot_ctx_t g_iap_boot;

/** @brief boot 主入口（不返回：跳转 app 或死等升级） */
void Iap_Boot_Run(void);

/** @brief 跳转 app（关中断/SysTick/NVIC，使能 IWDG 兜底，设置 MSP 后跳转） */
void Iap_JumpToApp(uint32_t app_base);

/** @brief 槽位 -> 分区基址 */
uint32_t Iap_SlotBase(uint8_t slot);

/** @brief 另一槽位 */
uint8_t Iap_OtherSlot(uint8_t slot);

#ifdef __cplusplus
}
#endif
