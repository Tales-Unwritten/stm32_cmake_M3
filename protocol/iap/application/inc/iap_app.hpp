#pragma once
/**
 * @file    iap_app.hpp
 * @brief   App 侧 IAP 接口（仅 APP_IAP 构建启用，裸跑版零影响）
 *
 * 调用点约定：
 *   - Iap_AppInit()      main() 第一行（systick_config 之前，VTOR 须先于任何中断）
 *   - Iap_AppFeedWdt()   function_init/loop（boot 跳转前已使能 IWDG，必须周期喂狗）
 *   - Iap_AppReportBootOk() 主循环每圈调用（内部判定首圈 + PENDING 窗口）
 *   - Iap_RequestUpgrade()  协议层触发（Modbus 寄存器 / String 命令）
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化：VTOR 重定位 + flash 安全区（仅参数区）+ 首次喂狗 */
void Iap_AppInit(void);

/** @brief 喂 IWDG（主循环周期调用） */
void Iap_AppFeedWdt(void);

/** @brief 上报启动成功（仅 PENDING 确认窗口内写 boot_ok，幂等） */
void Iap_AppReportBootOk(void);

/** @brief 请求升级：写 REQ_UPGRADE 标志后软复位（boot 进入升级模式） */
void Iap_RequestUpgrade(void);

/**
 * @brief 生成 IAP 状态文本（激活确认进度 / 固化生效），供启动时打印
 * @param buf  输出缓冲区
 * @param size 缓冲区大小
 * 示例：
 *   "IAP: active slot A v1.0.0 (confirmed)"
 *   "IAP: new firmware on slot B v1.0.0, confirming (1/3)"
 */
void Iap_AppGetStatus(char *buf, uint16_t size);

#ifdef __cplusplus
}
#endif
