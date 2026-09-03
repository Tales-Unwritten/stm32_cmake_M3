#pragma once

#ifdef __cplusplus

/**
 * @brief 片上 inter_flash(flash_port) 自测入口
 *
 * 仅在 FLASH_SELFTEST=ON 的固件里由 function_init() 调用。
 * 通过 soft_485(115200, 半双工) 输出机器可解析文本：
 *   [BOOT] / [STATE] / [PERSIST] / [READY] / [CASE] / [INFO] / [SUMMARY] / [DONE]
 *
 * @return 失败用例个数（0 = 全部通过；主流程可忽略）
 */
int flash_selftest_main(void);

#endif
