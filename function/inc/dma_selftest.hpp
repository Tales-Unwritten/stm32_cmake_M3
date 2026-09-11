#pragma once

#ifdef __cplusplus

/**
 * @brief DMA 内存搬运（M2M）自测入口
 *
 * 仅在 DMA_SELFTEST=ON 的固件里由 function_init() 调用（见 CMakeLists.txt：
 * 该开关追加 function/src/dma_selftest.cpp 并定义 ENABLE_DMA_SELFTEST）。
 * 通过 debug_uart(USART1 PA9/PA10, 115200) 输出机器可解析文本：
 *   [DMA] <用例> ... PASS/FAIL  /  [DMA] SUMMARY  /  [DMA] DONE
 *
 * 用 DMA1_Channel4（避开 ADC1 硬件固定的 DMA1_Channel1）验证：
 *   1. 数据宽度按地址对齐自动选择（WORD / HALFWORD / BYTE）
 *   2. 长度非宽度倍数 → 尾字节由 CPU 补齐
 *   3. 同通道连续搬运（Abort → 重配 → 重启链路）
 *   4. M2M 切回"外设→内存"时 CCR.MEM2MEM 残留已被清（F1 HAL 的坑）
 *   5. 4096B 搬运耗时：DMA M2M vs CPU memcpy
 *
 * @return 失败用例个数（0 = 全部通过；主流程可忽略）
 */
int dma_selftest_main(void);

#endif
