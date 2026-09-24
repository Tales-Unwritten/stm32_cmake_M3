#pragma once

#ifdef __cplusplus

/**
 * @brief CAN（inter_can / STM32F1 bxCAN）自测入口
 *
 * 仅在 CAN_SELFTEST=ON 的固件里由 app_setup() 调用（见 CMakeLists.txt：
 * 该开关追加 function/src/can_selftest.cpp 并定义 ENABLE_CAN_SELFTEST）。
 * 通过 debug_uart(USART1 PA9/PA10, 115200) 输出机器可解析文本：
 *   [CAN] <用例> ... PASS/FAIL  /  [CAN] SUMMARY  /  [CAN] VERDICT  /  [CAN] DONE
 *
 * 覆盖（无需收发器/总线，靠 bxCAN 片内 Loopback）：
 *   0. 未初始化保护 + 非法参数拒绝（空引脚/0 波特率/过滤器越界/TQ 越界/
 *      波特率不整除/过慢）
 *   1. init + 寄存器白盒校验：MSR/MCR/BTR（BRP/TS1/TS2/SJW/模式位）/验收滤波
 *      （FA1R/FM1R/FS1R）+ ESR 无错误
 *   2. 回环收发：标准帧 / 扩展帧 / ID 边界（0x000、0x7FF、0x00000000、
 *      0x1FFFFFFF）/ DLC 0~8 全档 / 数据逐字节比对 / FIFO 清空
 *   3. 连发管路：FIFO0 深度 3 连发 3 帧按序收回 + 16 帧递增 ID 扫描
 *   4. 各标准波特率回环：125k / 250k / 500k / 1M（BRP 与 PCLK1 精确匹配）
 *   5. 各工作模式：loopback / silent_loopback（片内回环）、silent（只听→发送被拒）
 *   6. deinit 后接口保护 + 再初始化可恢复
 *
 * @return 0 = 全部通过，1 = 有失败用例（失败明细见串口 [CAN] ... FAIL 行）
 *
 * @note 回环模式不需要外部接线；引脚仍会按要求配置（默认 CAN_TX=PB9 /
 *       CAN_RX=PB8，RM_CAN1_2）。可选的真实总线用例见 .cpp 头部说明。
 */
int can_selftest_main(void);

/**
 * @brief 常驻冒烟：重复做一次回环 发送→接收→比对，每次打印一行 [CAN] smoke
 *
 * 供 app_loop() 周期调用（默认每 1s），用于在台面上持续观察驱动是否正常；
 * 不做参数/寄存器校验（那部分在 can_selftest_main 里只跑一次）。
 */
void can_selftest_loop(void);

#endif
