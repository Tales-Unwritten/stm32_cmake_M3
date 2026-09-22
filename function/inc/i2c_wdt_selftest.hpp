#pragma once

#ifdef __cplusplus

/**
 * @brief 硬件 I2C + 看门狗 自测入口
 *
 * 仅在 I2C_WDT_SELFTEST=ON 的固件里由 app_setup() 调用（见 CMakeLists.txt：
 * 该开关追加 function/src/i2c_wdt_selftest.cpp 并定义 ENABLE_I2C_WDT_SELFTEST）。
 * 通过 debug_uart(USART1 PA9/PA10, 115200) 输出机器可解析文本：
 *   [I2C] / [WDT] <用例> ... PASS/FAIL  /  [I2C] SUMMARY  /  [I2C] DONE
 *
 * 覆盖：
 *   1. 硬件 I2C1(PB6/PB7) 初始化 + 地址探测（0xA0 写 / 0xA1 读均 ACK）
 *   2. device_eeprom 经 i2c_bus 抽象接口驱动硬件 I2C：读 → 写 → 回读比对 → 还原
 *   3. i2c_hw_port 设备级 API（i2c_write_reg / i2c_read_reg）1 / 2 / 8 字节收发
 *   4. IWDG：配置寄存器回读 + 按时喂狗存活
 *   5. WWDG：配置寄存器回读 + 窗口内喂狗存活
 *   6. 可选：饿死看门狗验证复位（WDT_PROVE_RESET，见 .cpp 头部说明）
 *
 * @note 本函数不会返回：看门狗一旦启动无法关闭，函数末尾进入"持续喂狗"循环
 *       （或按 WDT_PROVE_RESET 故意饿狗触发一次复位）。
 */
int i2c_wdt_selftest_main(void);

#endif
