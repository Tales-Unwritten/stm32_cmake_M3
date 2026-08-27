#pragma once

#ifdef __cplusplus


#endif //__cplusplus



#ifdef __cplusplus
extern "C" {
#endif

extern void function_init(void);
extern void function_loop(void);

/* UpdateModbusRegisters: 将 INA228 传感器值刷新到 Modbus StatusBlock 寄存器
 * 仅在 PROTOCOL_MODBUS 模式下编译，由 function_loop() 每周期调用 */
#include "protocol_conf.hpp"
#if ACTIVE_PROTOCOL == PROTOCOL_MODBUS
void UpdateModbusRegisters(void);
#endif

#ifdef __cplusplus
}
#endif
