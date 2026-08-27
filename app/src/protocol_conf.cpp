/**
 * @file    protocol.cpp
 * @brief   多协议公共入口 —— 初始化 & 任务分发
 *
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  切换协议: 修改 app/inc/protocol_conf.hpp 中 ACTIVE_PROTOCOL 宏   ║
 * ║  新增协议: 在下方 "DISPATCH" 区添加一个 #elif 分支即可        ║
 * ║           其余文件（Protocol_Init/Task 接口）无需改动         ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * 数据流:
 *   USART ISR → uart_buffer_t.rx_flag = 1
 *     → Protocol_Task() → _proto_task()
 *       → Modbus_Task()     (middleware/modbus/)
 *       → StringCmd_Task()  (protocol_string.cpp)
 *       → YourNew_Task()    (protocol_your_new.cpp)  ← 新协议在此接入
 */

#include "protocol_conf.hpp"

/* ══════════════════════════════════════════════════════════════
 *  DISPATCH — 每种协议仅需在此处添加一个 #elif 分支
 *
 *  分支格式:
 *    #elif ACTIVE_PROTOCOL == PROTOCOL_XXX
 *    #  include "xxx.hpp"                   // 协议栈头文件（若有）
 *       void XxxInit_Forward(void);         // 若 init/task 在其他 .cpp 中，前向声明
 *       static void _proto_init(void) { Xxx_Init(); }
 *       static void _proto_task(void) { Xxx_Task(); }
 * ══════════════════════════════════════════════════════════════ */

#if ACTIVE_PROTOCOL == PROTOCOL_MODBUS
/* ── Modbus RTU 从机 ────────────────────────────────────────── */
#  include "modbus.hpp"
   static void _proto_init(void) { Modbus_Init(); }
   static void _proto_task(void) { Modbus_Task(); }

#elif ACTIVE_PROTOCOL == PROTOCOL_STRING_CMD
/* ── ASCII 字符串指令协议 ───────────────────────────────────── */
#  include "string_pro.hpp"
   static void _proto_init(void) { StringCmd_Init(); }
   static void _proto_task(void) { StringCmd_Task(); }

/* ── 新增协议模板 ──────────────────────────────────────────────
 * #elif ACTIVE_PROTOCOL == PROTOCOL_YOUR_NEW
 * #  include "your_new.hpp"
 *    static void _proto_init(void) { YourNew_Init(); }
 *    static void _proto_task(void) { YourNew_Task(); }
 * ──────────────────────────────────────────────────────────── */

#else
#  error "Unknown ACTIVE_PROTOCOL — add a dispatch entry in app/src/protocol_conf.cpp"
#endif

/* ══════════════════════════════════════════════════════════════
 *  公共接口实现 —— 上层永远只调用这两个函数，无需感知协议细节
 * ══════════════════════════════════════════════════════════════ */

void Protocol_Init(void) { _proto_init(); }
void Protocol_Task(void) { _proto_task(); }
