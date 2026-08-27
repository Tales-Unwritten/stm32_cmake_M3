#pragma once
/**
 * @file    protocol.hpp
 * @brief   多协议抽象层 —— 协议类型定义 + 公共接口 + 扩展指南
 *
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  新增协议三步走：                                            ║
 * ║  1. 在本文件 "协议 ID" 区追加:                               ║
 * ║       #define PROTOCOL_YOUR_NEW  N                          ║
 * ║  2. 创建 app/src/protocol_your_new.cpp，实现：               ║
 * ║       void YourNew_Init(void);  void YourNew_Task(void);    ║
 * ║       （参考 protocol_string.cpp 的写法）                    ║
 * ║  3. 在 app/src/protocol.cpp 的 DISPATCH 段添加 #elif 分支    ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * 协议实现文件:
 *   protocol.cpp          公共入口 (Protocol_Init / Protocol_Task)
 *   protocol_string.cpp   ASCII 字符串指令协议
 *   middleware/modbus/    Modbus RTU 协议栈
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════
 *  协议 ID —— 新增协议在此追加 #define
 * ══════════════════════════════════════════════════════════ */
#define PROTOCOL_NONE         0
#define PROTOCOL_MODBUS       1
#define PROTOCOL_STRING_CMD   2

/** @brief 当前活跃协议（修改此宏后重新编译即可切换） */
#ifndef ACTIVE_PROTOCOL
#define ACTIVE_PROTOCOL  PROTOCOL_STRING_CMD
#endif

/* ══════════════════════════════════════════════════════════
 *  协议操作接口类型
 *  每种协议在 protocol.cpp 内提供一对 init/task 静态函数，
 *  上层只调用 Protocol_Init() / Protocol_Task()，无需感知具体协议。
 * ══════════════════════════════════════════════════════════ */
#ifdef __cplusplus
struct protocol_ops_t {
    void (*init)(void);  ///< 协议初始化（上电/切换时调用一次）
    void (*task)(void);  ///< 协议轮询（主循环每周期调用）
};
#endif

/* ══════════════════════════════════════════════════════════
 *  公共接口（所有协议模式通用）
 * ══════════════════════════════════════════════════════════ */
void Protocol_Init(void);   ///< 初始化活跃协议
void Protocol_Task(void);   ///< 协议轮询（主循环中周期调用）

#ifdef __cplusplus
}
#endif
