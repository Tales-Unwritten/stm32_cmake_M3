#pragma once

/**
 * @file    pc_task.hpp
 * @brief   PC 任务调度 —— 所有通信协议的入口
 *
 * APP_PC_Task() 是主循环中周期性调用的任务函数，内部根据
 * protocol.hpp 中 ACTIVE_PROTOCOL 宏分发给对应协议栈。
 *
 * 扩展方式:
 *   新增协议只需修改 protocol.hpp 和 protocol.cpp，
 *   本文件无需任何改动。
 */

extern void APP_PC_Task(void);
