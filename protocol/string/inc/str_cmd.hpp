#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 命令处理函数
   args:  命令名后面的参数字符串，无参数时为 NULL
   reply: 填充回复内容
   size:  reply 缓冲区大小 */
typedef void (*command_handler_t)(const char *args, char *reply, size_t size);

/* 命令表条目 */
typedef struct {
    const char         *name;
    command_handler_t   handler;
} command_entry_t;

extern const command_entry_t g_command_table[];
extern const uint8_t         g_command_count;

/* 解析一行命令（已去除 \r\n），匹配并执行
   找到命令返回 true，reply 由 handler 填充
   未找到返回 false，reply 填充错误提示 */
bool command_process(const char *line, char *reply, size_t size);

#ifdef __cplusplus
}
#endif
