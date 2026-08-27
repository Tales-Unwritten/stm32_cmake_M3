#pragma once
/**
 * @file    iap_conf.hpp
 * @brief   IAP 升级系统配置 —— 分区布局 / 协议常量 / 版本定义
 *
 * 分区布局（GD32F470ZI，2MB Flash，双 bank，每 bank: 4x16K+1x64K+7x128K）：
 *   Boot    0x08000000  128K   S0~S4（仅 SWD 更新）
 *   AppA    0x08020000  512K   S5~S8（链接基址 0x08020000）
 *   AppB    0x080A0000  512K   S9~S11 + bank2 S0~S4（链接基址 0x080A0000）
 *   Param0  0x08120000  128K   参数记录扇区 0（双扇区日志）
 *   Param1  0x08140000  128K   参数记录扇区 1（双扇区日志）
 *   预留    0x08160000  640K   未来扩展
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════
 *  分区布局
 * ══════════════════════════════════════════════════════════ */

#define IAP_FLASH_BASE      0x08000000u

#define IAP_BOOT_ADDR       0x08000000u
#define IAP_BOOT_SIZE       (128u * 1024u)

#define IAP_APP_A_ADDR      0x08020000u
#define IAP_APP_A_SIZE      (512u * 1024u)

#define IAP_APP_B_ADDR      0x080A0000u
#define IAP_APP_B_SIZE      (512u * 1024u)

#define IAP_PARAM0_ADDR     0x08120000u
#define IAP_PARAM1_ADDR     0x08140000u
#define IAP_PARAM_SIZE      (128u * 1024u)

/* ══════════════════════════════════════════════════════════
 *  槽位定义
 * ══════════════════════════════════════════════════════════ */

#define IAP_SLOT_A          'A'
#define IAP_SLOT_B          'B'

/* app 编译期槽位宏：APP_SLOT_A / APP_SLOT_B（由构建系统注入） */
#ifdef APP_SLOT_A
#  define IAP_APP_SLOT      IAP_SLOT_A
#  define IAP_APP_ADDR      IAP_APP_A_ADDR
#  define IAP_APP_SIZE      IAP_APP_A_SIZE
#elif defined(APP_SLOT_B)
#  define IAP_APP_SLOT      IAP_SLOT_B
#  define IAP_APP_ADDR      IAP_APP_B_ADDR
#  define IAP_APP_SIZE      IAP_APP_B_SIZE
#else
#  define IAP_APP_SLOT      0
#  define IAP_APP_ADDR      IAP_APP_A_ADDR   /* 非 IAP 构建（裸跑版）默认 A 基址，仅编译期占位 */
#  define IAP_APP_SIZE      IAP_APP_A_SIZE
#endif

/* ══════════════════════════════════════════════════════════
 *  版本 / 平台
 * ══════════════════════════════════════════════════════════ */

#define IAP_VERSION_MAJOR   1
#define IAP_VERSION_MINOR   0
#define IAP_VERSION_PATCH   0

/** 版本号打包为 u32：(主<<16)|(次<<8)|(补丁) */
#define IAP_VERSION_PACK() \
    (((uint32_t)(IAP_VERSION_MAJOR) << 16) | \
     ((uint32_t)(IAP_VERSION_MINOR) << 8)  | \
      (uint32_t)(IAP_VERSION_PATCH))

/** 平台标识：GD32F470ZI（可读 ASCII 大端 "GDF4"） */
#define IAP_PLATFORM_ID     0x47444634u

/* ══════════════════════════════════════════════════════════
 *  镜像头部（见 iap_image.hpp）
 * ══════════════════════════════════════════════════════════ */

#define IAP_IMAGE_MAGIC     0x31504149u   /* "IAP1" */

/* ══════════════════════════════════════════════════════════
 *  参数记录（见 iap_param.hpp）
 * ══════════════════════════════════════════════════════════ */

#define IAP_PARAM_MAGIC     0x32504149u   /* "IAP2" */

/* ══════════════════════════════════════════════════════════
 *  升级协议（boot 侧 USART0 通道）
 *  帧格式：0xA5 0x5A + CMD + LEN(2B LE) + PAYLOAD + CRC16(2B, 覆盖整帧)
 * ══════════════════════════════════════════════════════════ */

#define IAP_FRAME_HEAD0     0xA5u
#define IAP_FRAME_HEAD1     0x5Au

#define IAP_CMD_QUERY       0x01u   /* 查询：active 槽 / 版本 / 状态 */
#define IAP_CMD_START       0x02u   /* 开始：版本+长度+CRC32+目标槽 */
#define IAP_CMD_DATA        0x03u   /* 数据：包序号(2B)+负载 */
#define IAP_CMD_END         0x04u   /* 结束：触发全量 CRC32 校验 */
#define IAP_CMD_ACTIVATE    0x05u   /* 激活：切换 active 并复位 */
#define IAP_CMD_ABORT       0x06u   /* 放弃：状态复位，跳回 app */
#define IAP_CMD_REBOOT      0x07u   /* 复位：跳 active app */

/* 应答码 */
#define IAP_ACK_OK              0x00u
#define IAP_ACK_ERR_PARAM       0x01u   /* 参数错误 */
#define IAP_ACK_ERR_SLOT        0x02u   /* 目标槽错误/镜像头 slot 不符 */
#define IAP_ACK_ERR_CRC         0x03u   /* 校验失败 */
#define IAP_ACK_ERR_STATE       0x04u   /* 状态不允许该命令 */
#define IAP_ACK_ERR_BUSY        0x05u   /* 正在传输中 */
#define IAP_ACK_NEED_RESTART    0x10u   /* 需从头重传（无断点续传） */

/* 数据通道参数 */
#define IAP_DATA_PAYLOAD_MAX    256u    /* 单包负载上限（字节） */
#define IAP_CMD_TIMEOUT_MS      60000u  /* 升级模式命令超时（ms） */
#define IAP_BAUDRATE            115200u /* boot 串口波特率（USART0） */

/* 激活确认 / 回滚阈值 */
#define IAP_CONFIRM_THRESHOLD   3u      /* PENDING 下连续成功启动次数，达到后固化 */
#define IAP_ROLLBACK_THRESHOLD  2u      /* PENDING 下连续失败启动次数，达到后回滚 */

#ifdef __cplusplus
}
#endif
