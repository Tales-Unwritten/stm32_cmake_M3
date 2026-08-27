#pragma once
/**
 * @file    iap_param.hpp
 * @brief   参数记录结构 —— Param0/Param1 双扇区日志，任一时刻至少一份有效
 *
 * 双扇区交替写策略：
 *   - 每扇区 128KB / 512B = 256 槽，顺序写入（seq 递增）
 *   - 当前扇区写满或下一槽为脏数据时：擦除另一扇区，从槽 0 重新写
 *   - 读取：两扇区扫描所有已写槽，取「seq 最大且 CRC16 有效」的记录
 *   - 掉电安全：擦除瞬间断电，最新记录仍在未被擦除的扇区中
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IAP_PARAM_RECORD_SIZE   512u

/* 升级状态机 */
#define IAP_STATE_IDLE              0u  /* 空闲：正常双分区运行 */
#define IAP_STATE_REQ_UPGRADE       1u  /* app 请求升级：boot 进入升级模式等待命令 */
#define IAP_STATE_TRANSFERRING      2u  /* 传输中：目标分区正在接收数据 */
#define IAP_STATE_PENDING_ACTIVATE  3u  /* 待确认：active 已切到新分区，等 app 上报 */

typedef struct {
    uint32_t magic;          /* IAP_PARAM_MAGIC ("IAP2") */
    uint32_t seq;            /* 记录序号（递增，读最大有效者） */

    uint8_t  state;          /* IAP_STATE_* */
    uint8_t  active_slot;    /* 当前激活槽：IAP_SLOT_A/B */
    uint8_t  pending_slot;   /* 升级目标槽（仅 PENDING 有效） */
    uint8_t  boot_ok;        /* 1=app 已上报本次启动成功（仅 PENDING 有效） */
    uint8_t  attempt_slot;   /* 上次跳转的槽 */
    uint8_t  reserved0[3];

    uint16_t attempt_count;  /* PENDING 下连续启动尝试次数 */
    uint16_t success_count;  /* PENDING 下连续成功次数（>=3 固化） */

    uint32_t image_len;      /* 待激活镜像总长 */
    uint32_t image_crc32;    /* 待激活镜像负载 CRC32 */
    uint32_t image_version;  /* 待激活镜像版本 */

    uint32_t reserved1[10];  /* 预留：版本信息等扩展字段 */

    uint16_t crc16;          /* 记录自身 CRC16（覆盖本结构体 crc16 之前所有字节） */
    uint16_t reserved2;
} iap_param_t;               /* 512 字节槽 */

/**
 * @brief 读取最新有效参数记录（无有效记录返回 false）
 * @note  使用前须完成 flash_port::init（安全区覆盖 Param0/Param1）
 */
bool Iap_Param_Read(iap_param_t *out);

/**
 * @brief 写入新参数记录（seq 自动递增，双扇区交替）
 * @return true=写入并验证通过
 */
bool Iap_Param_Write(const iap_param_t *rec);

#ifdef __cplusplus
}
#endif
