#pragma once
/**
 * @file    iap_image.hpp
 * @brief   升级镜像头部结构 —— 由 tools/iap_pack.py 生成，boot 校验
 *
 * 镜像文件布局（iap_app_a.bin / iap_app_b.bin）：
 *   [0 .. 63]   镜像头部（本结构体）
 *   [64 .. ]    固件负载（app bin 原始内容，CRC32 覆盖此段）
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IAP_IMAGE_HEADER_SIZE   64u

typedef struct {
    uint32_t magic;        /* IAP_IMAGE_MAGIC ("IAP1") */
    uint32_t version;      /* IAP_VERSION_PACK() 打包版本 */
    uint32_t platform;     /* IAP_PLATFORM_ID 平台标识 */
    uint32_t image_len;    /* 镜像总长（含头部），必须为 4 对齐 */
    uint32_t crc32;        /* 头部之后全部负载的 CRC32 */
    uint8_t  slot;         /* 目标槽：IAP_SLOT_A / IAP_SLOT_B */
    uint8_t  reserved[43]; /* 保留（0 填充） */
} iap_image_header_t;      /* 64 字节 */

#ifdef __cplusplus
}
#endif
