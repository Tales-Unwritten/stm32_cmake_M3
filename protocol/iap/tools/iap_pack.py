#!/usr/bin/env python3
"""
IAP 升级包打包工具 —— 给 app bin 前置 64B 镜像头部

镜像布局：
  [0 .. 63]   iap_image_header_t（magic/version/platform/image_len/crc32/slot/reserved）
  [64 .. ]    固件负载（CRC32 覆盖此段；自动 4 字节对齐，填充 0xFF）

用法：
  python3 tools/iap_pack.py --input app_a.bin --output iap_app_a.bin --slot A --version 1.0.0

与固件侧约定（protocol/iap/inc/iap_conf.hpp / iap_image.hpp）：
  magic    = 0x31504149 ("IAP1")
  platform = 0x47444634 ("GDF4", GD32F470ZI)
  version  = (MAJOR<<16)|(MINOR<<8)|PATCH
"""

import argparse
import struct
import sys
import zlib

HEADER_SIZE = 64
IMAGE_MAGIC = 0x31504149  # "IAP1"
PLATFORM_ID = 0x47444634  # "GDF4"
SLOTS       = {"A": ord("A"), "B": ord("B")}


def pack(input_path, output_path, slot, version):
    with open(input_path, "rb") as f:
        payload = f.read()
    if not payload:
        sys.exit(f"error: empty input file: {input_path}")

    # 负载 4 字节对齐（镜像总长须为 4 的倍数，保证 32 位写边界）
    pad = (-len(payload)) % 4
    payload += b"\xff" * pad

    image_len = HEADER_SIZE + len(payload)
    crc32     = zlib.crc32(payload) & 0xFFFFFFFF
    major, minor, patch = version
    ver = (major << 16) | (minor << 8) | patch

    header = struct.pack(
        "<IIIIIB43s",
        IMAGE_MAGIC,
        ver,
        PLATFORM_ID,
        image_len,
        crc32,
        slot,
        b"\x00" * 43,
    )
    assert len(header) == HEADER_SIZE

    with open(output_path, "wb") as f:
        f.write(header)
        f.write(payload)

    print(f"packed : {output_path}")
    print(f"  slot    = {chr(slot)}")
    print(f"  version = {major}.{minor}.{patch}")
    print(f"  payload = {len(payload)} B (pad {pad})")
    print(f"  total   = {image_len} B")
    print(f"  crc32   = 0x{crc32:08X}")


def parse_version(text):
    parts = text.split(".")
    if len(parts) != 3:
        sys.exit("error: --version must be MAJOR.MINOR.PATCH (e.g. 1.0.0)")
    try:
        return tuple(int(p) for p in parts)
    except ValueError:
        sys.exit("error: --version parts must be integers")


def main():
    ap = argparse.ArgumentParser(description="IAP image packer (bin -> upgrade image)")
    ap.add_argument("--input", required=True, help="raw app bin (e.g. gd32f470_app_a.elf.bin)")
    ap.add_argument("--output", required=True, help="packed upgrade image")
    ap.add_argument("--slot", required=True, choices=["A", "B"], help="target slot")
    ap.add_argument("--version", required=True, help="MAJOR.MINOR.PATCH")
    args = ap.parse_args()

    pack(args.input, args.output, SLOTS[args.slot], parse_version(args.version))


if __name__ == "__main__":
    main()
