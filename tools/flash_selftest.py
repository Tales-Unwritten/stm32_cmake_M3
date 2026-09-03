#!/usr/bin/env python3
# ============================================================
# inter_flash 最小自测运行器
#   构建 -> probe-rs 下载 -> 复位 -> 从串口收 [R] 结果 -> 判定
# 用法:
#   python3 tools/flash_selftest.py                # 构建+下载+跑
#   python3 tools/flash_selftest.py --no-build     # 用现有固件
#   python3 tools/flash_selftest.py --port /dev/ttyUSB1
# 退出码: 0=全过 1=有用例FAIL 2=环境/通信错误
# ============================================================
import argparse
import glob
import os
import re
import subprocess
import sys
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF = os.path.join(ROOT, "build", "selftest", "stm32_cmake_M3.elf")
CHIP = "STM32F103VE"
TEST_BASE_OFF = 0x0807F800 - 0x08000000  # 测试页偏移(末页起点)


def sh(args):
    print("+", " ".join(args))
    r = subprocess.run(args, cwd=ROOT, text=True, capture_output=True)
    if r.returncode != 0:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit(f"cmd failed: {' '.join(args)}")
    return r.stdout + r.stderr


def find_ports():
    return sorted(p for p in glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
                  if os.path.exists(p))


def wake(ser):
    """部分 CH340 兼容片需要抖动 DTR/RTS 才恢复接收。"""
    for r, d in ((True, True), (False, False), (True, False), (False, True)):
        try:
            ser.dtr = bool(d)
            ser.rts = bool(r)
        except Exception:
            pass
        time.sleep(0.03)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass


def read_until_done(ser, timeout):
    lines = []
    buf = b""
    t0 = time.time()
    while time.time() - t0 < timeout:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                ln = raw.decode("utf-8", "ignore").strip()
                if ln:
                    lines.append(ln)
            if any("[DONE]" in ln for ln in lines):
                break
        else:
            time.sleep(0.05)
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--elf", default=ELF)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args()

    if not args.no_build:
        sh(["cmake", "--build", "--preset", "selftest"])
    if not os.path.exists(args.elf):
        raise SystemExit(f"not found: {args.elf}")

    # 代码区必须远离测试页
    size = sh(["arm-none-eabi-size", args.elf]).splitlines()[-1]
    text, data = (int(x) for x in re.findall(r"\d+", size)[:2])
    if text + data > TEST_BASE_OFF:
        raise SystemExit("firmware too large, would overlap test page")
    print(f"flash text+data={text+data} B (test page @{hex(0x0807F800)})")

    sh(["probe-rs", "download", "--chip", CHIP, "--non-interactive",
        "--disable-progressbars", "--verify", args.elf])

    def reset():
        sh(["probe-rs", "reset", "--chip", CHIP, "--non-interactive"])

    ports = [args.port] if args.port else find_ports()
    if not ports:
        raise SystemExit("no serial port")

    ser = None
    for p in ports:
        try:
            s = serial.Serial(p, args.baud, timeout=0.2)
        except Exception as e:
            print(f"skip {p}: {e}")
            continue
        wake(s)
        reset()
        lines = read_until_done(s, 8)
        if lines:
            ser, chosen = s, p
            break
        s.close()
    if ser is None:
        raise SystemExit("no selftest output on any port (check USB1/PA9-PA10 wiring)")

    # 复位一次正式采集（清掉扫描阶段的重复输出）
    ser.reset_input_buffer()
    reset()
    lines = read_until_done(ser, args.timeout)
    ser.close()

    results = [ln for ln in lines if ln.startswith("[R]")]
    for ln in results:
        print("  " + ln)
    m = [ln for ln in results if "SUMMARY" in ln]
    if not m:
        print("no SUMMARY; raw:")
        for ln in lines:
            print(" ", ln)
        raise SystemExit(2)
    mm = re.search(r"total=(\d+) pass=(\d+) fail=(\d+)", m[0])
    total, pas, fail = (int(x) for x in mm.groups())
    if fail == 0 and total == pas:
        print("RESULT: PASS")
        raise SystemExit(0)
    print("RESULT: FAIL")
    raise SystemExit(1)


if __name__ == "__main__":
    main()
