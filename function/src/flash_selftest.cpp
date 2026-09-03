// ============================================================
// inter_flash (flash_port) 最小自测 —— STM32F103VET6
// ============================================================
// 只在芯片末页(0x0807F800, 2KB, 专用测试页)做擦→写→读往返。
// 输出单行结果： [R] <name> PASS|FAIL <detail>  结尾 [R] SUMMARY + [DONE]
// 通信通道：debug_uart（USART1 PA9/PA10 115200，由调用方 DebugPort_Init 初始化）
// ============================================================

#include "flash_selftest.hpp"
#include "inter_flash.hpp"
#include "device_serial.hpp"
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

static constexpr uint32_t kPage = 0x0807F800u; // 测试页（末页，页对齐）
static constexpr uint32_t kSize = 0x800u;      // 2KB
static constexpr uint32_t kEnd  = kPage + kSize; // 0x08080000（芯片 Flash 末地址+1）

static int g_total = 0, g_pass = 0, g_fail = 0;

static void out(const char *s)
{
    debug_uart.send_data((const uint8_t *)s, (uint16_t)strlen(s));
    debug_uart.send_data((const uint8_t *)"\r\n", 2);
}

static void outf(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    out(line);
}

static void report(const char *name, bool ok, const char *fmt, ...)
{
    char msg[120];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    g_total++;
    if (ok)
        g_pass++;
    else
        g_fail++;
    outf("[R] %s %s %s", name, ok ? "PASS" : "FAIL", msg);
}

static bool page_is_ff()
{
    for (uint32_t i = 0; i < kSize; i++)
        if (flash_port::read_byte(kPage + i) != 0xFFu)
            return false;
    return true;
}

int flash_selftest_main(void)
{
    flash_port::init(kPage, kSize); // 解锁 + 安全区 = 整页

    // ── 1 擦除整页 + 读回全 0xFF ────────────────────────────
    {
        bool ok = flash_port::erase(kPage) && page_is_ff();
        report("erase_readback_ff", ok, "erase+verify whole page");
    }
    // ── 2 write_word / read_word 往返 ───────────────────────
    {
        static const uint32_t v[] = {0x00000000u, 0xFFFFFFFFu,
                                     0xA5C35A5Au, 0x12345678u};
        bool ok = true;
        for (int i = 0; i < 4; i++)
            if (!flash_port::write_word(kPage + (uint32_t)i * 4u, v[i]) ||
                flash_port::read_word(kPage + (uint32_t)i * 4u) != v[i])
                ok = false;
        report("word_roundtrip", ok, "4 words @page+0..15");
    }
    // ── 3 write_byte 往返（契约内：每个 2B 半字每擦除周期只写一次） ──
    {
        flash_port::erase(kPage);
        bool ok = true;
        int bad_i = -1, got = -1, exp = -1;
        // 8 个独立半字各写 1 字节（偶地址，步长 2）
        for (int i = 0; i < 8 && ok; i++)
        {
            uint8_t b = (uint8_t)(0xA0u + i);
            bool w = flash_port::write_byte(kPage + (uint32_t)i * 2u, b);
            uint8_t r = flash_port::read_byte(kPage + (uint32_t)i * 2u);
            if (!w || r != b)
            {
                ok = false;
                bad_i = i;
                got = (int)r;
                exp = (int)b;
            }
        }
        // 奇地址独立半字首次写（+0x11 所在半字 +0x10 未被占用）
        bool w2 = flash_port::write_byte(kPage + 0x11u, 0x5Au);
        bool r2 = flash_port::read_byte(kPage + 0x11u) == 0x5Au;
        report("byte_roundtrip", ok && w2 && r2,
               "first_mismatch i=%d got=0x%02X exp=0x%02X odd=%d/%d",
               bad_i, got, exp, (int)w2, (int)r2);
    }
    // ── 4 write_bytes 批量 1000B + 整段读回比对 ──────────────
    {
        static uint8_t pat[1000];
        for (int i = 0; i < 1000; i++)
            pat[i] = (uint8_t)(3u + i * 7u);
        flash_port::erase(kPage);
        bool w = flash_port::write_bytes(kPage + 0x100u, pat, 1000);
        int bad_i = -1, got = -1, exp = -1;
        for (int i = 0; i < 1000; i++)
        {
            uint8_t r = flash_port::read_byte(kPage + 0x100u + (uint32_t)i);
            if (r != pat[i] && bad_i < 0)
            {
                bad_i = i;
                got = (int)r;
                exp = (int)pat[i];
            }
        }
        report("bytes_roundtrip", w && bad_i < 0,
               "w=%d first_mismatch i=%d got=0x%02X exp=0x%02X",
               (int)w, bad_i, got, exp);
    }
    // ── 5 越界/写保护：返回 false 且 diag 正确 ───────────────
    {
        // 页外擦除（低一页 0x0807F000 不在安全区）→ stage2
        bool e1 = flash_port::erase(kPage - 0x800u);
        bool d1 = !e1 && flash_port::diag_fail_stage == 2;
        // 芯片外擦除 → stage1
        bool e2 = flash_port::erase(kEnd);
        bool d2 = !e2 && flash_port::diag_fail_stage == 1;
        // 区域外写 → false
        bool w1 = flash_port::write_byte(kPage - 1u, 0x11);
        bool w2 = flash_port::write_word(kEnd, 0x11223344u);
        report("oob_rejected", (d1 && d2 && !w1 && !w2),
               "erase_below=%d/%u erase_chipend=%d/%u w_byte=%d w_word=%d",
               (int)e1, (unsigned)flash_port::diag_fail_stage,
               (int)e2, (unsigned)flash_port::diag_fail_stage, (int)w1, (int)w2);
        // 上面误报 stage 显示取最后一次，仅在 FAIL 时作参考
    }
    // ── 6 0→1 写保护语义：erase 后写 0x00，再写 0xFF 应被拒 ──
    {
        flash_port::erase(kPage);
        bool w0 = flash_port::write_byte(kPage, 0x00);
        bool wf = flash_port::write_byte(kPage, 0xFF); // 0→1 应 false
        bool r0 = flash_port::read_byte(kPage) == 0x00;
        report("bit0to1_guard", w0 && !wf && r0,
               "w0=%d wFF=%d read0=%d", (int)w0, (int)wf, (int)r0);
    }
    // ── 7 半字二次编程契约：同半字第二次编程必须被硬件拒绝 ──
    {
        // a) 同地址 1→0 重写（0x11 -> 0x00）：非擦除态 → 应被拒且内容不变
        flash_port::erase(kPage);
        bool a1 = flash_port::write_byte(kPage, 0x11);
        bool a2 = !flash_port::write_byte(kPage, 0x00);
        bool a3 = flash_port::read_byte(kPage) == 0x11;
        // b) 偶字节写后再写同半字奇字节：应被拒
        flash_port::erase(kPage);
        bool b1 = flash_port::write_byte(kPage, 0x11);
        bool b2 = !flash_port::write_byte(kPage + 1u, 0x36);
        bool b3 = flash_port::read_byte(kPage + 1u) == 0xFF;
        // c) 奇地址独立半字首次写：应成功
        flash_port::erase(kPage);
        bool c1 = flash_port::write_byte(kPage + 1u, 0x36);
        bool c2 = flash_port::read_byte(kPage + 1u) == 0x36;
        report("reprogram_contract", a1 && a2 && a3 && b1 && b2 && b3 && c1 && c2,
               "same_hw=%d/%d/%d odd_after_even=%d/%d/%d odd_first=%d/%d",
               (int)a1, (int)a2, (int)a3, (int)b1, (int)b2, (int)b3, (int)c1, (int)c2);
    }

    outf("[R] SUMMARY total=%d pass=%d fail=%d", g_total, g_pass, g_fail);
    out("[DONE]");
    return g_fail;
}
