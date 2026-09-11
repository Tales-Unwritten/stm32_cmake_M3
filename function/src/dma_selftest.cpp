// ============================================================
// DMA 内存搬运（M2M）自测 —— STM32F103VET6
// ============================================================
// 仅在 DMA_SELFTEST=ON 的固件里由 function_init() 调用。
// 通信通道：debug_uart（USART1 PA9/PA10 115200，由调用方 DebugPort_Init 初始化）。
//
// 通道选 DMA1_Channel4（0-based 3）：避开 ADC1 硬件固定的 DMA1_Channel1。
// 覆盖用例：
//   1) 4 字节对齐 + 长度为 4 的倍数 → 自动选 WORD 宽度
//   2) 长度非 4 的倍数              → 尾字节由 CPU 补齐
//   3) 源/目的非 4 字节对齐          → 自动降为 BYTE 宽度
//   4) 2 字节对齐（非 4）            → 自动降为 HALFWORD 宽度
//   5) 同通道连续搬运 4096B          → Abort → 重配 → 重启 链路
//   6) M2M 之后切回"外设→内存"       → 断言 CCR.MEM2MEM 已清（F1 HAL 残留坑）
//   7) 4096B 耗时：DMA M2M vs CPU memcpy
//      实测（STM32F103 @72MHz, Release）：DMA 6991cyc / CPU 36883cyc（DMA 侧含每次配置开销）。
//      注意 CPU 侧用的是当前工具链的 memcpy（newlib-nano 的 C 实现，约 9cyc/B）；
//      若换成 LDM/STM 优化版 memcpy，这个差距会明显缩小，不要拿本行当"DMA 一定更快"的结论。
//   8) remaining()（读 CNDTR）：搬完后必须为 0；循环模式下是活值（不定长接收的基石）
//   9) ISR 路由：给 DMA1_CH4 开 NVIC + TC 中断，验证强符号 ISR → HAL_DMA_IRQHandler → 回调
//  10) USART1 DMA 收发回环（需上位机配合）：DMA 收 → DMA 原样回发
//  11) SPI1 + W25Q64：同一段数据，字节路径 vs DMA 路径逐字节一致
// 输出：每项一行 [DMA] ... PASS/FAIL，结尾 [DMA] SUMMARY + [DMA] DONE
// ============================================================

#include "dma_selftest.hpp"

#include "device_serial.hpp"
#include "device_w25qxx.hpp"
#include "inter_dma.hpp"
#include "inter_nvic.hpp"
#include "inter_spi.hpp"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace
{
constexpr uint32_t kBufBytes = 4096; // 测试缓冲大小
constexpr uint32_t kTimeoutMs = 100; // 单次搬运超时

uint8_t g_src[kBufBytes + 8]; // 多 8 字节：给"非对齐起点"用例留余量
uint8_t g_dst[kBufBytes + 8];
uint32_t g_fail = 0;

volatile uint32_t g_isr_hits = 0; // 用例 9：ISR 命中次数

/* 用例 9 用：挂到 DMA 句柄上的完成回调（HAL 会判空，此处故意设一个） */
void on_dma_cplt(DMA_HandleTypeDef *)
{
    g_isr_hits = g_isr_hits + 1U;
}

void dma_out(const char *s)
{
    debug_uart.send_data((const uint8_t *)s, (uint16_t)strlen(s));
    debug_uart.send_data((const uint8_t *)"\r\n", 2);
}

void dma_outf(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    dma_out(line);
}

/* DWT 周期计数（仅本文件自测用；CYCCNT@72MHz 约 59s 回绕） */
void dwt_init()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
uint32_t dwt_cycles()
{
    return DWT->CYCCNT;
}

/* 一次搬运用例：清空目的缓冲 → 搬运 → 逐字节比对 */
void run_case(dma_channel &dma, const char *name, uint32_t src_off, uint32_t dst_off, uint32_t bytes)
{
    memset(g_dst, 0, sizeof(g_dst));
    const HAL_StatusTypeDef st = dma.copy_memory(g_dst + dst_off, g_src + src_off, bytes, kTimeoutMs);

    // 从 CCR 反查实际选中的数据宽度（PSIZE：0=byte 1=halfword 2=word）
    const uint32_t psize = (dma.hal_handle()->Instance->CCR & DMA_CCR_PSIZE) >> DMA_CCR_PSIZE_Pos;
    const char *w = (psize == 2U) ? "word" : ((psize == 1U) ? "half" : "byte");

    uint32_t bad = 0;
    bool ok = (st == HAL_OK);
    for (uint32_t i = 0; ok && (i < bytes); i++)
    {
        if (g_dst[dst_off + i] != g_src[src_off + i])
        {
            ok = false;
            bad = i;
        }
    }

    if (!ok)
        g_fail++;
    if (ok)
        dma_outf("[DMA] %-13s %5luB off=%lu/%lu st=%d w=%-4s PASS", name, (unsigned long)bytes, (unsigned long)src_off,
                 (unsigned long)dst_off, (int)st, w);
    else
        dma_outf("[DMA] %-13s %5luB off=%lu/%lu st=%d w=%-4s FAIL @%lu", name, (unsigned long)bytes,
                 (unsigned long)src_off, (unsigned long)dst_off, (int)st, w, (unsigned long)bad);
}
} // namespace

int dma_selftest_main(void)
{
    static dma_channel dma({dma_id::dma1, 3, DMA_PRIORITY_LOW}); // DMA1_Channel4

    dma_out("[DMA] memcpy selftest: M2M on DMA1_CH4 (ADC keeps DMA1_CH1)");
    dma.init();
    if (!dma.is_initialized())
    {
        dma_out("[DMA] FAIL init");
        return 1;
    }

    // 源数据含 0x00/0xA5 交替，避免"没搬也像成功"（目的缓冲每次清零）
    for (uint32_t i = 0; i < sizeof(g_src); i++)
        g_src[i] = (uint8_t)(i * 7U + ((i & 1U) ? 0xA5U : 0x00U));

    run_case(dma, "align-256B", 0, 0, 256); // 4 对齐 + 4 的倍数 → word
    run_case(dma, "len-259B", 0, 0, 259);   // 尾 3 字节由 CPU 补
    run_case(dma, "off1-130B", 1, 1, 130);  // 起点非 4 对齐 → byte
    run_case(dma, "off2-66B", 2, 2, 66);    // 2 对齐非 4 → halfword
    run_case(dma, "repeat-4096B", 0, 0, 4096);

    // M2M → 外设模式：CCR.MEM2MEM 必须已清。F1 的 HAL_DMA_Init() 清位掩码不含
    // 该位，靠 dma_channel::_apply_config() 先 DeInit 兜底；否则之后配成外设模式时
    // DMA 会脱离外设请求自由狂奔（缓冲区瞬间填满）。
    dma.set_direction(DMA_PERIPH_TO_MEMORY);
    dma.set_increment(false, true);
    dma.set_circular(true);
    const HAL_StatusTypeDef st_cfg = dma.hal_configure();
    const uint32_t ccr = dma.hal_handle()->Instance->CCR;
    const uint32_t m2m = (ccr & DMA_CCR_MEM2MEM) ? 1U : 0U;
    const bool residue_ok = (st_cfg == HAL_OK) && (m2m == 0U);
    if (!residue_ok)
        g_fail++;
    dma_outf("[DMA] %-13s MEM2MEM=%lu DIR=%lu st=%d %s", "mem2mem-residue", (unsigned long)m2m,
             (unsigned long)((ccr & DMA_CCR_DIR) ? 1U : 0U), (int)st_cfg, residue_ok ? "PASS" : "FAIL");

    // 4096B 搬运耗时对比（DMA 侧含每次配置开销）
    dwt_init();
    memset(g_dst, 0, kBufBytes);
    uint32_t t0 = dwt_cycles();
    memcpy(g_dst, g_src, kBufBytes);
    const uint32_t cyc_cpu = dwt_cycles() - t0;

    memset(g_dst, 0, kBufBytes);
    t0 = dwt_cycles();
    (void)dma.copy_memory(g_dst, g_src, kBufBytes, kTimeoutMs);
    const uint32_t cyc_dma = dwt_cycles() - t0;

    const uint32_t us = SystemCoreClock / 1000000U;
    dma_outf("[DMA] 4096B cpu=%lucyc(%luus) dma=%lucyc(%luus)", (unsigned long)cyc_cpu, (unsigned long)(cyc_cpu / us),
             (unsigned long)cyc_dma, (unsigned long)(cyc_dma / us));

    // 8) remaining()（读 CNDTR）
    // 8a 普通模式搬完后必须归零（确定性）
    dma.set_width(DMA_PDATAALIGN_WORD);
    dma.set_direction(DMA_MEMORY_TO_MEMORY);
    dma.set_increment(true, true);
    dma.set_circular(false);
    (void)dma.copy_memory(g_dst, g_src, kBufBytes, kTimeoutMs);
    const uint32_t rem_done = dma.remaining();
    const bool rem_done_ok = (rem_done == 0U);
    if (!rem_done_ok)
        g_fail++;
    dma_outf("[DMA] %-13s after-done rem=%lu %s", "remaining", (unsigned long)rem_done, rem_done_ok ? "PASS" : "FAIL");

    // 8b 循环模式下是活值：刚启动时应落在 (0, count]（这是"已收字节数 = count - remaining"的基石）
    dma.set_circular(true);
    dma.start(g_src, g_dst, kBufBytes / 4); // 1024 个 word，循环自由奔跑
    const uint32_t rem_a = dma.remaining();
    HAL_Delay(1); // 1ms 后已跑很多圈，取第二个采样看是否为活值
    const uint32_t rem_b = dma.remaining();
    dma.stop();
    const bool rem_live_ok = (rem_a <= kBufBytes / 4) && (rem_a > 0U);
    if (!rem_live_ok)
        g_fail++;
    dma_outf("[DMA] %-13s circular rem=%lu->%lu max=%lu %s", "remaining", (unsigned long)rem_a,
             (unsigned long)rem_b, (unsigned long)(kBufBytes / 4), rem_live_ok ? "PASS" : "FAIL");

    // 9) ISR 路由：给新通道开 NVIC + TC 中断，验证 强符号 ISR → HAL_DMA_IRQHandler → 回调
    //    （M2 外设走的就是这条链；此处用 M1 自挂回调等价验证，不依赖任何外设）
    g_isr_hits = 0;
    dma.set_circular(false);
    dma.hal_handle()->XferCpltCallback = on_dma_cplt;
    __HAL_DMA_ENABLE_IT(dma.hal_handle(), DMA_IT_TC); // 只有 Start_IT 才自动开中断，这里手动开
    nvic().set_priority(DMA1_Channel4_IRQn, 6, 0);
    nvic().enable(DMA1_Channel4_IRQn);
    dma.start(g_src, g_dst, 1024); // 1024 个 word，约 100us 后完成
    HAL_Delay(2);                  // 等 TC 中断
    nvic().disable(DMA1_Channel4_IRQn);
    dma.hal_handle()->XferCpltCallback = nullptr;
    const bool isr_ok = (g_isr_hits == 1U);
    if (!isr_ok)
        g_fail++;
    dma_outf("[DMA] %-13s hooks=%lu %s", "isr-routing", (unsigned long)g_isr_hits, isr_ok ? "PASS" : "FAIL");

    // 10) USART1 DMA 收发回环：上位机在窗口内发字节 → DMA 收 → 原样 DMA 回发
    //     需要上位机配合；没发数据则报 SKIP（不计入失败）
    dma.deinit(); // 先让出 DMA1_CH4：通道是独占资源，USART1-TX 正要用它

    if (!debug_uart.enable_dma())
    {
        dma_out("[DMA] uart-loopback SKIP (enable_dma failed)");
    }
    else
    {
        static uint8_t echo[64];
        if (!debug_uart.receive_dma_start(echo, sizeof(echo)))
        {
            g_fail++;
            dma_out("[DMA] uart-loopback FAIL (receive_dma_start)");
        }
        else
        {
            dma_out("[DMA] uart-loopback: send bytes from PC within 3s ...");
            uint16_t got = 0;
            uint16_t last = 0;
            const uint32_t t0 = HAL_GetTick();
            uint32_t t_last = t0;
            while ((uint32_t)(HAL_GetTick() - t0) < 3000U)
            {
                const uint16_t n = debug_uart.rx_dma_count();
                if (n != last)
                {
                    last = n;
                    t_last = HAL_GetTick();
                }
                if ((n > 0U) && ((uint32_t)(HAL_GetTick() - t_last) > 50U))
                {
                    got = n; // 连续 50ms 无新字节 → 当作一帧结束
                    break;
                }
                HAL_Delay(1);
            }
            debug_uart.receive_dma_stop();

            if (got == 0U)
            {
                dma_out("[DMA] uart-loopback SKIP (no data from PC)");
            }
            else
            {
                const bool sent = debug_uart.send_data_dma(echo, got);
                if (!sent)
                    g_fail++;
                dma_outf("[DMA] uart-loopback rx=%u tx=%s %s", (unsigned)got, sent ? "ok" : "ng",
                         sent ? "PASS" : "FAIL");
            }
        }
    }

    // 11) SPI1 + W25Q64：同一段数据，字节路径 vs DMA 路径必须逐字节一致
    //     从机接线见 function.cpp：W25Q64 接在 PA5(SCK)/PA7(MOSI)/PA6(MISO)/PA4(CS)
    //     只读不写：不动 flash 里的任何数据
    static spi_port spi_flash({.periph = spi_id::spi1,
                               .sck_port = GPIOA,
                               .sck_pin = pin5,
                               .mosi_port = GPIOA,
                               .mosi_pin = pin7,
                               .miso_port = GPIOA,
                               .miso_pin = pin6,
                               .cs_port = GPIOA,
                               .cs_pin = pin4,
                               .cs_active_level = active_low,
                               .af = afio_enum_t::NONE,
                               .prescaler = SPI_BAUDRATEPRESCALER_4, // 72/4 = 18MHz（F103 SPI1 上限）
                               .clock_polarity = SPI_POLARITY_LOW,
                               .clock_phase = SPI_PHASE_1EDGE, // Mode 0，同软 SPI 的 soft_mode_0
                               .first_bit = SPI_FIRSTBIT_MSB,
                               .data_size = SPI_DATASIZE_8BIT});
    static w25qxx flash_hw(spi_flash);

    spi_flash.init();
    if (!spi_flash.is_initialized() || !spi_flash.enable_dma())
    {
        g_fail++;
        dma_outf("[DMA] %-13s FAIL (spi init=%d dma=%d)", "spi-w25q64", (int)spi_flash.is_initialized(),
                 (int)spi_flash.dma_enabled());
    }
    else
    {
        flash_hw.init();
        const uint32_t id_byte = flash_hw.jedec_id(); // 字节路径：期望 0xEF4017(W25Q64)

        // DMA 路径读同一条命令（0x9F + 3 字节全双工）
        uint8_t id_tx[4] = {0x9FU, 0xFFU, 0xFFU, 0xFFU};
        uint8_t id_rx[4] = {0};
        spi_flash.cs_select();
        const bool id_dma_ok = spi_flash.transfer_dma(id_tx, id_rx, sizeof(id_tx));
        spi_flash.cs_deselect();
        const uint32_t id_dma = ((uint32_t)id_rx[1] << 16) | ((uint32_t)id_rx[2] << 8) | (uint32_t)id_rx[3];

        // 读 512 字节：字节路径 vs DMA 路径（一次 CS 帧：命令+地址+数据）
        constexpr uint16_t kRd = 512;
        static uint8_t rd_byte[kRd];
        static uint8_t frame_tx[4 + kRd];
        static uint8_t frame_rx[4 + kRd];

        flash_hw.read(0x000000U, rd_byte, kRd); // 字节路径

        frame_tx[0] = 0x03U; // READ DATA
        frame_tx[1] = 0x00U; // 地址 0x000000（注意要右移：设备驱动里有处 << 写反了）
        frame_tx[2] = 0x00U;
        frame_tx[3] = 0x00U;
        for (uint16_t i = 4; i < sizeof(frame_tx); i++)
            frame_tx[i] = 0xFFU;

        spi_flash.cs_select();
        const bool rd_dma_ok = spi_flash.transfer_dma(frame_tx, frame_rx, sizeof(frame_tx));
        spi_flash.cs_deselect();

        uint32_t sum = 0;
        bool same = rd_dma_ok;
        for (uint16_t i = 0; i < kRd; i++)
        {
            sum += frame_rx[4 + i];
            if (frame_rx[4 + i] != rd_byte[i])
                same = false;
        }

        const bool id_ok = id_dma_ok && (id_byte == 0xEF4017U) && (id_dma == id_byte);
        if (!id_ok || !same)
            g_fail++;
        dma_outf("[DMA] %-13s id=%06lX/%06lX rd=%u sum=%08lX %s", "spi-w25q64", (unsigned long)id_byte,
                 (unsigned long)id_dma, (unsigned)kRd, (unsigned long)sum, (id_ok && same) ? "PASS" : "FAIL");

        // ── 强验证（会写 flash，测完擦回空白复原）────────────────
        //  擦除最后一扇区 → 写入 256B 非常量图案 → DMA 读回逐字节比对
        //  只有 ID 正常（确认是预期器件、容量可信）才动它，避免误擦未知器件
        if (id_ok)
        {
            const uint32_t cap_bytes = 1UL << (id_byte & 0xFFU); // 0x17 → 2^23 = 8MB
            const uint32_t sector = cap_bytes - 4096U;           // 最后一个 4KB 扇区
            static uint8_t pat[256];
            for (uint32_t i = 0; i < sizeof(pat); i++)
                pat[i] = (uint8_t)(i * 37U + 0x5AU);

            flash_hw.write_enable(); // 该驱动约定：擦/写前由调用方发 WREN
            flash_hw.sector_rease(sector);
            flash_hw.write_enable();
            flash_hw.write(sector, pat, sizeof(pat));

            frame_tx[0] = 0x03U; // READ DATA @ sector
            frame_tx[1] = (uint8_t)(sector >> 16);
            frame_tx[2] = (uint8_t)(sector >> 8);
            frame_tx[3] = (uint8_t)sector;
            for (uint32_t i = 4; i < (4U + sizeof(pat)); i++)
                frame_tx[i] = 0xFFU;

            spi_flash.cs_select();
            const bool rd_pat_ok = spi_flash.transfer_dma(frame_tx, frame_rx, (uint16_t)(4U + sizeof(pat)));
            spi_flash.cs_deselect();

            uint32_t bad = 0;
            for (uint32_t i = 0; i < sizeof(pat); i++)
                if (frame_rx[4 + i] != pat[i])
                    bad++;

            flash_hw.write_enable(); // 恢复原状：擦回空白
            flash_hw.sector_rease(sector);

            const bool wr_ok = rd_pat_ok && (bad == 0U);
            if (!wr_ok)
                g_fail++;
            dma_outf("[DMA] %-13s sector=%06lX pat=%u bad=%lu %s", "w25q64-rw", (unsigned long)sector,
                     (unsigned)sizeof(pat), (unsigned long)bad, wr_ok ? "PASS" : "FAIL");
        }

        spi_flash.deinit(); // 归还 DMA 通道(Ch2/Ch3)与 GPIO
    }

    dma_outf("[DMA] SUMMARY fail=%lu", (unsigned long)g_fail);
    dma_out("[DMA] DONE");
    return (int)g_fail;
}
