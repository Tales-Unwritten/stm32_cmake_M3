#include "device_serial.hpp"
#include "device_w25qxx.hpp"
#include "function.hpp"
#include "inter_adc.hpp"
#include "inter_flash.hpp"
#include "inter_soft_spi.hpp"
#include "inter_spi_bus.hpp"
#include "stdlib.h"
#include "string"
#include "string.h"

#include "delay.h"

#include <stdarg.h>
#include <stdio.h>

#ifdef ENABLE_FLASH_SELFTEST
#include "flash_selftest.hpp"
#endif

// ============================================================
// [ADC 多通道实测代码，保留供查看；不需要时请删除本段 + function_init()
//  中的 adc_multi_verify() 调用 + 上方 inter_adc.hpp/stdarg/stdio 三个头]
// 接线：PC0~PC5（ADC1_IN10~15，6 通道）任意接 3.3V/悬空均可，
//       仅打印各通道采集电压供观察；PC6 非 ADC 引脚已跳过。
// 输出：scan 轮询每通道一次 + DMA 循环(每通道 32 帧)统计 min/avg/max
// ============================================================
static void adc_out(const char *s)
{
    debug_uart.send_data((const uint8_t *)s, (uint16_t)strlen(s));
    debug_uart.send_data((const uint8_t *)"\r\n", 2);
}

static void adc_outf(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    adc_out(line);
}

static void adc_multi_verify(void)
{
    static const char *kPinName[6] = {"PC0", "PC1", "PC2", "PC3", "PC4", "PC5"};
    static const uint16_t kFrames = 32;
    static uint16_t dma_buf[6 * 32]; // 每帧 6 通道连续排列

    static adc_port adc(AdcPortConfig{
        .periph = adc_id::adc1, // ADC1
        .channels =
            {
                {GPIOC, pin0, ADC_CHANNEL_10, rank1, ADC_SAMPLETIME_55CYCLES_5},
                {GPIOC, pin1, ADC_CHANNEL_11, rank2, ADC_SAMPLETIME_55CYCLES_5},
                {GPIOC, pin2, ADC_CHANNEL_12, rank3, ADC_SAMPLETIME_55CYCLES_5},
                {GPIOC, pin3, ADC_CHANNEL_13, rank4, ADC_SAMPLETIME_55CYCLES_5},
                {GPIOC, pin4, ADC_CHANNEL_14, rank5, ADC_SAMPLETIME_55CYCLES_5},
                {GPIOC, pin5, ADC_CHANNEL_15, rank6, ADC_SAMPLETIME_55CYCLES_5},
            },
        .channel_count = 6,
        .use_dma = true, // ADC1 -> DMA1_Channel1
        .dma_controller = dma_id::dma1,
        .dma_channel = 0,
    });

    adc_out("[ADC] multi-ch verify: PC0..PC5 (ADC1_IN10..15)");
    adc.init();

    // 路径 1：scan 轮询（逐通道软件触发，F1 无 EOCM 故逐通道转换）
    uint32_t mv[6];
    adc.scan_mv(mv);
    for (uint8_t i = 0; i < 6; i++)
    {
        adc_outf("[ADC] POLL ch%u(%s) = %lu mV", (unsigned)i, kPinName[i], (unsigned long)mv[i]);
    }

    // 路径 2：DMA 多通道循环采集（每帧 6 通道顺序刷新）
    if (!adc.dma_start(dma_buf, kFrames))
    {
        adc_out("[ADC] FAIL dma_start");
        adc_out("[DONE]");
        return;
    }
    uint32_t tmo = 0;
    while (!adc.dma_done())
    {
        if (++tmo > 200)
            break; // ~200ms 超时保护
        delay_ms(1);
    }
    if (!adc.dma_done())
    {
        adc_out("[ADC] FAIL dma_done timeout");
        adc.dma_stop();
        adc_out("[DONE]");
        return;
    }
    for (uint8_t ch = 0; ch < 6; ch++)
    {
        uint32_t sum = 0, mn = 0xFFFF, mx = 0;
        for (uint16_t f = 0; f < kFrames; f++)
        {
            const uint16_t v = dma_buf[f * 6 + ch];
            sum += v;
            if (v < mn)
                mn = v;
            if (v > mx)
                mx = v;
        }
        const uint32_t avg = sum / kFrames;
        const uint32_t avg_mv = avg * 3300UL / 4095UL;
        adc_outf("[ADC] DMA  ch%u(%s) min=%u avg=%u max=%u  ~%lu mV", (unsigned)ch, kPinName[ch], (unsigned)mn,
                 (unsigned)avg, (unsigned)mx, (unsigned long)avg_mv);
    }
    adc.dma_stop();
    adc_out("[DONE]");
}

// SCK=PA5, MOSI=PA7, MISO=PA6, CS=PA4, 低有效, MODE0, MSB, 最快
soft_spi_bus spi_bus_64({GPIOA, pin5, GPIOA, pin7, GPIOA, pin6, GPIOA, pin4, active_low, soft_mode_0, MSB, 0});

w25qxx flash64(spi_bus_64);
flash_port mimo_flash;
flash_port deep_flash;

void function_init(void)
{
    // DebugPort_Init();
#ifdef ENABLE_FLASH_SELFTEST
    // ── 片上 flash_port 自测固件：跑完用例后进主循环空转 ──────
    // 通信通道：debug_uart（USART1 PA9/PA10，与 device_serial 唯一实例共用）
    DebugPort_Init();
    flash_selftest_main();
#else

    uint8_t abs[] = {0x99, 0x89, 0x77, 0x43, 0x18};
    uint8_t momo_id = 0;
    spi_bus_64.init();
    DebugPort_Init();
    adc_multi_verify(); // [ADC 多通道实测，保留；不需要时删除此行]
    soft_485_init();
    // flash64.init();
    mimo_flash.init(0x0807F800, 0x800);
    deep_flash.init(0x0807E800, 0x800);

    mimo_flash.write_byte(0x0807F800, 0x67);
    deep_flash.erase(0x0807E800);
    // deep_flash.write_bytes(0x0807E800, abs, sizeof(abs));
    deep_flash.write_word(0x0807E804, 0x8823);

    soft_485_send(&momo_id, 1);
    delay_ms(10);
    momo_id = mimo_flash.read_byte(0x0807F800);
#endif
}

void function_loop(void)
{
    // debug_uart.send_data((uint8_t *)"data\n", strlen("data\n"));
    // soft_485_send((uint8_t *)"soft_rs485\r\n", strlen("soft_rs485\r\n"));
    adc_multi_verify();
    delay_ms(800);
}
