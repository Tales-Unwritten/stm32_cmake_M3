# ============================================================
# IAP 构建（由根 CMakeLists 在 BUILD_IAP=ON 时 add_subdirectory 引入）
#
# 产出三个独立固件：
#   gd32f470_boot.elf    Bootloader（0x08000000, 128K, 仅 SWD 更新）
#   gd32f470_app_a.elf   App 槽 A（0x08020000, 512K）
#   gd32f470_app_b.elf   App 槽 B（0x080A0000, 512K）
#
# 完全独立于主 target，不影响现有 debug/release 构建。
# ============================================================

set(IAP_MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# ── 路径 ──────────────────────────────────────────────────

set(IAP_FW_DIR   ${CMAKE_SOURCE_DIR}/Firmware)
set(IAP_CMSIS    ${IAP_FW_DIR}/CMSIS)
set(IAP_LIB      ${IAP_FW_DIR}/library)

set(IAP_STARTUP  ${IAP_CMSIS}/GD/GD32F4xx/Source/GCC/startup_gd32f450_470.S)
set(IAP_SYSCALLS ${IAP_CMSIS}/GD/GD32F4xx/Source/GCC/newlib/syscalls.c)
set(IAP_SYSTEM   ${IAP_CMSIS}/GD/GD32F4xx/Source/system_gd32f4xx.c)

set(IAP_INCLUDES
    ${IAP_CMSIS}
    ${IAP_CMSIS}/GD/GD32F4xx/Include
    ${IAP_LIB}/Include
    ${CMAKE_SOURCE_DIR}/code/inc
    ${CMAKE_SOURCE_DIR}/hardware/inc
    ${CMAKE_SOURCE_DIR}/interface/inc
    ${CMAKE_SOURCE_DIR}/device/inc
    ${CMAKE_SOURCE_DIR}/user/inc
    ${CMAKE_SOURCE_DIR}/protocol/modbus/inc
    ${CMAKE_SOURCE_DIR}/protocol/string/inc
    ${CMAKE_SOURCE_DIR}/protocol/hex/inc
    ${CMAKE_SOURCE_DIR}/app/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/common/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/boot/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/application/inc
)

# GD32 标准外设库
set(IAP_LIB_SOURCES
    ${IAP_LIB}/Source/gd32f4xx_rcu.c
    ${IAP_LIB}/Source/gd32f4xx_gpio.c
    ${IAP_LIB}/Source/gd32f4xx_misc.c
    ${IAP_LIB}/Source/gd32f4xx_usart.c
    ${IAP_LIB}/Source/gd32f4xx_fmc.c
    ${IAP_LIB}/Source/gd32f4xx_timer.c
    ${IAP_LIB}/Source/gd32f4xx_adc.c
    ${IAP_LIB}/Source/gd32f4xx_dac.c
    ${IAP_LIB}/Source/gd32f4xx_dma.c
    ${IAP_LIB}/Source/gd32f4xx_exti.c
    ${IAP_LIB}/Source/gd32f4xx_syscfg.c
    ${IAP_LIB}/Source/gd32f4xx_spi.c
    ${IAP_LIB}/Source/gd32f4xx_can.c
    ${IAP_LIB}/Source/gd32f4xx_i2c.c
    ${IAP_LIB}/Source/gd32f4xx_rtc.c
    ${IAP_LIB}/Source/gd32f4xx_pmu.c
    ${IAP_LIB}/Source/gd32f4xx_fwdgt.c
)

# App 应用源（与主 target 相同，注入 APP_IAP / APP_SLOT_* 宏）
file(GLOB IAP_DEVICE_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/device/src/*.cpp
)

set(IAP_APP_SOURCES
    ${IAP_STARTUP}
    ${IAP_SYSCALLS}
    ${IAP_SYSTEM}
    ${IAP_LIB_SOURCES}

    # ── 内核 / 调度 ──
    ${CMAKE_SOURCE_DIR}/code/src/main.c
    ${CMAKE_SOURCE_DIR}/code/src/gd32f4xx_it.c
    ${CMAKE_SOURCE_DIR}/code/src/systick.c
    ${CMAKE_SOURCE_DIR}/code/src/link.c

    # ── 硬件层（旧 C 驱动） ──
    ${CMAKE_SOURCE_DIR}/hardware/src/hard_bsp_led.c
    ${CMAKE_SOURCE_DIR}/hardware/src/hard_eeprom.c

    # ── 接口抽象层 ──
    ${CMAKE_SOURCE_DIR}/interface/src/inter_io_ctrl.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_usart.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_spi.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_timer.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_can.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_i2c_hw.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_adc.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_dac.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_dma.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_exti.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_wdt.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_rtc.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_flash.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_i2c_bus.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_i2c_dev.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_soft_spi.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_soft_uart.cpp

    # ── device 设备驱动 ──
    ${IAP_DEVICE_SOURCES}

    # ── 协议栈 ──
    ${CMAKE_SOURCE_DIR}/protocol/modbus/src/modbus_crc.cpp
    ${CMAKE_SOURCE_DIR}/protocol/modbus/src/modbus_map.cpp
    ${CMAKE_SOURCE_DIR}/protocol/modbus/src/modbus.cpp
    ${CMAKE_SOURCE_DIR}/protocol/string/src/string_pro.cpp
    ${CMAKE_SOURCE_DIR}/protocol/string/src/str_cmd.cpp
    ${CMAKE_SOURCE_DIR}/protocol/hex/src/hex.cpp

    # ── 应用协议层 ──
    ${CMAKE_SOURCE_DIR}/app/src/private.cpp
    ${CMAKE_SOURCE_DIR}/app/src/protocol_conf.cpp
    ${CMAKE_SOURCE_DIR}/app/src/pc_task.cpp

    # ── 用户应用 ──
    ${CMAKE_SOURCE_DIR}/user/src/founction.cpp

    # ── IAP 应用侧（application + common） ──
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/iap_crc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/iap_param.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/iap_app.cpp
)

# ── 公共选项 ──────────────────────────────────────────────

set(IAP_COMMON_FLAGS
    ${IAP_MCU_FLAGS}
    -Wall
    -Wextra
    -fdata-sections
    -ffunction-sections
    -fno-common
)

set(IAP_COMMON_DEFS
    GD32F470
    USE_STDPERIPH_DRIVER
    HSE_VALUE=25000000U
)

set(IAP_COMMON_LINK
    ${IAP_MCU_FLAGS}
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# ============================================================
# Boot
# ============================================================

add_executable(gd32f470_boot.elf)

target_sources(gd32f470_boot.elf PRIVATE
    ${IAP_STARTUP}
    ${IAP_SYSCALLS}
    ${IAP_SYSTEM}
    ${IAP_LIB}/Source/gd32f4xx_rcu.c
    ${IAP_LIB}/Source/gd32f4xx_gpio.c
    ${IAP_LIB}/Source/gd32f4xx_misc.c
    ${IAP_LIB}/Source/gd32f4xx_usart.c
    ${IAP_LIB}/Source/gd32f4xx_fmc.c
    ${IAP_LIB}/Source/gd32f4xx_fwdgt.c

    # ── 系统 / 驱动 ──
    ${CMAKE_SOURCE_DIR}/code/src/systick.c
    ${CMAKE_SOURCE_DIR}/interface/src/inter_io_ctrl.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_usart.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_flash.cpp
    ${CMAKE_SOURCE_DIR}/interface/src/inter_wdt.cpp

    # ── IAP 核心（boot + common） ──
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/iap_crc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/iap_param.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/boot/src/iap_proto.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/boot/src/iap_boot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/boot/src/boot_main.cpp
)

target_include_directories(gd32f470_boot.elf PRIVATE ${IAP_INCLUDES})
target_compile_definitions(gd32f470_boot.elf PRIVATE ${IAP_COMMON_DEFS})

target_compile_options(gd32f470_boot.elf PRIVATE
    ${IAP_COMMON_FLAGS}
    -Os            # boot 固定尺寸优化，防止超出 128K 分区
)

target_link_options(gd32f470_boot.elf PRIVATE
    ${IAP_COMMON_LINK}
    -T${CMAKE_CURRENT_SOURCE_DIR}/ld/boot.ld
    -Wl,-Map=${CMAKE_BINARY_DIR}/gd32f470_boot.map
)

add_custom_command(TARGET gd32f470_boot.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
        $<TARGET_FILE:gd32f470_boot.elf>
        ${CMAKE_BINARY_DIR}/gd32f470_boot.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:gd32f470_boot.elf>
    COMMENT "Generating boot BIN"
)

# ============================================================
# App 双槽（同一源码两次链接，宏区分槽位）
# ============================================================

# 应用版本（升级包头部 + 烧录产物）
set(IAP_VERSION_STRING "1.0.0")

function(iap_add_app target_name slot_macro ld_file slot_char)
    add_executable(${target_name})

    target_sources(${target_name} PRIVATE ${IAP_APP_SOURCES})
    target_include_directories(${target_name} PRIVATE ${IAP_INCLUDES})

    target_compile_definitions(${target_name} PRIVATE
        ${IAP_COMMON_DEFS}
        APP_IAP
        ${slot_macro}
    )

    target_compile_options(${target_name} PRIVATE
        ${IAP_COMMON_FLAGS}
        $<$<CONFIG:Debug>:-Og>
        $<$<CONFIG:Debug>:-g3>
        $<$<CONFIG:Release>:-O2>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    )

    target_link_options(${target_name} PRIVATE
        ${IAP_COMMON_LINK}
        -T${CMAKE_CURRENT_SOURCE_DIR}/ld/${ld_file}
        -Wl,-Map=${CMAKE_BINARY_DIR}/${target_name}.map
    )

    # 输出 bin 名去掉 .elf 后缀（与烧录 target 引用一致）
    string(REPLACE ".elf" "" bin_name ${target_name})

    # 生成裸 bin + 带头部升级包（boot 校验格式）
    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${target_name}>
            ${CMAKE_BINARY_DIR}/${bin_name}.bin
        COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/iap_pack.py
            --input ${CMAKE_BINARY_DIR}/${bin_name}.bin
            --output ${CMAKE_BINARY_DIR}/iap_${bin_name}.bin
            --slot ${slot_char}
            --version ${IAP_VERSION_STRING}
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target_name}>
        COMMENT "Generating app BIN + upgrade image (${slot_macro})"
    )
endfunction()

iap_add_app(gd32f470_app_a.elf APP_SLOT_A app_a.ld A)
iap_add_app(gd32f470_app_b.elf APP_SLOT_B app_b.ld B)

# ============================================================
# 烧录（DAP-Link；bin 无地址信息，须显式指定分区基址）
# ============================================================

set(OCD openocd)
set(OCD_DIR ${CMAKE_SOURCE_DIR}/openocd)

add_custom_target(flash-iap-boot
    COMMAND ${OCD}
        -f ${OCD_DIR}/interface-daplink.cfg
        -f ${OCD_DIR}/target-gd32f4xx.cfg
        -c "program ${CMAKE_BINARY_DIR}/gd32f470_boot.bin 0x08000000 verify reset exit"
    DEPENDS gd32f470_boot.elf
    USES_TERMINAL
)

add_custom_target(flash-iap-app-a
    COMMAND ${OCD}
        -f ${OCD_DIR}/interface-daplink.cfg
        -f ${OCD_DIR}/target-gd32f4xx.cfg
        -c "program ${CMAKE_BINARY_DIR}/iap_gd32f470_app_a.bin 0x08020000 verify reset exit"
    DEPENDS gd32f470_app_a.elf
    USES_TERMINAL
)

add_custom_target(flash-iap-app-b
    COMMAND ${OCD}
        -f ${OCD_DIR}/interface-daplink.cfg
        -f ${OCD_DIR}/target-gd32f4xx.cfg
        -c "program ${CMAKE_BINARY_DIR}/iap_gd32f470_app_b.bin 0x080A0000 verify reset exit"
    DEPENDS gd32f470_app_b.elf
    USES_TERMINAL
)

# 整片初始化烧录：boot + app_a 升级包（带镜像头，boot 校验所需）
add_custom_target(flash-iap
    COMMAND ${OCD}
        -f ${OCD_DIR}/interface-daplink.cfg
        -f ${OCD_DIR}/target-gd32f4xx.cfg
        -c "program ${CMAKE_BINARY_DIR}/gd32f470_boot.bin 0x08000000 verify"
        -c "program ${CMAKE_BINARY_DIR}/iap_gd32f470_app_a.bin 0x08020000 verify reset exit"
    DEPENDS gd32f470_boot.elf gd32f470_app_a.elf
    USES_TERMINAL
)
