cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
#
# C++20：支持 designated initializers，Config 可只配置部分字段
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32_cmake_M3)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages（C/C++ 混编）
enable_language(C ASM CXX)

# ── 片上 Flash 驱动自测开关（默认 OFF，不影响正常固件） ──────────
# ON 时：
#   1. 追加 function/src/flash_selftest.cpp（片上自测主体）
#   2. 定义 ENABLE_FLASH_SELFTEST，function_init() 改为执行自测
#   3. 使用 preset Selftest 构建到 build/selftest
option(FLASH_SELFTEST "Build with on-chip flash_port selftest harness" OFF)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ----------------------------------------------------------------------------
# 源码添加与管理方式：
#   1) 路径一律相对于项目根目录（CMakeLists.txt 所在目录）
#   2) 按“层”分区管理，每层一段，方便查找与开关
#   3) .c   源文件自动按 C11   编译
#      .cpp 源文件自动按 C++20 编译
#   4) 新增文件：先按分区写好路径（先用 # 注释掉），需要时取消注释即可
# ----------------------------------------------------------------------------
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # ---- function：基础功能 / 算法 ----------------
    function/src/function.cpp

    # ---- app：应用任务层 ---------------------------
    # app/src/pc_task.cpp
    # app/src/private.cpp
    # app/src/protocol_conf.cpp

    # ---- interface：外设接口封装层 -----------------
    interface/src/inter_io_ctrl.cpp
    interface/src/inter_i2c_bus.cpp
    interface/src/inter_i2c_dev.cpp
    interface/src/inter_soft_spi.cpp
    interface/src/inter_soft_uart.cpp
    interface/src/inter_usart.cpp
    interface/src/inter_soft_uart.cpp
    interface/src/inter_spi.cpp
    interface/src/inter_flash.cpp


    # interface/src/inter_adc.cpp
    # interface/src/inter_timer.cpp

    # ---- device：器件驱动层 ------------------------
    # device/src/device_w25q128.cpp
    device/src/device_ds18b20.cpp
    device/src/device_ina226.cpp
    device/src/device_ina228.cpp
    device/src/device_serial.cpp
    device/src/device_w25qxx.cpp

    # ---- protocol：通信协议层 ----------------------
    # protocol/modbus/src/xxx.cpp
    # protocol/string/src/xxx.cpp
)

# Add include paths（头文件搜索路径，同样按分区管理）
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    function/inc
    interface/inc
    device/inc
    app/inc
    protocol/modbus/inc
    protocol/string/inc
)

# clangd 使用 clang 前端，不知道 arm-none-eabi-g++ 的隐式标准库路径，
# 会报 'cstdint' file not found。这里把编译器隐式 include 目录以 -isystem
# 显式写进编译命令（compile_commands.json），仅作用于 CXX 编译；
# 这些目录本就是 g++ 默认搜索路径，加入后不改变实际构建行为。
# 注：不能用 target_include_directories(SYSTEM) —— CMake 会自动去重隐式目录，
# 必须用 target_compile_options 注入，才能进入 compile_commands.json。
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-isystem$<JOIN:${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES},;-isystem>>
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    # 芯片实际容量：STM32F103VET6 = 512KB（inter_flash.hpp 默认 128KB 仅适用于
    # C8T6/RCT6，必须在此覆盖，否则驱动对 >128KB 地址的擦除会静默失败）
    FLASH_CAPACITY_KB=512
)

# 自测固件：仅在 FLASH_SELFTEST=ON 时追加源文件与编译宏
if(FLASH_SELFTEST)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        function/src/flash_selftest.cpp
    )
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        ENABLE_FLASH_SELFTEST=1
    )
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)



# ============================================================================
# 构建后处理：生成 .hex / .bin 并显示内存占用
# ============================================================================

# 获取目标文件名（使用生成器表达式，自动处理 .elf 后缀等问题）
set(TARGET_NAME ${CMAKE_PROJECT_NAME})

# 生成 .hex 文件
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.hex
    COMMAND echo "  [+] ${TARGET_NAME}.hex generated"
    COMMENT "Generating HEX file..."
    BYPRODUCTS ${TARGET_NAME}.hex
    VERBATIM
)

# 生成 .bin 文件
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.bin
    COMMAND echo "  [+] ${TARGET_NAME}.bin generated"
    COMMENT "Generating BIN file..."
    BYPRODUCTS ${TARGET_NAME}.bin
    VERBATIM
)

# 显示内存占用
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
    COMMAND echo ""
    COMMAND echo "=================================================="
    COMMAND echo "  Memory Usage Summary"
    COMMAND echo "=================================================="
    COMMENT "Analyzing memory usage..."
    VERBATIM
)
