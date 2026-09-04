# stm32_cmake_M3

> ⚠️ **开发状态声明：本工程持续移植/开发中**
> 主固件源码一律通过 `CMakeLists.txt` **显式列出**后才参与编译，未列出或被注释的源码默认不参与构建。
> 目前 `interface/`（外设接口封装层）中仍有**大量模块未从原 GD32F450/470 工程移植完成**（详见 [interface 层移植状态](#3-interface-层移植状态)），`app/`、`protocol/` 等层多为预留/未接入，请以本文状态表为准，勿将「目录里存在源码」等同于「已移植可用」。

---

## 1. 项目简介

基于 **STM32F103VET6** 的 CMake 嵌入式工程，目标是把一套以 **GD32F450/470** 为目标平台编写的
`interface`（硬件外设封装）+ `device`（器件驱动）分层驱动库，**逐模块移植到 STM32F1xx HAL 平台**。

- **主控**：STM32F103VET6（Cortex-M3，512KB Flash；CubeMX 工程器件宏 `STM32F103xE`）
- **构建**：CMake（≥3.22）+ Ninja + `arm-none-eabi-gcc`，C11 / C++20 混编，clangd 索引（`compile_commands.json`）
- **烧录/调试**：`probe-rs`
- **驱动分层思想**：器件驱动只依赖 `interface` 的抽象接口（如 `spi_bus` 基类、`io_ctrl`），
  可在**软/硬件**外设之间透明切换；平台相关差异被收敛在 `interface` 层内部。
- **当前已完成路线**（对应 git 历史）：
  软串口/硬件串口 → SPI（软/硬统一接口）→ W25Qxx 外部 Flash → 片上 Flash 安全读写驱动（含可选自测固件）。

## 2. 目录架构

```
stm32_cmake_M3/
├── CMakeLists.txt              # 主构建入口：按“层”分块列出源文件（接入开关所在）
├── CMakePresets.json           # 构建预设：Debug / Release / selftest（片上 Flash 自测固件）
├── stm32_cmake_M3.ioc          # STM32CubeMX 工程（改引脚/外设后重新生成 Core/）
├── startup_stm32f103xe.s       # 启动文件（CubeMX 生成）
├── STM32F103xx_FLASH.ld        # 链接脚本（512KB Flash）
├── .clangd / .gitignore / .gitattributes
│
├── Core/                       # 【CubeMX 生成】main / 中断 / gpio / delay / syscalls
│   ├── Inc/  Src/              #   （经 cmake/stm32cubemx/CMakeLists.txt 自动收集编译）
├── Drivers/                    # 【CubeMX 生成】STM32F1xx HAL + CMSIS（含 CMSIS/DSP）
├── cmake/
│   ├── stm32cubemx/CMakeLists.txt   # 收集 Core/、Drivers/、startup 与 HAL 源码
│   ├── gcc-arm-none-eabi.cmake      # arm-none-eabi-gcc 工具链文件
│   └── starm-clang.cmake            # clang 工具链（备选）
│
├── app/                        # 应用任务层（预留，未接入构建）
│   ├── inc/  src/              #   pc_task / private / protocol_conf（多协议抽象头）
├── function/                   # 基础功能层：启动/轮询入口 function_init/loop
│   ├── inc/function.hpp  src/function.cpp
│   ├── inc/flash_selftest.hpp src/flash_selftest.cpp   # 仅 selftest 预设参与构建
├── interface/                  # ⚠️ 外设接口封装层（核心移植对象，部分“开发中”）
│   ├── inc/  src/              #   见第 3 节状态表
├── device/                     # 器件驱动层（基于 interface，少量已接入）
│   ├── inc/  src/              #   见第 4 节状态表
├── protocol/                   # 通信协议层
│   ├── modbus/  string/  hex/  #   源码随包携带，主工程未接入
│   └── iap/                    #   独立子工程（boot/app 分区升级，含自身 README/ld/CMakeLists）
│
├── docs/                       # 芯片手册 / 器件规格书（PDF）
├── tools/                      # 辅助脚本（flash_selftest.py 等）
├── .zed/                       # Zed 编辑器任务（构建 / 烧录 / 三远程推拉）
└── build/                      # 构建产物（Debug/Release/selftest 三个子目录）
```

| 层/文件 | 职责 | 构建状态 |
|---|---|---|
| `Core/`、`Drivers/`、`startup`、`ld` | CubeMX 生成代码、HAL/CMSIS、启动与链接 | ✅ 始终参与构建 |
| `function/` | `function_init/loop` 初始化与主循环功能 | ✅ `function.cpp`；`flash_selftest.cpp` 仅 `selftest` 预设 |
| `interface/` | 外设接口封装（GPIO/SPI/I2C/UART/Flash/…） | 🟡 部分接入，见第 3 节 |
| `device/` | 器件驱动（W25Qxx/DS18B20/INA226/INA228/…） | 🟡 少量接入，见第 4 节 |
| `app/` | 应用任务（pc_task 等，多协议任务抽象） | 🟡 预留，`CMakeLists.txt` 中已注释 |
| `protocol/` | modbus / string / hex / iap | 🟡 主工程未接入；`iap` 为独立子工程 |
| `docs/`、`tools/` | 资料与脚本 | — |
| `.zed/` | 构建 / 烧录 / 多远程 git 任务 | — |

> 源码管理约定见 `CMakeLists.txt` 顶部注释：**新增源文件先在对应分区以 `#` 注释写好路径，需要时取消注释**即可接入。

## 3. interface 层移植状态

判断依据：源码是否仍引用 `gd32*` 头文件/寄存器，以及是否已在 `CMakeLists.txt` 中接入编译。

### ✅ 已移植完成（参与构建、当前固件在使用）

| 模块 | 说明 |
|---|---|
| `inter_io_ctrl` | GPIO 输入/输出/复用（AFIO）抽象，软 I2C/软 SPI/软串口等共用 |
| `inter_spi_bus.hpp` | SPI 总线抽象基类（软/硬 SPI 统一入口，纯接口） |
| `inter_spi` | 硬件 SPI 主机（STM32 HAL） |
| `inter_soft_spi` | 软件 SPI（IO 模拟） |
| `inter_soft_uart` | 软件串口（基于 `HAL_GetTick` 超时） |
| `inter_usart` | 硬件串口（HAL UART，模板化收发缓冲） |
| `inter_i2c_bus` / `inter_i2c_dev` | 软件 I2C 总线 / 器件访问封装 |
| `inter_flash` | 片上 Flash 安全读写（F1 HAL；工程以 `FLASH_CAPACITY_KB=512` 适配 VET6） |

### 🟡 开发中（未移植完 / 未接入构建）

> 以下模块源码**源自 GD32F450/470 工程**，仍引用 `gd32*` 头文件/寄存器（如 `inter_adc.hpp` 直接 `#include "gd32f4xx.h"`），
> **尚未改造为 STM32F1 HAL**，也不在 `CMakeLists.txt` 的编译列表中——`inter_adc`/`inter_timer` 留有注释行占位，其余未列出。

| 模块 | 现状 |
|---|---|
| `inter_adc` | GD32F4 三 ADC + DMA 通道映射代码，待按 F1 重写 |
| `inter_can` | GD32 bxCAN 代码 + F1 引脚注释混杂，待移植 |
| `inter_dac` | GD32 DAC 配置，F1 无 DAC，待改造/裁剪 |
| `inter_dma` | GD32 DMA 通道/请求源映射，待移植 |
| `inter_exti` | GD32 EXTI 参考实现（含 LVD/以太网等系统线），待精简移植 |
| `inter_i2c_hw` | GD32 硬件 I2C，待移植（当前软 I2C 已够用） |
| `inter_i2c_test_simple.hpp` | I2C 自测代码（header 单测），未整理 |
| `inter_rtc` | GD32 RTC，待移植 |
| `inter_timer` | GD32 定时器（含 PWM/编码器能力），待移植 |
| `inter_wdt` | GD32 IWDG 看门狗，待移植 |

## 4. device 层接入状态

`device/inc` 下携带 50 余个器件驱动头文件（57 个头文件 / 47 个源文件，多为软 I2C/软 SPI/IO 类器件的通用实现，不直接依赖 GD32 头文件），
但**只有显式接入 `CMakeLists.txt` 的才参与编译**，其余未在本平台联调验证：

| 状态 | 器件 |
|---|---|
| ✅ 已接入编译 | `device_w25qxx`（SPI 外部 Flash）、`device_ds18b20`（单总线温度）、`device_ina226`、`device_ina228`（I2C 电流/功率监测）、`device_serial`（串口资源实例：debug/rs232/软 rs485） |
| 🟡 未接入（代码随包携带） | `device_w25q128` 及其余约 60 个驱动（24LC/AT24 EEPROM、ADS1115、BH1750、DS3232、LCD1602、MAX31855、SHT3x/4x、PCF8574、MCP23x17 等），需要时在 `CMakeLists.txt` 取消注释并实测 |

## 5. 其余层状态

- **`app/`**：`pc_task` / `private` / `protocol_conf` 为多协议应用任务框架的预留代码，`CMakeLists.txt` 已注释，**未接入**。
- **`protocol/`**：`modbus`、`string`（字符串指令协议）、`hex` 源码随包携带，主工程**未接入**；
  `protocol/iap/` 是**独立子工程**（boot/app 分区串口升级），拥有自己的 `CMakeLists.txt`、分区链接脚本（`boot.ld`/`app_a.ld`/`app_b.ld`）与说明文档，请阅读 `protocol/iap/README.md`。
- **`function/`**：`function_init()` 目前用于板级初始化与片上 Flash 读写验证；`flash_selftest.cpp` 为片上 Flash 自测主体，
  通过 `FLASH_SELFTEST=ON`（即 `selftest` 预设）单独构建。

## 6. 构建与烧录

```sh
# 构建（Debug / Release）
cmake --preset Debug && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release

# 片上 Flash 自测固件（FLASH_SELFTEST=ON）
cmake --preset selftest && cmake --build --preset selftest

# 烧录（probe-rs；构建后自动生成 .elf/.hex/.bin 并打印内存占用）
probe-rs download --speed 4000 --chip <目标芯片> build/Release/stm32_cmake_M3.elf
probe-rs reset --chip <目标芯片>
```

- Zed 中可直接运行 `.zed/tasks.json` 的 **Build / flash** 任务（含 `release and flash` 一键流程）。
- 构建后处理会自动生成 `stm32_cmake_M3.hex/.bin` 并打印 RAM/Flash 占用摘要。

## 7. 多远程同步

代码同时镜像到三个托管平台，日常同步使用 `.zed/tasks.json` 中的 git 任务：

| Remote | 地址 |
|---|---|
| `gitcode`（main 上游） | `git@gitcode.com:2402_89737597/stm32_cmake_M3.git` |
| `origin`（Gitee） | `git@gitee.com:addiction_28/stm32_cmake_-m3.git` |
| `github` | `git@github.com:Tales-Unwritten/stm32_cmake_M3.git` |

- 任务 **`git: push to all remotes (Gitee + GitHub + GitCode)`** / **`git: pull from all remotes (...)`** 一次推/拉三个远程。

## 8. 移植/接入新模块速查

1. 硬件外设（interface）：将目标外设源文件改造成 STM32F1 HAL 实现，去掉 `gd32*` 依赖；
2. 在 `CMakeLists.txt` 对应分区块取消注释（新增文件先按注释规范写好路径）；
3. 器件驱动（device）：确认其依赖的 interface 模块已接入，按同一方式列出 `device/src/xxx.cpp`；
4. 重新 `cmake --preset <x>` 构建验证，并在目标板上实测。
