# IAP 升级系统开发说明书

> 本文档用大白话 + 打比方的方式，讲清楚本工程 IAP（In-Application Programming，应用内编程）升级是怎么工作的、代码怎么组织的、以及移植到别的板子要改哪里。
> 如果你对 IAP 完全陌生，请从第 1 章顺序读到第 7 章，再回头看代码会轻松很多。

---

## 1. 这是什么

**IAP 一句话**：设备运行的时候，通过串口接收新的固件程序，自己把自己"脑子"换掉，不需要拆机、不需要仿真器。

打个比方：

- 普通烧录（用 J-Link/DAP 下载）＝ 开颅手术，医生（仿真器）直接动手换脑子；
- IAP 升级 ＝ 微创手术，医生在体外远程指挥，设备自己给自己换脑子，而且换坏了还能换回旧脑子。

这个工程里有两个"医生"：

| 角色 | 是谁 | 干什么 |
|---|---|---|
| 手术室 | Bootloader（boot） | 开机先跑它，负责收新固件、校验、写入、决定启动哪个固件 |
| 病人 | Application（app） | 你平时写的业务程序（Modbus、字符串协议、自检等等都在这里面） |

**分工一句话**：boot 管"换脑子"，app 管"过日子"。

---

## 2. 代码结构

```
protocol/iap/
├── boot/                    # 手术室（bootloader 私有代码）
│   ├── inc/
│   │   ├── iap_boot.hpp        # boot 状态机/跳转接口
│   │   └── iap_proto.hpp       # 升级协议接口（端口表定义）
│   └── src/
│       ├── boot_main.cpp       # boot 入口 main() + SysTick/HardFault 处理
│       ├── iap_boot.cpp        # boot 主流程：初始化/状态机/分区校验/跳转 app
│       └── iap_proto.cpp       # 升级协议：帧解析/命令处理/分区擦除
├── application/             # 病人侧（app 私有代码）
│   ├── inc/
│   │   └── iap_app.hpp         # app 侧 IAP 接口声明
│   └── src/
│       └── iap_app.cpp         # VTOR 重定位/喂狗/boot_ok 上报/升级请求/状态提示
├── common/                  # 手术室和病人共用的"医疗器械"
│   ├── inc/
│   │   ├── iap_conf.hpp        # 分区布局/协议常量/版本号（移植第一站）
│   │   ├── iap_image.hpp       # 升级包镜像头结构（64 字节）
│   │   ├── iap_param.hpp       # 参数区记录结构（升级状态账本）
│   │   └── iap_crc.hpp         # CRC16/CRC32 校验工具
│   └── src/
│       ├── iap_param.cpp       # 参数区双扇区日志读写
│       └── iap_crc.cpp         # 校验实现
├── ld/                      # 三个独立链接脚本
│   ├── boot.ld                 # boot 链接：0x08000000，128K
│   ├── app_a.ld                # app A 槽：0x08020040，512K
│   └── app_b.ld                # app B 槽：0x080A0040，512K
├── tools/
│   └── iap_pack.py             # 打包脚本：裸 bin → 带 64B 头部的升级包
├── CMakeLists.txt              # 三个固件 target（boot / app_a / app_b）
└── README.md                   # 本文档
```

**每个文件干什么（速查表）**：

| 文件 | 职责 | 类比 |
|---|---|---|
| `iap_conf.hpp` | 分区地址、协议常量、版本号 | 手术方案总纲 |
| `iap_boot.cpp` | 开机决策：升级 or 启动？启动哪个？ | 主刀医生 |
| `iap_proto.cpp` | 收帧、拆包、写 flash、应答 | 护士递器械 |
| `iap_app.cpp` | app 报告"我活着"、请求升级、提示状态 | 病人自报平安 |
| `iap_param.cpp` | 把升级状态记到 flash，断电不丢 | 手术记录本 |
| `iap_crc.cpp` | 校验数据有没有传错 | 清点器械 |
| `iap_pack.py` | 把编译产物加工成升级包 | 术前打包器械箱 |

---

## 3. 核心概念：分区布局（先看懂这张表）

```
Flash 2MB（GD32F470ZI）
┌──────────────────────────────────────────────┐
│ 0x08000000  Boot   128K   手术室（仅 SWD 更新）│
│ 0x08020000  AppA   512K   抽屉 A（出厂固件）   │
│ 0x080A0000  AppB   512K   抽屉 B（升级目标）   │
│ 0x08120000  Param0 128K   手术记录本 ①        │
│ 0x08140000  Param1 128K   手术记录本 ②        │
│ 0x08160000  预留   640K   未来扩展             │
└──────────────────────────────────────────────┘
```

**双分区（A/B）思想**——这是"严格可回滚"的根基：

把固件想象成两个抽屉：A 抽屉放着出厂固件，B 抽屉空着。

1. 升级时：新固件只写进 **B 抽屉**（A 抽屉全程不碰，哪怕升级到一半断电，A 抽屉里的老固件都完好无损）；
2. 校验通过后：把"激活指针"指向 B 抽屉（只是改一个标志，不搬数据）；
3. 新固件跑起来连续 3 次都正常：**固化**（确认 B 转正）；
4. 新固件跑不起来：把激活指针拨回 A 抽屉——**回滚**，老固件一直在那儿等着。

**升级包格式**（分区里存的不是裸 bin，是带头部的镜像）：

```
┌─────────────┬───────────────────────────┐
│ 64B 镜像头部 │  固件（app 链接基址 = 分区+0x40）│
└─────────────┴───────────────────────────┘
头部内容：magic("IAP1") + 版本号 + 平台ID("GDF4") + 总长 + CRC32 + 目标槽位
```

> 为什么固件链接基址要 +0x40（64 字节）？因为头部占掉了分区开头，固件的向量表得往后挪 64 字节。这决定了 app 的链接脚本 `ORIGIN = 分区基址 + 0x40`，以及跳转/中断向量表（VTOR）都要 +0x40。

---

## 4. 升级全流程（重点）

### 4.1 一图流

```
上位机串口发字符串命令:  iap_upgrade
        │
        ▼
app 的字符串协议解析到命令
        │  Iap_RequestUpgrade()
        ├── 在参数区写一行 "我要升级"（REQ_UPGRADE）
        └── 软复位（NVIC_SystemReset）
        │
        ▼
boot 启动（每次开机都先跑 boot）
        │  读参数区 → 看到"升级请求"
        ├── 进入升级模式，等待上位机发升级包
        │
        ▼
上位机与 boot 对话（二进制帧协议，不再是字符串）
  QUERY    → 问状态（当前激活哪个槽、版本号）
  START    → 报"我要传：长度/CRC32/目标槽"，boot 擦除目标分区
  DATA×N   → 分包发送（每包 256B），boot 逐包写入 flash 并 ACK
  END      → boot 把收到的整个固件重新算一遍 CRC32，全部对上才通过
  ACTIVATE → boot 把激活指针拨到新槽，复位
        │
        ▼
boot 重新启动 → 校验新固件完整 → 跳进新固件
        │
        ▼
新 app 运行，串口打印：  IAP: NEW firmware on slot B v1.0.0, confirming (1/3)
        │  之后每次正常开机都确认一次
        ▼
连续 3 次成功 → 固化 → 打印:  IAP: active slot B v1.0.0 (confirmed)
```

### 4.2 逐段解释（用比喻）

| 阶段 | 发生了什么 | 比喻 |
|---|---|---|
| 触发 | 字符串命令 `iap_upgrade`（或 Modbus 寄存器 0x000A 写 0x5AA5） | 病人按下"我要做手术"按钮 |
| 移交 | app 写标志后复位，boot 接管 | 病人躺上手术台，主刀医生进场 |
| 接收 | boot 用二进制帧协议收包、每包 ACK | 护士一块一块递器械，递一块清点一块 |
| 校验 | END 时全量 CRC32 复查 | 手术结束前把所有器械再数一遍 |
| 激活 | 改激活指针 + 复位 | 宣布"换脑成功"，叫醒病人 |
| 确认窗口 | 新固件每次开机上报"我活着"，连续 3 次 | 术后观察期，3 天没事才出院 |
| 固化 | 3 次确认后状态清零 | 正式出院，档案归档 |
| 回滚 | 2 次启动失败自动切回旧槽 | 排异反应，立刻换回原装脑子 |

### 4.3 升级模式超时

- boot 进入升级模式后 **60 秒**内没有收到有效命令 → 自动放弃（丢弃半包）→ 跳回当前激活的 app 正常运行。
- 也就是说：误触发了升级请求也不用怕，等一分钟它自己就回去了。

---

## 5. 回滚机制（为什么"严格可回滚"）

这个工程回滚的底气来自一条铁律：

> **升级全程，活动分区（当前在用的固件）一个字节都不会被擦写。**

回滚不是"恢复备份"，而是**把激活指针拨回去**——旧固件从头到尾都好好躺在抽屉里。

| 场景 | 发生了什么 | 结果 |
|---|---|---|
| 传输中断电 | 目标分区写了一半，参数区记着"传输中"；下次 START 直接重来 | 老固件照常跑 |
| END 校验不过 | 拒绝激活，清状态 | 老固件照常跑 |
| 新固件跳转即死 | boot 使能的看门狗（IWDG）6.5 秒后复位 → 再启动发现没上报"我活着" → 累计 2 次失败 → 回滚 | 自动切回老固件 |
| 新固件运行中崩溃 | 确认窗口内（3 次以内）失败 → 回滚；已固化则视为 app 自身 bug | 窗口内可回滚 |
| 两个分区都坏了 | boot 校验都失败 → 进入升级模式死等上位机（防变砖） | 刷入新固件自救 |
| 参数区损坏 | 双扇区日志保证至少一份有效；全坏则重建（选有效的分区跑） | 不会变砖 |

---

## 6. 升级协议（boot 与上位机的对话语言）

**帧格式**（所有命令统一）：

```
┌──────┬──────┬──────┬─────────┬──────────────┬──────────┐
│ 0xA5 │ 0x5A │ CMD  │ LEN(2B) │ PAYLOAD[LEN] │ CRC16(2B)│
└──────┴──────┴──────┴─────────┴──────────────┴──────────┘
  帧头    帧头   命令   负载长度    负载数据      整帧校验
```

**命令表**：

| 命令 | 码 | 负载 | 干什么 |
|---|---|---|---|
| QUERY | 0x01 | 无 | 问状态：激活槽/版本/升级状态 |
| START | 0x02 | 版本+总长+CRC32+目标槽 | 报参数，boot 擦除目标分区 |
| DATA | 0x03 | 包序号(2B)+数据(≤256B) | 传一包数据，boot 写 flash 后 ACK |
| END | 0x04 | 无 | 触发全量 CRC32 校验 |
| ACTIVATE | 0x05 | 无 | 激活新槽并复位 |
| ABORT | 0x06 | 无 | 放弃升级，跳回 app |
| REBOOT | 0x07 | 无 | 复位 |

**应答格式**：`[状态码][下一包序号(2B)]`——状态码 0=OK，其余为各类错误（见 `iap_conf.hpp` IAP_ACK_*）。

**传输规矩**（停等协议）：
- 上位机发一包 → 等 boot 应答 → 再发下一包；
- 重复包（序号小于期望）boot 会直接 ACK 当前进度（去重）；
- 乱序包（序号大于期望）会被拒；
- 坏帧（CRC 错）静默丢弃，上位机超时重发。

**为什么要"逐包停等"**：boot 写 flash 时要关中断几毫秒，如果上位机狂发，这期间到达的串口字节会丢。一包一等就不会丢。

---

## 7. 参数区（手术记录本）

参数区是"升级状态账本"，记录：当前激活槽、升级状态、尝试/成功次数、镜像信息等。**每次状态变化就追加写一条记录**。

**双扇区日志（Param0 + Param1）**：

- 每个扇区 128KB 可以顺序写 256 条记录（每条 512 字节）；
- 当前扇区写满了，就擦另一个扇区接着写；
- 读取时扫描两个扇区，取"序号最大且校验通过"的记录。

比喻：两本轮流用的手术记录本——A 本写满了就擦 B 本接着写，任何时候至少有一本完整可读，断电也不怕（擦 A 本时 B 本还完好）。

**状态机四个状态**：

```
IDLE ──(app 请求)──> REQ_UPGRADE ──(START)──> TRANSFERRING
                                                  │
                                     (END通过+ACTIVATE)
                                                  ▼
IDLE <──(固化/回滚/放弃)── PENDING_ACTIVATE
```

| 状态 | 含义 |
|---|---|
| IDLE | 正常双分区运行 |
| REQ_UPGRADE | app 请求升级（等 boot 处理） |
| TRANSFERRING | 正在接收新固件 |
| PENDING_ACTIVATE | 已切到新槽，等连续 3 次成功确认 |

---

## 8. 移植指南（换板子要改什么）

### 8.1 换芯片（不同 MCU）

| 要改的地方 | 文件 | 说明 |
|---|---|---|
| 分区地址/大小 | `common/inc/iap_conf.hpp` | 按新芯片 flash 容量重排分区，保证扇区边界对齐 |
| 平台 ID | `common/inc/iap_conf.hpp` | `IAP_PLATFORM_ID`，随便换一个自己的魔数 |
| 链接脚本 | `ld/boot.ld`、`app_a.ld`、`app_b.ld` | ORIGIN/LENGTH 与分区表一一对应 |
| flash 容量宏 | `interface/inc/inter_flash.hpp` | `FLASH_CAPACITY_KB` |
| flash 扇区表 | `interface/src/inter_flash.cpp` | 换成新芯片的扇区布局；**擦除函数和 SN 编码务必核对**（见第 9、10 章） |
| 启动文件/系统时钟 | `protocol/iap/CMakeLists.txt` 里的 startup/system 路径 | 换芯片系列要换启动文件和时钟初始化 |
| VTOR 支持 | `application/src/iap_app.cpp` | Cortex-M3/M4 直接 `SCB->VTOR`；**M0/M0+ 没有 VTOR**，需要额外处理（见第 9 章） |

### 8.2 换升级串口

| 要改的地方 | 文件 | 说明 |
|---|---|---|
| boot 串口实例 | `boot/src/iap_boot.cpp` 顶部 `s_boot_uart` | 换外设号/引脚/AF/波特率 |
| 波特率常量 | `common/inc/iap_conf.hpp` | `IAP_BAUDRATE` |
| app 侧协议端口 | `protocol/string/src/string_pro.cpp`（或 modbus 端口表） | app 阶段的命令通道（当前是软串口 suart） |

### 8.3 换按键 / 去掉按键

| 要改的地方 | 文件 | 说明 |
|---|---|---|
| 按键引脚 | `boot/src/iap_boot.cpp` 顶部 `s_key` | 换端口/引脚；低电平有效 |
| 去掉按键 | `boot/src/iap_boot.cpp` `Iap_Boot_Run()` | 删掉 `Key_Pressed()` 判断即可，命令触发不受影响 |

### 8.4 改分区大小 / 换芯片容量

1. `iap_conf.hpp` 改分区宏（**必须扇区对齐**，见 8.1）；
2. 三个 `.ld` 改 ORIGIN/LENGTH；
3. `iap_proto.cpp` 的 `Erase_Partition()` 里擦除扇区清单（AppB 是 3×128K+4×16K+64K 混合布局，换布局要同步改，那里有 `static_assert` 兜底）；
4. `inter_flash.hpp` 的容量宏。

### 8.5 换业务协议（Modbus ↔ 字符串）

| 要改的地方 | 文件 | 说明 |
|---|---|---|
| 激活协议 | `app/inc/protocol_conf.hpp` | `ACTIVE_PROTOCOL` 宏 |
| Modbus 触发 | `protocol/modbus/src/modbus_map.cpp` | 触发寄存器 0x000A / 0x5AA5 |
| 字符串触发 | `protocol/string/src/str_cmd.cpp` | `iap_upgrade` 命令 |

### 8.6 版本号

| 要改的地方 | 文件 | 说明 |
|---|---|---|
| 固件版本 | `common/inc/iap_conf.hpp` | `IAP_VERSION_MAJOR/MINOR/PATCH` |
| 打包版本 | `protocol/iap/CMakeLists.txt` | `IAP_VERSION_STRING`（必须两处一致） |

改完重新编译，`POST_BUILD` 会自动生成带新版本号的升级包 `build/iap/iap_gd32f470_app_*.bin`。

### 8.7 常用构建/烧录命令

```bash
cmake --preset iap                 # 配置 IAP 工程（build/iap）
cmake --build build/iap --target gd32f470_boot.elf gd32f470_app_a.elf gd32f470_app_b.elf
cmake --build build/iap --target flash-iap        # 整片烧录 boot + app_a（出厂）
cmake --build build/iap --target flash-iap-boot   # 只烧 boot
cmake --build build/iap --target flash-iap-app-b  # 只烧 app_b
python3 tests/iap_test.py /dev/ttyACM0 query      # 查询升级状态
python3 tests/iap_test.py /dev/ttyACM0 upgrade --file build/iap/iap_gd32f470_app_b.bin
```

---

## 9. 不同单片机的闪存原理差异（重要）

IAP 的核心操作就是"擦 flash、写 flash、改向量表"，而**不同芯片这三件事的做法完全不同**。移植时最容易在这里翻车。

### 9.1 擦除粒度

| 芯片 | 最小擦除单位 | 说明 |
|---|---|---|
| GD32F4xx | **4KB 页** 和 **扇区**（16K/64K/128K，兼容 STM32F4 布局） | 两种擦除都提供！`fmc_page_erase` 只擦 4KB 页，`fmc_sector_erase` 擦整个扇区 |
| STM32F4 | 扇区（4×16K + 64K + 7×128K 每 bank） | 只有扇区擦除 |
| STM32F1 | 页（1KB 或 2KB） | 小页擦除 |
| STM32F0/G0 | 页（1KB/2KB 等） | 小页擦除 |

**教训**：擦除粒度必须和"你心里的扇区表"一致。本工程曾误用页擦除（4KB）去擦"以为的 128K 扇区"，结果同一扇区残留旧数据，写入校验时爆炸（详见第 10 章坑 1）。

### 9.2 扇区编号编码（GD32 独有的大坑）

- GD32F4xx 的扇区擦除，FMC 控制寄存器里的 **SN 位域编码不是连续的**：bank0 = 0~11，bank1 = **16~27**（12~15 是无效值）；
- STM32F4 是连续的 0~23。

**表现**：把扇区号 12 直接当编码用，硬件会认为"SN 无效"，擦除静默失败（WPERR）；把裸号 OR 进寄存器还会错位，可能擦到别的扇区（本工程曾因此误擦 boot 区，见坑 2/3）。

### 9.3 向量表（VTOR）与中断

| 芯片 | 向量表重定位 | 说明 |
|---|---|---|
| Cortex-M3/M4（GD32F4、STM32F4/F1/F3） | M3/M4 有 `SCB->VTOR`；**F1 特殊**（无 VTOR，需要启动时把向量表从 flash 拷到 RAM 或用 IAP 专用跳转方式） | 本工程直接在 `Iap_AppInit()` 里写 `SCB->VTOR = 分区基址+0x40` |
| Cortex-M0/M0+（STM32F0/G0 等） | **没有 VTOR** | 中断向量表只能在 0x00000000（或 flash 映射区），需要把新固件向量表拷贝到 RAM 并重映射，或 boot 与 app 共用一套向量表（把 app 的 ISR 编进 boot）——这是另一套玩法 |

### 9.4 写保护 / 选项字节

- GD32F4xx 有扇区写保护（选项字节 WP0/WP1，某位写 0 = 保护对应扇区），受保护扇区擦/写会置 WPERR；
- 出厂默认不保护。本工程靠软件安全区（`flash_port::init`）防止误擦，没开硬件保护（开了之后 SWD 更新 boot 要先解保护，流程变繁琐）；
- 量产加固时可以考虑用选项字节把 boot 区硬件保护起来。

### 9.5 时钟 / 波特率

- boot 是独立程序，有自己的时钟初始化（`system_gd32f4xx.c` 的 SystemInit）；
- 串口波特率误差影响升级稳定性，换芯片后要按新时钟树核对波特率。

---

## 10. 实测踩坑记录（全是真金白银换来的）

### 坑 1：页擦除 vs 扇区擦除（升级写一半失败）
- **现象**：目标分区有旧固件时，DATA 写到第 17 包（4KB 边界）报 ERR_BUSY。
- **原因**：`flash_port::erase` 用了 `fmc_page_erase`（只擦 4KB 页），而扇区表按 128K 管理 → 同一"扇区"里只有前 4KB 被擦，其余残留旧数据 → 写入时 flash 无法把 0 变回 1。
- **修复**：改用 `fmc_sector_erase`（真擦整个扇区）。
- **为什么之前没暴露**：出厂烧录后目标分区是全 0xFF 的"新扇区"，不需要擦除也能写，所以首次升级一切正常；第二次升级才炸。

### 坑 2：GD32 SN 编码非连续（bank2 擦除失败）
- **现象**：擦 AppB（跨 bank）时 bank1 的扇区正常，bank2 第一个扇区（0x08100000）擦除失败（WPERR）。
- **原因**：GD32 的 SN 位域编码 bank1 = 扇区号+4（12→16），12~15 是无效值；按 STM32 的连续编号传 12 属于无效操作。
- **修复**：`inter_flash.cpp` 的 `sector_to_sn()`（扇区号 ≥12 时 +4）。

### 坑 3：裸扇区号 OR 进控制寄存器（误擦 boot 区事故）
- **现象**：某次升级后 boot 起不来，`0x08004000` 整片 0xFF。
- **原因**：把裸扇区号 9 直接 `FMC_CTL |= 9`，bit3 恰好落在 SN 位域上，硬件以为是扇区 1 → 把 boot 的扇区 1 擦了。
- **修复**：用 `CTL_SN()` 宏正确编码（`sector<<3 & 0xF8`）+ `sector_to_sn()` 映射；事后加了 `diag_fail_addr/stage` 诊断字段，以后擦除失败能直接读出失败地址和阶段。
- **教训**：写 FMC 控制寄存器前，把编码规则查清楚再动手。

### 坑 4：USB 转串口拆包导致长帧全丢
- **现象**：小命令（QUERY/START）都能通，265 字节的 DATA 帧必挂。
- **原因**：DAP-Link 的虚拟串口走 USB，USB 每 64 字节一个包，包间间隙 ~1ms，远大于串口一个字节的时间（86us @115200）→ 硬件 USART 判定"帧空闲"（IDLE）→ 长帧被拆成好几段"半帧"，每段都校验失败被丢弃。
- **修复**：boot 判帧改为"长度驱动"——先解析帧头里的 LEN，等收满这么多字节再处理，不再依赖 IDLE 中断。

### 坑 5：boot 缺 SysTick/异常处理函数（开机死循环）
- **现象**：烧录后 PC 卡死在 0x08001230（Default_Handler）。
- **原因**：`SysTick_Handler`/`HardFault_Handler` 原来在 app 文件 `gd32f4xx_it.c` 里，boot 没编这个文件 → 向量表指向默认死循环 handler → SysTick 一触发就死。
- **修复**：boot 自己在 `boot_main.cpp` 里实现了这两个 handler。
- **教训**：boot 是独立程序，凡是用到的中断都要自己带全。

### 坑 6：升级包头部与裸 bin 混用
- **现象**：出厂烧裸 bin，boot 校验镜像头失败 → 死等升级。
- **原因**：boot 校验的是"带头部的升级包"（分区开头 64B 是头部），裸 bin 开头是向量表（MSP 值），校验当然不过。
- **修复**：`flash-iap` 烧录目标改为烧 `iap_gd32f470_app_*.bin`（带头部）；app 链接基址 = 分区 + 0x40。

### 坑 7：IWDG 生命周期（复位后看门狗还在跑）
- **现象**（推演后预防）：app 崩溃触发 IWDG 复位后，boot 进升级模式做长耗时操作（擦除 4~12 秒）会被 6.5 秒的看门狗打断。
- **原因**：IWDG 一旦使能，**系统复位不会停止它**。
- **处理**：boot 启动即喂狗、升级模式循环/擦除每扇区后/参数写入前都喂狗；跳转 app 后由 app 接管喂狗。

### 坑 8：ACTIVATE 后确认窗口被误清
- **现象**：升级"成功"了但 PENDING 状态被清成 IDLE，回滚保护失效。
- **原因**：ACTIVATE 写参数时 `attempt_slot=0`，boot 首启判定 `attempt_slot==active` 不成立，误走"参数不一致"分支清掉了确认窗口。
- **修复**：ACTIVATE 时 `attempt_slot` 直接指向新槽。

---

## 11. 调试工具与排查方法

### 11.1 协议验证工具 `tests/iap_test.py`

```bash
python3 tests/iap_test.py /dev/ttyACM0 query               # 查状态（槽/版本/状态机）
python3 tests/iap_test.py /dev/ttyACM0 upgrade --file build/iap/iap_gd32f470_app_b.bin
                                                           # 一键完整升级（自动分包+停等+重试）
```

### 11.2 用 openocd 看现场（板子连 DAP-Link 时）

```bash
# 看 CPU 停在哪（boot 区=还在 boot；0x0802xxxx=app A；0x080Axxxx=app B）
openocd -f openocd/interface-daplink.cfg -f openocd/target-gd32f4xx.cfg \
        -c "init" -c "halt" -c "reg pc" -c "shutdown"

# 看参数区最新记录（magic 应=0x32504149 "IAP2"；seq 越大越新）
#   槽地址 = 0x08120000 + 槽号×512（从 0 开始扫，找 seq 最大的有效记录）
openocd ... -c "mdw 0x08120000 4" ...

# 看分区内容（0x08020000 应是 "IAP1" 头部；0x08020040 是 app 向量表）
openocd ... -c "mdw 0x08020000 4" -c "mdw 0x08020040 4" ...
```

### 11.3 参数记录字段速查

```
偏移  字段          含义
0     magic         0x32504149 ("IAP2")
4     seq           记录序号（越大越新）
8     state         0=IDLE 1=REQ_UPGRADE 2=TRANSFERRING 3=PENDING_ACTIVATE
9     active_slot   'A'/'B'（当前激活槽）
10    pending_slot  升级目标槽
11    boot_ok       1=app 已上报启动成功
12    attempt_slot  上次跳转的槽
16    attempt_count 确认窗口内连续失败次数（≥2 回滚）
18    success_count 确认窗口内连续成功次数（≥3 固化）
20    image_len / 24 image_crc32 / 28 image_version   待激活镜像信息
72    crc16         记录自身校验
```

### 11.4 复位原因速查

- 用 openocd 读 `0x40007000`（RSTSCK），bit27 附近是看门狗复位标志——如果反复复位，先看是不是 IWDG 没喂上。

---

## 12. 安全设计总结（层层防护）

| 层 | 措施 | 防什么 |
|---|---|---|
| 1 | boot 的 flash 安全区排除自身扇区 | 代码 bug 误擦 boot |
| 2 | app 的 flash 安全区仅限参数区 | app bug 误擦固件区 |
| 3 | START 拒绝"升级当前活动分区" | 升级时活动分区被破坏 |
| 4 | 包 0 写入后校验镜像头（magic/平台/槽位/长度/CRC） | 发错槽位/发错包 |
| 5 | END 全量 CRC32 复查 | 传输错误 |
| 6 | 激活前活动分区零写入 + 激活后 3 次确认才固化 | 新固件有问题可随时回滚 |
| 7 | 连续 2 次启动失败自动回滚 | 新固件跳转即死/崩溃 |
| 8 | 双分区都坏 → 死等升级（不跳变砖） | 出厂空片/误擦 |
| 9 | 参数区双扇区日志 | 断电丢状态 |
| 10 | boot 仅 SWD 更新 | 升级链路的最后底线 |

---

## 附：常见问题（FAQ）

**Q：升级到一半断电了怎么办？**
A：老固件完好无损（活动分区零写入）。重新上电 boot 发现"传输中"状态残留，60 秒无命令后自动清掉并跑老固件；想继续升级就重新发 START，从头传。

**Q：怎么手动进升级模式？**
A：两种：① 按住 PC13 按键复位（boot 启动时检测到按键）；② app 运行中发字符串命令 `iap_upgrade`（或 Modbus 写 0x000A=0x5AA5）。命令触发是产品形态，按键是兜底。

**Q：升级包从哪来？**
A：编译 app 时 `POST_BUILD` 自动调用 `tools/iap_pack.py` 生成：`build/iap/iap_gd32f470_app_a.bin` / `..._app_b.bin`。升级目标 = 当前非活动槽，**发对应的包**（active=A 就发 B 包）。

**Q：boot 怎么更新？**
A：只支持 SWD（J-Link/DAP）烧录：`cmake --build build/iap --target flash-iap-boot`。串口不升级 boot，这是回滚的底线。

**Q：裸跑版（无 boot 的开发构建）受影响吗？**
A：完全不受。`BUILD_IAP` 默认 OFF，裸跑版不带任何 IAP 代码（`APP_IAP` 宏未定义，相关代码不编译）。
