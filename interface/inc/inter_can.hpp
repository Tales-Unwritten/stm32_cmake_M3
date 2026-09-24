#pragma once
// ============================================================
// @platform STM32F1xx（当前平台）
//   从 GD32F4xx 版移植：公共 API（can_port 方法签名、CanPortConfig 字段语义）
//   保持一致，仅将平台相关项替换为 F1 映射：
//     - 外设:      can_id::can0/can1                → 仅 can_id::can1（F103 高密度器件单 CAN）
//     - 时钟:      rcu_periph_clock_enable(RCU_CANx) → __HAL_RCC_CAN1_CLK_ENABLE()
//     - 初始化:    can_parameter_struct / can_init   → CAN_HandleTypeDef + HAL_CAN_Init/Start
//     - 过滤器:    can_filter_parameter_struct       → CAN_FilterTypeDef + HAL_CAN_ConfigFilter
//     - 发送:      can_message_transmit + 轮询 TSTAT → HAL_CAN_AddTxMessage + IsTxMessagePending
//     - 接收:      can_message_receive(CAN_FIFO0)    → HAL_CAN_GetRxMessage(CAN_RX_FIFO0)
//     - 工作模式:  CAN_NORMAL_MODE 等宏              → can_mode 枚举（映射 HAL CAN_MODE_*）
//     - 波特率:    公式不变，PCLK1 由 HAL_RCC_GetPCLK1Freq() 提供（本例 72MHz 主频 → 36MHz）
//   依赖：HAL CAN 驱动（需在 stm32f1xx_hal_conf.h 打开 HAL_CAN_MODULE_ENABLED，
//         并把 Drivers/.../stm32f1xx_hal_can.c 加入构建，见 CMakeLists.txt）
// ============================================================

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
// #include "stm32f1xx_hal.h" // IWYU pragma: keep —— HAL 基础 +（经 hal_conf）CAN_HandleTypeDef / HAL_CAN_* / CAN_MODE_*
#include <cstdint>

// ============================================================
//  平台中性类型
// ============================================================

/** @brief CAN 外设 ID（STM32F103 高密度器件仅 1 个 CAN，即 CAN1） */
enum class can_id : uint8_t
{
    can1
};

/** @brief 工作模式（语义与原 GD32 版 CAN_NORMAL_MODE 等一致） */
enum class can_mode : uint8_t
{
    normal = 0,      ///< 正常收发（HAL: CAN_MODE_NORMAL）
    loopback,        ///< 自测环回（HAL: CAN_MODE_LOOPBACK）
    silent,          ///< 只听模式（HAL: CAN_MODE_SILENT）
    silent_loopback, ///< 静默环回（HAL: CAN_MODE_SILENT_LOOPBACK）
};

// ============================================================
//  配置结构体（POD）
// ============================================================

struct CanPortConfig
{
    can_id periph = can_id::can1;       ///< 外设 ID（F1 仅 can1）
    GPIO_TypeDef *tx_port = nullptr;    ///< CAN TX GPIO 端口
    pin_enum_t tx_pin = pin_none;       ///< CAN TX 引脚掩码
    GPIO_TypeDef *rx_port = nullptr;    ///< CAN RX GPIO 端口
    pin_enum_t rx_pin = pin_none;       ///< CAN RX 引脚掩码
    afio_enum_t af = afio_enum_t::NONE; ///< CAN1 AFIO 重映射：RM_CAN1_1/2/3
    uint32_t baudrate = 500000;         ///< 波特率: 125000, 250000, 500000, 1000000
    can_mode mode = can_mode::normal;   ///< 工作模式
    uint8_t filter_bank = 0;            ///< 过滤器 bank（F1 单 CAN：0~13）

    // ── 时序参数（对齐 GD32 版：外部指定保证整除） ──
    // 波特率 = PCLK1 / prescaler / (1 + bs1_tq + bs2_tq)
    // 默认 12TQ（采样点 75%）：PCLK1=36MHz 时 125k→24, 250k→12, 500k→6, 1M→3，全部整除
    uint8_t bs1_tq = 8;    ///< 时间段1 TQ 数（0 = 用默认 8）
    uint8_t bs2_tq = 3;    ///< 时间段2 TQ 数（0 = 用默认 3）
    uint8_t prescaler = 0; ///< 0 = 自动计算（不整除时 init 返回 false）；非 0 = 手动指定
};

// ============================================================
//  CAN 端口
// ============================================================

/**
 * @brief CAN 总线端口（阻塞发送 + 查询 FIFO0 接收）
 *
 * 使用示例（CAN1, PB8/PB9 重映射, 500kbps，标准帧）：
 *   static can_port can({
 *       can_id::can1, GPIOB, pin9, GPIOB, pin8,
 *       afio_enum_t::RM_CAN1_2, 500000
 *   });
 *   can.init();
 *   can.send(0x123, false, data, 8);
 *
 * 使用示例（环回自测，无需外部收发器/总线）：
 *   can.send(0x7FF, false, "HELLO123", 8);
 *   uint32_t id; bool ext; uint8_t rx[8], len;
 *   if (can.recv(id, ext, rx, len)) { ... }
 *
 * @note F1 的 TX 引脚须配为复用推挽、RX 引脚须配为输入（本类已按 CubeMX 的
 *       CAN 引脚配置处理，无需调用方干预）。
 */
class can_port
{
  public:
    explicit can_port(const CanPortConfig &cfg);
    ~can_port();

    can_port(const can_port &) = delete;
    can_port &operator=(const can_port &) = delete;

    // ── 生命周期 ──────────────────────────────────────────

    /** @brief 初始化 CAN（幂等）
     *  @return true = 成功；false = 波特率无法整除（prescaler 非整数）或参数非法
     *  @note  波特率公式：PCLK1 / prescaler / (1+bs1+bs2)，
     *         自动计算时要求整除，否则返回 false（可手动指定 prescaler）
     */
    bool init();
    void deinit();
    [[nodiscard]] bool is_initialized() const noexcept
    {
        return _initialized;
    }

    // ── 发送（阻塞） ──────────────────────────────────────

    /** @brief 发送 CAN 帧
     *  @param id        帧 ID（标准帧 11-bit 或扩展帧 29-bit）
     *  @param is_ext    是否为扩展帧
     *  @param data      数据指针
     *  @param len       数据长度（0~8 字节）
     *  @return          发送成功返回 true
     */
    bool send(uint32_t id, bool is_ext, const uint8_t *data, uint8_t len);

    // ── 接收（非阻塞，查询 FIFO0） ────────────────────────

    /** @brief 查询是否有接收帧就绪 */
    [[nodiscard]] bool rx_ready() const;

    /** @brief 接收 CAN 帧（非阻塞）
     *  @param[out] id       接收到的帧 ID
     *  @param[out] is_ext   是否为扩展帧
     *  @param[out] data     数据缓冲区（至少 8 字节）
     *  @param[out] len      实际数据长度
     *  @return              有帧时返回 true
     */
    bool recv(uint32_t &id, bool &is_ext, uint8_t *data, uint8_t &len);

    // ── 查询 ──────────────────────────────────────────────

    [[nodiscard]] can_id periph() const noexcept
    {
        return _cfg.periph;
    }
    [[nodiscard]] uint32_t hw() const noexcept
    {
        return _periph;
    }

  private:
    void _enable_clock();
    [[nodiscard]] uint32_t _can_clock() const; // CAN 外设时钟频率（Hz）

    CanPortConfig _cfg;
    io_ctrl _tx;
    io_ctrl _rx;
    bool _initialized;
    uint32_t _periph;
    CAN_HandleTypeDef _hcan; ///< HAL CAN 句柄（本类自持，不依赖 CubeMX 生成的 hcan）
};

#endif /* __cplusplus */
