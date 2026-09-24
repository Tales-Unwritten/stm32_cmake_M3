// ============================================================
// @platform STM32F1xx（当前平台）
//   基于 STM32 HAL CAN 驱动（HAL_CAN_*）实现，公共接口签名与 GD32 版一致。
//   GD32 → F1 映射：
//     can_parameter_struct/can_init        → CAN_HandleTypeDef + HAL_CAN_Init/Start
//     can_filter_parameter_struct          → CAN_FilterTypeDef + HAL_CAN_ConfigFilter
//     can_message_transmit + 轮询 TSTAT    → HAL_CAN_AddTxMessage + IsTxMessagePending
//     can_message_receive(CAN_FIFO0)       → HAL_CAN_GetRxMessage(CAN_RX_FIFO0)
//   注意：
//     - F1 的 CAN_TX 引脚配复用推挽、CAN_RX 引脚配输入（CubeMX 同款）；
//     - CAN 时钟在 APB1(PCLK1)，本工程 72MHz 主频 → 36MHz；
//     - HAL CAN 句柄由本类持有，不占用 CubeMX 会生成的全局 hcan。
// ============================================================

#include "inter_can.hpp"

// 等待空闲邮箱 / 发送完成的循环上限（非绝对时间）
static constexpr uint32_t CAN_TIMEOUT = 0xFFFF;

// F1 单 CAN 的过滤器 bank 上限（14 个：0 ~ 13）
static constexpr uint8_t CAN_FILTER_BANK_MAX = 13;

// 模式映射：中性枚举 → HAL 宏
static uint32_t _to_hal_mode(can_mode m)
{
    switch (m)
    {
    case can_mode::loopback:
        return CAN_MODE_LOOPBACK;
    case can_mode::silent:
        return CAN_MODE_SILENT;
    case can_mode::silent_loopback:
        return CAN_MODE_SILENT_LOOPBACK;
    case can_mode::normal:
    default:
        return CAN_MODE_NORMAL;
    }
}

// ============================================================
//  构造 / 析构
// ============================================================

can_port::can_port(const CanPortConfig &cfg)
    : _cfg(cfg), _tx(cfg.tx_port, cfg.tx_pin), _rx(cfg.rx_port, cfg.rx_pin), _initialized(false), _periph(0),
      _hcan{} // 句柄清零
{
}

can_port::~can_port()
{
    deinit();
}

// ============================================================
//  时钟使能 / 频率
// ============================================================

void can_port::_enable_clock()
{
    // F103 高密度器件只有一个 CAN（CAN1）
    __HAL_RCC_CAN1_CLK_ENABLE();
}

uint32_t can_port::_can_clock() const
{
    // F1 的 CAN 挂在 APB1(PCLK1) 上
    return HAL_RCC_GetPCLK1Freq();
}

// ============================================================
//  init
// ============================================================

bool can_port::init()
{
    if (_initialized)
        return true;

    // 参数校验
    if (_cfg.periph != can_id::can1)
        return false; // F1 仅 CAN1
    if (_cfg.tx_port == nullptr || _cfg.tx_pin == pin_none || _cfg.rx_port == nullptr || _cfg.rx_pin == pin_none)
        return false;
    if (_cfg.baudrate == 0)
        return false;
    if (_cfg.filter_bank > CAN_FILTER_BANK_MAX)
        return false;

    _periph = (uint32_t)CAN1;

    _enable_clock();

    // GPIO：TX = 复用推挽（CAN 引脚标准无上拉）；RX = 输入
    _tx.init(mode_af_pp, nopull, speed_high);
    _tx.set_af(_cfg.af); // CAN1 重映射是全局 AFIO 设置，经 TX 应用一次即可
    _rx.init(mode_input, pullup, speed_high);

    // ── 时序参数（对齐 GD32 版：保证整除，波特率精确） ──
    // 波特率 = PCLK1 / prescaler / (1 + bs1 + bs2)
    const uint8_t bs1 = (_cfg.bs1_tq == 0) ? 8 : _cfg.bs1_tq;
    const uint8_t bs2 = (_cfg.bs2_tq == 0) ? 3 : _cfg.bs2_tq;
    if (bs1 < 1 || bs1 > 16 || bs2 < 1 || bs2 > 8)
        return false;
    const uint32_t total_tq = (uint32_t)1 + bs1 + bs2;

    uint32_t prescaler = _cfg.prescaler;
    if (prescaler == 0)
    {
        // 自动计算：要求整除，否则返回 false（避免 468.75k≠500k 的问题）
        const uint64_t div = (uint64_t)_cfg.baudrate * total_tq;
        if (div == 0)
            return false;
        prescaler = (uint32_t)(_can_clock() / div);
        if ((uint64_t)_can_clock() % div != 0)
            return false; // 无法整除：请手动指定 bs1/bs2/prescaler
        if (prescaler == 0 || prescaler > 1024)
            return false;
    }

    // ── HAL 外设初始化（逐字段对应 CubeMX MX_CAN_Init 生成的 hcan.Init） ──
    _hcan.Instance = CAN1;                                        // 外设实例：F103 高密度器件唯一 CAN
    _hcan.Init.Prescaler = prescaler;                             // 波特率预分频：上面算出的分频系数（HAL 内部 -1 写入 BTR）
    _hcan.Init.Mode = _to_hal_mode(_cfg.mode);                    // 工作模式：normal → CAN_MODE_NORMAL（另有 loopback/silent）
    _hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;                       // 重同步跳转宽度 = 1TQ（与 12TQ 位时间配套，容错够用）
    _hcan.Init.TimeSeg1 = (uint32_t)(bs1 - 1) << CAN_BTR_TS1_Pos; // 时间段1：HAL 期望「TQ 数 - 1」（bs1=8 → 7<<16）
    _hcan.Init.TimeSeg2 = (uint32_t)(bs2 - 1) << CAN_BTR_TS2_Pos; // 时间段2：同上（bs2=3 → 采样点 75%）
    _hcan.Init.TimeTriggeredMode = DISABLE;                       // 时间触发通信模式关（普通 CAN 2.0B）
    _hcan.Init.AutoBusOff = ENABLE;                               // 自动离线恢复 ABOM=1（对齐 GD32 auto_bus_off_recovery=ENABLE）
    _hcan.Init.AutoWakeUp = DISABLE;                              // 自动唤醒关（睡眠唤醒由 HAL_CAN_WakeUp 主动触发）
    _hcan.Init.AutoRetransmission = ENABLE;                       // 发送失败自动重传 NART=0（对齐 GD32 auto_retrans=ENABLE）
    _hcan.Init.ReceiveFifoLocked = DISABLE;                       // FIFO 满时新帧覆盖最旧帧 RFLM=0（不锁定，避免卡 FIFO）
    _hcan.Init.TransmitFifoPriority = DISABLE;                    // 发送优先级按报文 ID，而非邮箱顺序 TXFP=0

    // 初始化外设（含 MspInit；注意 HAL_CAN_Init 后外设仍处于 init 模式，须再调 HAL_CAN_Start）
    if (HAL_CAN_Init(&_hcan) != HAL_OK)
    {
        _hcan.Instance = nullptr;
        return false;
    }

    // ── 接收过滤器：接收所有帧 → FIFO0（掩码全 0 = 全部通过，对齐 GD32 版） ──
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = _cfg.filter_bank;               // 使用的过滤器组编号（F1 单 CAN：0~13）
    filter.FilterMode = CAN_FILTERMODE_IDMASK;          // 掩码模式（另有 CAN_FILTERMODE_IDLIST 列表模式）
    filter.FilterScale = CAN_FILTERSCALE_32BIT;         // 32 位宽：一个组 = 一条过滤规则（16 位可放两条）
    filter.FilterIdHigh = 0x0000;                       // 期望 ID 的高 16 位（含 STID/IDE/RTR/EXID 高位）
    filter.FilterIdLow = 0x0000;                        // 期望 ID 的低 16 位
    filter.FilterMaskIdHigh = 0x0000;                   // 对应掩码高 16 位：全 0 = 高位不参与比较
    filter.FilterMaskIdLow = 0x0000;                    // 对应掩码低 16 位：全 0 = 标准帧/扩展帧全部通过
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;     // 命中帧送入 FIFO0（与 recv() 查询的 FIFO 一致）
    filter.FilterActivation = ENABLE;                   // 使能本过滤器组
    filter.SlaveStartFilterBank = 14;                   // 从 CAN 起始 bank（仅双 CAN 器件有意义，单 CAN 无意义）

    if (HAL_CAN_ConfigFilter(&_hcan, &filter) != HAL_OK)
    {
        (void)HAL_CAN_DeInit(&_hcan);
        _hcan.Instance = nullptr;
        return false;
    }

    // 进入正常收发状态（清 INRQ，State → LISTENING）
    if (HAL_CAN_Start(&_hcan) != HAL_OK)
    {
        (void)HAL_CAN_DeInit(&_hcan);
        _hcan.Instance = nullptr;
        return false;
    }

    _initialized = true;
    return true;
}

// ============================================================
//  deinit
// ============================================================

void can_port::deinit()
{
    if (!_initialized)
        return;

    // 先取消所有待发请求（重要）：HAL_CAN_DeInit 只把 MCR.RESET 置位复位 CAN *核*，
    // 并不清发送邮箱的 TXRQ；残留的"发不出去"的帧（如只听模式/无 ACK 时超时的帧）
    // 会在下次 init 后"迟到发出"，污染 RX FIFO 甚至意外上线。必须显式中止。
    (void)HAL_CAN_AbortTxRequest(&_hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);

    (void)HAL_CAN_Stop(&_hcan);
    (void)HAL_CAN_DeInit(&_hcan); // 内部调用 HAL_CAN_MspDeInit（弱函数，默认空）
    __HAL_RCC_CAN1_CLK_DISABLE();

    _tx.deinit();
    _rx.deinit();

    _periph = 0;
    _initialized = false;
}

// ============================================================
//  发送
// ============================================================

bool can_port::send(uint32_t id, bool is_ext, const uint8_t *data, uint8_t len)
{
    if (!_initialized || !data || len > 8)
        return false;

    CAN_TxHeaderTypeDef tx = {0};
    tx.StdId = is_ext ? 0x000U : (id & 0x7FFU);
    tx.ExtId = is_ext ? (id & 0x1FFFFFFFU) : 0x00000000U;
    tx.IDE = is_ext ? CAN_ID_EXT : CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = len;
    tx.TransmitGlobalTime = DISABLE;

    // 等待空闲邮箱（超时保护）
    uint32_t timeout = CAN_TIMEOUT;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&_hcan) == 0)
    {
        if (--timeout == 0)
            return false;
    }

    uint32_t mailbox = 0;
    if (HAL_CAN_AddTxMessage(&_hcan, &tx, data, &mailbox) != HAL_OK)
        return false;

    // 等本帧真正发完（对齐 GD32 版“等 TSTAT 完成”语义）
    timeout = CAN_TIMEOUT;
    while (HAL_CAN_IsTxMessagePending(&_hcan, mailbox) != 0)
    {
        if (--timeout == 0)
            return false;
    }
    return true;
}

// ============================================================
//  接收（查询 FIFO0）
// ============================================================

bool can_port::rx_ready() const
{
    if (!_initialized)
        return false;
    return HAL_CAN_GetRxFifoFillLevel(&_hcan, CAN_RX_FIFO0) > 0;
}

bool can_port::recv(uint32_t &id, bool &is_ext, uint8_t *data, uint8_t &len)
{
    if (!_initialized || !data)
        return false;

    if (HAL_CAN_GetRxFifoFillLevel(&_hcan, CAN_RX_FIFO0) == 0)
        return false;

    CAN_RxHeaderTypeDef rx = {0};
    if (HAL_CAN_GetRxMessage(&_hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK)
        return false;

    is_ext = (rx.IDE == CAN_ID_EXT);
    id = is_ext ? rx.ExtId : rx.StdId;
    len = (uint8_t)rx.DLC;
    return true;
}
