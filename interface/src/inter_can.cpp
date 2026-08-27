// ============================================================
// @platform GD32F4xx（当前平台）
//   参考实现：王海涛 can_bsp.c（时序参数外部指定保证整除）
// ============================================================

#include "inter_can.hpp"

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_can.h"

// ============================================================
//  构造 / 析构
// ============================================================

can_port::can_port(const CanPortConfig &cfg)
    : _cfg(cfg)
    , _tx(cfg.tx_port, cfg.tx_pin)
    , _rx(cfg.rx_port, cfg.rx_pin)
    , _initialized(false)
    , _periph(0)
{
}

can_port::~can_port()
{
    deinit();
}

// ============================================================
//  时钟使能
// ============================================================

void can_port::_enable_clock()
{
    if (_cfg.periph == can_id::can0) {
        rcu_periph_clock_enable(RCU_CAN0);
    } else {
        // ⚠️ CAN1 的过滤器寄存器位于 CAN0 外设地址空间（共享过滤器），
        //    必须同时使能 CAN0 时钟，否则过滤器配置写入无效（FW 恒 0）
        rcu_periph_clock_enable(RCU_CAN0);
        rcu_periph_clock_enable(RCU_CAN1);
    }
}

// ============================================================
//  CAN 外设时钟频率（APB1，GD32F470 @240MHz = 60MHz）
// ============================================================

uint32_t can_port::_can_clock() const
{
    return rcu_clock_freq_get(CK_APB1);
}

// ============================================================
//  init
// ============================================================

bool can_port::init()
{
    if (_initialized) return true;

    // 参数校验
    if (_cfg.tx_port == nullptr || _cfg.tx_pin == pin_none || _cfg.rx_port == nullptr || _cfg.rx_pin == pin_none)
        return false;
    if (_cfg.baudrate == 0)
        return false;

    // 外设基址
    if (_cfg.periph == can_id::can0) {
        _periph = CAN0;
    } else {
        _periph = CAN1;
    }

    _enable_clock();

    // GPIO AF（CAN 引脚标准无上拉）
    _tx.init(mode_af_pp, nopull, speed_high);
    _tx.set_af(_cfg.af);
    _rx.init(mode_af_pp, nopull, speed_high);
    _rx.set_af(_cfg.af);

    // ── 时序参数（对齐参考实现：保证整除，波特率精确） ──
    // 波特率 = PCLK1 / prescaler / (1 + bs1 + bs2)
    // ⚠️ GD32 枚举值 = TQ 数 - 1（CAN_BT_BS1_8TQ == 0x07）
    const uint8_t bs1 = (_cfg.bs1_tq == 0) ? 8 : _cfg.bs1_tq;
    const uint8_t bs2 = (_cfg.bs2_tq == 0) ? 3 : _cfg.bs2_tq;
    if (bs1 < 1 || bs1 > 16 || bs2 < 1 || bs2 > 8)
        return false;
    const uint32_t total_tq = (uint32_t)1 + bs1 + bs2;

    uint32_t prescaler = _cfg.prescaler;
    if (prescaler == 0)
    {
        // 自动计算：要求整除，否则返回 false（避免旧版 468.75k≠500k 的问题）
        const uint64_t div = (uint64_t)_cfg.baudrate * total_tq;
        if (div == 0) return false;
        prescaler = (uint32_t)(_can_clock() / div);
        if ((uint64_t)_can_clock() % div != 0)
            return false;   // 无法整除：请手动指定 bs1/bs2/prescaler
        if (prescaler == 0 || prescaler > 1024)
            return false;
    }

    // CAN 外设初始化
    can_parameter_struct can_cfg;
    can_struct_para_init(CAN_INIT_STRUCT, &can_cfg);
    can_cfg.working_mode       = _cfg.mode;
    can_cfg.resync_jump_width  = CAN_BT_SJW_1TQ;
    can_cfg.time_segment_1     = (uint8_t)(bs1 - 1);   // 寄存器值 = TQ 数 - 1
    can_cfg.time_segment_2     = (uint8_t)(bs2 - 1);   // 寄存器值 = TQ 数 - 1
    can_cfg.time_triggered     = DISABLE;
    can_cfg.auto_bus_off_recovery = ENABLE;
    can_cfg.auto_wake_up       = DISABLE;
    can_cfg.auto_retrans       = ENABLE;      // 对齐参考实现：总线错误自动重传
    can_cfg.rec_fifo_overwrite = DISABLE;
    can_cfg.trans_fifo_order   = DISABLE;
    can_cfg.prescaler          = prescaler;

    // CAN1 必须分配过滤器 bank（必须在 can_init 之前）
    if (_cfg.periph == can_id::can1) {
        can1_filter_start_bank(_cfg.filter_bank);
    }

    can_init(_periph, &can_cfg);

    // 配置过滤器：接收所有帧 → FIFO0（掩码全 0）
    can_filter_parameter_struct filter_cfg;
    can_struct_para_init(CAN_FILTER_STRUCT, &filter_cfg);
    filter_cfg.filter_number      = _cfg.filter_bank;
    filter_cfg.filter_mode        = CAN_FILTERMODE_MASK;
    filter_cfg.filter_bits        = CAN_FILTERBITS_32BIT;
    filter_cfg.filter_list_high   = 0x0000;
    filter_cfg.filter_list_low    = 0x0000;
    filter_cfg.filter_mask_high   = 0x0000;
    filter_cfg.filter_mask_low    = 0x0000;
    filter_cfg.filter_fifo_number = CAN_FIFO0;
    filter_cfg.filter_enable      = ENABLE;
    can_filter_init(&filter_cfg);

    _initialized = true;
    return true;
}

// ============================================================
//  deinit
// ============================================================

void can_port::deinit()
{
    if (!_initialized) return;

    can_deinit(_periph);

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
    if (!_initialized || !data || len > 8) return false;

    can_transmit_message_struct tx_msg;
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx_msg);

    if (is_ext) {
        tx_msg.tx_efid = id;
        tx_msg.tx_ff   = CAN_FF_EXTENDED;
    } else {
        tx_msg.tx_sfid = id;
        tx_msg.tx_ff   = CAN_FF_STANDARD;
    }
    tx_msg.tx_ft = CAN_FT_DATA;
    tx_msg.tx_dlen = len;
    for (uint8_t i = 0; i < len && i < 8; i++) {
        tx_msg.tx_data[i] = data[i];
    }

    // 等待空闲邮箱（超时保护）
    uint32_t timeout = 0xFFFF;
    while (can_message_transmit(_periph, &tx_msg) == CAN_TRANSMIT_NOMAILBOX) {
        if (--timeout == 0) return false;
    }

    // 先清发送完成标志，再等待真正完成（防止残留标志假成功）
    CAN_TSTAT(_periph) = CAN_TSTAT_MTF0 | CAN_TSTAT_MTF1 | CAN_TSTAT_MTF2;
    timeout = 0xFFFF;
    while (!(CAN_TSTAT(_periph) & (CAN_TSTAT_MTF0 | CAN_TSTAT_MTF1 | CAN_TSTAT_MTF2))) {
        if (--timeout == 0) return false;
    }
    return true;
}

// ============================================================
//  接收（查询 FIFO0）
// ============================================================

bool can_port::rx_ready() const
{
    if (!_initialized) return false;
    return (can_receive_message_length_get(_periph, CAN_FIFO0) > 0);
}

bool can_port::recv(uint32_t &id, bool &is_ext, uint8_t *data, uint8_t &len)
{
    if (!_initialized || !data) return false;

    if (can_receive_message_length_get(_periph, CAN_FIFO0) == 0) {
        return false;
    }

    can_receive_message_struct rx_msg;
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx_msg);
    can_message_receive(_periph, CAN_FIFO0, &rx_msg);

    is_ext = (rx_msg.rx_ff == CAN_FF_EXTENDED);
    id     = is_ext ? rx_msg.rx_efid : rx_msg.rx_sfid;
    len    = rx_msg.rx_dlen;
    for (uint8_t i = 0; i < len && i < 8; i++) {
        data[i] = rx_msg.rx_data[i];
    }

    return true;
}
