// ============================================================
// @platform STM32F1xx（当前平台）
//   硬件 I2C 主机端口，基于 STM32Cube LL 驱动（stm32f1xx_ll_i2c.h）实现。
//   公共接口签名与 GD32 版本一致，内部映射关系：
//     - 外设基址:   I2C0/I2C1/I2C2       → I2C1 / I2C2
//     - 时钟使能:   rcu_periph_clock_enable(RCU_I2Cx)
//                                        → __HAL_RCC_I2Cx_CLK_ENABLE()
//     - 外设初始化:  i2c_clock_config / i2c_mode_addr_config
//                                        → LL_I2C_ConfigSpeed / LL_I2C_SetOwnAddress1
//     - 总线操作:   start_on_bus/stop_on_bus, I2C_DATA
//                                        → LL_I2C_GenerateStart/StopCondition,
//                                          LL_I2C_Transmit/ReceiveData8
//     - 状态标志:   I2C_FLAG_SBSEND/ADDSEND/TBE/RBNE/AERR/I2CBSY
//                                        → LL_I2C_IsActiveFlag_SB/ADDR/TXE/RXNE/AF/BUSY
//   注意：F1 主模式没有 STOPF（仅从模式），且单/双字节接收时序特殊，
//         详见 stop() / i2c_read_reg() 内注释。
// ============================================================

#include "inter_i2c_hw.hpp"

// HAL 宏（__HAL_RCC_I2Cx_CLK_ENABLE / HAL_RCC_GetPCLK1Freq）经 inter_io_ctrl.hpp 引入
#include "stm32f1xx_ll_i2c.h"

#include "delay.h"

// 标志等待的默认超时（循环次数，非绝对时间）
static constexpr uint16_t I2C_TIMEOUT = 0xFFFF;

// ============================================================
//  构造 / 析构
// ============================================================

i2c_hw_port::i2c_hw_port(const I2cHwConfig &cfg)
    : _cfg(cfg)
    , _scl(cfg.scl_port, cfg.scl_pin)
    , _sda(cfg.sda_port, cfg.sda_pin)
    , _initialized(false)
    , _addr_phase(false)
    , _last_addr(false)
    , _addr_pending(false)
    , _periph(nullptr)
    , _busy(0)
{
}

i2c_hw_port::~i2c_hw_port()
{
    deinit();
}

// ============================================================
//  时钟使能 / 关闭
// ============================================================

void i2c_hw_port::_enable_clock()
{
    switch (_cfg.periph) {
        case i2c_hw_id::i2c1: __HAL_RCC_I2C1_CLK_ENABLE(); break;
        case i2c_hw_id::i2c2: __HAL_RCC_I2C2_CLK_ENABLE(); break;
    }
}

void i2c_hw_port::_disable_clock()
{
    switch (_cfg.periph) {
        case i2c_hw_id::i2c1: __HAL_RCC_I2C1_CLK_DISABLE(); break;
        case i2c_hw_id::i2c2: __HAL_RCC_I2C2_CLK_DISABLE(); break;
    }
}

// ============================================================
//  外设配置（init / bus_recovery 共用）
// ============================================================

void i2c_hw_port::_configure_peripheral()
{
    // 配置寄存器必须在 PE=0 时写入
    LL_I2C_Disable(_periph);

    // 时钟：F1 的 I2C 挂在 APB1(PCLK1) 上
    LL_I2C_ConfigSpeed(_periph, HAL_RCC_GetPCLK1Freq(),
                       _cfg.clock_speed, LL_I2C_DUTYCYCLE_2);

    // 自身地址（主机模式不参与寻址，但 OAR1 仍需配置）
    LL_I2C_SetOwnAddress1(_periph, 0, LL_I2C_OWNADDRESS1_7BIT);
    LL_I2C_DisableOwnAddress2(_periph);

    // 使能时钟延展，兼容会拉低 SCL 的从机
    LL_I2C_EnableClockStretching(_periph);

    LL_I2C_Enable(_periph);
    LL_I2C_AcknowledgeNextData(_periph, LL_I2C_ACK);

    _addr_phase   = false;
    _last_addr    = false;
    _addr_pending = false;
}

// ============================================================
//  init
// ============================================================

void i2c_hw_port::init()
{
    if (_initialized) return;

    switch (_cfg.periph) {
        case i2c_hw_id::i2c1: _periph = I2C1; break;
        case i2c_hw_id::i2c2: _periph = I2C2; break;
        default: return;
    }

    _enable_clock();

    // GPIO：I2C 必须复用开漏；上拉用内部弱上拉兜底，建议外部 4.7k
    _scl.init(mode_af_od, pullup, speed_high);
    _scl.set_af(_cfg.af);
    _sda.init(mode_af_od, pullup, speed_high);
    _sda.set_af(_cfg.af);

    _configure_peripheral();

    _initialized = true;
}

// ============================================================
//  deinit
// ============================================================

void i2c_hw_port::deinit()
{
    if (!_initialized) return;

    LL_I2C_AcknowledgeNextData(_periph, LL_I2C_NACK);
    LL_I2C_Disable(_periph);

    _scl.deinit();
    _sda.deinit();

    // 关闭外设时钟，避免干扰后续软 I2C
    _disable_clock();

    _periph = nullptr;
    _initialized = false;
}

// ============================================================
//  互斥
// ============================================================

void i2c_hw_port::lock()
{
    while (_busy);
    _busy = 1;
}

void i2c_hw_port::unlock()
{
    _busy = 0;
}

// ============================================================
//  状态标志等待
// ============================================================

bool i2c_hw_port::_wait_sb(uint16_t timeout)
{
    while (!LL_I2C_IsActiveFlag_SB(_periph)) {
        if (timeout-- == 0U) return false;
    }
    return true;
}

bool i2c_hw_port::_wait_txe(uint16_t timeout)
{
    while (!LL_I2C_IsActiveFlag_TXE(_periph)) {
        if (timeout-- == 0U) return false;
    }
    return true;
}

bool i2c_hw_port::_wait_rxne(uint16_t timeout)
{
    while (!LL_I2C_IsActiveFlag_RXNE(_periph)) {
        if (timeout-- == 0U) return false;
    }
    return true;
}

bool i2c_hw_port::_wait_btf(uint16_t timeout)
{
    while (!LL_I2C_IsActiveFlag_BTF(_periph)) {
        // 数据阶段被从机 NACK：BTF 不会置位，提前中止并清标志
        if (LL_I2C_IsActiveFlag_AF(_periph)) {
            LL_I2C_ClearFlag_AF(_periph);
            return false;
        }
        if (timeout-- == 0U) return false;
    }
    return true;
}

void i2c_hw_port::_wait_bus_idle(uint16_t timeout)
{
    while (LL_I2C_IsActiveFlag_BUSY(_periph)) {
        if (timeout-- == 0U) return;
    }
}

// ============================================================
//  总线控制（STM32 LL 实现）
// ============================================================

void i2c_hw_port::start()
{
    if (!_initialized) return;

    // POS 只用于双字节接收，每次传输前复位
    LL_I2C_DisableBitPOS(_periph);
    _addr_pending = false;

    // 等待总线空闲
    _wait_bus_idle(I2C_TIMEOUT);

    // 生成 START（或重复 START）
    LL_I2C_GenerateStartCondition(_periph);
    if (_wait_sb(I2C_TIMEOUT))
    {
        _addr_phase = true;
        _last_addr = false;
    }
}

void i2c_hw_port::i2c_restart()
{
    if (!_initialized) return;

    _addr_pending = false;

    // 重复 START：总线已被本主机占有，直接发 START
    LL_I2C_GenerateStartCondition(_periph);
    if (_wait_sb(I2C_TIMEOUT))
    {
        _addr_phase = true;
        _last_addr = false;
    }
}

void i2c_hw_port::stop()
{
    if (!_initialized) return;

    // 若地址阶段的 ADDR 尚未清除（例如 ping() 只发地址就收尾），先补清再发 STOP
    if (_addr_pending)
    {
        LL_I2C_ClearFlag_ADDR(_periph);
        _addr_pending = false;
    }

    LL_I2C_GenerateStopCondition(_periph);

    // F1 主模式不产生 STOPF（STOPF 仅从模式），改为等待总线真正释放
    _wait_bus_idle(I2C_TIMEOUT);

    // 清掉传输过程中遗留的 NACK，保证下次传输起点干净
    LL_I2C_ClearFlag_AF(_periph);

    _addr_phase = false;
    _last_addr = false;
}

void i2c_hw_port::write_byte(uint8_t data)
{
    if (!_initialized) return;

    // 地址阶段的 ADDR 在此之前（wait_ack）只置位、不立即清除，
    // 因为 F1 要求接收方向要能「在清 ADDR 之前」配好 ACK/POS；
    // 发送方向则在此处先清 ADDR（对齐 HAL 的 EV8：清 ADDR 后再写 DR）。
    if (_addr_pending)
    {
        LL_I2C_ClearFlag_ADDR(_periph);
        _addr_pending = false;
    }

    // ── STM32F1 关键时序 ──────────────────────────
    // START 之后的第一个字节是器件地址：此时 TxE 尚未置位（需到 ADDR 阶段
    // 结束、EV8 时 TxE 才有效），必须直接写 DR；否则会一直等 TxE 超时，
    // 地址一个字节都发不出去（表现为扫描/探测全部超时失败）。
    // 地址字节之后的寄存器/数据字节才走「等 TxE 再写」。
    if (_addr_phase)
    {
        _addr_phase = false;
        _last_addr = true;
    }
    else
    {
        _last_addr = false;
        if (!_wait_txe(I2C_TIMEOUT))
            return;
    }

    LL_I2C_TransmitData8(_periph, data);
}

uint8_t i2c_hw_port::read_byte()
{
    if (!_initialized) return 0U;

    // 兼容用法：地址阶段后直接 read_byte()。单字节读的 NACK 由调用方
    // 通过 write_ack() 给出；多字节读请改用 read_bytes()。
    if (_addr_pending)
    {
        LL_I2C_ClearFlag_ADDR(_periph);
        _addr_pending = false;
    }

    if (!_wait_rxne(I2C_TIMEOUT)) return 0U;
    return LL_I2C_ReceiveData8(_periph);
}

bool i2c_hw_port::wait_ack(uint16_t timeout)
{
    if (!_initialized) return false;

    // ── 地址阶段：ADDR 置位 / AF 置位（被 NACK） ──────
    if (_last_addr)
    {
        while (!LL_I2C_IsActiveFlag_ADDR(_periph)) {
            if (LL_I2C_IsActiveFlag_AF(_periph)) {
                LL_I2C_ClearFlag_AF(_periph);
                return false;
            }
            if (timeout-- == 0U) return false;
        }

        // ── 此处故意“不清 ADDR”：推迟到知道方向的下一瞬 ──
        // 写方向：write_byte()/stop() 清；
        // 读方向：read_bytes() 需要在清 ADDR 的同一临界区内按 len 配好 POS/ACK。
        // 直接在这里清掉的话，多字节接收就来不及配置，len>=2 会错位。
        _addr_pending = true;
        return true;
    }

    // ── 数据阶段：数据字节不会置 ADDR！ ──────────
    // 应答状态改用 BTF（字节传输完成 = 被 ACK）＋ AF（被 NACK）判定。
    // 这是软/硬 I2C 的语义差异：软 I2C 的 wait_ack 等第 9 个时钟，
    // 对地址与数据字节都适用；而硬件 F1 只有地址阶段才有 ADDR 标志。
    // （_wait_btf 内部已含 AF 检测并会清 AF）
    return _wait_btf(timeout);
}

void i2c_hw_port::write_ack(uint8_t ack)
{
    if (!_initialized) return;

    LL_I2C_AcknowledgeNextData(_periph, ack ? LL_I2C_NACK : LL_I2C_ACK);
}

// ============================================================
//  批量接收（F1 EV7 时序）
//  对照树内 HAL：Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
//                HAL_I2C_Master_Receive() 的 XferSize==1 / ==2 / else
//                以及 while(XferSize>0) 接收循环，逐分支照搬。
// ============================================================

void i2c_hw_port::read_bytes(uint8_t *buf, uint16_t len)
{
    if (!_initialized || buf == nullptr || len == 0U) return;

    uint16_t idx       = 0U;
    uint16_t remaining = len;

    // 接收阶段一律先关 POS，下面 len==2 再单独打开
    LL_I2C_DisableBitPOS(_periph);

    // ── 进入接收（ADDR 仍由 wait_ack 挂起）：按字节数在“清 ADDR 之前”配好 ACK/POS ──
    // 这一段必须与清 ADDR 处在同一临界区：F1 的应答位必须在当前字节结束前生效，
    // 这正是“先 read_byte 再 write_ack”表达不了的地方。
    if (_addr_pending)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        if (remaining == 1U)
        {
            // 单字节：先 NACK → 清 ADDR → 发 STOP → （数据阶段）最后才读 DR
            LL_I2C_AcknowledgeNextData(_periph, LL_I2C_NACK);
            LL_I2C_ClearFlag_ADDR(_periph);
            LL_I2C_GenerateStopCondition(_periph);
        }
        else if (remaining == 2U)
        {
            // 双字节：必须用 POS，否则第二个字节会被第一字节覆盖
            LL_I2C_EnableBitPOS(_periph);
            LL_I2C_ClearFlag_ADDR(_periph);
            LL_I2C_AcknowledgeNextData(_periph, LL_I2C_NACK);
        }
        else
        {
            // 三字节及以上：正常 ACK，靠 BTF 成对搬运 + 末三字节特殊处理
            LL_I2C_AcknowledgeNextData(_periph, LL_I2C_ACK);
            LL_I2C_ClearFlag_ADDR(_periph);
        }

        __set_PRIMASK(primask);
        _addr_pending = false;
    }

    // ── 数据阶段 ──────────────────────────────────────────
    while (remaining > 0U)
    {
        if (remaining > 3U)
        {
            // 还有多于 3 字节：RXNE 驱一个；若 BTF 已置再补一个（防落后于移位寄存器）
            if (!_wait_rxne(I2C_TIMEOUT)) break;
            buf[idx++] = LL_I2C_ReceiveData8(_periph);
            remaining--;

            if (LL_I2C_IsActiveFlag_BTF(_periph))
            {
                // 只剩 3 字节时提前把应答位切成 NACK
                if (remaining == 3U) LL_I2C_AcknowledgeNextData(_periph, LL_I2C_NACK);
                buf[idx++] = LL_I2C_ReceiveData8(_periph);
                remaining--;
            }
        }
        else if (remaining == 3U)
        {
            // 末三字节：BTF → NACK → 搬 1 → BTF → STOP → 搬 2
            if (!_wait_btf(I2C_TIMEOUT)) break;
            LL_I2C_AcknowledgeNextData(_periph, LL_I2C_NACK);

            uint32_t primask = __get_PRIMASK();
            __disable_irq();

            buf[idx++] = LL_I2C_ReceiveData8(_periph);
            remaining--;

            if (!_wait_btf(I2C_TIMEOUT)) { __set_PRIMASK(primask); break; }

            LL_I2C_GenerateStopCondition(_periph);
            buf[idx++] = LL_I2C_ReceiveData8(_periph);
            remaining--;
            buf[idx++] = LL_I2C_ReceiveData8(_periph);
            remaining--;

            __set_PRIMASK(primask);
        }
        else if (remaining == 2U)
        {
            // POS 已置：等 BTF 后先 STOP 再把两个字节搬出
            if (!_wait_btf(I2C_TIMEOUT)) break;

            uint32_t primask = __get_PRIMASK();
            __disable_irq();

            LL_I2C_GenerateStopCondition(_periph);
            buf[idx++] = LL_I2C_ReceiveData8(_periph);
            remaining--;
            buf[idx++] = LL_I2C_ReceiveData8(_periph);
            remaining--;

            __set_PRIMASK(primask);
        }
        else // remaining == 1
        {
            // 单字节：STOP 已在前面发出，这里等 RXNE 后读 DR
            if (!_wait_rxne(I2C_TIMEOUT)) break;
            buf[idx++] = LL_I2C_ReceiveData8(_periph);
            remaining--;
        }
    }

    // 收尾：恢复默认 ACK/POS，等总线释放，清掉可能的 NACK
    LL_I2C_DisableBitPOS(_periph);
    LL_I2C_AcknowledgeNextData(_periph, LL_I2C_ACK);
    _wait_bus_idle(I2C_TIMEOUT);
    LL_I2C_ClearFlag_AF(_periph);

    _addr_phase = false;
    _last_addr  = false;
}

// ============================================================
//  总线恢复
// ============================================================

void i2c_hw_port::bus_recovery()
{
    if (!_initialized) return;

    // 1) 关外设：把 SCL/SDA 从复用功能切回普通开漏 GPIO，手动发时钟
    LL_I2C_Disable(_periph);
    _scl.reinit(mode_out_od, pullup, speed_high);
    _sda.reinit(mode_out_od, pullup, speed_high);

    // 2) 释放两条线
    _sda.high();
    _scl.high();
    delay_us(5);

    // 3) 9 个时钟脉冲：让卡在数据字节中间的从机把剩余位移出
    for (uint8_t i = 0; i < 9U; i++) {
        _scl.low();
        delay_us(5);
        _scl.high();
        delay_us(5);
    }

    // 4) 手动产生 STOP：SDA 低 → SCL 高 → SDA 高
    _sda.low();
    delay_us(5);
    _scl.high();
    delay_us(5);
    _sda.high();
    delay_us(5);

    // 5) 恢复复用开漏并重新配置外设（PE 被清零，配置需重写）
    _scl.reinit(mode_af_od, pullup, speed_high);
    _scl.set_af(_cfg.af);
    _sda.reinit(mode_af_od, pullup, speed_high);
    _sda.set_af(_cfg.af);

    _configure_peripheral();
}

// ============================================================
//  设备级操作
// ============================================================

void i2c_hw_port::i2c_write_reg(uint8_t dev_addr_7bit, uint8_t reg,
                                const uint8_t *data, uint16_t len)
{
    if (!_initialized) return;
    if (data == nullptr && len != 0U) return;

    lock();

    // ── START → ADDR(W) ──────────────────────────────────────
    start();
    write_byte((uint8_t)(dev_addr_7bit << 1));   // 7-bit → 8-bit 写地址
    if (!wait_ack()) { stop(); unlock(); return; }

    // ── 寄存器地址（每字节等 BTF 确认已真正发出） ────────────
    write_byte(reg);
    if (!_wait_btf(I2C_TIMEOUT)) { stop(); unlock(); return; }

    // ── 数据字节 ────────────────────────────────────────────
    for (uint16_t i = 0; i < len; i++) {
        write_byte(data[i]);
        if (!_wait_btf(I2C_TIMEOUT)) break;   // 从机 NACK 或超时
    }

    stop();
    unlock();
}

void i2c_hw_port::i2c_read_reg(uint8_t dev_addr_7bit, uint8_t reg,
                               uint8_t *data, uint16_t len)
{
    if (!_initialized || data == nullptr || len == 0U) return;

    const uint8_t addr_w = (uint8_t)(dev_addr_7bit << 1);
    const uint8_t addr_r = (uint8_t)(addr_w | 0x01U);

    lock();

    // ── 阶段 1：START → ADDR(W) → REG ───────────────────────
    start();
    write_byte(addr_w);
    if (!wait_ack()) { stop(); unlock(); return; }

    write_byte(reg);
    if (!_wait_btf(I2C_TIMEOUT)) { stop(); unlock(); return; }

    // ── 阶段 2：RESTART → ADDR(R) ───────────────────────────
    i2c_restart();
    write_byte(addr_r);

    // wait_ack() 只等 ADDR 置位（不清），随后由 read_bytes() 按 len
    // 在“清 ADDR 之前”配好 POS/ACK 并完成接收与 STOP。
    if (!wait_ack()) { stop(); unlock(); return; }

    // ── 阶段 3：接收（含末字节 NACK + STOP，见 read_bytes） ──
    read_bytes(data, len);

    unlock();
}
