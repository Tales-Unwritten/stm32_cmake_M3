#include "modbus_map.hpp"

#ifdef APP_IAP
#include "iap_app.hpp"
/* IAP 升级触发：保持寄存器 0x000A（ControlBlock[0]）写 0x5AA5 触发进入升级模式 */
#define MODBUS_IAP_TRIGGER_ADDR    0x000A
#define MODBUS_IAP_TRIGGER_VALUE   0x5AA5
#endif

/** 设备信息区： */
uint16_t DeviceBlock[10] =
{
        1,
        100,
};
/** 控制参数 */
uint16_t ControlBlock[50];

/** 实时数据 */
uint16_t StatusBlock[50];

/**
 * 线圈存储区（bit-packed，1 字节 = 8 个线圈）
 * 总容量: 32 字节 = 256 个线圈
 * 用途: 继电器、LED、电机等开关量输出
 */
static uint8_t CoilBlock[32] = {0};

/**
 * 离散输入存储区（bit-packed，1 字节 = 8 个离散输入）
 * 总容量: 32 字节 = 256 个离散输入
 * 用途: 按钮、限位开关、传感器等开关量输入
 */
static uint8_t DiscreteInputBlock[32] = {0};

/** 线圈/离散输入最大地址（用于安全校验） */
#define COIL_MAX_ADDR            (sizeof(CoilBlock) * 8)            /* 256 */
#define DISCRETE_INPUT_MAX_ADDR  (sizeof(DiscreteInputBlock) * 8)  /* 256 */

/** 区块表 */
typedef struct
{
    uint16_t            start_addr;
    uint16_t            length;
    modbus_block_type_t type;
    uint16_t            *data;

} modbus_block_t;

modbus_block_t ModbusBlockTable[] =
    {

        {0x0000, 10, MODBUS_BLOCK_HOLDING,  DeviceBlock},

        {0x000A, 50, MODBUS_BLOCK_HOLDING,  ControlBlock},

        {0x0040, 50, MODBUS_BLOCK_INPUT,    StatusBlock},

        /* 注意: 线圈和离散输入使用独立的位操作函数（Modbus_ReadCoilBit 等），
         * 不通过通用区块表访问，因此不在此表中注册。
         * 枚举值 MODBUS_BLOCK_COIL / MODBUS_BLOCK_DISCRETE_INPUT 保留供未来扩展。 */

};

/** 区块数量 */
const uint16_t BlockCount = sizeof(ModbusBlockTable) / sizeof(modbus_block_t);

uint16_t Modbus_ReadRegister(uint16_t addr,modbus_block_type_t type)
{

    for (uint16_t i = 0; i < BlockCount; i++)
    {

        modbus_block_t *block = &ModbusBlockTable[i];

        if (block->type != type)
        {
            continue;
        }
        if (addr >= block->start_addr && addr < block->start_addr + block->length)
        {
            uint16_t index = addr - block->start_addr;
            return block->data[index];
        }
    }
    return 0;
}


void Modbus_WriteRegister(uint16_t addr, uint16_t value)
{
    for (uint16_t i = 0; i < BlockCount; i++)
    {
        modbus_block_t *block = &ModbusBlockTable[i];
        if (block->type != MODBUS_BLOCK_HOLDING)
        {
            continue;
        }
        if (addr >= block->start_addr && addr < block->start_addr + block->length)
        {
            uint16_t index = addr - block->start_addr;
            block->data[index] = value;
            break;
        }
    }

#ifdef APP_IAP
    /* IAP 触发钩子：值已写入，主机可读回验证；触发后软复位进 boot 升级模式 */
    if (addr == MODBUS_IAP_TRIGGER_ADDR && value == MODBUS_IAP_TRIGGER_VALUE)
    {
        Iap_RequestUpgrade();
    }
#endif
}

/* ============================================================
 * 线圈 / 离散输入 位操作函数
 *
 * 存储布局：bit-packed，小端位序
 *   byte[0] 的 bit0 = 地址 0
 *   byte[0] 的 bit1 = 地址 1
 *   ...
 *   byte[0] 的 bit7 = 地址 7
 *   byte[1] 的 bit0 = 地址 8
 *   依此类推
 *
 * 安全策略：
 *   - 所有函数均对地址做边界检查
 *   - 越界读返回 0，越界写静默忽略
 * ============================================================ */

uint8_t Modbus_ReadCoilBit(uint16_t addr)
{
    /* 【安全】地址越界检查 */
    if (addr >= COIL_MAX_ADDR)
    {
        return 0; /* 越界返回 0（安全降级） */
    }

    uint16_t byte_index = addr >> 3;   /* addr / 8 */
    uint8_t  bit_offset = addr & 0x07; /* addr % 8 */

    /* 【安全】二次校验：防止编译器优化后越界 */
    if (byte_index >= sizeof(CoilBlock))
    {
        return 0;
    }

    return (CoilBlock[byte_index] >> bit_offset) & 0x01;
}

void Modbus_WriteCoilBit(uint16_t addr, uint8_t value)
{
    /* 【安全】地址越界检查 */
    if (addr >= COIL_MAX_ADDR)
    {
        return; /* 越界静默忽略 */
    }

    uint16_t byte_index = addr >> 3;
    uint8_t  bit_offset = addr & 0x07;

    /* 【安全】二次校验 */
    if (byte_index >= sizeof(CoilBlock))
    {
        return;
    }

    if (value)
    {
        CoilBlock[byte_index] |= (1 << bit_offset);  /* 置 1 */
    }
    else
    {
        CoilBlock[byte_index] &= ~(1 << bit_offset); /* 清 0 */
    }
}

uint8_t Modbus_ReadDiscreteInputBit(uint16_t addr)
{
    /* 【安全】地址越界检查 */
    if (addr >= DISCRETE_INPUT_MAX_ADDR)
    {
        return 0;
    }

    uint16_t byte_index = addr >> 3;
    uint8_t  bit_offset = addr & 0x07;

    /* 【安全】二次校验 */
    if (byte_index >= sizeof(DiscreteInputBlock))
    {
        return 0;
    }

    return (DiscreteInputBlock[byte_index] >> bit_offset) & 0x01;
}

void Modbus_WriteDiscreteInputBit(uint16_t addr, uint8_t value)
{
    /* 【安全】地址越界检查 */
    if (addr >= DISCRETE_INPUT_MAX_ADDR)
    {
        return; /* 越界静默忽略 */
    }

    uint16_t byte_index = addr >> 3;
    uint8_t  bit_offset = addr & 0x07;

    /* 【安全】二次校验 */
    if (byte_index >= sizeof(DiscreteInputBlock))
    {
        return;
    }

    if (value)
    {
        DiscreteInputBlock[byte_index] |= (1 << bit_offset);
    }
    else
    {
        DiscreteInputBlock[byte_index] &= ~(1 << bit_offset);
    }
}

uint16_t Modbus_GetCoilMaxAddr(void)
{
    return COIL_MAX_ADDR;
}

uint16_t Modbus_GetDiscreteMaxAddr(void)
{
    return DISCRETE_INPUT_MAX_ADDR;
}

uint8_t Modbus_IsHoldingAddr(uint16_t addr)
{
    /* 遍历区块表，检查地址是否属于任何 HOLDING 块 */
    for (uint16_t i = 0; i < BlockCount; i++)
    {
        if (ModbusBlockTable[i].type == MODBUS_BLOCK_HOLDING &&
            addr >= ModbusBlockTable[i].start_addr &&
            addr <  ModbusBlockTable[i].start_addr + ModbusBlockTable[i].length)
        {
            return 1;
        }
    }
    return 0;
}

uint8_t Modbus_IsInputAddr(uint16_t addr)
{
    /* 遍历区块表，检查地址是否属于任何 INPUT 块 */
    for (uint16_t i = 0; i < BlockCount; i++)
    {
        if (ModbusBlockTable[i].type == MODBUS_BLOCK_INPUT &&
            addr >= ModbusBlockTable[i].start_addr &&
            addr <  ModbusBlockTable[i].start_addr + ModbusBlockTable[i].length)
        {
            return 1;
        }
    }
    return 0;
}
