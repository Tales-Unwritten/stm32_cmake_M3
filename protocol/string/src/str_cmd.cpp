/**
 * @file    str_cmd.c
 * @brief   字符串命令框架 —— 命令表 + 解析引擎 + 用户命令 handler
 *
 * 新增命令步骤：
 *   1. 在此文件写一个 handler 函数
 *   2. 在 g_command_table[] 中加一行
 * 删除命令步骤：
 *   1. 删掉 g_command_table[] 中对应行
 *   2. 删掉对应的 handler 函数
 */

#include "str_cmd.hpp"
#include <string.h>
#include <stdio.h>

#include "device_ina228.hpp"

#ifdef APP_IAP
#include "iap_app.hpp"
#endif

/* ============================================================
 *  用户命令处理函数 —— 在此处添加/删除
 * ============================================================ */

extern INA228 INA228_2;

/* newlib-nano 不支持 %lld，int64 统一用 (long) 转换 */

static void cmd_bus_voltage(const char *args, char *reply, size_t size) {
    (void)args;
    snprintf(reply, size, "bus_voltage=%ld mV\r\n",
             (long)INA228_2.getBusVoltage_mV());
}

static void cmd_current(const char *args, char *reply, size_t size) {
    (void)args;
    snprintf(reply, size, "current=%ld mA\r\n",
             (long)INA228_2.getCurrent_mA());
}

static void cmd_power(const char *args, char *reply, size_t size) {
    (void)args;
    snprintf(reply, size, "power=%ld mW\r\n",
             (long)INA228_2.getPower_mW());
}

static void cmd_temperature(const char *args, char *reply, size_t size) {
    (void)args;
    snprintf(reply, size, "temperature=%ld mC\r\n",
             (long)INA228_2.getTemperature_mC());
}

static void cmd_energy(const char *args, char *reply, size_t size) {
    (void)args;
    snprintf(reply, size, "energy=%ld mJ\r\n",
             (long)INA228_2.getEnergy_mJ());
}

static void cmd_charge(const char *args, char *reply, size_t size) {
    (void)args;
    snprintf(reply, size, "charge=%ld mC\r\n",
             (long)INA228_2.getCharge_mC());
}

static void cmd_shunt_voltage(const char *args, char *reply, size_t size) {
    (void)args;
    snprintf(reply, size, "shunt_voltage=%ld uV\r\n",
             (long)INA228_2.getShuntVoltage_uV());
}

static void cmd_mfr_id(const char *args, char *reply, size_t size) {
    (void)args;
    snprintf(reply, size, "manufacturer_id=0x%04X\r\n",
             INA228_2.getManufacturerID());
}

static void cmd_device_id(const char *args, char *reply, size_t size) {
    (void)args;
    snprintf(reply, size, "device_id=0x%04X\r\n",
             INA228_2.getDeviceID());
}

static void cmd_alert(const char *args, char *reply, size_t size) {
    (void)args;
    INA228_AlertFlags flags = INA228_2.readAlertFlags();
    snprintf(reply, size,
        "alert=0x%04X over_volt:%d under_volt:%d over_bus:%d "
        "under_bus:%d over_pwr:%d over_temp:%d conv_ready:%d\r\n",
        INA228_2.readDiagAlertRaw(),
        flags.shuntOverLimit, flags.shuntUnderLimit,
        flags.busOverLimit, flags.busUnderLimit,
        flags.powerOverLimit, flags.tempOverLimit,
        flags.conversionReady);
}

static void cmd_clear_alert(const char *args, char *reply, size_t size) {
    (void)args;
    INA228_2.clearAlertLatch();
    snprintf(reply, size, "alert_cleared\r\n");
}

static void cmd_all(const char *args, char *reply, size_t size) {
    (void)args;
    long bus_mv  = (long)INA228_2.getBusVoltage_mV();
    long shunt_uv = (long)INA228_2.getShuntVoltage_uV();
    long cur_ma  = (long)INA228_2.getCurrent_mA();
    long pwr_mw  = (long)INA228_2.getPower_mW();
    long temp_mc = (long)INA228_2.getTemperature_mC();
    long eng_mj  = (long)INA228_2.getEnergy_mJ();
    long chg_mc  = (long)INA228_2.getCharge_mC();
    snprintf(reply, size,
        "=== INA228 ===\r\n"
        "bus_voltage=%ld mV\r\nshunt_voltage=%ld uV\r\n"
        "current=%ld mA\r\npower=%ld mW\r\n"
        "temperature=%ld mC\r\nenergy=%ld mJ\r\ncharge=%ld mC\r\n"
        "manufacturer_id=0x%04X\r\ndevice_id=0x%04X\r\nalert=0x%04X\r\n",
        bus_mv, shunt_uv, cur_ma, pwr_mw, temp_mc, eng_mj, chg_mc,
        INA228_2.getManufacturerID(), INA228_2.getDeviceID(),
        INA228_2.readDiagAlertRaw());
}

#ifdef APP_IAP
/* IAP：触发进入升级模式（写请求标志后复位，boot 接管） */
static void cmd_iap_upgrade(const char *args, char *reply, size_t size) {
    (void)args;
    snprintf(reply, size, "iap_upgrade triggered, rebooting into bootloader...\r\n");
    Iap_RequestUpgrade();
}
#endif

/* ============================================================
 *  命令表 —— 在此处增删条目
 * ============================================================ */

const command_entry_t g_command_table[] = {
    { "ina228_bus_voltage",   cmd_bus_voltage   },
    { "ina228_current",       cmd_current       },
    { "ina228_power",         cmd_power         },
    { "ina228_temperature",   cmd_temperature   },
    { "ina228_energy",        cmd_energy        },
    { "ina228_charge",        cmd_charge        },
    { "ina228_shunt_voltage", cmd_shunt_voltage },
    { "ina228_mfr_id",        cmd_mfr_id        },
    { "ina228_device_id",     cmd_device_id     },
    { "ina228_alert",         cmd_alert         },
    { "ina228_clear_alert",   cmd_clear_alert   },
    { "ina228_all",           cmd_all           },
#ifdef APP_IAP
    { "iap_upgrade",          cmd_iap_upgrade   },
#endif
};

const uint8_t g_command_count =
    sizeof(g_command_table) / sizeof(g_command_table[0]);

/* ============================================================
 *  命令解析引擎
 * ============================================================ */

bool command_process(const char *line, char *reply, size_t size)
{
  if (!line || !*line)
  {
    snprintf(reply, size, "Error: empty command\r\n");
    return false;
  }

  /* 纯空白 → 等同空命令 */
  {
    const char *check = line;
    while (*check == ' ') check++;
    if (!*check) {
      snprintf(reply, size, "Error: empty command\r\n");
      return false;
    }
  }

  /* 提取命令名（第一个空格前的部分） */
  const char *arguments = NULL;
  const char *cursor = line;
  
  while (*cursor && *cursor != ' ')
  {
    cursor++;
  }
  size_t name_length = (size_t)(cursor - line);

  /* 提取参数部分 */
  if (*cursor == ' ')
  {
    arguments = cursor + 1;
    
    while (*arguments == ' ')
    {
      arguments++;
    }
    if (!*arguments)
    {
      arguments = NULL;
    }
  }

  /* 遍历命令表匹配 */
  for (uint8_t i = 0; i < g_command_count; i++) 
  {
    if (strlen(g_command_table[i].name) == name_length && memcmp(line, g_command_table[i].name, name_length) == 0) 
    {
      g_command_table[i].handler(arguments, reply, size);
      return true;
    }
  }

  /* 未知命令 */
  snprintf(reply, size, "Error: unknown command '%.*s'\r\n", (int)name_length,line);
  
  return false;
}
