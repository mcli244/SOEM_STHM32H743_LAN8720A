#include "ecatuser.h"

#include "stdio.h"
#include "string.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"

#include "stm32h7xx_hal.h"
// #include "gpio.h"

#include "osal.h"
#include "tim.h"
#include "motor.h"
#include "ecatutils.h"

#define ECAT_MOTOR_STEP_MAX (81920)

uint64 app_time_base = 0;
uint64 ref_time_base = 0;
uint64 sync_start_time = 0;
int64 app_time_offset = 0;

int32_t step_increment = 10;
int32_t step_is_enabled = 0;
int32_t stop_flag = 0;

// 注意：为了和SOEM内部一致，这里索引号为0的实际上没有被使用
#define MOTOR_CNT 6
PDO_Output *outputs[MOTOR_CNT];
PDO_Input *inputs[MOTOR_CNT];

char IOmap[200];
int oloop, iloop;
static motor_pos_t cur_motor_info = {0, 0, 0, 0, 0};
static uint8_t cur_motor_info_update = 0;


typedef enum
{
  ECAT_MOTOR_INITIALIZING = 0, // 初始化阶段
  ECAT_MOTOR_CSP_READY,      // CSP模式准备就绪, 定时器开始周期收发数据
  ECAT_MOTOR_CSP_RUNNING,   // CSP模式运行中
  ECAT_MOTOR_CSP_PAUSED,    // CSP模式暂停
  ECAT_MOTOR_ERROR
} motor_state_t;
static motor_state_t motor_state = ECAT_MOTOR_INITIALIZING;

motor_state_t ecat_motor_state_get(void)
{
  return motor_state;
}

void ecat_motor_state_set(motor_state_t sta)
{
  motor_state = sta;
}

int Servosetup(uint16 slave)
{
  drive_write8(slave, 0x1c12, 0, 0);
  drive_write16(slave, 0x1c12, 0, 0x1600);
  drive_write8(slave, 0x1c12, 0, 1);

  drive_write8(slave, 0x1c13, 0, 0);
  drive_write16(slave, 0x1c13, 0, 0x1A00);
  drive_write8(slave, 0x1c13, 0, 1);

  drive_write8(slave, 0x1A00, 0, 0);
  drive_write32(slave, 0x1A00, 1, 0x60410010);
  drive_write32(slave, 0x1A00, 2, 0x60640020);
  drive_write32(slave, 0x1A00, 3, 0x60610008);
  drive_write32(slave, 0x1A00, 4, 0x60770010);
  drive_write32(slave, 0x1A00, 5, 0x606C0020);
  drive_write32(slave, 0x1A00, 6, 0x60b90010);
  drive_write32(slave, 0x1A00, 7, 0x60bb0020);
  drive_write32(slave, 0x1A00, 8, 0x60ba0020);
  drive_write32(slave, 0x1A00, 9, 0x213f0010);
  drive_write8(slave, 0x1A00, 0, 9);

  drive_write8(slave, 0x1600, 0, 0);
  drive_write32(slave, 0x1600, 1, 0x60400010);
  drive_write32(slave, 0x1600, 2, 0x607a0020);
  drive_write32(slave, 0x1600, 3, 0x60600008);
  drive_write32(slave, 0x1600, 4, 0x60710010);
  drive_write32(slave, 0x1600, 5, 0x60b80010);
  drive_write32(slave, 0x1600, 6, 0x60ff0020);
  drive_write8(slave, 0x1600, 0, 6);

  return 1;
}

void dump_slave_pdo(uint16 slave)
{
    uint16 rxpdo_index = 0x1C12;
    uint16 txpdo_index = 0x1C13;
    uint8 n_entries = 0;
    uint32 pdo;
    int size;
    
    rt_kprintf("=== 从站 %d (%s) PDO 映射信息 ===\n", slave, ec_slave[slave].name);

    // 打印 SyncManager 信息
    for (int i = 0; i < EC_MAXSM; i++)
    {
        if (ec_slave[slave].SM[i].SMlength > 0)
        {
            rt_kprintf("  SM[%d]: Addr=0x%04X, Len=%d, Type=%d\n",
                       i, ec_slave[slave].SM[i].StartAddr,
                       ec_slave[slave].SM[i].SMlength,
                       ec_slave[slave].SMtype[i]);
        }
    }

    // 打印 RxPDO 映射项
    size = sizeof(n_entries);
    if (ec_SDOread(slave, rxpdo_index, 0x00, FALSE, &size, &n_entries, EC_TIMEOUTRXM) > 0)
    {
        rt_kprintf("  RxPDO 映射数量: %d (Index: 0x1C12)\n", n_entries);
        for (uint8 i = 1; i <= n_entries; i++)
        {
            size = sizeof(pdo);
            if (ec_SDOread(slave, rxpdo_index, i, FALSE, &size, &pdo, EC_TIMEOUTRXM) > 0)
            {
                uint16 index = pdo & 0xFFFF;
                rt_kprintf("    RxPDO[%d]: 0x%04X\n", i, index);

                // 获取该 PDO 映射的子项
                uint8 sub_entries = 0;
                size = sizeof(sub_entries);
                if (ec_SDOread(slave, index, 0x00, FALSE, &size, &sub_entries, EC_TIMEOUTRXM) > 0)
                {
                    for (uint8 j = 1; j <= sub_entries; j++)
                    {
                        uint32 map_entry;
                        size = sizeof(map_entry);
                        if (ec_SDOread(slave, index, j, FALSE, &size, &map_entry, EC_TIMEOUTRXM) > 0)
                        {
                            uint16 map_index = (map_entry >> 16) & 0xFFFF;
                            uint8 map_sub = (map_entry >> 8) & 0xFF;
                            uint8 map_bits = map_entry & 0xFF;
                            rt_kprintf("      -> 0x%04X:%02X (%d bits)\n", map_index, map_sub, map_bits);
                        }
                    }
                }
            }
        }
    }

    // 打印 TxPDO 映射项（0x1C13）
    size = sizeof(n_entries);
    if (ec_SDOread(slave, txpdo_index, 0x00, FALSE, &size, &n_entries, EC_TIMEOUTRXM) > 0)
    {
        rt_kprintf("  TxPDO 映射数量: %d (Index: 0x1C13)\n", n_entries);
        for (uint8 i = 1; i <= n_entries; i++)
        {
            size = sizeof(pdo);
            if (ec_SDOread(slave, txpdo_index, i, FALSE, &size, &pdo, EC_TIMEOUTRXM) > 0)
            {
                uint16 index = pdo & 0xFFFF;
                rt_kprintf("    TxPDO[%d]: 0x%04X\n", i, index);

                // 获取该 PDO 映射的子项
                uint8 sub_entries = 0;
                size = sizeof(sub_entries);
                if (ec_SDOread(slave, index, 0x00, FALSE, &size, &sub_entries, EC_TIMEOUTRXM) > 0)
                {
                    for (uint8 j = 1; j <= sub_entries; j++)
                    {
                        uint32 map_entry;
                        size = sizeof(map_entry);
                        if (ec_SDOread(slave, index, j, FALSE, &size, &map_entry, EC_TIMEOUTRXM) > 0)
                        {
                            uint16 map_index = (map_entry >> 16) & 0xFFFF;
                            uint8 map_sub = (map_entry >> 8) & 0xFF;
                            uint8 map_bits = map_entry & 0xFF;
                            rt_kprintf("      -> 0x%04X:%02X (%d bits)\n", map_index, map_sub, map_bits);
                        }
                    }
                }
            }
        }
    }
}

void ecat_init(void)
{
  static int frist_in = 1;
  int slc;
  //    int i,chk;
  int cnt = 1;
  int expectedWKC;

  /* initialise SOEM, bind socket to ifname */
  if (ec_init("eth0"))
  {
    rt_kprintf("ec_init succeeded.\r\n");
    ecat_delay_ms(100);
  _start:
    if (ec_config_init(TRUE) > 0)
    {
      rt_kprintf("%d slaves found and configured.\r\n", ec_slavecount);
      if (ec_slavecount >= 1)
      {
        for (slc = 1; slc <= ec_slavecount; slc++)
        {
          rt_kprintf("Found %s at position %d\n", ec_slave[slc].name, slc);
          ec_slave[slc].PO2SOconfig = &Servosetup;
        }
      }

      ec_config_map(&IOmap);
      ec_configdc();

      for (int i = 1; i <= ec_slavecount; i++)
      {
        if (ec_slave[i].hasdc)
        {
          ec_dcsync0(i, TRUE, SYNC0TIME, 250000);
        }
      }

      if (frist_in)
      {
        frist_in = 0;
        goto _start;
      }

      rt_kprintf("Slaves mapped, state to SAFE_OP.\n \r");
      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
      /* read indevidual slave state and store in ec_slave[] */
      ec_readstate();
      for (cnt = 1; cnt <= ec_slavecount; cnt++)
      {
        rt_kprintf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n \r",
                   cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                   ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
      }

      oloop = ec_slave[0].Obytes;
      if ((oloop == 0) && (ec_slave[0].Obits > 0))
        oloop = 1;
      if (oloop > 30)
        oloop = 30;

      iloop = ec_slave[0].Ibytes;
      if ((iloop == 0) && (ec_slave[0].Ibits > 0))
        iloop = 1;
      if (iloop > 30)
        iloop = 30;

      rt_kprintf("oloop:%d iloop:%d\n\r", oloop, iloop);
      rt_kprintf("segments : %d : %d %d %d %d\n \r", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);
      rt_kprintf("Request operational state for all slaves\n \r");
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      rt_kprintf("Calculated workcounter %d\n \r", expectedWKC);

      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_writestate(0);
      rt_kprintf("DC capable : %d\r\n", ec_configdc());
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_writestate(0);
      int chk = 200;
      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
      } while (chk-- && ec_slave[0].state != EC_STATE_OPERATIONAL);
      rt_kprintf("\r\n%d %d\r\n", ec_slave[0].state, ec_slave[1].state);

      if (ec_slave[0].state == EC_STATE_OPERATIONAL)
      {
        // for (slc = 1; slc <= ec_slavecount; slc++)
        for (slc = 1; slc < MOTOR_CNT; slc++)
        {
          outputs[slc] = (PDO_Output *)ec_slave[slc].outputs;
          inputs[slc] = (PDO_Input *)ec_slave[slc].inputs;
        }
        ecat_motor_state_set(ECAT_MOTOR_CSP_READY);
        rt_thread_delay(10);
        _ec_start_csp(); // 启动CSP模式
        rt_kprintf("all slaves reached operational state.\r\n");
      }
      else
      {
        rt_kprintf("Not all slaves reached operational state.\n \r");
      }
    }
    else
    {
      rt_kprintf("No slaves found!\r\n");
    }
  }
  else
  {
    rt_kprintf("No socket connection Excecute as root\r\n");
  }
}

void ecat_loop(void)
{
  motor_state_t sta = ecat_motor_state_get();

  if (sta == ECAT_MOTOR_INITIALIZING || sta == ECAT_MOTOR_ERROR)
    return; 

  ec_receive_processdata(EC_TIMEOUTRET);
  if(step_is_enabled)
  {
    for (int i = 1; i <= ec_slavecount; i++)
    {
      rt_base_t level = rt_hw_interrupt_disable();
      outputs[i]->TargetPos += step_increment;
      rt_hw_interrupt_enable(level);
    }
  }
  else
  {
    if (cur_motor_info_update)
    {
      // rt_base_t level = rt_hw_interrupt_disable();
      outputs[1]->TargetPos = cur_motor_info.x;
      outputs[2]->TargetPos = cur_motor_info.y;
      outputs[3]->TargetPos = cur_motor_info.z;
      outputs[4]->TargetPos = cur_motor_info.a;
      outputs[5]->TargetPos = cur_motor_info.b;
      // rt_hw_interrupt_enable(level);
      cur_motor_info_update = 0;
    }
  }
  ec_send_processdata();
  
}

int _ec_mode_check(int slave, int mode, int timeout_ms)
{
  // 检查所有电机是否已经退出CSP模式
  int chk = timeout_ms/50;
  int all_exited = 1;
  do
  {
    all_exited = 1;
    if(slave == 0)
    {
      for (int i = 1; i <= ec_slavecount; i++)
      {
        if(inputs[i]->TargetMode != mode)
          all_exited = 0;
      }
    }
    else
    {
      if(inputs[slave]->TargetMode != mode)
        all_exited = 0;
    }
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_send_processdata();
    rt_thread_mdelay(50); 
  } while (all_exited == 0 && chk--);
  
  if(all_exited == 0)
    return -1;
  
  return 0;
}

int _ec_start_csp_comm(int slave, int force)
{
  motor_state_t sta = ecat_motor_state_get();
  if ((sta == ECAT_MOTOR_CSP_RUNNING || sta == ECAT_MOTOR_CSP_PAUSED )&& slave == 0 && force == 0)
  {
    rt_kprintf("ecat already in CSP mode\r\n");
    return 0;
  }

  if(slave == 0)
  {
    for (int i = 1; i <= ec_slavecount; i++)
    {
      outputs[i]->TargetMode = 8;  
      outputs[i]->ControlWord |= (1<<7); // bit7 0 -> 1 清除错误
    }
    ec_send_processdata();
    rt_thread_mdelay(10);
  }
  else
  {
    outputs[slave]->TargetMode = 8;    
    outputs[slave]->ControlWord |= (1<<7); // bit7 0 -> 1 清除错误
    ec_send_processdata();
    rt_thread_mdelay(10);
  }
  

  if(slave == 0)
  {
    for (int i = 1; i <= ec_slavecount; i++)
    {
      outputs[i]->TargetMode = 8;     // CSP模式
      outputs[i]->ControlWord = 0x06; // Shutdown
    }
  }
  else
  {
    outputs[slave]->TargetMode = 8;     // CSP模式
    outputs[slave]->ControlWord = 0x06; // Shutdown
  }
  ec_send_processdata();
  rt_thread_mdelay(50); // 进入CSP模式后, 需要至少等待20ms， 才能进行位置更新
  
  if(slave == 0)
  {
    for (int i = 1; i <= ec_slavecount; i++)
    {
      outputs[i]->ControlWord = 0x07;
      outputs[i]->TargetPos = inputs[i]->CurrentPosition; // 初始对齐
    }
  }
  else
  {
    outputs[slave]->ControlWord = 0x07;
    outputs[slave]->TargetPos = inputs[slave]->CurrentPosition; // 初始对齐
  }
  ec_send_processdata();
  rt_thread_mdelay(10);

  if(slave == 0)
  {
    for (int i = 1; i <= ec_slavecount; i++)
    {
      outputs[i]->ControlWord = 0x0F;
    }
  }
  else
  {
    outputs[slave]->ControlWord = 0x0F;
  }
  ec_send_processdata();
  rt_thread_mdelay(10);

  int ret = _ec_mode_check(slave, 8, 1000); // 检查所有电机是否已经进入CSP模式
  if (ret < 0)
  {
    rt_kprintf("❌ 部分电机未能成功进入CSP模式，请检查连接或配置\n");
    for (int i = 1; i <= ec_slavecount; i++)
      rt_kprintf("电机 %d 当前 TargetMode: %d\n", i, inputs[i]->TargetMode, inputs[i]);
    return -1;
  }

  ecat_motor_state_set(ECAT_MOTOR_CSP_RUNNING);
  return 0;
}


#define MAX_SAFEOP_RETRY 10
#define SAFEOP_TIMEOUT_MS 100

/**
 * 尝试将所有从站设置为 SAFE_OP 状态，包含重试机制
 * 返回 0 表示成功，-1 表示失败
 */
int ec_try_set_state_safe_op(void)
{
  int retry = 0;
  int result = 0;

  // 先检查是否已经处于 SAFE_OP 状态
  result = ec_statecheck(0, EC_STATE_SAFE_OP, SAFEOP_TIMEOUT_MS / 2);
  if (result == EC_STATE_SAFE_OP)
  {
      rt_kprintf("ℹ️ 当前已处于 SAFE_OP 状态，无需切换\n");
      return 0;
  }
  rt_kprintf("result:%d\n", result);

  while (retry < MAX_SAFEOP_RETRY)
  {
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);

    result = ec_statecheck(0, EC_STATE_SAFE_OP, SAFEOP_TIMEOUT_MS);
    if (result == EC_STATE_SAFE_OP)
    {
      rt_kprintf("✅ 所有从站成功进入 SAFE_OP 状态（尝试 %d 次）\n", retry + 1);
      return 0;
    }

    rt_kprintf("⚠️ 第 %d 次尝试 SAFE_OP 失败，返回状态 0x%04X，重试...\n", retry + 1, result);
    retry++;
  }

  rt_kprintf("❌ 所有重试失败，无法进入 SAFE_OP 状态\n");
  return -1;
}

#define MAX_OP_RETRY 20
#define OP_TIMEOUT_MS 100

/**
 * 尝试将所有从站设置为 OPERATIONAL 状态，包含重试机制
 * 返回 0 表示成功，-1 表示失败
 */
int ec_try_set_state_op(void)
{
  int result = 0;

  // ✅ 先检查是否已经处于 OPERATIONAL 状态
  result = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTRET / 2);
  if (result == EC_STATE_OPERATIONAL)
  {
      rt_kprintf("ℹ️ 当前已处于 OPERATIONAL 状态，无需切换\n");
      return 0;
  }

  // ✅ 设置期望状态为 OP
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  ec_writestate(0);

  int chk = 100;
  do
  {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
  } while (chk-- && ec_slave[0].state != EC_STATE_OPERATIONAL);

  rt_kprintf("\r\nchk: %d  master: 0x%04X  slave1: 0x%04X\r\n",
              chk, ec_slave[0].state, ec_slave[1].state);

  if (ec_slave[0].state != EC_STATE_OPERATIONAL)
  {
      rt_kprintf("❌ ec_try_set_state_op: 设置 OPERATIONAL 失败，当前状态: 0x%04X\r\n", ec_slave[0].state);
      return -1;
  }

  rt_kprintf("✅ 所有从站成功进入 OPERATIONAL 状态\n");
  return 0;
}

int _ec_start_op(int slave_index)
{
  ec_slave[slave_index].state = EC_STATE_SAFE_OP;
  ec_writestate(slave_index);
  if (ec_statecheck(slave_index, EC_STATE_SAFE_OP, EC_TIMEOUTRET3) != EC_STATE_SAFE_OP) {
      printf("从站未进入 SAFE-OP\n");
      return -1;
  }

  int retry = 100;
  do
  {
    ec_slave[slave_index].state = EC_STATE_OPERATIONAL;
    ec_writestate(slave_index);
    if (ec_statecheck(slave_index, EC_STATE_OPERATIONAL, EC_TIMEOUTRET3) != EC_STATE_OPERATIONAL) {
        retry --;
    }
    else
    {
        break;
    }
    rt_thread_delay(10); // 等待10ms后重试
  } while (retry);
  if (retry <= 0)
  {
      printf("从站 %d 无法进入 OP 模式\n", slave_index);
      return -1;
  }
  printf("从站 %d 成功进入 OP 模式\n", slave_index);
  return 0;
}

int _ec_start_csp(void)
{
  _ec_start_csp_comm(0, 0); // 传入0表示对所有电机进行操作
  return 0;
}
MSH_CMD_EXPORT(_ec_start_csp, "_ec_start_csp");

int _ec_csp_continue(void)
{
  motor_state_t sta = ecat_motor_state_get();
  if (sta != ECAT_MOTOR_CSP_PAUSED)
  {
    rt_kprintf("ecat not is csp paused\r\n");
    return -1;
  }

  // 检测出错的电机
  for (int i = 1; i <= ec_slavecount; i++)
  {
    if (inputs[i]->StatusWord & 0x0008) // bit3伺服故障: 0：无故障，1：有故障
    {
      rt_kprintf("电机 %d 出现错误，状态字: 0x%04X state of slave:%d\n", i, inputs[i]->StatusWord, ec_slave[i].state);
      if(ec_slave[i].state != EC_STATE_OPERATIONAL)
      {
        rt_kprintf("电机 %d 未处于 OP 状态，尝试重启...\n", i);
        int ret = _ec_start_op(i); // 尝试将出错的电机设置为 OP 状态
        if (ret < 0)
        {
          rt_kprintf("❌ 电机 %d 无法进入 OP 状态，请检查连接或配置\n", i);
        }
        else
        {
          rt_kprintf("✅ 电机 %d 已成功重启并进入 OP 状态\n", i);
        }
      }
      int ret = _ec_start_csp_comm(i, 1); // 尝试对出错的电机进行CSP模式重启
      if(ret < 0)
      {
        rt_kprintf("❌ 电机 %d 无法进入CSP模式，请检查连接或配置\n", i);
        return -1; // 如果有任何电机无法进入CSP模式，则返回错误
      }
      else
      {
        rt_kprintf("✅ 电机 %d 已成功重启并进入CSP模式\n", i);
      }
    }
  }

  for (int i = 1; i <= ec_slavecount; i++)
  {
    outputs[i]->TargetMode = 8;     // CSP模式
    outputs[i]->ControlWord = 0x0F; // Shutdown
  }
  ec_send_processdata();
  rt_thread_mdelay(50); // 进入CSP模式后, 需要至少等待20ms， 才能进行位置更新

  step_is_enabled = 1; // 允许步进控制
  ecat_motor_state_set(ECAT_MOTOR_CSP_RUNNING);

  return 0;
}
MSH_CMD_EXPORT(_ec_csp_continue, "_ec_csp_continue");

int _ec_csp_pause(void)
{
  motor_state_t sta = ecat_motor_state_get();
  if (sta != ECAT_MOTOR_CSP_RUNNING)
  {
    rt_kprintf("ecat not is csp running\r\n");
    return -1;
  }

  if(step_is_enabled)
    step_is_enabled = 0;
  
  for (int i = 1; i <= ec_slavecount; i++)
  {
    outputs[i]->TargetMode = 1;     // 退出CSP模式
    outputs[i]->ControlWord = 0x0F; // 保持电机力矩
  }
  ec_send_processdata();

  int ret = _ec_mode_check(0, 1, 1000); 
  if(ret < 0)
  {
    rt_kprintf("❌ 部分电机未能成功暂停CSP模式，请检查连接或配置\n");
    for (int i = 1; i <= ec_slavecount; i++)
    {
      rt_kprintf("电机 %d 当前 TargetMode: %d\n", i, inputs[i]->TargetMode, inputs[i]);
    }
    return -1;
  }

  rt_kprintf("✅ 所有电机已成功暂停CSP模式\n");
  ecat_motor_state_set(ECAT_MOTOR_CSP_PAUSED);

  return 0;
}
MSH_CMD_EXPORT(_ec_csp_pause, "_ec_csp_pause");


void ecat_test_main(void *parameter)
{
  rt_hw_stm32_tim();
  HAL_TIM_Base_Start_IT(&htim4);
  ecat_delay_ms(100);
  ecat_init();
}

int ecat_start(void)
{
  rt_thread_t tid;
  tid = rt_thread_create("ecat_test",
                         ecat_test_main,
                         RT_NULL,
                         1024 * 8,
                         5,
                         2);
  if (tid != RT_NULL)
  {
    rt_thread_startup(tid);
  }
  else
  {
    rt_kprintf("state = -RT_ERROR\n");
  }
  return 0;
}
MSH_CMD_EXPORT(ecat_start, "ecat_start");



int ecat_continue(void)
{
  if (stop_flag)
  {
    stop_flag = 0;
    rt_kprintf("ecat_continue\r\n");
  }
  return 0;
}
MSH_CMD_EXPORT(ecat_continue, "ecat_continue");

int ecat_status(void)
{
  rt_kprintf("stop_flag:%d\r\n", stop_flag);
  rt_kprintf("ec_slavecount:%d\r\n", ec_slavecount);
  rt_kprintf("cur_motor_info x:%08d y:%08d z:%08d a:%08d b:%08d\r\n",
             cur_motor_info.x, cur_motor_info.y, cur_motor_info.z, cur_motor_info.a, cur_motor_info.b);

  motor_state_t sta = ecat_motor_state_get();
  if (sta != ECAT_MOTOR_INITIALIZING)
  {
    for (int i = 1; i <= ec_slavecount; i++)
    {
      rt_kprintf("state:%d; StatusWord:%x, CurrentPosition:%8ld,TargetMode:%d CurrentTorque:%8d CurrentSpeed:%8d ServoError:%d\r\n",
                 ec_slave[i].state, inputs[i]->StatusWord, inputs[i]->CurrentPosition, inputs[i]->TargetMode,
                 inputs[i]->CurrentTorque, inputs[i]->CurrentSpeed, inputs[i]->ServoError);
    }
  }
  else
  {
    rt_kprintf("ecat not running\r\n");
    return -1;
  }
  return 0;

  return 0;
}
MSH_CMD_EXPORT(ecat_status, "ecat_status");

void ecat_set_pos(int argc, char **argv)
{
  motor_state_t sta = ecat_motor_state_get();
  if (sta != ECAT_MOTOR_CSP_RUNNING)
  {
    rt_kprintf("ecat_set_pos: motor not in csp running\r\n");
    return;
  }

  if (argc != 2)
  {
    rt_kprintf("Usage: ecat_set_pos step_increment\r\n");
    rt_kprintf("rg: ecat_set_pos 100\r\n");
    return;
  }

  step_is_enabled = 1;
  step_increment = atoi(argv[1]);
  rt_kprintf("ecat_set_pos step_increment:%d\r\n", step_increment);
}
MSH_CMD_EXPORT(ecat_set_pos, "ecat_set_pos");

//////////////////////////////// 电机控制对外接口 ////////////////////////////////////////////////////////////
int motor_set(motor_pos_t *pos)
{
  if (pos == NULL)
  {
    rt_kprintf("ecat_motor_ctrl: pos is NULL\r\n");
    return -1;
  }

  motor_state_t sta = ecat_motor_state_get();
  if (sta != ECAT_MOTOR_CSP_RUNNING)
  {
    rt_kprintf("ecat_set_pos: motor not in csp running\r\n");
    return -1;
  }

  // if (abs(pos->x) > ECAT_MOTOR_STEP_MAX ||
  //     abs(pos->y) > ECAT_MOTOR_STEP_MAX ||
  //     abs(pos->z) > ECAT_MOTOR_STEP_MAX ||
  //     abs(pos->a) > ECAT_MOTOR_STEP_MAX ||
  //     abs(pos->b) > ECAT_MOTOR_STEP_MAX)
  // {
  //   rt_kprintf("ecat_motor_ctrl: position out of range ECAT_MOTOR_STEP_MAX:%d\r\n", ECAT_MOTOR_STEP_MAX);
  //   rt_kprintf("ecat_motor_ctrl: x:%d y:%d z:%d a:%d b:%d\r\n",
  //              pos->x, pos->y, pos->z,
  //              pos->a, pos->b);
  //   return -1;
  // }

  if(motor_get(&cur_motor_info) < 0)
  {
    rt_kprintf("ecat_motor_ctrl: motor_get failed\r\n");
    return -1;
  }

  cur_motor_info_update = 0;
  cur_motor_info.x += pos->x;
  cur_motor_info.y += pos->y;
  cur_motor_info.z += pos->z;
  cur_motor_info.a += pos->a;
  cur_motor_info.b += pos->b;
  cur_motor_info_update = 1;

  return 0;
}

int motor_get(motor_pos_t *pos)
{
  if (pos == NULL)
  {
    rt_kprintf("ecat_get_motor_info: pos is NULL\r\n");
    return -1;
  }

  motor_state_t sta = ecat_motor_state_get();
  if (sta != ECAT_MOTOR_CSP_RUNNING)
  {
    rt_kprintf("ecat_set_pos: motor not in csp running\r\n");
    return -1;
  }

  pos->x = inputs[1]->CurrentPosition;
  pos->y = inputs[2]->CurrentPosition;
  pos->z = inputs[3]->CurrentPosition;
  pos->a = inputs[4]->CurrentPosition;
  pos->b = inputs[5]->CurrentPosition;

  return 0;
}

void motor_set_test(int argc, char **argv)
{

  if (argc != 6)
  {
    rt_kprintf("Usage: motor_set_test x y z a b\r\n");
    rt_kprintf("rg: motor_set_test 100 200 300 400 500\r\n");
    return;
  }

  step_is_enabled = 0;

  motor_pos_t m_info = {0, 0, 0, 0, 0};
  m_info.x = atoi(argv[1]);
  m_info.y = atoi(argv[2]);
  m_info.z = atoi(argv[3]);
  m_info.a = atoi(argv[4]);
  m_info.b = atoi(argv[5]);
  rt_kprintf("motor_set_test x:%d y:%d z:%d a:%d b:%d\r\n",
             m_info.x, m_info.y, m_info.z,
             m_info.a, m_info.b);
  int ret = motor_set(&m_info);
  if (ret < 0)
  {
    rt_kprintf("ecat_motor_ctrl failed: %d\r\n", ret);
  }
  else
  {
    rt_kprintf("ecat_motor_ctrl success\r\n");
  }
}
MSH_CMD_EXPORT(motor_set_test, "motor_set_test");

void motor_get_test(int argc, char **argv)
{
  motor_pos_t m_info = {0, 0, 0, 0, 0};
  int ret = motor_get(&m_info);
  if (ret < 0)
  {
    rt_kprintf("motor_get_test failed: %d\r\n", ret);
  }
  else
  {
    rt_kprintf("motor_get_test success\r\n");
    rt_kprintf("Motor Info - x:%d y:%d z:%d a:%d b:%d\r\n",
               m_info.x, m_info.y, m_info.z,
               m_info.a, m_info.b);
  }
}
MSH_CMD_EXPORT(motor_get_test, "motor_get_test");

int motor_init(void)
{
  motor_state_t sta = ecat_motor_state_get();
  if (sta != ECAT_MOTOR_INITIALIZING)
  {
    rt_kprintf("motor_init: motor already initialized\r\n");
    return -1;
  }

  ecat_start();

  return RT_EOK;
}
MSH_CMD_EXPORT(motor_init, "motor_init");

int motor_deinit(void)
{
  return RT_EOK;
}
MSH_CMD_EXPORT(motor_deinit, "motor_deinit");

int motor_ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex,
                      boolean CA, int psize, void *p, int Timeout)
{
  motor_state_t sta = ecat_motor_state_get();
  if (sta != ECAT_MOTOR_CSP_PAUSED)
  {
    rt_kprintf("motor_ec_SDOwrite: The SDO operation is disabled in current state: %d\r\n", sta);
    return -1;
  }

  int wkc = ec_SDOwrite(Slave, Index, SubIndex, CA, psize, p, Timeout);
  if(wkc <= 0)
  {
    rt_kprintf("motor_ec_SDOwrite failed wkc:%d\r\n", wkc);
    return -1;
  }

  return 0;
}

int motor_ec_SDOread(uint16 Slave, uint16 Index, uint8 SubIndex,
                     boolean CA, int *psize, void *p, int Timeout)
{
  motor_state_t sta = ecat_motor_state_get();
  if (sta != ECAT_MOTOR_CSP_PAUSED)
  {
    rt_kprintf("motor_ec_SDOwrite: The SDO operation is disabled in current state: %d\r\n", sta);
    return -1;
  }

  int wkc = ec_SDOread(Slave, Index, SubIndex, CA, psize, p, Timeout);
  if(wkc <= 0)
  {
    rt_kprintf("ec_SDOread failed wkc:%d\r\n", wkc);
    return -1;
  }

  return 0;
}

int motor_SDO_rw_test(void)
{
  // TODO: 需要判断当前状态，PDO模式下不能读取，会打断周期同步
  uint32_t cur_value = 0;
  int size = sizeof(cur_value);
  int ret = 0;

  // 1. 读取原始值
  ret = motor_ec_SDOread(1, 0x6065, 0, FALSE, &size, &cur_value, EC_TIMEOUTRXM);  // 用户位置偏差过大阈值
  if (ret < 0)
  {
    rt_kprintf("motor_ec_SDOread failed ret:%d\r\n", ret);
    return -1;
  }
  rt_kprintf("0x6065[0]:0x%x ret:%d\r\n", cur_value, ret);

  // 2. 写入测试值
  uint32_t test_val = 0x1234;
  ret = motor_ec_SDOwrite(1, 0x6065, 0, FALSE, size, &test_val, EC_TIMEOUTRXM);
  if (ret < 0)
  {
    rt_kprintf("motor_ec_SDOwritefailed ret:%d\r\n", ret);
    return -1;
  }

  // 3. 读取验证
  uint32_t tmp_val = 0;
  ret = motor_ec_SDOread(1, 0x6065, 0, FALSE, &size, &tmp_val, EC_TIMEOUTRXM);
  if (ret < 0)
  {
    rt_kprintf("motor_ec_SDOread failed ret:%d\r\n", ret);
    return -1;
  }
  rt_kprintf("0x6065[0]:0x%x ret:%d\r\n", tmp_val, ret);

  // 4. 写回原始值
  ret = motor_ec_SDOwrite(1, 0x6065, 0, FALSE, size, &cur_value, EC_TIMEOUTRXM);
  if (ret < 0)
  {
    rt_kprintf("motor_ec_SDOwrite failed ret:%d\r\n", ret);
    return -1;
  }
  rt_kprintf("motor_SDO_rw_test write back 0x6065[0]:0x%x ret:%d\r\n", cur_value, ret);

  if (tmp_val != test_val)
  {
    rt_kprintf("check failed expected:0x%x, got:0x%x\r\n", test_val, tmp_val);
    return -1;
  }
  rt_kprintf("motor_SDO_rw_test successful tmp_val:0x%x test_val:0x%x\r\n", tmp_val, test_val);

  return 0;
}
MSH_CMD_EXPORT(motor_SDO_rw_test, "motor_SDO_rw_test");

int motor_ec_csp_pause(void)
{
  return _ec_csp_pause();
}
MSH_CMD_EXPORT(motor_ec_csp_pause, "motor_ec_csp_pause");

int motor_ec_csp_continue(void)
{
  return _ec_csp_continue();
}
MSH_CMD_EXPORT(motor_ec_csp_continue, "motor_ec_csp_continue");
