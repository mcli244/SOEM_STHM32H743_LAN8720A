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
uint8 wkc;

int32_t step_increment = 10;
int32_t step_is_enabled = 0;
int32_t stop_flag = 0;

// 注意：为了和SOEM内部一致，这里索引号为0的实际上没有被使用
#define MOTOR_CNT 6
PDO_Output *outputs[MOTOR_CNT];
PDO_Input *inputs[MOTOR_CNT];

char IOmap[200];
uint32_t dorun = 0;
int oloop, iloop;

// motor control


typedef enum
{
  ECAT_MOTOR_INITIALIZING = 0, // 初始化阶段
  ECAT_MOTOR_IDEL = 1,
  ECAT_MOTOR_READY,
  ECAT_MOTOR_SWITCH_ON,
  ECAT_MOTOR_OPERATION_ENABLED,
  ECAT_MOTOR_ROTATION_STOP,
  ECAT_MOTOR_STOP_1,
  ECAT_MOTOR_STOP_2,
  ECAT_MOTOR_STOP_3,
  ECAT_MOTOR_STOP_4, // 保持周期同步
  ECAT_MOTOR_ERROR
} motor_state_t;
static motor_state_t motor_state = ECAT_MOTOR_INITIALIZING;

uint8_t motor_is_stop = 0;
static motor_pos_t cur_motor_info = {0, 0, 0, 0, 0};
static uint8_t cur_motor_info_update = 0;

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
        dorun = 1;
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


/*
CIA402状态机
Not ready to switch on	  驱动器正在初始化，不能进行通讯
Switch on disabled	      驱动器初始化完成，可进行通讯
Ready to switch on	      等待进入Switch On状态，电机未被励磁
Switched on	              驱动器已准备好
Operation enabled	        驱动器给电机励磁信号，电机使能
Quick stop active	        驱动器根据设定的方式停机
Fault reaction active	    驱动器检测到报警，按设定方式停机，电机处于励磁状态
Fault	                    故障，电机无励磁
*/

void motor_state_set(motor_state_t sta)
{
  motor_state = sta;
}

motor_state_t motor_state_get(void)
{
  return motor_state;
}

void ecat_loop(void)
{
  static int statup_delay_cnt = 0;
  if (dorun > 0)
  {
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    int all_ready = 1;
    switch (motor_state)
    {
    case ECAT_MOTOR_IDEL: // 初始化 -> Ready to Switch On
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->TargetMode = 8;     // CSP模式
        outputs[i]->ControlWord = 0x06; // Shutdown
      }

      if (statup_delay_cnt < 50) // 50*loop周期(1ms) = 50ms
      {
        statup_delay_cnt++;
        break;
      }

      for (int i = 1; i <= ec_slavecount; i++)
      {
        uint16_t status = inputs[i]->StatusWord;
        if (!(status == 0x0231 || status == 0x0631 || status == 0x0221 || status == 0x1221))
        {
          all_ready = 0;
          break;
        }
      }

      if (all_ready)
      {
        statup_delay_cnt = 0;
        motor_state_set(ECAT_MOTOR_READY);
      }
      break;
    case ECAT_MOTOR_READY: // Ready to Switch On -> Switched On
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x07;
        outputs[i]->TargetPos = inputs[i]->CurrentPosition; // 初始对齐
      }

      cur_motor_info.x = inputs[1]->CurrentPosition;
      cur_motor_info.y = inputs[2]->CurrentPosition;
      cur_motor_info.z = inputs[3]->CurrentPosition;
      cur_motor_info.a = inputs[4]->CurrentPosition;
      cur_motor_info.b = inputs[5]->CurrentPosition;

      for (int i = 1; i <= ec_slavecount; i++)
      {
        uint16_t status = inputs[i]->StatusWord;
        if (!(status == 0x0233 || status == 0x0633 || status == 0x0223 || status == 0x1223 || status == 0x1233))
        {
          all_ready = 0;
          break;
        }
      }

      if (all_ready)
        motor_state_set(ECAT_MOTOR_SWITCH_ON);
      break;

    case ECAT_MOTOR_SWITCH_ON: // Switched On -> Operation Enabled
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x0F;
      }

      for (int i = 1; i <= ec_slavecount; i++)
      {
        uint16_t status = inputs[i]->StatusWord;
        if (!(status == 0x1637 || status == 0x1633 || status == 0x1237))
        {
          all_ready = 0;
          break;
        }
      }

      if (all_ready)
        motor_state_set(ECAT_MOTOR_OPERATION_ENABLED);
      break;

    case ECAT_MOTOR_OPERATION_ENABLED: // 正常控制阶段（位置模式）
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

      if (stop_flag)
      {
        motor_state_set(ECAT_MOTOR_ROTATION_STOP);
      }
      break;
    case ECAT_MOTOR_ROTATION_STOP: // 转动停止，保持当前的位置，不再接收新的位置
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x0F;                     // 保持 Operation Enabled
        outputs[i]->TargetPos = inputs[i]->CurrentPosition; // 停止在当前位置
      }
      motor_state_set(ECAT_MOTOR_STOP_1);
    case ECAT_MOTOR_STOP_1:
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x07; // Shutdown
      }
      motor_state_set(ECAT_MOTOR_STOP_2);
      break;
    case ECAT_MOTOR_STOP_2:
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x06; // Shutdown
      }
      motor_state_set(ECAT_MOTOR_STOP_3);
      break;
    case ECAT_MOTOR_STOP_3:
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x00;
      }
      motor_state_set(ECAT_MOTOR_STOP_4);
      motor_is_stop = 1;
      rt_kprintf("ecat_stop acti\r\n");
      break;
    case ECAT_MOTOR_STOP_4: // 保持周期同步
      if (stop_flag == 0)
      {
        motor_state_set(ECAT_MOTOR_INITIALIZING);
      }
      break;

    default:
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x80; // 带立即更新位
      }
      motor_state_set(ECAT_MOTOR_IDEL);
      break;
    }
    
    ec_send_processdata();
  }
}

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

int ecat_stop(void)
{
  if (dorun)
  {
    stop_flag = 1;
    rt_kprintf("ecat_stop\r\n");
  }
  return 0;
}
MSH_CMD_EXPORT(ecat_stop, "ecat_stop");

#define MAX_SAFEOP_RETRY 3
#define SAFEOP_TIMEOUT_MS 100

/**
 * 尝试将所有从站设置为 SAFE_OP 状态，包含重试机制
 * 返回 0 表示成功，-1 表示失败
 */
int ec_try_set_state_safe_op(void)
{
  int retry = 0;
  int result = 0;

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
  int retry = 0;
  int result = 0;

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
  rt_kprintf("\r\nchk:%d %d %d\r\n", chk, ec_slave[0].state, ec_slave[1].state);

  if (ec_slave[0].state != EC_STATE_OPERATIONAL)
  {
    rt_kprintf("ec_try_set_state_op: Failed to set state to OPERATIONAL, current state: 0x%04X\r\n", ec_slave[0].state);
    return -1;
  }

  return 0;
}

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
  rt_kprintf("dorun:%d\r\n", dorun);
  rt_kprintf("motor_state:%d\r\n", motor_state);
  rt_kprintf("stop_flag:%d\r\n", stop_flag);
  rt_kprintf("ec_slavecount:%d\r\n", ec_slavecount);
  rt_kprintf("cur_motor_info x:%08d y:%08d z:%08d a:%08d b:%08d\r\n",
             cur_motor_info.x, cur_motor_info.y, cur_motor_info.z, cur_motor_info.a, cur_motor_info.b);

  if (dorun > 0)
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
  if (dorun == 0)
  {
    rt_kprintf("ecat not runnig\r\n");
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

  if (dorun == 0)
  {
    rt_kprintf("ecat not running\r\n");
    return -1;
  }

  if (motor_state != ECAT_MOTOR_OPERATION_ENABLED)
  {
    rt_kprintf("ecat not in operation enabled state\r\n");
    return -1;
  }

  if (abs(pos->x) > ECAT_MOTOR_STEP_MAX ||
      abs(pos->y) > ECAT_MOTOR_STEP_MAX ||
      abs(pos->z) > ECAT_MOTOR_STEP_MAX ||
      abs(pos->a) > ECAT_MOTOR_STEP_MAX ||
      abs(pos->b) > ECAT_MOTOR_STEP_MAX)
  {
    rt_kprintf("ecat_motor_ctrl: position out of range ECAT_MOTOR_STEP_MAX:%d\r\n", ECAT_MOTOR_STEP_MAX);
    rt_kprintf("ecat_motor_ctrl: x:%d y:%d z:%d a:%d b:%d\r\n",
               pos->x, pos->y, pos->z,
               pos->a, pos->b);
    return -1;
  }

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

  if (dorun == 0)
  {
    rt_kprintf("ecat not running\r\n");
    return -1;
  }

  if (motor_state != ECAT_MOTOR_OPERATION_ENABLED)
  {
    rt_kprintf("ecat not in operation enabled state\r\n");
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
  rt_thread_t th = rt_thread_find("myth");
  if (th != RT_NULL)
  {
    rt_kprintf("motor_init: thread already exists\r\n");
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

int motor_exit_op(void)
{
  motor_is_stop = 0;
  int time_out = 0;
  ecat_stop();
  // 等待所有电机停止
  while (motor_is_stop == 0)
  {
    rt_thread_mdelay(10);
    time_out++;
    if (time_out > 3000)
    {
      rt_kprintf("ecat_exit_op: timeout waiting for motors to stop\r\n");
      return -1;
    }
  }

  // 进入safe state
  return ec_try_set_state_safe_op();
}
MSH_CMD_EXPORT(motor_exit_op, "motor_exit_op");

int motor_enter_op(void)
{
  // 1. 进入OP状态
  if (ec_try_set_state_op() < 0)
  {
    rt_kprintf("ecat_enter_op: Failed to enter OP state\r\n");
    return -1;
  }
  return ecat_continue();
}
MSH_CMD_EXPORT(motor_enter_op, "motor_enter_op");

int motor_ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex,
                      boolean CA, int psize, void *p, int Timeout)
{
  // TODO: 需要判断当前状态，PDO模式下不能读取，会打断周期同步
  return ec_SDOwrite(Slave, Index, SubIndex, CA, psize, p, Timeout);
}

int motor_ec_SDOread(uint16 Slave, uint16 Index, uint8 SubIndex,
                     boolean CA, int *psize, void *p, int Timeout)
{
  // TODO: 需要判断当前状态，PDO模式下不能读取，会打断周期同步
  return ec_SDOread(Slave, Index, SubIndex, CA, psize, p, Timeout);
}

int motor_SDO_rw_test(void)
{
  // TODO: 需要判断当前状态，PDO模式下不能读取，会打断周期同步
  uint32_t cur_value = 0;
  int wkc = 0;

  // 1. 读取原始值
  wkc = motor_ec_SDOread(1, 0x6065, 0, FALSE, sizeof(cur_value), &cur_value, EC_TIMEOUTRXM);  // 用户位置偏差过大阈值
  if (wkc <= 0)
  {
    rt_kprintf("motor_SDO_rw_test read16 failed wkc:%d\r\n", wkc);
    return -1;
  }
  rt_kprintf("0x6065[0]:0x%x wkc:%d\r\n", cur_value, wkc);

  // 2. 写入测试值
  uint32_t test_val = 0x1234;
  wkc = motor_ec_SDOwrite(1, 0x6065, 0, FALSE, sizeof(test_val), &test_val, EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    rt_kprintf("motor_SDO_rw_test write16 failed wkc:%d\r\n", wkc);
    return -1;
  }

  // 3. 读取验证
  uint32_t tmp_val = 0;
  wkc = motor_ec_SDOread(1, 0x6065, 0, FALSE, sizeof(tmp_val), &tmp_val, EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    rt_kprintf("motor_ec_SDOwrite failed wkc:%d\r\n", wkc);
    return -1;
  }
  rt_kprintf("0x6065[0]:0x%x wkc:%d\r\n", tmp_val, wkc);

  if (tmp_val != test_val)
  {
    rt_kprintf("check failed expected:0x%x, got:0x%x\r\n", test_val, tmp_val);
    return -1;
  }
  rt_kprintf("check succ tmp_val:0x%x test_val:0x%x\r\n", tmp_val, test_val);

  // 4. 写回原始值
  wkc = motor_ec_SDOwrite(1, 0x6065, 0, FALSE, sizeof(cur_value), &cur_value, EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    rt_kprintf("motor_ec_SDOwrite failed wkc:%d\r\n", wkc);
    return -1;
  }
  rt_kprintf("motor_SDO_rw_test write back 0x6065[0]:0x%x wkc:%d\r\n", cur_value, wkc);

  return 0;
}
MSH_CMD_EXPORT(motor_SDO_rw_test, "motor_SDO_rw_test");
