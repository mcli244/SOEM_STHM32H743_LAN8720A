#include "ecatuser.h"

#include "stdio.h"
#include "string.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"

#include "stm32h7xx_hal.h"
// #include "gpio.h"

#include "osal.h"
#include "ecatuser.h"
#include "tim.h"
#include "motor.h"

#define ECAT_MOTOR_STEP_MAX (81920)

uint64 app_time_base = 0;
uint64 ref_time_base = 0;
uint64 sync_start_time = 0;
int64 app_time_offset = 0;
uint8 wkc;
int32_t start_pos = 0;
int32_t step_increment = 10;
int32_t stop_flag = 0;

// 注意：为了和SOEM内部一致，这里索引号为0的实际上没有被使用
#define MOTOR_CNT 6
PDO_Output *outputs[MOTOR_CNT];
PDO_Input *inputs[MOTOR_CNT];

char IOmap[200];
uint32_t dorun = 0;
int oloop, iloop;

// motor control
uint8_t startup_step = 0;
static motor_pos_t cur_motor_info = {0, 0, 0, 0, 0};
static uint8_t cur_motor_info_update = 0;

static void _dm9000_delay_ms(uint16_t nms)
{
  /**  tips:
   * 这里主频480MHz rt_hw_us_delay() 大于1000就会卡死
   * 这里把它限制在500us，大于的部分就拆分
   * **/
  rt_uint32_t us = nms * 1000;
  while (us > 500)
  {
    rt_hw_us_delay(500);
    us -= 500;
  }
  rt_hw_us_delay(us);
}

static int drive_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
  int wkc = 0;
  wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    rt_kprintf("write8 failed slave:%d index:0x%x subindex:%d value:0x%x wkc:%d\r\n", slave, index, subindex, value, wkc);
  }
  return wkc;
}

static int drive_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
  int wkc = 0;
  wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    rt_kprintf("write16 failed slave:%d index:0x%x subindex:%d value:0x%x wkc:%d\r\n", slave, index, subindex, value, wkc);
  }
  return wkc;
}

static int drive_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value)
{
  int wkc = 0;
  wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    rt_kprintf("write32 failed slave:%d index:0x%x subindex:%d value:0x%x wkc:%d\r\n", slave, index, subindex, value, wkc);
  }
  return wkc;
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
    _dm9000_delay_ms(100);
  _start:
    if (ec_config_init(TRUE) > 0)
    {
      rt_kprintf("%d slaves found and configured.\r\n", ec_slavecount);

      if (ec_slavecount >= 1)
      {
        for (slc = 1; slc <= ec_slavecount; slc++)
        {

          for (slc = 1; slc <= ec_slavecount; slc++)
          {
            rt_kprintf("Found %s at position %d\n", ec_slave[slc].name, slc);
            ec_slave[slc].PO2SOconfig = &Servosetup;
          }
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

      rt_kprintf("Slave 0 State=0x%04x\r\n", ec_slave[0].state);
      rt_kprintf("Slave 1 State=0x%04x\r\n", ec_slave[1].state);

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

void ecat_loop(void)
{
  static int statup_delay_cnt = 0;
  if (dorun > 0)
  {
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    int all_ready = 1;

    switch (startup_step)
    {
    case 1: // 初始化 -> Ready to Switch On
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
        startup_step = 2;
      }

      break;

    case 2: // Ready to Switch On -> Switched On
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
        startup_step = 3;
      break;

    case 3: // Switched On -> Operation Enabled
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
        startup_step = 4;
      break;

    case 4: // 正常控制阶段（位置模式）
      // for (int i = 1; i <= ec_slavecount; i++)
      // {
      //   rt_base_t level = rt_hw_interrupt_disable();
      //   outputs[i]->TargetPos += step_increment;
      //   rt_hw_interrupt_enable(level);
      // }
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

      if (stop_flag)
      {
        startup_step = 5;
      }
      break;
    case 5: // 关闭电机
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x0F;                     // 保持 Operation Enabled
        outputs[i]->TargetPos = inputs[i]->CurrentPosition; // 停止在当前位置
      }
      startup_step = 6;
    case 6:
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x07; // Shutdown
      }
      startup_step = 7;
      break;
    case 7:
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x06; // Shutdown
      }
      startup_step = 8;
      break;
    case 8:
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x00;
      }
      startup_step = 9;
      rt_kprintf("ecat_stop acti\r\n");
      break;
    case 9:
      if (stop_flag == 0)
      {
        startup_step = 0;
      }
      break;

    default:
      for (int i = 1; i <= ec_slavecount; i++)
      {
        outputs[i]->ControlWord = 0x80; // 带立即更新位
      }
      startup_step = 1; // 重置状态机

      break;
    }

    ec_send_processdata();
  }
}

void ecat_test_main(void *parameter)
{
  rt_hw_stm32_tim();
  HAL_TIM_Base_Start_IT(&htim4);
  _dm9000_delay_ms(100);
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
  rt_kprintf("startup_step:%d\r\n", startup_step);
  rt_kprintf("stop_flag:%d\r\n", stop_flag);
  rt_kprintf("ec_slavecount:%d\r\n", ec_slavecount);
  rt_kprintf("cur_motor_info x:%08d y:%08d z:%08d a:%08d b:%08d\r\n", 
    cur_motor_info.x, cur_motor_info.y, cur_motor_info.z, cur_motor_info.a, cur_motor_info.b);

  if (dorun > 0)
  {
    for(int i=1; i<=ec_slavecount; i++)
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

  if (startup_step != 4)
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

  if (startup_step != 4)
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

  if(argc != 6)
  {
    rt_kprintf("Usage: motor_set_test x y z a b\r\n");
    rt_kprintf("rg: motor_set_test 100 200 300 400 500\r\n");
    return;
  }

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
