/*
 * @Author       : lixiangjun@up3d.com
 * @Date         : 2025-06-04
 * @FilePath     : ecatutils.c
 * @Description  : Description
 */
#include "ecatutils.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"
#include "ethercat.h"


int drive_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
  int wkc = 0;
  wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    rt_kprintf("write8 failed slave:%d index:0x%x subindex:%d value:0x%x wkc:%d\r\n", slave, index, subindex, value, wkc);
  }
  return wkc;
}

int drive_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
  int wkc = 0;
  wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    rt_kprintf("write16 failed slave:%d index:0x%x subindex:%d value:0x%x wkc:%d\r\n", slave, index, subindex, value, wkc);
  }
  return wkc;
}

int drive_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value)
{
  int wkc = 0;
  wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    rt_kprintf("write32 failed slave:%d index:0x%x subindex:%d value:0x%x wkc:%d\r\n", slave, index, subindex, value, wkc);
  }
  return wkc;
}

void ecat_delay_ms(uint16_t nms)
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

