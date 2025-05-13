/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-08     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"


#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define LED_PIN  GET_PIN(A, 9)


#include "osal.h"
#include "ecatuser.h"
uint64 app_time_base = 0;
uint64 ref_time_base = 0;
uint64 sync_start_time = 0;
int64 app_time_offset = 0;

#include "tim.h"
extern void rt_hw_stm32_tim(void);
int main(void)
{
    int count = 1;
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);      // 设置为输出模式
    while (count++)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);            // 输出高电平
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);             // 输出低电平
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}

void ecat_test_main(void *parameter)
{
    rt_kprintf("SOEM (Simple Open EtherCAT Master)\nSlaveinfo\n");
    rt_hw_stm32_tim();
    HAL_TIM_Base_Start_IT(&htim4);
    rt_thread_mdelay(100);
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