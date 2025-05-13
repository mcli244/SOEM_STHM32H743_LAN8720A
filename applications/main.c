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
