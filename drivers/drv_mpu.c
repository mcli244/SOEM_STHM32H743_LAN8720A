/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-18     lixiao       the first version
 */
#include <rtthread.h>
#include "stm32h7xx.h"
#include "board.h"

int mpu_init(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* Disable the MPU */
    HAL_MPU_Disable();

    /* Configure the MPU attributes as WT for AXI SRAM */
    MPU_InitStruct.Enable            = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress       = 0x24000000;
    MPU_InitStruct.Size              = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.AccessPermission  = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable      = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable       = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable       = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number            = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField      = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable  = 0X00;
    MPU_InitStruct.DisableExec       = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

#ifdef BSP_USING_SDRAM
    /* Configure the MPU attributes as WT for SDRAM */
    MPU_InitStruct.Enable            = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress       = 0xC0000000;
    MPU_InitStruct.Size              = MPU_REGION_SIZE_32MB;
    MPU_InitStruct.AccessPermission  = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable      = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable       = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable       = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number            = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField      = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable  = 0x00;
    MPU_InitStruct.DisableExec       = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
#endif

#ifdef BSP_USING_ETH
    #if 0
    /* Configure the MPU attributes as Device not cacheable
       for ETH DMA descriptors and RX Buffers*/
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = 0x30040000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_1KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = 0x30040400;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_8KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    #endif
    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = 0x30040000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_256B;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
    MPU_InitStruct.BaseAddress      = 0x30044000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_16KB;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);


    // 配置FMC区域
//    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
//    MPU_InitStruct.BaseAddress = 0x60000000;  // 这是FMC NE1的基地址，确保与实际配置匹配
//    MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;  // 根据实际的FMC区域大小设置
//    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
//    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;  // 不缓冲
//    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;    // 不缓存
//    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;        // 可共享
//    MPU_InitStruct.Number = MPU_REGION_NUMBER3;  // 使用的MPU区域编号
//    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;  // 设置为Strongly Ordered
//    MPU_InitStruct.SubRegionDisable = 0x00;
//    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
//    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* 配置 FMC 扩展 IO 的 MPU 属性为 Device 或者 Strongly Ordered */
    // MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    // MPU_InitStruct.BaseAddress      = 0x60000000;
    // MPU_InitStruct.Size             = ARM_MPU_REGION_SIZE_64KB;
    // MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    // MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    // MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    // MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    // MPU_InitStruct.Number           = MPU_REGION_NUMBER3;
    // MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    // MPU_InitStruct.SubRegionDisable = 0x00;
    // MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;


    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = 0x60000000;  // 根据你挂载的地址修改
    MPU_InitStruct.Size             = ARM_MPU_REGION_SIZE_64KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;   // ✅ 关键：必须 NOT_BUFFERABLE
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;    // ✅ 关键：必须 NOT_CACHEABLE
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;        // 推荐设置为 Shareable
    MPU_InitStruct.Number           = MPU_REGION_NUMBER3;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;              // Device 类型
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE; // 保险起见，禁止执行

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
#endif

//    /* Configure the MPU attributes as WT for QSPI */
//    MPU_InitStruct.Enable            = MPU_REGION_ENABLE;
//    MPU_InitStruct.BaseAddress       = 0x90000000;
//    MPU_InitStruct.Size              = MPU_REGION_SIZE_8MB;
//    MPU_InitStruct.AccessPermission  = MPU_REGION_FULL_ACCESS;
//    MPU_InitStruct.IsBufferable      = MPU_ACCESS_NOT_BUFFERABLE;
//    MPU_InitStruct.IsCacheable       = MPU_ACCESS_CACHEABLE;
//    MPU_InitStruct.IsShareable       = MPU_ACCESS_NOT_SHAREABLE;
//    MPU_InitStruct.Number            = MPU_REGION_NUMBER3;
//    MPU_InitStruct.TypeExtField      = MPU_TEX_LEVEL0;
//    MPU_InitStruct.SubRegionDisable  = 0X00;
//    MPU_InitStruct.DisableExec       = MPU_INSTRUCTION_ACCESS_ENABLE;
//
//    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Enable the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

    /* Enable CACHE */
    SCB_EnableICache();
    SCB_EnableDCache();

    return RT_EOK;

}
INIT_BOARD_EXPORT(mpu_init);


