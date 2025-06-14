#include "lan8742.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_eth.h"
#include<rtthread.h>
#include<rtdevice.h>
#include "drv_common.h"
// #include "main.h"
extern ETH_HandleTypeDef heth;

void rt_hw_ms_delay(int nms)
{
    int us = nms * 1000;
    while (us > 500)
    {
        rt_hw_us_delay(500);
        us -= 500;
    }
    rt_hw_us_delay(us);
} 


uint32_t lan8742addr;

int LAN8742_Init(void)
{
    uint32_t regvalue = 0, addr = 0, cnt = 0;
    HAL_StatusTypeDef rtn;

    HAL_ETH_SetMDIOClockRange(&heth);
    
    lan8742addr = 32;
    for(addr = 0; addr <= lan8742addr; addr ++)
    {
        rtn = HAL_ETH_ReadPHYRegister(&heth, addr, LAN8742_SMR, &regvalue);
        if( rtn != HAL_OK)
        {
            rt_kprintf("Read Address %d error \r\n",addr);
            continue;
        }
        
        if((regvalue & LAN8742_SMR_PHY_ADDR) == addr)
        {
            lan8742addr = addr;
            break;
        }
    }
    
    // rt_kprintf("lan8742addr = %d\r\n",addr);
    if( lan8742addr > 31 )
        return -1;
    
    //软件复位一下
    // rt_kprintf("start soft reset\r\n");
    rtn = HAL_ETH_WritePHYRegister(&heth,lan8742addr,LAN8742_BCR,LAN8742_BCR_SOFT_RESET);
    if( rtn != HAL_OK )
    {
        rt_kprintf("HAL_ETH_WritePHYRegister LAN8742_BCR LAN8742_BCR_SOFT_RESET Error \r\n");
        return -1;
    }

    //查询是否复位完成
    // rt_kprintf("check soft reset is finish or not\r\n");
    do
    {
        rtn = HAL_ETH_ReadPHYRegister(&heth, lan8742addr, LAN8742_BCR, &regvalue);
        if( rtn != HAL_OK )
        {
            rt_kprintf("HAL_ETH_ReadPHYRegister LAN8742_BCR Error \r\n");
            return -1;
        }
        rt_hw_ms_delay(10);
        cnt ++;
        if( cnt > 1000 )
        {
            rt_kprintf("LAN8742 BCR soft reset timeout\r\n");
            return -1;
        }
    }while( regvalue & LAN8742_BCR_SOFT_RESET );
    // rt_kprintf("soft reset is finish\r\n");
    
    //HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
    //HAL_NVIC_EnableIRQ(ETH_IRQn);
    
    return 0;
}

int LAN8742_GetLinkState(void)
{
    uint32_t readval = 0;
    int32_t rtn;
    
    //读状态寄存器
    rtn = HAL_ETH_ReadPHYRegister(&heth, lan8742addr, LAN8742_BSR, &readval);
    if( rtn != HAL_OK )
    {
        rt_kprintf("HAL_ETH_ReadPHYRegister LAN8742_BSR Error \r\n");
        return LAN8742_STATUS_READ_ERROR;
    }
    
    //再读一遍状态寄存器
    rtn = HAL_ETH_ReadPHYRegister(&heth, lan8742addr, LAN8742_BSR, &readval);
    if( rtn != HAL_OK )
    {
        rt_kprintf("HAL_ETH_ReadPHYRegister LAN8742_BSR Error \r\n");
        return LAN8742_STATUS_READ_ERROR;
    }
    
    if((readval & LAN8742_BSR_LINK_STATUS) == 0)
    {
        return LAN8742_STATUS_LINK_DOWN;    
    }
    
    /* Check Auto negotiaition */
    
    rtn = HAL_ETH_ReadPHYRegister(&heth, lan8742addr, LAN8742_BCR, &readval);
    if( rtn != HAL_OK )
    {
        return LAN8742_STATUS_READ_ERROR;
    }
    
    if((readval & LAN8742_BCR_AUTONEGO_EN) != LAN8742_BCR_AUTONEGO_EN)
    {
        if(((readval & LAN8742_BCR_SPEED_SELECT) == LAN8742_BCR_SPEED_SELECT) && ((readval & LAN8742_BCR_DUPLEX_MODE) == LAN8742_BCR_DUPLEX_MODE)) 
        {
            return LAN8742_STATUS_100MBITS_FULLDUPLEX;
        }
        else if ((readval & LAN8742_BCR_SPEED_SELECT) == LAN8742_BCR_SPEED_SELECT)
        {
            return LAN8742_STATUS_100MBITS_HALFDUPLEX;
        }        
        else if ((readval & LAN8742_BCR_DUPLEX_MODE) == LAN8742_BCR_DUPLEX_MODE)
        {
            return LAN8742_STATUS_10MBITS_FULLDUPLEX;
        }
        else
        {
            return LAN8742_STATUS_10MBITS_HALFDUPLEX;
        }  		
    }
    else /* Auto Nego enabled */
    {
        rtn = HAL_ETH_ReadPHYRegister(&heth, lan8742addr, LAN8742_PHYSCSR, &readval);
        if( rtn != HAL_OK )
        {
            return LAN8742_STATUS_READ_ERROR;
        }
        
        /* Check if auto nego not done */
        if((readval & LAN8742_PHYSCSR_AUTONEGO_DONE) == 0)
        {
            return LAN8742_STATUS_AUTONEGO_NOTDONE;
        }
        
        if((readval & LAN8742_PHYSCSR_HCDSPEEDMASK) == LAN8742_PHYSCSR_100BTX_FD)
        {
            return LAN8742_STATUS_100MBITS_FULLDUPLEX;
        }
        else if ((readval & LAN8742_PHYSCSR_HCDSPEEDMASK) == LAN8742_PHYSCSR_100BTX_HD)
        {
            return LAN8742_STATUS_100MBITS_HALFDUPLEX;
        }
        else if ((readval & LAN8742_PHYSCSR_HCDSPEEDMASK) == LAN8742_PHYSCSR_10BT_FD)
        {
            return LAN8742_STATUS_10MBITS_FULLDUPLEX;
        }
        else
        {
            return LAN8742_STATUS_10MBITS_HALFDUPLEX;
        }				
    }
}


