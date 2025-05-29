
#include "drv_dm9000.h"
#include <rtdevice.h>
#include <rtthread.h>
#include <string.h>
#include "drv_common.h"

#include "stdio.h"
#include "stm32h7xx_hal_sram.h"
#include <netif/ethernetif.h>
#include <lwip/inet.h>
#include <lwip/netif.h>
#include <lwip/pbuf.h>
#include <lwip/opt.h>
#include <netdev.h>

#define LOG_TAG "drv.dm9000"
#include <drv_log.h>

// #define DM9000_DEBUG

#ifdef DM9000_DEBUG
#define DM9000_DUG rt_kprintf
#else
#define DM9000_DUG(...) \
    do                  \
    {                   \
    } while (0)
#endif

/***************** netdev **********************/
#define DM900NET_NAME "dm9000"
struct dm9000_net_eth
{
    struct eth_device parent;
    enum DM9000_PHY_mode mode; // 工作模式
    u8 imr_all;                // 中断类型
    u16 queue_packet_len;      // 每个数据包大小
    u8 mac_addr[6];            // MAC地址
    u8 multicase_addr[8];      // 组播地址
    rt_timer_t poll_link_timer;
};
static struct dm9000_net_eth dm9000_net_dev =
    {
        .mac_addr = {0x00, 0x80, 0xE1, 0x00, 0x00, 0x00},
        .multicase_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        .mode = DM9000_100MFD,
};

int dm_irq_cnt = 0;
/***************** netdev end**********************/

#ifndef DM9000_USE_LWIP
static uint8_t rxbuf[1540];
#endif

#define PIN_NRESET GET_PIN(A, 10) // 复位引脚
#define PIN_IRQ GET_PIN(A, 15)    // 例如：PA0 作为中断引脚

void irq_callback(void *args);
static void _dm9000_delay_ms(u16 nms)
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

static uint32_t FMC_Initialized = 0;

static void DM9000_GPIO_Init(void)
{
    /* USER CODE BEGIN FMC_MspInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /* USER CODE END FMC_MspInit 0 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (FMC_Initialized)
    {
        return;
    }
    FMC_Initialized = 1;

    /* Peripheral clock enable */
    __HAL_RCC_FMC_CLK_ENABLE();

    /** FMC GPIO Configuration
    PF0   ------> FMC_A0
    PE7   ------> FMC_D4
    PE8   ------> FMC_D5
    PE9   ------> FMC_D6
    PE10   ------> FMC_D7
    PE11   ------> FMC_D8
    PE12   ------> FMC_D9
    PE13   ------> FMC_D10
    PE14   ------> FMC_D11
    PE15   ------> FMC_D12
    PD8   ------> FMC_D13
    PD9   ------> FMC_D14
    PD10   ------> FMC_D15
    PD14   ------> FMC_D0
    PD15   ------> FMC_D1
    PC7   ------> FMC_NE1
    PD0   ------> FMC_D2
    PD1   ------> FMC_D3
    PD4   ------> FMC_NOE
    PD5   ------> FMC_NWE
    */
    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FMC;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN FMC_MspInit 1 */

    /* USER CODE END FMC_MspInit 1 */
}

SRAM_HandleTypeDef hsram1;

/* FMC initialization function */
void DM9000_FMC_Config(void)
{
    /* USER CODE BEGIN FMC_Init 0 */

    /* USER CODE END FMC_Init 0 */

    FMC_NORSRAM_TimingTypeDef Timing = {0};

    /* USER CODE BEGIN FMC_Init 1 */
    DM9000_GPIO_Init();
    /* USER CODE END FMC_Init 1 */

    hsram1.Instance = FMC_NORSRAM_DEVICE;
    hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
    /* hsram1.Init */
    hsram1.Init.NSBank              = FMC_NORSRAM_BANK1;
    hsram1.Init.DataAddressMux      = FMC_DATA_ADDRESS_MUX_DISABLE;
    hsram1.Init.MemoryType          = FMC_MEMORY_TYPE_SRAM;
    hsram1.Init.MemoryDataWidth     = FMC_NORSRAM_MEM_BUS_WIDTH_16; 
    hsram1.Init.BurstAccessMode     = FMC_BURST_ACCESS_MODE_DISABLE;
    hsram1.Init.WaitSignalPolarity  = FMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram1.Init.WaitSignalActive    = FMC_WAIT_TIMING_BEFORE_WS;
    hsram1.Init.WriteOperation      = FMC_WRITE_OPERATION_ENABLE;
    hsram1.Init.WaitSignal          = FMC_WAIT_SIGNAL_DISABLE;
    hsram1.Init.ExtendedMode        = FMC_EXTENDED_MODE_DISABLE;
    hsram1.Init.AsynchronousWait    = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram1.Init.WriteBurst          = FMC_WRITE_BURST_DISABLE;
    hsram1.Init.ContinuousClock     = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
    hsram1.Init.WriteFifo           = FMC_WRITE_FIFO_DISABLE;
    hsram1.Init.PageSize            = FMC_PAGE_SIZE_NONE;


    // 使用的HCLK 120M = 8.3ns
    /* Timing */
    Timing.AddressSetupTime         = 10;       // DM9000手册建议地址建立时间为大于80ns F0寄存器
    Timing.AddressHoldTime          = 0;        // 模式A没用上
    Timing.DataSetupTime            = 2;        // DM9000手册建议数据建立时间为大于10ns  
    Timing.BusTurnAroundDuration    = 2;        // 片选信号，高脉宽
    Timing.CLKDivision              = 0;        // 模式A没用上
    Timing.DataLatency              = 0;        // 模式A没用上
    Timing.AccessMode               = FMC_ACCESS_MODE_A;
    if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN FMC_Init 2 */
    //设置引脚为输入模式（下降沿触发）
    rt_pin_mode(PIN_IRQ, PIN_MODE_INPUT);
    rt_pin_attach_irq(PIN_IRQ, PIN_IRQ_MODE_FALLING, irq_callback, RT_NULL);
    rt_pin_irq_enable(PIN_IRQ, PIN_IRQ_ENABLE);
    /* USER CODE END FMC_Init 2 */
}

// 读取DM9000指定寄存器的值
// reg:寄存器地址
// 返回值：DM9000指定寄存器的值
uint16_t DM9000_ReadReg(uint16_t reg)
{
    DM9000->REG = reg; //__DSB();
    return DM9000->DATA;
}

// 向DM9000指定寄存器中写入指定值
// reg:要写入的寄存器
// data:要写入的值
void DM9000_WriteReg(uint16_t reg, uint16_t data)
{
    DM9000->REG = reg; //__DSB();
    DM9000->DATA = data; //__DSB();
}

// 读取DM9000的PHY的指定寄存器
// reg:要读的PHY寄存器
// 返回值:读取到的PHY寄存器值
u16 DM9000_PHY_ReadReg(u16 reg)
{
    u16 temp;
    DM9000_WriteReg(DM9000_EPAR, DM9000_PHY | reg);
    DM9000_WriteReg(DM9000_EPCR, 0X0C); // 选中PHY，发送读命令
    _dm9000_delay_ms(10);
    DM9000_WriteReg(DM9000_EPCR, 0X00); // 清除读命令
    temp = (DM9000_ReadReg(DM9000_EPDRH) << 8) | (DM9000_ReadReg(DM9000_EPDRL));
    return temp;
}

// 向DM9000的PHY寄存器写入指定值
// reg:PHY寄存器
// data:要写入的值
void DM9000_PHY_WriteReg(u16 reg, u16 data)
{
    DM9000_WriteReg(DM9000_EPAR, DM9000_PHY | reg);
    DM9000_WriteReg(DM9000_EPDRL, (data & 0xff));        // 写入低字节
    DM9000_WriteReg(DM9000_EPDRH, ((data >> 8) & 0xff)); // 写入高字节
    DM9000_WriteReg(DM9000_EPCR, 0X0A);                  // 选中PHY,发送写命令
    _dm9000_delay_ms(50);
    DM9000_WriteReg(DM9000_EPCR, 0X00); // 清除写命令
}

// 获取DM9000的芯片ID
// 返回值：DM9000的芯片ID值
u32 DM9000_Get_DeiviceID(void)
{
    u32 value;
    value = DM9000_ReadReg(DM9000_VIDL);
    value |= DM9000_ReadReg(DM9000_VIDH) << 8;
    value |= DM9000_ReadReg(DM9000_PIDL) << 16;
    value |= DM9000_ReadReg(DM9000_PIDH) << 24;
    return value;
}

int dm9000_check_link_status(void);

// 获取DM9000的连接速度和双工模式
// 返回值：  0,100M半双工
//           1,100M全双工
//           2,10M半双工
//           3,10M全双工
//           0XFF,连接失败！
u8 DM9000_Get_SpeedAndDuplex(void)
{
    u8 temp;
    u8 i = 0;
    if (dm9000_net_dev.mode == DM9000_AUTO) // 如果开启了自动协商模式一定要等待协商完成
    {
        while (!(DM9000_PHY_ReadReg(0X01) & 0X0020)) // 等待自动协商完成
        {
            _dm9000_delay_ms(10);
            i++;
            if (i > 100)
                return 0XFF; // 自动协商失败
        }
    }
    else // 自定义模式,一定要等待连接成功
    {
        while (!(DM9000_ReadReg(DM9000_NSR) & 0X40)) // 等待连接成功
        {
            _dm9000_delay_ms(10);
            i++;
            if (i > 100)
                return 0XFF; // 连接失败
        }
    }
    temp = ((DM9000_ReadReg(DM9000_NSR) >> 6) & 0X02);  // 获取DM9000的连接速度
    temp |= ((DM9000_ReadReg(DM9000_NCR) >> 3) & 0X01); // 获取DM9000的双工状态
    return temp;
}

// 设置DM900的PHY工作模式
// mode:PHY模式
void DM9000_Set_PHYMode(u8 mode)
{
    u16 BMCR_Value, ANAR_Value;
    switch (mode)
    {
    case DM9000_10MHD: // 10M半双工
        BMCR_Value = 0X0000;
        ANAR_Value = 0X21;
        break;
    case DM9000_10MFD: // 10M全双工
        BMCR_Value = 0X0100;
        ANAR_Value = 0X41;
        break;
    case DM9000_100MHD: // 100M半双工
        BMCR_Value = 0X2000;
        ANAR_Value = 0X81;
        break;
    case DM9000_100MFD: // 100M全双工
        BMCR_Value = 0X2100;
        ANAR_Value = 0X101;
        break;
    case DM9000_AUTO: // 自动协商模式
        BMCR_Value = 0X1000;
        ANAR_Value = 0X01E1;
        break;
    }
    DM9000_PHY_WriteReg(DM9000_PHY_BMCR, BMCR_Value);
    DM9000_PHY_WriteReg(DM9000_PHY_ANAR, ANAR_Value);
    DM9000_WriteReg(DM9000_GPR, 0X00); // 使能PHY
}

// 设置DM9000的MAC地址
// macaddr:指向MAC地址
void DM9000_Set_MACAddress(u8 *macaddr)
{
    u8 i;
    for (i = 0; i < 6; i++)
    {
        DM9000_WriteReg(DM9000_PAR + i, macaddr[i]);
    }
}
// 设置DM9000的组播地址
// multicastaddr:指向多播地址
void DM9000_Set_Multicast(u8 *multicastaddr)
{
    u8 i;
    for (i = 0; i < 8; i++)
    {
        DM9000_WriteReg(DM9000_MAR + i, multicastaddr[i]);
    }
}

// 复位DM9000
void DM9000_Reset(void)
{
    // 复位DM9000,复位步骤参考<DM9000 Application Notes V1.22>手册29页
    int cnt = 0;
    rt_pin_mode(PIN_NRESET, PIN_MODE_OUTPUT);
    rt_pin_write(PIN_NRESET, PIN_LOW);
    _dm9000_delay_ms(10);
    rt_pin_write(PIN_NRESET, PIN_HIGH);
    _dm9000_delay_ms(100);

    DM9000_WriteReg(DM9000_GPCR, 0x01);            // 第一步:设置GPCR寄存器(0X1E)的bit0为1
    DM9000_WriteReg(DM9000_GPR, DM9000_PHY_ON);    // 第二步:设置GPR寄存器(0X1F)的bit1为0，DM9000内部的PHY上电
    DM9000_WriteReg(DM9000_NCR, (0x02 | NCR_RST)); // 第三步:软件复位DM9000
    do
    {
        _dm9000_delay_ms(25);
        cnt++;
        if (cnt >= 3)
        {
            rt_kprintf("DM9000_ReadReg:0x%x State failed\n", DM9000_ReadReg(DM9000_NCR));
            return;
        }
    } while (DM9000_ReadReg(DM9000_NCR) & 1); // 等待DM9000软复位完成

    DM9000_WriteReg(DM9000_NCR, 0);
    DM9000_WriteReg(DM9000_NCR, (0x02 | NCR_RST)); // DM9000第二次软复位
    do
    {
        _dm9000_delay_ms(25);
        cnt++;
        if (cnt >= 3)
        {
            rt_kprintf("DM9000_ReadReg:0x%x State failed\n", DM9000_ReadReg(DM9000_NCR));
            return;
        }
    } while (DM9000_ReadReg(DM9000_NCR) & 1);
}

int DM9000_Init(void)
{
    u32 temp;
    DM9000_FMC_Config();
    DM9000_Reset();
    _dm9000_delay_ms(100);

    temp = DM9000_Get_DeiviceID(); // 获取DM9000ID
    // rt_kprintf("DM9000 ID:%#x\r\n",temp);
    if (temp != DM9000_ID)
    {
        DM9000_DUG("DM9000_Get_DeiviceID faild\r\n");
        return 1; // 读取ID错误
    }
    DM9000_DUG("DM9000 ID:0x%x\r\n", temp);
    //    temp=*(vu32*)(0x1FFFF7E8);              //获取STM32的唯一ID的前24位作为MAC地址后三字节
    dm9000_net_dev.mode = DM9000_AUTO;
    dm9000_net_dev.queue_packet_len = 0;
    // DM9000的SRAM的发送和接收指针自动返回到开始地址，并且开启接收中断
    dm9000_net_dev.imr_all = IMR_PAR | IMR_PRI;
    DM9000_Set_PHYMode(dm9000_net_dev.mode); // 设置PHY工作模式
    DM9000_WriteReg(DM9000_NCR, 0X00);
    DM9000_WriteReg(DM9000_TCR, 0X00); // 发送控制寄存器清零
    DM9000_WriteReg(DM9000_BPTR, 0X3F);
    DM9000_WriteReg(DM9000_FCTR, 0X38);
    DM9000_WriteReg(DM9000_FCR, 0X00);
    DM9000_WriteReg(DM9000_SMCR, 0X00);                                // 特殊模式
    DM9000_WriteReg(DM9000_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END); // 清除发送状态
    DM9000_WriteReg(DM9000_ISR, 0X0F);                                 // 清除中断状态
    DM9000_WriteReg(DM9000_TCR2, 0X80);                                // 切换LED到mode1

    // 设置MAC地址和组播地址
    DM9000_Set_MACAddress(dm9000_net_dev.mac_addr);      // 设置MAC地址
    DM9000_Set_Multicast(dm9000_net_dev.multicase_addr); // 设置组播地址
    DM9000_WriteReg(DM9000_RCR,RCR_DIS_LONG|RCR_RXEN);
    // DM9000_WriteReg(DM9000_RCR, 0x3B);  // 开启 promiscuous 接收所有包
    // DM9000_WriteReg(DM9000_RCR, 0x1F); // 启用所有接收模式，包括广播、接收长度较小的数据包等

    DM9000_WriteReg(DM9000_IMR, IMR_PAR);
    // temp = DM9000_Get_SpeedAndDuplex(); // 获取DM9000的连接速度和双工状态
    // if (temp != 0XFF)                   // 连接成功，通过串口显示连接速度和双工状态
    // {
    //     DM9000_DUG("DM9000 Speed:%dMbps,Duplex:%s duplex mode\r\n", (temp & 0x02) ? 10 : 100, (temp & 0x01) ? "Full" : "Half");
    // }
    // else
    //     rt_kprintf("DM9000 Establish Link Failed!\r\n");
    DM9000_WriteReg(DM9000_IMR,dm9000_net_dev.imr_all);  //设置中断
    return 0;
}

void DM9000_SendPacket(struct pbuf *p)
{
    struct pbuf *q;
    u16 pbuf_index = 0;
    u8 word[2], word_index = 0;

    DM9000->REG = DM9000_MWCMD;
    q = p;

    while (q)
    {
        if (pbuf_index < q->len)
        {
            word[word_index++] = ((u8_t *)q->payload)[pbuf_index++];

            if (word_index == 2)
            {
                DM9000->DATA = ((u16)word[1] << 8) | word[0];
                word_index = 0;
            }
        }
        else
        {
            q = q->next;
            pbuf_index = 0;
        }
    }

    if (word_index == 1)
    {
        DM9000->DATA = word[0];
    }

    DM9000_WriteReg(DM9000_TXPLL, p->tot_len & 0xFF);
    DM9000_WriteReg(DM9000_TXPLH, (p->tot_len >> 8) & 0xFF);
    DM9000_WriteReg(DM9000_TCR, 0x01);
    while ((DM9000_ReadReg(DM9000_ISR) & 0x02) == 0)
        ;
    DM9000_WriteReg(DM9000_ISR, 0x02);
}

// DM9000接收数据包
// 接收到的数据包存放在DM9000的RX FIFO中，地址为0X0C00~0X3FFF
// 接收到的数据包的前四个字节并不是真实的数据，而是有特定含义的
// byte1:表明是否接收到数据，为0x00或者0X01，如果两个都不是的话一定要软件复位DM9000
//		0x01，接收到数据
//		0x00，未接收到数据
// byte2:第二个字节表示一些状态信息，和DM9000的RSR(0X06)寄存器一致的
// byte3:本帧数据长度的低字节
// byte4:本帧数据长度的高字节
// 返回值：pbuf格式的接收到的数据包
#if 0
struct pbuf *DM9000_Receive_Packet(void)
{
    struct pbuf *p;
    struct pbuf *q;
    u32 rxbyte;
    vu16 rx_status, rx_length;
    u16 *data;
    u16 dummy;
    int len, i;

    p = NULL;

    DM9000_DUG("DM9000_Receive_Packet\r\n");

    __error_retry:
        DM9000_ReadReg(DM9000_MRRH); // 读取这两个寄存器
        DM9000_ReadReg(DM9000_MRRL);
        DM9000_ReadReg(DM9000_MRCMDX); // 假读
        rxbyte = (u8)DM9000->DATA;     // 进行第二次读取
        //__DSB();
        if (rxbyte)                    // 接收到数据
        {
            if (rxbyte > 1) // rxbyte大于1，接收到的数据错误,挂了
            {
                rt_kprintf("dm9000 rx: rx error, stop device rxbyte:0x%x\r\n", rxbyte);
                DM9000_WriteReg(DM9000_RCR, 0x00);
                DM9000_WriteReg(DM9000_ISR, 0x80);
                DM9000_WriteReg(DM9000_NCR, NCR_RST);  // 软复位
                _dm9000_delay_ms(5);
                DM9000_WriteReg(DM9000_RCR, 0x39);     // 恢复接收
                return (struct pbuf *)p;
            }
            DM9000->REG = DM9000_MRCMD;//__DSB();
            rx_status = DM9000->DATA;//__DSB();
            rx_length = DM9000->DATA;//__DSB();
            DM9000_DUG("rx_status:0x%x rx_length: %d\r\n", rx_status, rx_length);
            // if(rx_length>512)rt_kprintf("rxlen:%d\r\n",rx_length);
            p = pbuf_alloc(PBUF_RAW, rx_length, PBUF_POOL); // pbufs内存池分配pbuf
            if (p != NULL)                                  // 内存申请成功
            {
                for (q = p; q != NULL; q = q->next)
                {
                    data = (u16 *)q->payload;
                    len = (q->len + 1) / 2;
                    for (i = 0; i < len; i++)
                    {
                        ((u16 *) data)[i] = DM9000->DATA;
                        //__DSB();
                    }
                        
                }
            }
            else // 内存申请失败
            {
                rt_kprintf("pbuf内存申请失败:%d\r\n", rx_length);
                data = &dummy;
                len = rx_length;
                while (len)
                {
                    *data = DM9000->DATA;
                    len -= 2;
                }
            }
            // 根据rx_status判断接收数据是否出现如下错误：FIFO溢出、CRC错误
            // 对齐错误、物理层错误，如果有任何一个出现的话丢弃该数据帧，
            // 当rx_length小于64或者大于最大数据长度的时候也丢弃该数据帧
            if ((rx_status & 0XBF00) || (rx_length < 0X40) || (rx_length > DM9000_PKT_MAX))
            {
                rt_kprintf("rx_status:%#x\r\n", rx_status);
                if (rx_status & 0x100)
                    rt_kprintf("rx fifo error\r\n");
                if (rx_status & 0x200)
                    rt_kprintf("rx crc error\r\n");
                if (rx_status & 0x8000)
                    rt_kprintf("rx length error\r\n");
                if (rx_length > DM9000_PKT_MAX)
                {
                    rt_kprintf("rx length too big\r\n");
                    DM9000_WriteReg(DM9000_NCR, NCR_RST); // 复位DM9000
                    _dm9000_delay_ms(5);
                }
                if (p != NULL)
                    pbuf_free((struct pbuf *)p); // 释放内存
                p = NULL;
                goto __error_retry;
            }
        }
        else
        {
            DM9000_WriteReg(DM9000_ISR,ISR_PTS);			//清除所有中断标志位
            dm9000_net_dev.imr_all=IMR_PAR|IMR_PRI;				//重新接收中断
            DM9000_WriteReg(DM9000_IMR, dm9000_net_dev.imr_all);
        }

    return (struct pbuf *)p;
}

#else
struct pbuf *DM9000_Receive_Packet(void)
{
    struct pbuf *head = NULL;  // 链表头
    struct pbuf *tail = NULL;  // 链表尾
    struct pbuf *p, *q;
    u32 rxbyte;
    vu16 rx_status, rx_length;
    u16 *data;
    u16 dummy;
    int len, i;

    DM9000_DUG("DM9000_Receive_Packets\r\n");

    while (1)
    {
        DM9000_ReadReg(DM9000_MRRH);
        DM9000_ReadReg(DM9000_MRRL);
        DM9000_ReadReg(DM9000_MRCMDX);  // 假读
        rxbyte = (u8)DM9000->DATA;
        //__DSB();

        if (rxbyte == 0)
        {
            // FIFO空，清除中断，允许接收中断
            // DM9000_WriteReg(DM9000_ISR, ISR_PTS);
            // dm9000_net_dev.imr_all = IMR_PAR | IMR_ROOI | IMR_POI | IMR_PTI | IMR_PRI;
            // DM9000_WriteReg(DM9000_IMR, dm9000_net_dev.imr_all);
            break;
        }
        if (rxbyte > 1)
        {
            rt_kprintf("dm9000 rx: rx error, stop device rxbyte:0x%x\r\n", rxbyte);
            DM9000_WriteReg(DM9000_RCR, 0x00);
            DM9000_WriteReg(DM9000_ISR, 0x80);
            DM9000_WriteReg(DM9000_NCR, NCR_RST);
            _dm9000_delay_ms(5);
            DM9000_WriteReg(DM9000_RCR, 0x39);
            break;  // 出错直接退出
        }

        DM9000->REG = DM9000_MRCMD; //__DSB();
        rx_status = DM9000->DATA; //__DSB();
        rx_length = DM9000->DATA; //__DSB();

        DM9000_DUG("rx_status:0x%x rx_length: %d\r\n", rx_status, rx_length);

        if ((rx_status & 0xBF00) || (rx_length < 0x40) || (rx_length > DM9000_PKT_MAX))
        {
            rt_kprintf("rx_status error:%#x\r\n", rx_status);
            if (rx_status & 0x100) rt_kprintf("rx fifo error\r\n");
            if (rx_status & 0x200) rt_kprintf("rx crc error\r\n");
            if (rx_status & 0x8000) rt_kprintf("rx length error\r\n");
            if (rx_length > DM9000_PKT_MAX)
            {
                rt_kprintf("rx length too big\r\n");
                DM9000_WriteReg(DM9000_NCR, NCR_RST);
                _dm9000_delay_ms(5);
            }
            // 丢弃这包数据
            for (i = 0; i < (rx_length + 1) / 2; i++)
            {
                dummy = DM9000->DATA;
                //__DSB();
            }
            continue;  // 继续读下一包
        }

        p = pbuf_alloc(PBUF_RAW, rx_length, PBUF_POOL);
        if (p == NULL)
        {
            rt_kprintf("pbuf allocation failed:%d\r\n", rx_length);
            // 读出数据但不存储，避免死锁
            for (i = 0; i < (rx_length + 1) / 2; i++)
            {
                dummy = DM9000->DATA;
                //__DSB();
            }
            continue;
        }

        for (q = p; q != NULL; q = q->next)
        {
            data = (u16 *)q->payload;
            len = (q->len + 1) / 2;
            for (i = 0; i < len; i++)
            {
                data[i] = DM9000->DATA;
                //__DSB();
            }
        }

        // 将新包加入链表尾
        if (head == NULL)
        {
            head = p;
            tail = p;
        }
        else
        {
            tail->next = p;
            tail = p;
        }
    }

    return head;
}
#endif

int DM9000_Receive_Packet2(uint8_t *buf, int buf_size)
{
    u32 rxbyte;
    vu16 rx_status, rx_length = 0;
    u16 *data;
    u16 dummy;
    int len;
    static int first = 1;

    // for(int i=0; i<10000; i++);
__error_retry:
    DM9000_ReadReg(DM9000_MRRH); // 读取这两个寄存器
    DM9000_ReadReg(DM9000_MRRL);
    DM9000_ReadReg(DM9000_MRCMDX); // 假读
    rxbyte = (u8)DM9000->DATA;     // 进行第二次读取
    if (rxbyte)                    // 接收到数据
    {
        if (rxbyte > 1) // rxbyte大于1，接收到的数据错误,挂了
        {
            rt_kprintf("dm9000 rx: rx error, stop device rxbyte:0x%x\r\n", rxbyte);
            DM9000_WriteReg(DM9000_RCR, 0x00);
            DM9000_WriteReg(DM9000_ISR, 0x80);
            return 0;
        }
        DM9000->REG = DM9000_MRCMD;
        rx_status = DM9000->DATA;
        rx_length = DM9000->DATA;
        DM9000_DUG("rx_status:0x%x rx_length: %d\r\n", rx_status, rx_length);
        // if(rx_length>512)rt_kprintf("rxlen:%d\r\n",rx_length);
        data = (u16 *)buf;
        len = rx_length;
        while (len > 0)
        {
            *data = DM9000->DATA;
            data++;
            len -= 2;
        }
        // 根据rx_status判断接收数据是否出现如下错误：FIFO溢出、CRC错误
        // 对齐错误、物理层错误，如果有任何一个出现的话丢弃该数据帧，
        // 当rx_length小于64或者大于最大数据长度的时候也丢弃该数据帧
        if ((rx_status & 0XBF00) || (rx_length < 0X40) || (rx_length > DM9000_PKT_MAX))
        {
            rt_kprintf("rx_status:%#x\r\n", rx_status);
            if (rx_status & 0x100)
                rt_kprintf("rx fifo error\r\n");
            if (rx_status & 0x200)
                rt_kprintf("rx crc error\r\n");
            if (rx_status & 0x8000)
                rt_kprintf("rx length error\r\n");
            if (rx_length > DM9000_PKT_MAX)
            {
                rt_kprintf("rx length too big\r\n");
                DM9000_WriteReg(DM9000_NCR, NCR_RST); // 复位DM9000
                _dm9000_delay_ms(5);
            }
            goto __error_retry;
        }
    }
    else
    {
        // DM9000_WriteReg(DM9000_ISR,ISR_PTS);			//清除所有中断标志位
        // dm9000_net_dev.imr_all=IMR_PAR|IMR_PRI;				//重新接收中断
        // DM9000_WriteReg(DM9000_IMR, dm9000_net_dev.imr_all);
    }

    return rx_length;
}

#ifndef DM9000_USE_LWIP
extern void _netif_input_dat(u8 *dat, u16 len); // 定义在net_hook.h中
#endif


// 中断处理函数
void DM9000_ISRHandler(void)
{
    u16 int_status;
    u16 last_io;
    last_io = DM9000->REG;
    int_status = DM9000_ReadReg(DM9000_ISR);
    DM9000_WriteReg(DM9000_ISR, int_status); // 清除中断标志位，DM9000的ISR寄存器的bit0~bit5写1清零
    if (int_status & ISR_ROS)
        rt_kprintf("overflow \r\n");
    if (int_status & ISR_ROOS)
        rt_kprintf("overflow counter overflow \r\n");
    if (int_status & ISR_PRS) // 接收中断
    {
// DM9000_DUG("ISR_PRS\r\n");
#ifdef DM9000_USE_LWIP
        rt_err_t result;
        result = eth_device_ready(&(dm9000_net_dev.parent));
        dm_irq_cnt ++;
//        if (result != RT_EOK)
            // rt_kprintf("RxCpltCallback err = %d", result);
#else

        // **进入临界区，屏蔽所有中断**
        // rt_base_t level = rt_hw_interrupt_disable();
        for (int i = 0; i < 5; i++)
        {
            int rx_cnt = DM9000_Receive_Packet2(rxbuf, sizeof(rxbuf));
            if (rx_cnt > 0)
            {
                _netif_input_dat((u8 *)rxbuf, rx_cnt);
            }
            else
                break;
        }

// **退出临界区，恢复中断**
// rt_hw_interrupt_enable(level);
#endif
    }
    if (int_status & ISR_PTS) // 发送中断
    {
        // 发送完成中断，用户自行添加所需代码
    }
    DM9000->REG = last_io;
}

void irq_callback(void *args)
{
    DM9000_ISRHandler();
}

int dm9000_check_link_status(void)
{
    int status = DM9000_ReadReg(DM9000_NSR); // 读取网络状态寄存器
    return (status & 0x40) ? 1 : 0;          // Bit6 = 1 表示 Link Up
}

#ifdef DM9000_USE_LWIP
/************************************************ rt netdev init***********************************************************/
static rt_err_t dm9000_eth_tx(rt_device_t dev, struct pbuf *p)
{
    DM9000_DUG("dm9000_eth_tx\r\n");
    DM9000_SendPacket(p);
    return RT_EOK;
}

int debug_flag = 0;
struct pbuf *dm9000_eth_rx(rt_device_t dev)
{
    struct pbuf *p = NULL;
    DM9000_DUG("dm9000_eth_rx\r\n");
    p = DM9000_Receive_Packet();

    if(debug_flag)
    {
        rt_kprintf("dm9000_eth_rx\r\n");
        if (p != NULL)
        {
            struct pbuf *q;
            for (q = p; q != NULL; q = q->next)
            {
                rt_kprintf("pbuf len: %d\r\n", q->len);
                // rt_kprintf("pbuf payload: ");
                // for (int i = 0; i < q->len; i++)
                // {
                //     rt_kprintf("%02x ", ((u8_t *)q->payload)[i]);
                // }
                // rt_kprintf("\r\n");
            }
        }
        else
        {
            rt_kprintf("No packet received\r\n");
        }
    }
        

    return p;
}

/** 初始化网卡 */
static rt_err_t dm9000_eth_init(rt_device_t dev)
{
    if (0 != DM9000_Init())
        return RT_ERROR;

    // eth_device_linkchange(&dm9000_net_dev.parent, RT_TRUE);
    return RT_EOK;
}

static rt_err_t dm9000_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
    rt_kprintf("emac open\r\n");
    return RT_EOK;
}

static rt_err_t dm9000_eth_close(rt_device_t dev)
{
    rt_kprintf("emac close\r\n");
    return RT_EOK;
}

static rt_size_t dm9000_eth_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_kprintf("emac read\r\n");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t dm9000_eth_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct pbuf p;

    rt_kprintf("pos:0x%x size:0x%x\r\n", pos, size);

    // 直接使用 buffer，无需分配新内存
    p.payload = (void *)buffer;
    p.len = size;
    p.tot_len = size;
    p.next = NULL;

    // 发送数据
    dm9000_eth_tx(dev, &p);

    return size;
}

/** 控制接口 */
static rt_err_t dm9000_eth_control(rt_device_t dev, int cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR: // 获取 MAC 地址
        if (args)
            rt_memcpy(args, dm9000_net_dev.mac_addr, 6);
        return RT_EOK;
    default:
        return RT_ERROR;
    }
}

static void phy_linkchange()
{
    static rt_uint8_t phy_speed = 0;
    rt_uint8_t phy_speed_new = 0;
    rt_uint32_t status;
    u8 temp;

    temp = DM9000_Get_SpeedAndDuplex(); // 获取DM9000的连接速度和双工状态
    if (temp != 0XFF)                   // 连接成功，通过串口显示连接速度和双工状态
    {
        if(phy_speed != temp) // 如果连接状态发生变化
        {
            phy_speed_new = temp;
            phy_speed = temp;
            DM9000_DUG("DM9000 Speed:%dMbps,Duplex:%s duplex mode\r\n", (temp & 0x02) ? 10 : 100, (temp & 0x01) ? "Full" : "Half");
            eth_device_linkchange(&dm9000_net_dev.parent, RT_TRUE);
        }
        else
        {
            return; // 没有变化，直接返回
        }
    }
    else
    {
        DM9000_DUG("link down");
        phy_speed = 0;
        eth_device_linkchange(&dm9000_net_dev.parent, RT_FALSE);
    }
}


static void phy_monitor_thread_entry(void *parameter)
{
    phy_linkchange();
#ifdef PHY_USING_INTERRUPT_MODE
    /* configuration intterrupt pin */
    rt_pin_mode(PHY_INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(PHY_INT_PIN, PIN_IRQ_MODE_FALLING, eth_phy_isr, (void *)"callbackargs");
    rt_pin_irq_enable(PHY_INT_PIN, PIN_IRQ_ENABLE);

    /* enable phy interrupt */
    HAL_ETH_WritePHYRegister(&EthHandle, PHY_ADDR, PHY_INTERRUPT_MASK_REG, PHY_INT_MASK);
#if defined(PHY_INTERRUPT_CTRL_REG)
    HAL_ETH_WritePHYRegister(&EthHandle, PHY_ADDR, PHY_INTERRUPT_CTRL_REG, PHY_INTERRUPT_EN);
#endif
#else /* PHY_USING_INTERRUPT_MODE */
    dm9000_net_dev.poll_link_timer = rt_timer_create("phylnk", (void (*)(void*))phy_linkchange,
                                        NULL, RT_TICK_PER_SECOND, RT_TIMER_FLAG_PERIODIC);
    if (!dm9000_net_dev.poll_link_timer || rt_timer_start(dm9000_net_dev.poll_link_timer) != RT_EOK)
    {
        LOG_E("Start link change detection timer failed");
    }
#endif /* PHY_USING_INTERRUPT_MODE */
}

/** 注册网卡 */
static int rt_hw_dm9000_netdev_init(void)
{
    // 获取STM32的唯一ID的前24位作为MAC地址后三字节
    uint32_t id = HAL_GetUIDw0();
    dm9000_net_dev.mac_addr[0] = 0x00;
    dm9000_net_dev.mac_addr[1] = 0x80;
    dm9000_net_dev.mac_addr[2] = 0xE1;
    dm9000_net_dev.mac_addr[3] = (id >> 16) & 0XFF;
    dm9000_net_dev.mac_addr[4] = (id >> 8) & 0XFFF;
    dm9000_net_dev.mac_addr[5] = id & 0XFF;

    dm9000_net_dev.parent.parent.init = dm9000_eth_init;
    dm9000_net_dev.parent.parent.control = dm9000_eth_control;
    dm9000_net_dev.parent.eth_tx = dm9000_eth_tx;
    dm9000_net_dev.parent.eth_rx = dm9000_eth_rx;
    dm9000_net_dev.parent.parent.open = dm9000_eth_open;
    dm9000_net_dev.parent.parent.close = dm9000_eth_close;
    dm9000_net_dev.parent.parent.read = dm9000_eth_read;
    dm9000_net_dev.parent.parent.write = dm9000_eth_write;
    dm9000_net_dev.parent.parent.control = dm9000_eth_control;
    dm9000_net_dev.parent.parent.user_data = RT_NULL;

    eth_device_init(&dm9000_net_dev.parent, DM900NET_NAME);

    rt_kprintf("PBUF_POOL_BUFSIZE:%ld\r\n", PBUF_POOL_BUFSIZE);

    /* start phy monitor */
    rt_thread_t tid;
    rt_uint32_t state;
    tid = rt_thread_create("phy",
                           phy_monitor_thread_entry,
                           RT_NULL,
                           1024,
                           RT_THREAD_PRIORITY_MAX - 2,
                           2);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        state = -RT_ERROR;
    }

    return state;
}
#endif

#ifdef DM9000_USE_LWIP
INIT_DEVICE_EXPORT(rt_hw_dm9000_netdev_init);
#else
// INIT_DEVICE_EXPORT(DM9000_Init);
#endif

/************************************* Debug *********************************************** */
#if 1
void dm9000_reg_show(void)
{
    rt_kprintf("dm9000_reg_show \n");

    rt_kprintf("NCR   (%02X): %02x\n", DM9000_NCR,  DM9000_ReadReg(DM9000_NCR));
    rt_kprintf("NSR   (%02X): %02x\n", DM9000_NSR,  DM9000_ReadReg(DM9000_NSR));
    rt_kprintf("TCR   (%02X): %02x\n", DM9000_TCR,  DM9000_ReadReg(DM9000_TCR));
    rt_kprintf("TSRI  (%02X): %02x\n", DM9000_TSRI, DM9000_ReadReg(DM9000_TSRI));
    rt_kprintf("TSRII (%02X): %02x\n", DM9000_TSRII, DM9000_ReadReg(DM9000_TSRII));
    rt_kprintf("RCR   (%02X): %02x\n", DM9000_RCR,  DM9000_ReadReg(DM9000_RCR));
    rt_kprintf("RSR   (%02X): %02x\n", DM9000_RSR,  DM9000_ReadReg(DM9000_RSR));
    rt_kprintf("ORCR  (%02X): %02x\n", DM9000_ROCR, DM9000_ReadReg(DM9000_ROCR));
    rt_kprintf("CRR   (%02X): %02x\n", DM9000_CHIPR,DM9000_ReadReg(DM9000_CHIPR));
    rt_kprintf("CSCR  (%02X): %02x\n", DM9000_TCSCR, DM9000_ReadReg(DM9000_TCSCR));
    rt_kprintf("RCSSR (%02X): %02x\n", DM9000_RCSCSR,DM9000_ReadReg(DM9000_RCSCSR));
    rt_kprintf("ISR   (%02X): %02x\n", DM9000_ISR,  DM9000_ReadReg(DM9000_ISR));
    rt_kprintf("IMR   (%02X): %02x\n", DM9000_IMR,  DM9000_ReadReg(DM9000_IMR));
    rt_kprintf("ID : %04x\n", DM9000_Get_DeiviceID);
    rt_kprintf("dm9000_reg_show end\n");
}
MSH_CMD_EXPORT(dm9000_reg_show, "dm9000 register dump");

void dm9000_w_data(void)
{
    rt_kprintf("dm9000_data \n");
    DM9000->DATA = 0xFF;
    rt_kprintf("dm9000_data end\n");
}
MSH_CMD_EXPORT(dm9000_w_data, "dm9000_w_data");

void dm9000_w_reg(void)
{
    rt_kprintf("dm9000_reg \n");
    DM9000->REG = 0xFF;
    rt_kprintf("dm9000_reg end\n");
}
MSH_CMD_EXPORT(dm9000_w_reg, "dm9000_w_reg");

void dm9000_r_data(void)
{
    rt_kprintf("dat:0x%x \n", DM9000->DATA);
}
MSH_CMD_EXPORT(dm9000_r_data, "dm9000_r_data");

void dm9000_r_reg(void)
{
    rt_kprintf("dat:0x%x \n", DM9000->REG);
}
MSH_CMD_EXPORT(dm9000_r_reg, "dm9000_r_reg");

void dm9000_showID(void)
{
    rt_kprintf("DM9000 ID: 0x%x\r\n", DM9000_Get_DeiviceID());

}
MSH_CMD_EXPORT(dm9000_showID, "dm9000_showID");

void dm9000_dump(void)
{
    uint16_t *p = (volatile uint16_t *)(0x60000000);
    for(int i=0; i<16; i++)
    {
        rt_kprintf("%02d - 0x%4x: 0x%x\n", i, p, *p);
        p++;
    }

}
MSH_CMD_EXPORT(dm9000_dump, "dm9000_dump");


void dm9000_test(void)
{
    rt_kprintf("dm9000_test \n");
    DM9000->REG = 0xFF;

    rt_kprintf("dm9000_test end\n");
    rt_kprintf("FMC_BTR1: 0x%08lX\n", FMC_Bank1_R->BTCR[1]);  // 读取 NOR/SRAM 1 号区的时序寄存器
    rt_kprintf("FMC_BWTR1: 0x%08lX\n", FMC_Bank1E_R->BWTR[0]);  // 读取写时序
    rt_kprintf("FMC_BCR1: 0x%08lX\n", FMC_Bank1_R->BTCR[0]);  // 读取控制寄存器


}
MSH_CMD_EXPORT(dm9000_test, "dm9000_test");

void dm9000_debug(int argc, char **argv)
{
    if(argc != 2)
    {
        rt_kprintf("Usage: dm9000_debug 0|1\r\n");
        return;
    }

    debug_flag = atoi(argv[1]);
    rt_kprintf("dm9000_debug set to %d\r\n", debug_flag);
}
MSH_CMD_EXPORT(dm9000_debug, "dm9000_debug");

#endif

void dm9000_init(int argc, char **argv)
{
    DM9000_Init();
    rt_kprintf("dm9000_init ID:0x%x\r\n", DM9000_Get_DeiviceID());
}
MSH_CMD_EXPORT(dm9000_init, "dm9000_init");

void dm9000_readID(int argc, char **argv)
{
    rt_kprintf("dm9000_init ID:0x%x\r\n", DM9000_Get_DeiviceID());
}
MSH_CMD_EXPORT(dm9000_readID, "dm9000_readID");


int net_send(unsigned char *data, int len)
{
    int ret = -1;
    struct pbuf p={0};
    p.payload = (void *)data;
    p.len = len;
    p.tot_len = len;
    DM9000_SendPacket(&p);
    ret = len;

    return ret;
}

void dm9000_arp_test(int argc, char **argv)
{
    static const uint8_t arp_packet[42] = {
        // 以太网头
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,        // 目的 MAC（广播）
        0x02, 0x00, 0x00, 0x00, 0x00, 0x02,        // 源 MAC
        0x08, 0x06,                                // 类型：ARP = 0x0806

        // ARP payload
        0x00, 0x01,        // 硬件类型：以太网
        0x08, 0x00,        // 协议类型：IPv4
        0x06,              // 硬件地址长度：6
        0x04,              // 协议地址长度：4
        0x00, 0x01,        // 操作：ARP请求

        0x02, 0x00, 0x00, 0x00, 0x00, 0x02,        // 发送方 MAC
        192, 168, 8, 190,                            // 发送方 IP

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        // 目标 MAC（未知）
        192, 168, 8, 1                           // 目标 IP
    };
    net_send(arp_packet, sizeof(arp_packet));

    
    uint8_t rx_buf[1540];
    int pkg_cnt = 0;
    int pkg_send_cnt = 0;
    int pkg_recv_cnt = 0;
    int pkg_arp_cnt = 0;
    int time_out = 0;
    while(1)
    {
        pkg_send_cnt++;
        int rx_cnt = DM9000_Receive_Packet2(rx_buf, sizeof(rx_buf));
        if(rx_cnt > 0)
        {
            pkg_recv_cnt++;
            uint16_t ethertype = (rx_buf[12] << 8) | rx_buf[13];
            rt_kprintf("recv <<<--- total:%d ethertype:0x%x\n", rx_cnt, ethertype);
            if (ethertype == 0x0806)
            {
                pkg_arp_cnt++;
                // rt_kprintf("recv <<<--- total:%d\n", rx_cnt);
                // rt_kprintf(">>> 这是一帧 ARP 包 pkg_recv_cnt:%d time_out:%d\n", pkg_recv_cnt, time_out);
                // hex_dump(rx_buf, rx_cnt);
                // return ;
            }
            // else if(ethertype == 0x0800)
            // {
            //     rt_kprintf("recv <<<--- total:%d\n", rx_cnt);
            //     rt_kprintf(">>> 这是一帧 IPV4 包 pkg_recv_cnt:%d time_out:%d\n", pkg_recv_cnt, time_out);
            //     hex_dump(rx_buf, rx_cnt);
            // }
        }
        _dm9000_delay_ms(10);
        time_out ++;
        if(time_out >= 10)
        {
            rt_kprintf(">>> pkg_recv_cnt:%d pkg_send_cnt:%d pkg_arp_cnt:%d\n", pkg_recv_cnt, pkg_send_cnt, pkg_arp_cnt);
            return;
        } 
    }
    
}
MSH_CMD_EXPORT(dm9000_arp_test, "dm9000_arp_test");

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
