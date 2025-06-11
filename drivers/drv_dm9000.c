#include "drv_dm9000.h"

#define DBG_TAG                        "dm9k"
#define DBG_LVL                        DBG_LOG
#include <rtdbg.h>

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include <netif/ethernetif.h>

#include "stm32h7xx_hal_sram.h"

// #define DM9000_DEBUG
#ifdef DM9000_DEBUG
#define DM9000_TRACE    rt_kprintf
#else
#define DM9000_TRACE(...)
#endif

/* dm9000 reset pin : GPIOD PIN7, LOW is RESET */
#define DM9000_RST_0        rt_pin_write(GET_PIN(A, 10), PIN_LOW)
#define DM9000_RST_1        rt_pin_write(GET_PIN(A, 10), PIN_HIGH)

#define MAX_ADDR_LEN        6       /* max length of hw address */

#define DM9000_PHY          0x40    /* PHY address 0x01 */

#define PIN_NRESET GET_PIN(A, 10) // 复位引脚
#define PIN_IRQ GET_PIN(A, 15)    // 例如：PA0 作为中断引脚

enum DM9000_PHY_mode
{
    DM9000_10MHD = 0, DM9000_100MHD = 1,
    DM9000_10MFD = 4, DM9000_100MFD = 5,
    DM9000_AUTO  = 8, DM9000_1M_HPNA = 0x10
};

enum DM9000_TYPE
{
    TYPE_DM9000E,
    TYPE_DM9000A,
    TYPE_DM9000B
};

struct rt_dm9000_eth
{
    /* inherit from ethernet device */
    struct eth_device parent;

    enum DM9000_TYPE type;
    enum DM9000_PHY_mode mode;

    rt_uint8_t packet_cnt;                /* packet I or II */
    rt_uint16_t queue_packet_len;          /* queued packet (packet II) */

    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];     /* hw address   */
    rt_uint8_t  init_complete;           /* init complete flag */
};

static struct rt_dm9000_eth dm9000_device;
static struct rt_semaphore sem_ack, sem_lock;
int dm_irq_cnt, dm_pkg_max;

// 这个一定要放在全局作用域下
//static SRAM_HandleTypeDef DM9000_Handler;           //DM9000句柄

/* --- */

static inline void dm9000_delay_ms(rt_uint32_t ms)
{
    rt_thread_mdelay(ms); return;
}

/* Read a byte from I/O port */
rt_inline rt_uint16_t dm9000_io_read(rt_uint16_t reg) {
    DM9000_IO = reg;
    return DM9000_DATA;
}

/* Write a byte to I/O port */
rt_inline void dm9000_io_write(rt_uint16_t reg, rt_uint16_t value) {
    DM9000_IO = reg;
    DM9000_DATA = value;
}

/* Get DeviceID of DM9000 */
static rt_uint32_t dm9000_get_device_id(void)
{
    rt_uint32_t value;
    value  = dm9000_io_read(DM9000_VIDL);
    value |= dm9000_io_read(DM9000_VIDH) << 8;
    value |= dm9000_io_read(DM9000_PIDL) << 16;
    value |= dm9000_io_read(DM9000_PIDH) << 24;
    return value;
}

/* Reset DM9000 */
static void dm9000_reset(void) {
    DM9000_TRACE("enter dm9000_reset\n");
    
    DM9000_RST_0; // set rst pin low
    dm9000_delay_ms(10);

    DM9000_RST_1;
    dm9000_delay_ms(100);  // hardware rst over

    dm9000_io_write(DM9000_GPCR, 0x01);
    dm9000_io_write(DM9000_GPR, 0);
    dm9000_io_write(DM9000_NCR, (0x02 | NCR_RST)); // soft rst

    do
    {
        dm9000_delay_ms(25);
    }while(dm9000_io_read(DM9000_NCR) & 1); // wait for soft rst over

    dm9000_io_write(DM9000_NCR,0);
    dm9000_io_write(DM9000_NCR, (0x02 | NCR_RST)); // soft rst again

    do
    {
        dm9000_delay_ms(25);
    }while (dm9000_io_read(DM9000_NCR) & 1);
}


/* Read a word from phyxcer */
rt_inline rt_uint16_t dm9000_phy_read(rt_uint16_t reg) {
    rt_uint16_t val;

    /* Fill the phyxcer register into REG_0C */
    dm9000_io_write(DM9000_EPAR, DM9000_PHY | reg);
    dm9000_io_write(DM9000_EPCR, 0x0C); /* Issue phyxcer read command */

    dm9000_delay_ms(100);       /* Wait read complete */

    dm9000_io_write(DM9000_EPCR, 0x00); /* Clear phyxcer read command */
    val = (dm9000_io_read(DM9000_EPDRH) << 8) | dm9000_io_read(DM9000_EPDRL);

    return val;
}

/* Write a word to phyxcer */
rt_inline void dm9000_phy_write(rt_uint16_t reg, rt_uint16_t value)
{
    /* Fill the phyxcer register into REG_0C */
    dm9000_io_write(DM9000_EPAR, DM9000_PHY | reg);

    /* Fill the written data into REG_0D & REG_0E */
    dm9000_io_write(DM9000_EPDRL, (value & 0xFF));
    dm9000_io_write(DM9000_EPDRH, ((value >> 8) & 0xFF));
    dm9000_io_write(DM9000_EPCR, 0x0A); /* Issue phyxcer write command */

    dm9000_delay_ms(500);       /* Wait write complete */

    dm9000_io_write(DM9000_EPCR, 0x00); /* Clear phyxcer write command */
}

/* Set PHY operationg mode */
rt_inline void dm9000_phy_mode_set(rt_uint8_t mode)
{
    rt_uint16_t phy_BMCR, phy_ANAR;
    switch(mode)
    {
        case DM9000_10MHD:
            phy_BMCR = 0X0000;
            phy_ANAR = 0X21;
            break;
        case DM9000_10MFD:
            phy_BMCR = 0X0100;
            phy_ANAR = 0X41;
            break;
        case DM9000_100MHD:
            phy_BMCR = 0X2000;
            phy_ANAR = 0X81;
            break;
        case DM9000_100MFD:
            phy_BMCR = 0X2100;
            phy_ANAR = 0X101;
            break;
        case DM9000_AUTO:
            phy_BMCR = 0X1000;
            phy_ANAR = 0X01E1;
            break;
    }

    dm9000_phy_write(DM9000_PHY_BMCR, phy_BMCR);
    dm9000_phy_write(DM9000_PHY_ANAR, phy_ANAR); /* Set PHY media mode */

    dm9000_io_write(DM9000_GPCR, 0x01); /* Let GPIO0 output */
    dm9000_io_write(DM9000_GPR, 0X00);  /* enable PHY */
}

/* interrupt service routine */
void rt_dm9000_isr(void *arg)
{
    rt_uint16_t int_status;
    rt_uint16_t last_io;
//    rt_uint32_t eint_pend;

    last_io = DM9000_IO;

    /* Disable all interrupts */
    dm9000_io_write(DM9000_IMR, IMR_PAR);

    /* Got DM9000 interrupt status */
    int_status = dm9000_io_read(DM9000_ISR);    /* Got ISR */
    dm9000_io_write(DM9000_ISR, int_status);    /* Clear ISR status */

    DM9000_TRACE("dm9000 isr: int status %04x\n", int_status);

    /* receive overflow */
    if (int_status & ISR_ROS)
    {
        LOG_W("overflow, ISR:%02x", int_status);
    }

    if (int_status & ISR_ROOS)
    {
        LOG_W("overflow counter overflow, ISR:%02x", int_status);
    }

    /* Received the coming packet */
    if (int_status & ISR_PRS)
    {
        /* a frame has been received */
        eth_device_ready(&(dm9000_device.parent));
        dm_irq_cnt ++;
    }

    /* Transmit Interrupt check */
    if (int_status & ISR_PTS)
    {
        /* clear int_status */
        dm9000_io_write(DM9000_ISR, ISR_PTS);

        /* transmit done */
        int tx_status = dm9000_io_read(DM9000_NSR); /* Got TX status */

        if (tx_status & (NSR_TX2END | NSR_TX1END))
        {
            dm9000_device.packet_cnt --;
            if (dm9000_device.packet_cnt > 0)
            {
                DM9000_TRACE("dm9000 isr: tx second packet\n");

                /* transmit packet II */
                /* Set TX length to DM9000 */
                dm9000_io_write(DM9000_TXPLL, dm9000_device.queue_packet_len & 0xff);
                dm9000_io_write(DM9000_TXPLH, (dm9000_device.queue_packet_len >> 8) & 0xff);

                /* Issue TX polling command */
                dm9000_io_write(DM9000_TCR, TCR_TXREQ); /* Cleared after TX complete */
            }

            /* One packet sent complete */
            /* clear tx isr */
            if (sem_ack.value != 0) {
                LOG_W("isr: trying to release sem_ack while its value > 0 / failed");
            } else {
                rt_sem_release(&sem_ack);
            }
        }
    }

    /* Re-enable interrupt mask */
    dm9000_io_write(DM9000_IMR, IMR_PAR | IMR_PTM | IMR_PRM | ISR_ROS | ISR_ROOS);

    DM9000_IO = last_io;
}

static void dm9000_softrst_wait(rt_uint32_t ms)
{
    dm9000_io_write(DM9000_NCR, NCR_RST);
    do
    {
        rt_thread_mdelay(ms);
    } while (dm9000_io_read(DM9000_NCR) & 1); /* wait for soft rst over */

    /* initialize regs */

    /* GPIO0 on pre-activate PHY */
    dm9000_io_write(DM9000_GPR, 0x00);              /* REG_1F bit0 activate phyxcer */
    dm9000_io_write(DM9000_GPCR, GPCR_GEP_CNTL);    /* Let GPIO0 output */
    dm9000_io_write(DM9000_GPR, 0x00);               /* Enable PHY */

    /* Set PHY */
    dm9000_phy_mode_set(dm9000_device.mode);

    /* Program operating register */
    dm9000_io_write(DM9000_NCR, 0x0);   /* only intern phy supported by now */
    dm9000_io_write(DM9000_TCR, 0);     /* TX Polling clear */
    dm9000_io_write(DM9000_BPTR, 0x3f); /* Less 3Kb, 200us */
    dm9000_io_write(DM9000_FCTR, FCTR_HWOT(3) | FCTR_LWOT(8));  /* Flow Control : High/Low Water */
    dm9000_io_write(DM9000_FCR, 0x0);   /* SH FIXME: This looks strange! Flow Control */
    dm9000_io_write(DM9000_SMCR, 0);    /* Special Mode */
    dm9000_io_write(DM9000_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END);  /* clear TX status */
    dm9000_io_write(DM9000_ISR, 0x0f);  /* Clear interrupt status */
    dm9000_io_write(DM9000_TCR2, 0x80); /* Switch LED to mode 1 */

    /* Activate DM9000 */
    dm9000_io_write(DM9000_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN); /* RX enable */
    dm9000_io_write(DM9000_IMR, IMR_PAR);
}

/* RT-Thread Device Interface */
/* initialize the interface */
static rt_err_t rt_dm9000_init(rt_device_t dev)
{
    LOG_I("Driver dm9000 init / start");
    int i, oft, lnk;
    rt_uint32_t dm9000_id;

    /* RESET device */
    dm9000_reset();
    dm9000_delay_ms(100);

    /* identfy DM9000 */
    dm9000_id = dm9000_get_device_id();
    LOG_I("dm9000 id: 0x%x", dm9000_id);
    if (dm9000_id != DM9000_ID) {
        LOG_E("dm9000 id error");
        return -RT_ERROR;
    }

    /* set mac address */
    for (i = 0, oft = DM9000_PAR; i < 6; ++i, ++oft)
        dm9000_io_write(oft, dm9000_device.dev_addr[i]);
    /* set multicast address */
    for (i = 0, oft = DM9000_MAR; i < 8; ++i, ++oft)
        dm9000_io_write(oft, 0xff);

    dm9000_softrst_wait(25); /* init regs here */

    if (dm9000_device.mode == DM9000_AUTO)
    {
        i = 0;
        while (!(dm9000_phy_read(1) & 0x20))
        {
            /* autonegation complete bit */
            rt_thread_delay( RT_TICK_PER_SECOND/10 );
            i++;
            if (i > 30 ) /* wait 3s */
            {
                LOG_E("could not establish link");
                return 0;
            }
        }
    }

    /* send a notify */
    eth_device_linkchange(&dm9000_device.parent, RT_TRUE);

    /* see what we've got */
    lnk = dm9000_phy_read(17) >> 12;
    switch (lnk)
    {
    case 1:
        LOG_I("10M half duplex ");
        break;
    case 2:
        LOG_I("10M full duplex ");
        break;
    case 4:
        LOG_I("100M half duplex ");
        break;
    case 8:
        LOG_I("100M full duplex ");
        break;
    default:
        LOG_I("unknown: %d ", lnk);
        break;
    }

    /* Enable TX/RX interrupt mask */
    dm9000_io_write(DM9000_IMR, IMR_PAR | IMR_PTM | IMR_PRM | ISR_ROS | ISR_ROOS);

    LOG_I("Driver dm9000 init / end");
    dm9000_device.init_complete = 1;
    return RT_EOK;
}

static rt_err_t rt_dm9000_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_dm9000_close(rt_device_t dev)
{
    /* RESET devie */
    dm9000_phy_write(0, 0x8000);    /* PHY RESET */
    dm9000_io_write(DM9000_GPR, 0x01);  /* Power-Down PHY */
    dm9000_io_write(DM9000_IMR, 0x80);  /* Disable all interrupt */
    dm9000_io_write(DM9000_RCR, 0x00);  /* Disable RX */

    return RT_EOK;
}

static rt_size_t rt_dm9000_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t rt_dm9000_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t rt_dm9000_control(rt_device_t dev, int cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args) rt_memcpy(args, dm9000_device.dev_addr, 6);
        else return -RT_ERROR;
        break;

    default :
        break;
    }

    return RT_EOK;
}

/* ethernet device interface */
/* transmit packet. */
rt_err_t rt_dm9000_tx( rt_device_t dev, struct pbuf* p)
{
//    LOG_D("enter rt_dm9000_tx, p->tot_len: %d\n", p->tot_len);
    DM9000_TRACE("rt_dm9000_tx: %d\n", p->tot_len);

    /* lock DM9000 device */
    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

    /* disable dm9000a interrupt */
    dm9000_io_write(DM9000_IMR, IMR_PAR);

    /* Move data to DM9000 TX RAM */
    // DM9000_outb(DM9000_IO_BASE, DM9000_MWCMD);
    DM9000_IO = DM9000_MWCMD;

    {
        /* q traverses through linked list of pbuf's
         * This list MUST consist of a single packet ONLY */
        struct pbuf *q;
        rt_uint16_t pbuf_index = 0;
        rt_uint8_t word[2], word_index = 0;

        q = p;
        /* Write data into dm9000a, two bytes at a time
         * Handling pbuf's with odd number of bytes correctly
         * No attempt to optimize for speed has been made */
        while (q)
        {
            if (pbuf_index < q->len)
            {
                word[word_index++] = ((u8_t*)q->payload)[pbuf_index++];
                if (word_index == 2)
                {
                    // DM9000_outw(DM9000_DATA_BASE, (word[1] << 8) | word[0]);
                    DM9000_DATA = (word[1] << 8) | word[0]; // write two bytes to DM9000_DATA
                    word_index = 0;
                }
            }
            else
            {
                q = q->next;
                pbuf_index = 0;
            }
        }
        /* One byte could still be unsent */
        if (word_index == 1)
        {
            // DM9000_outw(DM9000_DATA_BASE, word[0]);
            DM9000_DATA = word[0]; // write one byte to DM9000_DATA
        }
    }

//    /* Set TX length to DM9000 */
//    dm9000_io_write(DM9000_TXPLL, p->tot_len & 0xff);
//    dm9000_io_write(DM9000_TXPLH, (p->tot_len >> 8) & 0xff);
//
//    /* Issue TX polling command */
//    dm9000_io_write(DM9000_TCR, TCR_TXREQ); /* Cleared after TX complete */

    if (dm9000_device.packet_cnt == 0)
    {
        DM9000_TRACE("dm9000 tx: first packet\n");

        dm9000_device.packet_cnt ++;
        /* Set TX length to DM9000 */
        dm9000_io_write(DM9000_TXPLL, p->tot_len & 0xff);
        dm9000_io_write(DM9000_TXPLH, (p->tot_len >> 8) & 0xff);

        /* Issue TX polling command */
        dm9000_io_write(DM9000_TCR, TCR_TXREQ); /* Cleared after TX complete */
    }
    else
    {
        DM9000_TRACE("dm9000 tx: second packet\n");

        dm9000_device.packet_cnt ++;
        dm9000_device.queue_packet_len = p->tot_len;
    }

    /* enable dm9000a all interrupt */
    dm9000_io_write(DM9000_IMR, IMR_PAR | IMR_PTM | IMR_PRM | ISR_ROS | ISR_ROOS);

    /* unlock DM9000 device */
    rt_sem_release(&sem_lock);

    /* wait ack */
    rt_sem_take(&sem_ack, RT_WAITING_FOREVER);

    DM9000_TRACE("rt_dm9000_tx done\n");

    return RT_EOK;
}

/* reception packet. */
struct pbuf *rt_dm9000_rx(rt_device_t dev)
{
    struct pbuf* p;
    rt_uint32_t rx_ready; /* first rx byte */
    rt_uint16_t rx_status, rx_len;
    rt_uint16_t* data;
    rt_uint8_t dummy_u8;
    rt_uint16_t dummy_u16; // used for dummy
    rt_int32_t len;

    /* init p pointer */
    p = RT_NULL;

    /* lock DM9000 device */
    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

    /* disable dm9000a interrupt */
    dm9000_io_write(DM9000_IMR, IMR_PAR);

    /* Check packet ready or not */
    // dm9000_io_read(DM9000_MRRH); // 读取这两个寄存器
    // dm9000_io_read(DM9000_MRRL);
    dm9000_io_read(DM9000_MRCMDX);              /* Dummy read */
    // rx_ready = DM9000_inb(DM9000_DATA_BASE);      /* Got most updated data */
    rx_ready = (u8)DM9000_DATA;      /* Got most updated data */
    if (rx_ready == 0x01)
    {
        /* A packet ready now  & Get status/length */
        // DM9000_outb(DM9000_IO_BASE, DM9000_MRCMD);
        // rx_status = DM9000_inw(DM9000_DATA_BASE) & 0xff00;
        // rx_len = DM9000_inw(DM9000_DATA_BASE);

        DM9000_IO = DM9000_MRCMD;
        rx_status = DM9000_DATA & 0xff00;
        rx_len = DM9000_DATA;

        DM9000_TRACE("dm9000 rx: status %04x len %d\n", rx_status, rx_len);

        /* error handle */
        if ((rx_status & 0xbf00) || (rx_len < 0x40) || (rx_len > DM9000_PKT_MAX))
        {
            LOG_E("rx error: status %04x, rx_len: %d", rx_status, rx_len);

            if (rx_status & 0x100)
            {
                LOG_E("rx fifo error");
            }
            if (rx_status & 0x200)
            {
                LOG_E("rx crc error");
            }
            if (rx_status & 0x8000)
            {
                LOG_E("rx length error");
            }
            if (rx_len > DM9000_PKT_MAX)
            {
                LOG_E("rx length too big");
            }

            /* software-reset and re-init */
            dm9000_softrst_wait(25);

            /* it issues an error, release pbuf */
            if (p != RT_NULL)
                pbuf_free(p);
            p = RT_NULL;
            goto _rx_end;
        }

        /* allocate buffer */
        // p = pbuf_alloc(PBUF_LINK, rx_len, PBUF_RAM);
        rx_len -= 4; // remove 4B CRC
        p = pbuf_alloc(PBUF_RAW, rx_len, PBUF_POOL);
        if (p != RT_NULL)
        {
            // RT_ASSERT(p->type == PBUF_RAM); /* set PBUF_RAM above */
            // if (p->type == PBUF_RAM) {
                /* p is one large chunk */
//                int i;

                // RT_ASSERT(p->next == RT_NULL);
                // RT_ASSERT(p->len == p->tot_len);

                data = (rt_uint16_t*)p->payload;
                len = p->len;

                while (len > 1) {
                    // *data = DM9000_inw(DM9000_DATA_BASE);
                    *data = DM9000_DATA;
                    data++;
                    len -= 2;
                }

                /* just read a byte, protect memory */
                if (len == 1) {
                    // dummy_u8 = DM9000_inb(DM9000_DATA_BASE);
                    dummy_u8 = (u8)DM9000_DATA;
                    ((rt_uint8_t*)p->payload)[p->len - 1] = dummy_u8;
                }

                dummy_u16 = DM9000_DATA;
                dummy_u16 = DM9000_DATA;

            // } else { /* p is not one large chunk */
            //     struct pbuf* q;
            //     rt_int32_t len;

            //     for (q = p; q != RT_NULL; q= q->next)
            //     {
            //         data = (rt_uint16_t*)q->payload;
            //         len = q->len;

            //         while (len > 0)
            //         {
            //             *data = DM9000_inw(DM9000_DATA_BASE);
            //             data ++;
            //             len -= 2;
            //         }
            //     }
            // }
        }
        else /* pbuf allocate failed */
        {
            LOG_E("dm9000 rx: no pbuf, rx_len:%d", rx_len);
            len = rx_len;

            /* no pbuf, discard data from DM9000 */
            while (len > 1)
            {
                // dummy_u16 = DM9000_inw(DM9000_DATA_BASE); /* dummy read 2 bytes */
                dummy_u16 = DM9000_DATA; /* dummy read 2 bytes */
                len -= 2;
            }

            /* len == 1, if remaining 1 byte not read */
            if (len == 1)
            {
                // dummy_u8 = DM9000_inb(DM9000_DATA_BASE); /* dummy read 1 byte */
                dummy_u8 = DM9000_DATA;
            }
        }
    }
    else if (rx_ready > 0x01) /* error, stop interface and wait to reset */
    {
        LOG_E("dm9000 rx: rx error, stop device rx_ready:%d", rx_ready);

        dm9000_io_write(DM9000_ISR, 0x80);  /* Stop INT request */
        dm9000_io_write(DM9000_ISR, 0x0F);  /* Clear ISR status */
        dm9000_io_write(DM9000_RCR, 0x00);  /* Stop Rx Function */

        dm9000_softrst_wait(5); /* software-reset and re-init */
        goto _rx_end;
    }
    /*else rx_ready == 0x00, no message should be read */

_rx_end:

    /* clear packet received latch status */
    dm9000_io_write(DM9000_ISR, ISR_PRS);

    /* restore receive interrupt */
    dm9000_io_write(DM9000_IMR, IMR_PAR | IMR_PTM | IMR_PRM | ISR_ROS | ISR_ROOS);

    /* unlock DM9000 device */
    rt_sem_release(&sem_lock);

    return p;
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
    rt_pin_attach_irq(PIN_IRQ, PIN_IRQ_MODE_FALLING, rt_dm9000_isr, RT_NULL);
    rt_pin_irq_enable(PIN_IRQ, PIN_IRQ_ENABLE);
    /* USER CODE END FMC_Init 2 */
}

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
    if (dm9000_device.mode == DM9000_AUTO) // 如果开启了自动协商模式一定要等待协商完成
    {
        while (!(dm9000_phy_read (0X01) & 0X0020)) // 等待自动协商完成
        {
            dm9000_delay_ms(10);
            i++;
            if (i > 100)
                return 0XFF; // 自动协商失败
        }
    }
    else // 自定义模式,一定要等待连接成功
    {
        while (!(dm9000_io_read(DM9000_NSR) & 0X40)) // 等待连接成功
        {
            dm9000_delay_ms(10);
            i++;
            if (i > 100)
                return 0XFF; // 连接失败
        }
    }
    temp = ((dm9000_io_read(DM9000_NSR) >> 6) & 0X02);  // 获取DM9000的连接速度
    temp |= ((dm9000_io_read(DM9000_NCR) >> 3) & 0X01); // 获取DM9000的双工状态
    return temp;
}

static void phy_linkchange()
{
    static rt_uint8_t phy_speed = 0;
    rt_uint8_t temp;

    temp = DM9000_Get_SpeedAndDuplex(); // 获取DM9000的连接速度和双工状态
    if (temp != 0XFF)                   // 连接成功，通过串口显示连接速度和双工状态
    {
        if(phy_speed != temp) // 如果连接状态发生变化
        {
            phy_speed = temp;
            LOG_D("DM9000 Speed:%dMbps,Duplex:%s duplex mode", (temp & 0x02) ? 10 : 100, (temp & 0x01) ? "Full" : "Half");
            eth_device_linkchange(&dm9000_device.parent, RT_TRUE);
        }
        else
        {
            return; // 没有变化，直接返回
        }
    }
    else
    {
        LOG_D("link down");
        phy_speed = 0;
        eth_device_linkchange(&dm9000_device.parent, RT_FALSE);
    }
}

void phy_state_check(void *arg)
{
    LOG_D("phy_state_check started");
    while(1)
    {
        if (dm9000_device.init_complete == 1) 
            phy_linkchange();
        rt_thread_mdelay(1000); // 每隔1秒检测一次PHY状态
    }
}

int rt_hw_dm9000_init(void) {
    /* stm32 hal lib dm9000 specific init */
    rt_uint32_t temp;
    DM9000_FMC_Config(); // FMC配置

    /* general dm9000 init */
    rt_sem_init(&sem_ack, "tx_ack", 0, RT_IPC_FLAG_FIFO);   // 同步信号量，初始为0，发送tx后等待中断释放信号量表示tx完成
    rt_sem_init(&sem_lock, "eth_lock", 1, RT_IPC_FLAG_FIFO); // 互斥信号量，初始为1，用于保护tx和rx过程不冲突
    
    dm9000_device.type  = TYPE_DM9000A;
    dm9000_device.mode  = DM9000_AUTO;
    dm9000_device.packet_cnt = 0;
    dm9000_device.queue_packet_len = 0;

    /*
     * SRAM Tx/Rx pointer automatically return to start address,
     * Packet Transmitted, Packet Received
     */
    // temp = *(volatile rt_uint16_t*)(0x1FFFF7E8);                //获取STM32的唯一ID的前24位作为MAC地址后三字节
    temp = HAL_GetUIDw0();
    dm9000_device.dev_addr[0] = 0x02;
    dm9000_device.dev_addr[1] = 0x00;
    dm9000_device.dev_addr[2] = 0x00;
    dm9000_device.dev_addr[3] = (temp >> 16) & 0xFF;    //低三字节用STM32的唯一ID
    dm9000_device.dev_addr[4] = (temp >> 8) & 0xFFF;
    dm9000_device.dev_addr[5] = temp  &0xFF;

    dm9000_device.parent.parent.init       = rt_dm9000_init;
    dm9000_device.parent.parent.open       = rt_dm9000_open;
    dm9000_device.parent.parent.close      = rt_dm9000_close;
    dm9000_device.parent.parent.read       = rt_dm9000_read;
    dm9000_device.parent.parent.write      = rt_dm9000_write;
    dm9000_device.parent.parent.control    = rt_dm9000_control;
    dm9000_device.parent.parent.user_data  = RT_NULL;

    dm9000_device.parent.eth_rx  = rt_dm9000_rx;
    dm9000_device.parent.eth_tx  = rt_dm9000_tx;
    
    eth_device_init(&(dm9000_device.parent), "e0");

    rt_thread_t tid;
    tid = rt_thread_create("phy", phy_state_check, RT_NULL, 512, RT_THREAD_PRIORITY_MAX - 2, 2);
    rt_thread_startup(tid);

    return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_dm9000_init);

void dm9000a(void)
{
    rt_kprintf("\n");
    rt_kprintf("NCR   (%02X): %02x\n", DM9000_NCR,   dm9000_io_read(DM9000_NCR));
    rt_kprintf("NSR   (%02X): %02x\n", DM9000_NSR,   dm9000_io_read(DM9000_NSR));
    rt_kprintf("TCR   (%02X): %02x\n", DM9000_TCR,   dm9000_io_read(DM9000_TCR));
    rt_kprintf("TSRI  (%02X): %02x\n", DM9000_TSR1,  dm9000_io_read(DM9000_TSR1));
    rt_kprintf("TSRII (%02X): %02x\n", DM9000_TSR2,  dm9000_io_read(DM9000_TSR2));
    rt_kprintf("RCR   (%02X): %02x\n", DM9000_RCR,   dm9000_io_read(DM9000_RCR));
    rt_kprintf("RSR   (%02X): %02x\n", DM9000_RSR,   dm9000_io_read(DM9000_RSR));
    rt_kprintf("ORCR  (%02X): %02x\n", DM9000_ROCR,  dm9000_io_read(DM9000_ROCR));
    rt_kprintf("CRR   (%02X): %02x\n", DM9000_CHIPR, dm9000_io_read(DM9000_CHIPR));
    rt_kprintf("CSCR  (%02X): %02x\n", DM9000_CSCR,  dm9000_io_read(DM9000_CSCR));
    rt_kprintf("RCSSR (%02X): %02x\n", DM9000_RCSSR, dm9000_io_read(DM9000_RCSSR));
    rt_kprintf("ISR   (%02X): %02x\n", DM9000_ISR,   dm9000_io_read(DM9000_ISR));
    rt_kprintf("IMR   (%02X): %02x\n", DM9000_IMR,   dm9000_io_read(DM9000_IMR));
    rt_kprintf("pin_G_6: %d\n", rt_pin_read(GET_PIN(G, 6)));
    rt_kprintf("\n");
}

#ifdef RT_USING_FINSH
#include <finsh.h>
MSH_CMD_EXPORT(dm9000a, dm9000a register dump);
#endif
