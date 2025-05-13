#include "ecatuser.h"

#include "stdio.h"
#include "string.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"

#include "stm32h7xx_hal.h"
//#include "gpio.h"

#include "osal.h"
#include "ecatuser.h"
#include "tim.h"

uint64 app_time_base = 0;
uint64 ref_time_base = 0;
uint64 sync_start_time = 0;
int64 app_time_offset = 0;

PDO_Output *outputs1;
PDO_Input *inputs1;


char IOmap[200];
uint32_t dorun=0;
uint32_t OpenReady=0;

int oloop, iloop;

//motor control 
uint16 cur_status;
uint8_t startup_step=0;
int32 cur_pos = 0;
uint16 csp_pos_delay;
int cmdpos_raw;

#define DEBUG 1


static int drive_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
    int wkc = 0;
    wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    if(wkc <= 0)
    {
        printf("write8 failed slave:%d index:0x%x subindex:%d value:0x%x wkc:%d\r\n", slave, index, subindex, value, wkc);
    }
    return wkc;
}

static int drive_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
    int wkc = 0;
    wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    if(wkc <= 0)
    {
        printf("write16 failed slave:%d index:0x%x subindex:%d value:0x%x wkc:%d\r\n", slave, index, subindex, value, wkc);
    }
    return wkc;
}

static int drive_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value)
{
    int wkc = 0;
    wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    if(wkc <= 0)
    {
        printf("write32 failed slave:%d index:0x%x subindex:%d value:0x%x wkc:%d\r\n", slave, index, subindex, value, wkc);
    }
    return wkc;
}

int Servosetup(uint16 slave)
{	
	#if 0
	  int retval;
    uint16 u16val;
    uint8  u8val;
    uint32 u32val;
    retval = 0;

    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1600;	
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

	
	  u8val = 0;
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  u32val = 0x60400010;	///ControlWord
    retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x607A0020;	///TargetPos
    retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60FF0020;	///TargetVelocity
    retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60600008;	///ModeOfOperation
    retval += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x00000008;	
    retval += ec_SDOwrite(slave, 0x1600, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);	
		u8val = 5;
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
		
		
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1a00;
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  
		u8val = 0;
    retval += ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  u32val = 0x60410010;	////Status Word
    retval += ec_SDOwrite(slave, 0x1a00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60640020;	///ActualPosition
    retval += ec_SDOwrite(slave, 0x1a00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x606C0020;	///ActualVelocity
    retval += ec_SDOwrite(slave, 0x1a00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	 	u32val = 0x60610008;	//ModeOfOperationDisplay
    retval += ec_SDOwrite(slave, 0x1a00, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x00000008;	///
    retval += ec_SDOwrite(slave, 0x1a00, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);		
//		u32val = 0x00000008;	
//    retval += ec_SDOwrite(slave, 0x1a00, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);		
		u8val = 5;
    retval += ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
		
    return 1;
	#endif

	#ifdef RPDO1_1600_TXPDO1_1A00
	  int retval;
    uint16 u16val;
    uint8  u8val;
    uint32 u32val;
    retval = 0;

    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1600;	
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

	
	  u8val = 0;
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  u32val = 0x60400010;	///ControlWord
    retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60600008;	///TargetPos
    retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x607A0020;	///TargetVelocity
    retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60B80010;	///ModeOfOperation
    retval += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u8val = 4;
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
		
		
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1a00;
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  
		u8val = 0;
    retval += ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  u32val = 0x603F0010;	////Status Word
    retval += ec_SDOwrite(slave, 0x1a00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60410010;	///ActualPosition
    retval += ec_SDOwrite(slave, 0x1a00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x606C0020;	///ActualVelocity
    retval += ec_SDOwrite(slave, 0x1a00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	 	u32val = 0x60610008;	//ModeOfOperationDisplay
    retval += ec_SDOwrite(slave, 0x1a00, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60B90010;	///
    retval += ec_SDOwrite(slave, 0x1a00, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);		
		u32val = 0x60BA0020;	
   retval += ec_SDOwrite(slave, 0x1a00, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);	
   u32val = 0x60F40020;	
   retval += ec_SDOwrite(slave, 0x1a00, 0x07, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
   u32val = 0x60FD0020;	
   retval += ec_SDOwrite(slave, 0x1a00, 0x08, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);	
   u32val = 0x213F0010;	
   retval += ec_SDOwrite(slave, 0x1a00, 0x09, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u8val = 9;
    retval += ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	#endif
	
	#ifdef RPDO5_1604_TXPDO5_1A04
	#if 1
	int retval;
    uint16 u16val;
    uint8  u8val;
    uint32 u32val;
    retval = 0;

    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1603;	
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

	  u8val = 0;
    retval += ec_SDOwrite(slave, 0x1604, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  u32val = 0x60400010;	///ControlWord
    retval += ec_SDOwrite(slave, 0x1604, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60600008;	///TargetPos
    retval += ec_SDOwrite(slave, 0x1604, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60710010;	///TargetVelocity
    retval += ec_SDOwrite(slave, 0x1604, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x607A0020;	///ModeOfOperation
    retval += ec_SDOwrite(slave, 0x1604, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x607F0020;	///ModeOfOperation
    retval += ec_SDOwrite(slave, 0x1604, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60B80010;	///ModeOfOperation
    retval += ec_SDOwrite(slave, 0x1604, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60E00010;	///ModeOfOperation
    retval += ec_SDOwrite(slave, 0x1604, 0x07, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60E10010;	///ModeOfOperation
    retval += ec_SDOwrite(slave, 0x1604, 0x08, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60FF0020;	///ModeOfOperation
    retval += ec_SDOwrite(slave, 0x1604, 0x08, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u8val = 9;
    retval += ec_SDOwrite(slave, 0x1604, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1A01;
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	
		u8val = 0;
    retval += ec_SDOwrite(slave, 0x1A04, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  u32val = 0x60410010;	////Status Word
    retval += ec_SDOwrite(slave, 0x1A04, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60610008;	///ActualPosition
    retval += ec_SDOwrite(slave, 0x1A04, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60640020;	///ActualVelocity
    retval += ec_SDOwrite(slave, 0x1A04, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	 	u32val = 0x606C0020;	//ModeOfOperationDisplay
    retval += ec_SDOwrite(slave, 0x1A04, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60770010;	///
    retval += ec_SDOwrite(slave, 0x1A04, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);		
		u32val = 0x60B90010;	
   retval += ec_SDOwrite(slave, 0x1A04, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);	
   u32val = 0x60BA0020;	
   retval += ec_SDOwrite(slave, 0x1A04, 0x07, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
   u32val = 0x60BC0020;	
   retval += ec_SDOwrite(slave, 0x1A04, 0x08, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);	
   u32val = 0x60F40020;	
   retval += ec_SDOwrite(slave, 0x1A04, 0x09, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
   u32val = 0x603F0010;	
   retval += ec_SDOwrite(slave, 0x1A04, 0x0A, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
   u32val = 0x60FD0020;	
   retval += ec_SDOwrite(slave, 0x1A04, 0x0B, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u8val = 11;
    retval += ec_SDOwrite(slave, 0x1A04, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	#else
	drive_write8(slave, 0x1c12, 0, 0);
	drive_write16(slave, 0x1c12, 0, 0x1604);
	drive_write8(slave, 0x1c12, 0, 1);

	drive_write8(slave, 0x1604, 0, 0);
	drive_write32(slave, 0x1604, 1, 0x60400010);	///ControlWord
	drive_write32(slave, 0x1604, 2, 0x60600008);	///ModeOfOperation
	drive_write32(slave, 0x1604, 3, 0x60710010);	///TargetTorque
	drive_write32(slave, 0x1604, 4, 0x607A0020);	///TargetPos
	drive_write32(slave, 0x1604, 5, 0x607F0020);	///MaximumContourSpeed
	drive_write32(slave, 0x1604, 6, 0x60B80010);	///ProbeFunction
	drive_write32(slave, 0x1604, 7, 0x60E00010);	///ForwardMaximumTorqueLimit
	drive_write32(slave, 0x1604, 8, 0x60E10010);	///NegativeMaximumTorqueLimit
	drive_write32(slave, 0x1604, 9, 0x60FF0020);	///TargetSpeed 
	drive_write8(slave, 0x1604, 9, 0);

	drive_write8(slave, 0x1c13, 0, 0);
	drive_write16(slave, 0x1c13, 0, 0x1A04);
	drive_write8(slave, 0x1c13, 0, 1);

	drive_write8(slave, 0x1A04, 0, 0);
	drive_write32(slave, 0x1A04, 1,  0x60410010);	///StatusWord
	drive_write32(slave, 0x1A04, 2,  0x60610008);	///ModeOfOperation
	drive_write32(slave, 0x1A04, 3,  0x60640020);	///CurrentPosition
	drive_write32(slave, 0x1A04, 4,  0x606C0020);	///CurrentSpeed
	drive_write32(slave, 0x1A04, 5,  0x60770010);	///CurrentTorque
	drive_write32(slave, 0x1A04, 6,  0x60B90010);	///ProbeStatus
	drive_write32(slave, 0x1A04, 7,  0x60BA0020);	///Probe1Position 
	drive_write32(slave, 0x1A04, 8,  0x60BC0020);	///Probe2Position
	drive_write32(slave, 0x1A04, 9,  0x60F40020);	///PositionDeviation 
	drive_write32(slave, 0x1A04, 10, 0x603F0010);	///ErrorCode 
	drive_write32(slave, 0x1A04, 11, 0x60FD0020);	///DIInputStatus 
	drive_write8(slave, 0x1A04, 0, 11);
	#endif
	#endif

	#if defined(RPDO2_1601_TXPDO2_1A01)
	drive_write8(slave, 0x1c12, 0, 0);
	drive_write16(slave, 0x1c12, 0, 0x1601);
	drive_write8(slave, 0x1c12, 0, 1);

	drive_write8(slave, 0x1601, 0, 0);
	drive_write32(slave, 0x1601, 1, 0x60400010);	///ControlWord
	drive_write32(slave, 0x1601, 2, 0x60600008);	///ModeOfOperation
	drive_write32(slave, 0x1601, 3, 0x60710010);	///TargetTorque
	drive_write32(slave, 0x1601, 4, 0x607A0020);	///TargetPos
	drive_write32(slave, 0x1601, 5, 0x60800020);	///MaximumContourSpeed
	drive_write32(slave, 0x1601, 6, 0x60B80010);	///ProbeFunction
	drive_write32(slave, 0x1601, 7, 0x60FF0020);	///TargetSpeed 
	drive_write8(slave, 0x1601, 7, 0);

	drive_write8(slave, 0x1c13, 0, 0);
	drive_write16(slave, 0x1c13, 0, 0x1A01);
	drive_write8(slave, 0x1c13, 0, 1);

	drive_write8(slave, 0x1A01, 0, 0);
	drive_write32(slave, 0x1A01, 1, 0x603F0010);	///ErrorCode 
	drive_write32(slave, 0x1A01, 2,  0x60410010);	///StatusWord
	drive_write32(slave, 0x1A01, 3,  0x60610008);	///TargetMode
	drive_write32(slave, 0x1A01, 4,  0x60640020);	///CurrentPosition
	drive_write32(slave, 0x1A01, 5,  0x606C0020);	///CurrentSpeed
	drive_write32(slave, 0x1A01, 6,  0x60770010);	///CurrentTorque
	drive_write32(slave, 0x1A01, 7,  0x60B90010);	///ProbeStatus
	drive_write32(slave, 0x1A01, 8,  0x60BA0020);	///Probe1Position 
	drive_write32(slave, 0x1A01, 9,  0x60BC0020);	///Probe2Position
	drive_write32(slave, 0x1A01, 10, 0x60FD0020);	///DIInputStatus 
	drive_write8(slave, 0x1A01, 0, 10);
	#endif

	#if defined(RPDO_TXPDO_PLC)
	drive_write8(slave, 0x1c12, 0, 0);
	drive_write16(slave, 0x1c12, 0, 0x1600);
	drive_write8(slave, 0x1c12, 0, 1);

	drive_write8(slave, 0x1c13, 0, 0);
	drive_write16(slave, 0x1c13, 0, 0x1A00);
	drive_write8(slave, 0x1c13, 0, 1);

		/*
1	0x60410010	// ״̬�� (6041h)
2	0x60640020	// λ�÷��� (6064h)
3	0x60610008	// ����ģʽ��ʾ��6061h��
4	0x60770010	// ת�ط���ֵ (6077h)
5	0x606c0020	// �ٶȷ���ֵ (606Ch)
6	0x60b90010	// ̽��״̬ (60B9h)
7	0x60bb0020	// ̽�� 1 �½���λ�÷��� (60BBh)
8	0x60ba0020	// ̽�� 1 ������λ�÷��� (60BAh)
9	0x213f0010	// �ŷ��ڲ�������루213Fh��
	*/
	drive_write8(slave, 0x1A00, 0, 0);
	drive_write32(slave, 0x1A00, 1, 0x60410010);	
	drive_write32(slave, 0x1A00, 2,  0x60640020);	
	drive_write32(slave, 0x1A00, 3,  0x60610008);	
	drive_write32(slave, 0x1A00, 4,  0x60770010);	
	drive_write32(slave, 0x1A00, 5,  0x606C0020);	
	drive_write32(slave, 0x1A00, 6,  0x60b90010);	
	drive_write32(slave, 0x1A00, 7,  0x60bb0020);	
	drive_write32(slave, 0x1A00, 8,  0x60ba0020);	
	drive_write32(slave, 0x1A00, 9,  0x213f0010);	
	drive_write8(slave, 0x1A00, 0, 9);

		/*
1	0x60400010	// ������ (6040h)
2	0x607a0020	// Ŀ��λ�� (607Ah)
3	0x60600008	// ����ģʽ (6060h)
4	0x60710010	// Ŀ��ת�� (6071h)
5	0x60b80010	// ̽�빦�� (60B8h)
6	0x60ff0020	// Ŀ���ٶ� (60FFh)
	 */
	drive_write8(slave, 0x1600, 0, 0);
	drive_write32(slave, 0x1600, 1, 0x60400010);	
	drive_write32(slave, 0x1600, 2, 0x607a0020);	
	drive_write32(slave, 0x1600, 3, 0x60600008);	
	drive_write32(slave, 0x1600, 4, 0x60710010);	
	drive_write32(slave, 0x1600, 5, 0x60b80010);	
	drive_write32(slave, 0x1600, 6, 0x60ff0020);	
	drive_write8(slave, 0x1600, 0, 6);

	// drive_write8(slave, 0x6060, 0, 0x8);
	// drive_write16(slave, 0x6040, 0, 0x0086);
	// drive_write32(slave, 0x6099, 2, 0x0000000a);
	// drive_write32(slave, 0x6099, 1, 0x00000014);
	// drive_write8(slave, 0x6098, 0, 0x1);
	// drive_write16(slave, 0x6040, 0, 0x0086);
	// drive_write8(slave, 0x6098, 0, 0x1);
	// drive_write32(slave, 0x6099, 1, 0x00000014);
	// drive_write32(slave, 0x6099, 2, 0x0000000a);
	#endif

    return 1;
}

void ecat_init(void)
{
    int slc;
//    int i,chk;
    int cnt = 1;
		int expectedWKC;
    
    /* initialise SOEM, bind socket to ifname */
	if (ec_init("eth0"))
	{
		printf("ec_init succeeded.\r\n");
         HAL_Delay(100);
		if ( ec_config_init(TRUE) > 0 )
		{
			printf("%d slaves found and configured.\r\n",ec_slavecount);
            
			if ( ec_slavecount >= 1 ) 
			{
                for(slc = 1; slc <= ec_slavecount; slc++)
                 {
                     
                    for(slc = 1; slc <= ec_slavecount; slc++)
										{
											printf("Found %s at position %d\n", ec_slave[slc].name, slc);
											ec_slave[slc].PO2SOconfig = &Servosetup;
										}
                 }
            }
            
            ec_configdc();//DC???????
            ec_config_map(&IOmap);
            ec_dcsync0(1, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 1
//						ec_dcsync0(2, TRUE, SYNC0TIME, 0); // SYNC0 on slave 2
						
						printf("Slaves mapped, state to SAFE_OP.\n \r");	
						/* wait for all slaves to reach SAFE_OP state */
			      ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
						/* read indevidual slave state and store in ec_slave[] */
						ec_readstate();
						
						printf("Slave 0 State=0x%04x\r\n",ec_slave[0].state);
						printf("Slave 1 State=0x%04x\r\n",ec_slave[1].state);			
						
						for(cnt = 1; cnt <= ec_slavecount ; cnt++){
						printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n \r",
							cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
							ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
						}
            
						oloop = ec_slave[0].Obytes;//???????????????????????SSC-IO??2
						if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
						if (oloop > 30) oloop = 30;
			
						iloop = ec_slave[0].Ibytes;//???????????????????????SSC-IO??6		
						if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
						if (iloop > 30) iloop = 30;
			
						printf("oloop:%d iloop:%d\n\r",oloop,iloop);
						
						printf("segments : %d : %d %d %d %d\n \r",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);
						printf("Request operational state for all slaves\n \r");
						expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
						printf("Calculated workcounter %d\n \r", expectedWKC);
						
						/* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            
            ec_writestate(0);
            
						printf("DC capable : %d\r\n",ec_configdc());
            
            HAL_Delay(100);
            
					/* wait for all slaves to reach OP state */
						do{
							ec_slave[0].state = EC_STATE_OPERATIONAL;
							ec_slave[1].state = EC_STATE_OPERATIONAL;
              ec_writestate(0);
							ec_writestate(1);
							printf("-%d %d-",ec_slave[0].state,ec_slave[1].state);
							}
						while ( (ec_slave[0].state != EC_STATE_OPERATIONAL)||(ec_slave[1].state != EC_STATE_OPERATIONAL));
//						while  (ec_slave[0].state != EC_STATE_OPERATIONAL);
						printf("\r\n%d %d\r\n",ec_slave[0].state,ec_slave[1].state);
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                
                outputs1 = (PDO_Output *)ec_slave[1].outputs;
                inputs1  = (PDO_Input *)ec_slave[1].inputs;
                dorun = 1;

                printf("all slaves reached operational state.\r\n");
            }
			
							else
						{
							printf("Not all slaves reached operational state.\n \r");
						}
		}
		else
		{
			printf("No slaves found!\r\n");
		}
	}
	else
	{
		printf("No socket connection Excecute as root\r\n");
	}
    
}





uint8 wkc;
int32_t start_pos = 0;
int32_t step_increment = 10;
//1ms isr
//int nihao = 1;
void ecat_loop(void)
{
  static uint32 i = 0;
	static uint8 flag=0;

	
    if (dorun>0)
    {
				wkc=ec_receive_processdata(EC_TIMEOUTRET);
			//printf("%d ",wkc);
			cur_status = inputs1->StatusWord;//0x6041
			
			switch(startup_step)
			{
				case 1:
				  outputs1->ControlWord = 0x06;//0x6040
				  if((cur_status==0x0631)||(cur_status==0x0231)||(cur_status==0x0221)||(cur_status==0x1221))
						 startup_step=2;
//				  printf("0x06\n");
					break;
			  case 2:
					outputs1->ControlWord = 0x07;
					if((cur_status==0x633)||(cur_status==0x0233)||(cur_status==0x0223)||(cur_status==0x1223)||(cur_status==0x1233))
						 startup_step=3;
					start_pos = inputs1->CurrentPosition;
					outputs1->TargetPos = start_pos;
//					printf("0x07\n");
					break;
				case 3:
					outputs1->ControlWord = 0x0f;
					if((cur_status==0x1637)||(cur_status==0x1633)||(cur_status==0x1237))
				 	//if((cur_status==0x1637)||(cur_status==0x1633))
						 startup_step=4;
//						 printf("0x0f\n");
					break;
				case 4:
					 outputs1->ControlWord = 0x1f;	
					 outputs1->TargetPos += step_increment;
					break;
				default :
					startup_step=1;
				//   outputs1->ControlWord = 0x03;//0x6040
					outputs1->ControlWord = 0x80;//0x6040
				break;
			}		
	 outputs1->TargetMode = 0x8;
	 ec_send_processdata();	
		}

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

extern uint32_t dorun;
int show_flag = 0;
int _do_ecat_show_rdo(void)
{
	show_flag = 1;
    while (1)
    {
        if (dorun>0){
			rt_kprintf("state:%d; StatusWord:%x, CurrentPosition:%ld,TargetMode:%d CurrentTorque:%d CurrentSpeed:%d ServoError:%d\r\n",
			ec_slave[1].state, inputs1->StatusWord, inputs1->CurrentPosition,inputs1->TargetMode,
			inputs1->CurrentTorque, inputs1->CurrentSpeed, inputs1->ServoError);
        }
		if(show_flag == 0)
			break;
        rt_thread_mdelay(1);
    }

    return RT_EOK;
}
int ecat_show_rdo(void)
{
    rt_thread_t tid;
    tid = rt_thread_create("ecat_show_rdo",
            _do_ecat_show_rdo,
                           RT_NULL,
                           1024 * 1,
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
MSH_CMD_EXPORT(ecat_show_rdo, "ecat_show_rdo");

int ecat_show_stop(void)
{
    show_flag = 0;
    return 0;
}
MSH_CMD_EXPORT(ecat_show_stop, "ecat_show_stop");

void ecat_set_pos(int argc, char **argv)
{
    if (dorun == 0)
    {
        rt_kprintf("ecat not runnig\r\n");
        return ;
    }

    if (argc != 2)
    {
        rt_kprintf("Usage: ecat_set_pos step_increment\r\n");
        rt_kprintf("rg: ecat_set_pos 100\r\n");
        return ;
    }

    step_increment = atoi(argv[1]);
    rt_kprintf("ecat_set_pos step_increment:%d\r\n", step_increment);
}
MSH_CMD_EXPORT(ecat_set_pos, "ecat_set_pos");




