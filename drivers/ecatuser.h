#ifndef _ECATUSER_H_
#define _ECATUSER_H_


#include "ethercat.h"

#include "osal.h"
#define SYNC0TIME 1000 
///1000us

extern uint64 app_time_base;
extern uint64 ref_time_base;
extern uint64 sync_start_time;
extern int64 app_time_offset;

#if 0
typedef struct __packed 
{
   uint16 ControlWord;
   int32 TargetPos;
	 int32 TargetVelocity;
   uint8 TargetMode;
	 uint8 relese;
}PDO_Output;

typedef struct __packed  
{
   uint16 StatusWord;
   int32 CurrentPosition;
   int32 CurrentVelocity;
   uint8 CurrentMode;
	 uint8 relese;
}PDO_Input;
#endif

// PDO参数组 选择一个进行映射
// #define RPDO1_1600_TXPDO1_1A00
//#define RPDO2_1601_TXPDO2_1A01
// #define RPDO5_1604_TXPDO5_1A04
#define RPDO_TXPDO_PLC


#if defined(RPDO1_1600_TXPDO1_1A00)
typedef struct __packed 
{
    uint16_t ControlWord;        // 伺服控制字
    uint8_t TargetMode;    // 运行模式设定
    int32_t TargetPos;      // 目标位置
    uint32_t TouchProbeFunction; // 探针功能
} PDO_Output;

typedef struct __packed 
{
    uint16_t ErrorCode;
    uint16_t StatusWord;
    int32_t CurrentPosition;
    uint8_t TargetMode;
    uint16_t TouchProbeStatus;
    int32_t TouchProbePos1PosValue;
    int32_t FollowingErrorActualValue;
    uint32_t DigitalInputs;
    uint16_t ServoError;
} PDO_Input;
#endif

#if defined(RPDO2_1601_TXPDO2_1A01)
typedef struct __packed 
{
   /*
      0x60400010);	///ControlWord
      0x60600008);	///TargetMode
      0x60710010);	///TargetTorque
      0x607A0020);	///TargetPos
      0x60800020);	///MaximumContourSpeed
      0x60B80010);	///ProbeFunction
      0x60FF0020);	///TargetSpeed 
   */
   uint16_t ControlWord;   // 伺服控制字
   uint8_t TargetMode;    // 运行模式设定
   int16_t  TargetTorque;
   int32_t TargetPos;   
   int32_t MaximumContourSpeed;
   int16_t ProbeFunction;
   int32_t TargetSpeed;
} PDO_Output;

typedef struct __packed 
{
   /*
      0x603F0010);	///ErrorCode
      0x60410010);	///StatusWord
      0x60610008);	///TargetMode
      0x60640020);	///CurrentPosition
      0x606C0020);	///CurrentSpeed
      0x60770010);	///CurrentTorque
      0x60B90010);	///ProbeStatus
      0x60BA0020);	///Probe1Position 
      0x60BC0020);	///Probe2Position
      0x60FD0020);	///DIInputStatus 
   */
   uint16_t ErrorCode;
   uint16_t StatusWord;
   uint8_t TargetMode;
   int32_t CurrentPosition;
   int32_t CurrentSpeed;
   int16_t CurrentTorque;
   int16_t ProbeStatus;
   int32_t Probe1Position;
   int32_t Probe2Position;
   int32_t PositionDeviation;
   int32_t DIInputStatus;
} PDO_Input;
#endif

#if defined(RPDO5_1604_TXPDO5_1A04)
typedef struct __packed 
{
   /*
      0x60400010);	///ControlWord
      0x60600008);	///TargetMode
      0x60710010);	///TargetTorque
      0x607A0020);	///TargetPos
      0x607F0020);	///MaximumContourSpeed
      0x60B80010);	///ProbeFunction
      0x60E00010);	///ForwardMaximumTorqueLimit
      0x60E10010);	///NegativeMaximumTorqueLimit
      0x60FF0020);	///TargetSpeed 
   */
   uint16_t ControlWord;   // 伺服控制字
   uint8_t TargetMode;    // 运行模式设定
   int16_t  TargetTorque;
   int32_t TargetPos;   
   int32_t MaximumContourSpeed;
   int16_t ProbeFunction;
   int16_t ForwardMaximumTorqueLimit;
   int16_t NegativeMaximumTorqueLimit;
   int32_t TargetSpeed;
} PDO_Output;

typedef struct __packed 
{
   /*
      0x60410010);	///StatusWord
      0x60610008);	///TargetMode
      0x60640020);	///CurrentPosition
      0x606C0020);	///CurrentSpeed
      0x60770010);	///CurrentTorque
      0x60B90010);	///ProbeStatus
      0x60BA0020);	///Probe1Position 
      0x60BC0020);	///Probe2Position
      0x60F40020);	///PositionDeviation
      0x603F0010);	///ErrorCode 
      0x60FD0020);	///DIInputStatus 
   */
   uint16_t StatusWord;
   uint8_t CurrentMode;
   int32_t CurrentPosition;
   int32_t CurrentSpeed;
   int16_t CurrentTorque;
   int16_t ProbeStatus;
   int32_t Probe1Position;
   int32_t Probe2Position;
   int32_t PositionDeviation;
   uint16_t ErrorCode;
   uint32_t DIInputStatus;
} PDO_Input;
#endif


#if defined(RPDO_TXPDO_PLC)
typedef struct __packed 
{
   int16_t ControlWord;        // 伺服控制字
   int32_t TargetPos;      // 目标位置
   int8_t TargetMode;    // 运行模式设定
   int16_t  TargetTorque;
   int16_t TouchProbeFunction; // 探针功能
   int32_t TargetSpeed;
} PDO_Output;

typedef struct __packed 
{
   uint16_t StatusWord;
   int32_t CurrentPosition;
   uint8_t TargetMode;
   int16_t CurrentTorque;
   int32_t CurrentSpeed;
   uint16_t TouchProbeStatus;
   int32_t TouchProbe1FallingEdgePos; // 探针1下降沿位置反馈
   int32_t TouchProbe1RisingEdgePos;   // 探针1上升沿位置反馈
   int16_t ServoError;
} PDO_Input;
#endif

void ecat_init(void);
void ecat_loop(void);
void ctrl_state(void);



#endif

