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


void ecat_init(void);
void ecat_loop(void);
void ctrl_state(void);

#endif

