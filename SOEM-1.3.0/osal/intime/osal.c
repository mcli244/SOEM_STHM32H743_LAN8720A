/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#include <osal.h>
#include <time.h>
#include "tim.h"

sys_timer_t sys_timer = {0};

#define  up3d_timercmp(a, b, CMP)                                \
  (((a)->usecH == (b)->usecH) ?                           \
   ((a)->usecL CMP (b)->usecL) :                        \
   ((a)->usecH CMP (b)->usecH))
#define  up3d_timeradd(a, b, result)                             \
  do {                                                      \
    (result)->usecH = (a)->usecH + (b)->usecH;           \
    (result)->usecL = (a)->usecL + (b)->usecL;        \
    if ((result)->usecL >= USECS_PER_HIGH)                       \
    {                                                       \
       ++(result)->usecH;                                  \
       (result)->usecL -= USECS_PER_HIGH;                        \
    }                                                       \
  } while (0)
#define  up3d_timersub(a, b, result)                             \
  do {                                                      \
    (result)->usecH = (a)->usecH - (b)->usecH;           \
    (result)->usecL = (a)->usecL - (b)->usecL;        \
    if ((result)->usecL < 0) {                            \
      --(result)->usecH;                                   \
      (result)->usecL += USECS_PER_HIGH;                         \
    }                                                       \
  } while (0)


void osal_getSysTime (sys_timer_t *timer)
{
    timer->usecH =  GetSec();
	timer->usecL =  GetUSec();
}

uint64_t osal_getSysTime_us(void)
{
	return GetSec() * USECS_PER_HIGH + GetUSec();
}

void osal_timer_start (osal_timert * self,uint32_t timeout_usec)
{  
   sys_timer_t start_time;
   sys_timer_t timeout;
   sys_timer_t stop_time;

   osal_getSysTime (&start_time);
   
   timeout.usecH = timeout_usec / USECS_PER_HIGH;
   timeout.usecL = timeout_usec % USECS_PER_HIGH;
   up3d_timeradd (&start_time, &timeout, &stop_time);

   self->stop_time.usecH = stop_time.usecH;
   self->stop_time.usecL = stop_time.usecL;
}

boolean osal_timer_is_expired (osal_timert * self)
{  
    sys_timer_t current_time;
    sys_timer_t stop_time;
    int is_not_yet_expired;

    osal_getSysTime (&current_time);
    stop_time.usecH = self->stop_time.usecH;
    stop_time.usecL = self->stop_time.usecL;
    is_not_yet_expired = up3d_timercmp (&current_time, &stop_time, <);

   return is_not_yet_expired == FALSE;
}

int osal_usleep (uint32 usec){
   osal_timert qtime;
   osal_timer_start(&qtime, usec);

   while(!osal_timer_is_expired(&qtime));
   return 1;
}




