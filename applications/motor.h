/*
 * @Author       : lixiangjun@up3d.com
 * @Date         : 2025-05-29
 * @FilePath     : motor.h
 * @Description  : 电机控制头文件
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__
#include <stdint.h>

typedef struct  
{
   int32_t x;
   int32_t y;
   int32_t z;
   int32_t a;
   int32_t b;
} motor_pos_t;

/** 电机初始化      
 * 
 * @return                = 0 on success, negative value on failure.
 */
int motor_init(void);

/** 电机位置设置  
 * 
 * @param[in] pos         = 增量式位置控制， 在当前位置基础上增加或减少位置
 * @return                = 0 on success, negative value on failure.
 */
int motor_set(motor_pos_t *pos);

/** 电机当前位置信息获取    
 * 
 * @param[in] pos         = 电机当前位置信息
 * @return                = 0 on success, negative value on failure.
 */
int motor_get(motor_pos_t *pos);


/** 电机反初始化      
 * 
 * @return                = 0 on success, negative value on failure.
 */
int motor_deinit(void);


/** EtherCAT 退出OP模式，转为安全操作模式。 
 *     
 * @return                = 0 on success, negative value on failure.
 * @note 
 *    退出后才可以读写SDO
 */
int motor_ec_exit_op(void);


/** EtherCAT 进入OP模式，开始发送周期PDO数据。
 *         
 * @return                = 0 on success, negative value on failure.
 */
int motor_ec_enter_op(void);

/** EtherCAT SDO写操作
 *
 * @param[in]  Slave      = 从站地址
 * @param[in]  Index      = 索引
 * @param[in]  SubIdx     = 子索引
 * @param[in]  CA         = CA 是否使用通信参数， 一般FALSE
 * @param[in]  psize      = 数据大小
 * @param[out] p          = 数据指针
 * @param[in]  Timeout    = Timeout 超时时间, 单位us
 * @return                = 0 on success, negative value on failure.
 */
int motor_ec_SDOread(uint16 Slave, uint16 Index, uint8 SubIndex,
                     boolean CA, int *psize, void *p, int Timeout);
                     
/** EtherCAT SDO写操作
 *
 * @param[in]  Slave      = 从站地址
 * @param[in]  Index      = 索引
 * @param[in]  SubIdx     = 子索引
 * @param[in]  CA         = CA 是否使用通信参数， 一般FALSE
 * @param[in]  psize      = 数据大小
 * @param[in]  p          = 数据指针
 * @param[in]  Timeout    = Timeout 超时时间, 单位us
 * @return                = 0 on success, negative value on failure.
 */
int motor_ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex,
                      boolean CA, int psize, void *p, int Timeout);

#endif /*__MOTOR_H__*/