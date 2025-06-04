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

/**
 * @description: 电机初始化
 * @return {*} 0 on success, negative value on failure.
 */
int motor_init(void);

/**
 * @description: 电机位置设置
 * @param {motor_pos_t} *pos = 增量式位置控制， 在当前位置基础上增加或减少位置
 * @return {*}  0 on success, negative value on failure.
 */
int motor_set(motor_pos_t *pos);

/**
 * @description: 电机当前位置信息获取
 * @param {motor_pos_t} *pos 电机当前位置信息
 * @return {*}  0 on success, negative value on failure.
 */
int motor_get(motor_pos_t *pos);

/**
 * @description: 电机反初始化
 * @return {*} 0 on success, negative value on failure.
 */
int motor_deinit(void);

/**
 * @description: EtherCAT 退出OP模式，转为安全操作模式。 注意：退出后才可以读写SDO
 * @return {*}0 on success, negative value on failure.
 */
int motor_ec_exit_op(void);

/**
 * @description: EtherCAT 进入OP模式， 开始发送周期PDO数据。
 * @return {*}0 on success, negative value on failure.
 */
int motor_ec_enter_op(void);


/**
 * @description: EtherCAT SDO读操作
 * @param {uint16} Slave 从站地址
 * @param {uint16} Index 索引
 * @param {uint8} SubIndex 子索引
 * @param {boolean} CA 是否使用通信参数
 * @param {int} psize 数据大小
 * @param {void*} p 数据指针
 * @param {int} Timeout 超时时间
 * @return {*} 0 on success, negative value on failure.
 */
int motor_ec_SDOread(uint16 Slave, uint16 Index, uint8 SubIndex,
                     boolean CA, int *psize, void *p, int Timeout);
                     
/**
 * @description: EtherCAT SDO写操作
 * @param {uint16} Slave 从站地址
 * @param {uint16} Index 索引
 * @param {uint8} SubIndex 子索引
 * @param {boolean} CA 是否使用通信参数
 * @param {int} psize 数据大小
 * @param {void*} p 数据指针
 * @param {int} Timeout 超时时间
 * @return {*} 0 on success, negative value on failure.
 */
int motor_ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex,
                      boolean CA, int psize, void *p, int Timeout);

#endif /*__MOTOR_H__*/