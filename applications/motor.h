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

#endif /*__MOTOR_H__*/