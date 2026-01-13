#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

/**
  * @brief  PID 控制器结构体定义
  */
typedef struct {
    float Target;       // 目标值（ROS 2 下发的期望速度）
    float Kp, Ki, Kd;   // PID 参数
    float Error;        // 当前偏差
    float LastError;    // 上次偏差
    float Integral;     // 积分累加
} PID_Controller;

// 全局变量声明，方便 main.c 调用
extern PID_Controller PID_A, PID_B;

// 函数声明
void Motor_All_Init(void);                      // 所有硬件初始化
void MotorA_SetSpeed(int16_t duty);             // 电机 A 速度接口 (-100 到 100)
void MotorB_SetSpeed(int16_t duty);             // 电机 B 速度接口 (-100 到 100)
int16_t Encoder_GetA(void);                     // 获取电机 A 脉冲增量
int16_t Encoder_GetB(void);                     // 获取电机 B 脉冲增量
float PID_Update(PID_Controller *p, float actual); // PID 计算核心算法

#endif
// 结尾必须保留一个空行