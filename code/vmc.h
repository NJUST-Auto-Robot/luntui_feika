#ifndef VMC_H
#define VMC_H

#include "stdio.h"
#include "math.h"
#include "string.h"

#include "BWR_config.h"
#include "servo.h"

// !腿的左关节电机的零位应映射为"y轴正半轴"
// !腿的右关节电机的零位应映射为"y轴负半轴"
typedef struct vmc_leg_s
{
    float theta1; // 表示右边的关节电机夹角 rad
    float theta2; // 表示左边的关节电机夹角 rad

    float theta1_bias; // 偏差的角度,用于补偿硬件
    float theta2_bias;

    float theta1_lim[2];    //表示关节电机的角度限幅,以VMC的模型为基准,[初值,末值]
    float theta2_lim[2];

    float length;
    float theta; // 表示轮子与车体的倾角

    servo_s *servo1; // 右边的关节电机
    servo_s *servo2; // 左边的关节电机

} vmc_leg_s;

void vmcInit(vmc_leg_s *leg, servo_s *servo1, servo_s *servo2);
void vmcAntiSol(vmc_leg_s *leg, float L, float theta);

void vmcToPos(vmc_leg_s *leg);

#endif
