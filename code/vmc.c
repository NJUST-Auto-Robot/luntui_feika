/*
 * @Author: skybase
 * @Date: 2025-02-15 14:51:11
 * @LastEditors: skybase
 * @LastEditTime: 2025-02-15 14:52:10
 * @Description:  ?(???)?? 
 * @FilePath: \luntui_feika\CODE\vmc.c
 */
#include "vmc.h"


void vmcInit(vmc_leg_s *leg, servo_s *servo1, servo_s *servo2)
{
    leg->servo1 = servo1;
    leg->servo2 = servo2;
}

void vmcAntiSol(vmc_leg_s *leg, float L, float theta)
{
    float l1 = pow((BWRLA * BWRLA + L * L + 2 * BWRLA * L * sin(theta)), 0.5);
    float l2 = pow((BWRLA * BWRLA + L * L - 2 * BWRLA * L * sin(theta)), 0.5);
    float theta1 = acos((l1 * l1 + BWRLA * BWRLA - L * L) / (2 * l1 * BWRLA)) + acos((l1 * l1 + BWRLU * BWRLU - BWRLD * BWRLD) / (2 * l1 * BWRLU));
    float theta2 = acos((l2 * l2 + BWRLA * BWRLA - L * L) / (2 * l2 * BWRLA)) + acos((l2 * l2 + BWRLU * BWRLU - BWRLD * BWRLD) / (2 * l2 * BWRLU));

    float t1, t2;
    t2 = (theta2 <= leg->theta2_lim[0]) ? leg->theta2_lim[0] : theta2;
    t2 = (theta2 >= leg->theta2_lim[1]) ? leg->theta2_lim[1] : theta2;
    t1 = (theta1 <= leg->theta1_lim[0]) ? leg->theta1_lim[0] : theta1;
    t1 = (theta1 >= leg->theta1_lim[1]) ? leg->theta1_lim[1] : theta1;

    leg->theta1 = t1;
    leg->theta2 = t2;
}

void vmcToPos(vmc_leg_s *leg)
{

    servoToDeg(leg->servo1, leg->theta2 + leg->theta2_bias - PI_2);
    servoToDeg(leg->servo2, PI3_2 - leg->theta1 + leg->theta1_bias);
}