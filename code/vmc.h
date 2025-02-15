#ifndef VMC_H
#define VMC_H

#include "stdio.h"
#include "math.h"
#include "string.h"

#include "BWR_config.h"
#include "servo.h"

// !�ȵ���ؽڵ������λӦӳ��Ϊ"y��������"
// !�ȵ��ҹؽڵ������λӦӳ��Ϊ"y�Ḻ����"
typedef struct vmc_leg_s
{
    float theta1; // ��ʾ�ұߵĹؽڵ���н� rad
    float theta2; // ��ʾ��ߵĹؽڵ���н� rad

    float theta1_bias; // ƫ��ĽǶ�,���ڲ���Ӳ��
    float theta2_bias;

    float theta1_lim[2];    //��ʾ�ؽڵ���ĽǶ��޷�,��VMC��ģ��Ϊ��׼,[��ֵ,ĩֵ]
    float theta2_lim[2];

    float length;
    float theta; // ��ʾ�����복������

    servo_s *servo1; // �ұߵĹؽڵ��
    servo_s *servo2; // ��ߵĹؽڵ��

} vmc_leg_s;

void vmcInit(vmc_leg_s *leg, servo_s *servo1, servo_s *servo2);
void vmcAntiSol(vmc_leg_s *leg, float L, float theta);

void vmcToPos(vmc_leg_s *leg);

#endif
