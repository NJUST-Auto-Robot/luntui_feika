#ifndef __FOC_H
#define __FOC_H

#include "zf_common_headfile.h"
#include "math.h"
#include "string.h"


#define _PI_2  1.57079632679
#define _PI    3.14159265359
#define _3PI_2 4.71238898038
#define _2PI   6.28318530718
#define _round(x) ((x) >= 0 ? (uint16_t)((x) + 0.5) : (uint16_t)((x) - 0.5))

#define sqr3        1.732050808
#define one_sqr3    0.5773502692
#define pi          3.1415926

#define DutyAmplitude   10000   //ռ�ձȵķ�ֵ����ARR
#define VBAT            12      //�����ѹ V
#define I_to_V          7

#define StanderedVoltage    3.3
#define AdcCycle            4096
#define AdcVoltageOffset    2048    //AdcCycle/2
#define SampleResistor      0.01    //��λ��ŷķ
#define OperationalAmplify  20      //�˷ŷŴ���

//����õ��ǶȵĻص�����(����ֵΪ������ԭʼֵ)
#define Angle_read()            1
#define PoleNum                 7       //������(������һ��)
#define MechanicalAngleOffset   312     //��λ/�� ���ڶ༶�Ե�����£����ڶ����λƫ��(�����Ƕ���λ)��ȡ����һ������  180
#define EncoderCycle            4096    //2^12


typedef struct FOC_MOTOR
{
    uint8_t ID;
    uint8_t spin_direction; //0ͣת 1��ת 2��ת

    float voltage_info[3];  //
    float voltage_control[3];
    float current_q;
    float current_d;
    float current_alpha;
    float current_beta;
    float voltage_q;
    float voltage_d;
    float voltage_alpha;
    float voltage_beta;

    float DutyA;
    float DutyB;
    float DutyC;

    uint16_t angle_raw;     //ԭʼ����
    float machanical_theta; //deg
    float electrical_theta; //rad

    uint8_t foc_sector;

}FOC_MOTOR;

void FOC_init(FOC_MOTOR *motor,uint8_t ID);
void Foc_iq_control(FOC_MOTOR *motor, float q);
void svpwm_test(FOC_MOTOR *motor, float spd_rad);

#endif

