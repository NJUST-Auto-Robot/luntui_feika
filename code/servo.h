/*
 * @Author: skybase
 * @Date: 2024-11-07 22:48:13
 * @LastEditors: skybase
 * @LastEditTime: 2025-02-15 17:45:12
 * @Description:  ?(???)??
 * @FilePath: \25_feika_luntui\code\servo.h
 */
#ifndef SERVO_H
#define SERVO_H

#include "m_math.h"
#include "zf_common_headfile.h"

typedef struct servo_s
{
    float deg;
    int degmode;  //  modeΪ0:����Ϊ������, modeΪ1:����Ϊ�Ƕ���
    int degrang;  // �������Ϊ90��/180��/270��
    int polarity; // ����ļ���,���ڽ�����ת����,0:���� 1:��ת

    pwm_channel_enum PWM_CHANNEL;

    float remap_deg_s;//��ʼ�Ƕ�
    float remap_deg_e;//�����Ƕ�

} servo_s;

void servoInit(servo_s *servo, pwm_channel_enum PWM_CHANNEL, float remap_deg_s, float remap_deg_e, int degrang, int degmode, int polority);
uint32_t servoSetDeg(servo_s *servo, float deg);
void servoToDeg(servo_s *servo, float deg);

#endif
