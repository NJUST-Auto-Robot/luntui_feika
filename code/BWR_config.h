/*
 * @Author: skybase
 * @Date: 2024-12-22 17:18:45
 * @LastEditors: skybase
 * @LastEditTime: 2025-02-15 15:07:27
 * @Description:  ?(???)??
 * @FilePath: \luntui_feika\CODE\BWR_config.h
 */
#ifndef BWRCONF_H
#define BWRCONF_H

// ���㳣��
#define PI 3.14
#define PI2 6.28
#define PI_2 1.57
#define PI3_2 4.71

// �Ȳ���������
#define BWRLA 0.018 //�ؽڵ�������ĵ�ľ���
#define BWRLU 0.06  //�ؽڵ��ȳ�
#define BWRLD 0.09  //�����ϵ��ȳ�

#define BWR_MB 0.400 // ��������
#define BWR_MW 0.136  // ��������
#define BWR_RW 0.0325 // ���Ӱ뾶
#define BWR_D 0.15   // ����֮��ľ���
#define BWR_H 0.15   // ����ĸ߶�
#define BWR_W 0.1    // ����Ŀ��

#define BWR_L 0 // ���ľݳ������ĵľ���
#define GRAVITY 9.81

#define BWR_JP BWR_MB *BWR_L *BWR_L * 2 / 3
#define BWR_JD BWR_MB *BWR_D *BWR_D / 12
#define BWR_I BWR_MW *BWR_RW *BWR_RW 2 / 2


#endif