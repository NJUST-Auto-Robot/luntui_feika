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

// 计算常量
#define PI 3.14
#define PI2 6.28
#define PI_2 1.57
#define PI3_2 4.71

// 腿部常量定义
#define BWRLA 0.018 //关节电机与中心点的距离
#define BWRLU 0.06  //关节的腿长
#define BWRLD 0.09  //轮子上的腿长

#define BWR_MB 0.400 // 车体重量
#define BWR_MW 0.136  // 轮子重量
#define BWR_RW 0.0325 // 轮子半径
#define BWR_D 0.15   // 两轮之间的距离
#define BWR_H 0.15   // 车体的高度
#define BWR_W 0.1    // 车体的宽度

#define BWR_L 0 // 质心据车体中心的距离
#define GRAVITY 9.81

#define BWR_JP BWR_MB *BWR_L *BWR_L * 2 / 3
#define BWR_JD BWR_MB *BWR_D *BWR_D / 12
#define BWR_I BWR_MW *BWR_RW *BWR_RW 2 / 2


#endif