/*********************************************************************************************************************
 * TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC377 开源库的一部分
 *
 * TC377 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          cpu0_main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.10.2
 * 适用平台          TC377TP
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-11-03       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"

#include "stm32_icm20948.h"
#include "bsp_log.h"
#include "servo.h"
#include "vmc.h"

#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// *腿部实例定义
servo_s servo1;
servo_s servo2;
servo_s servo3;
servo_s servo4;

vmc_leg_s leg_l;
vmc_leg_s leg_r;

// **************************** 代码区域 ****************************

void BWRInit(void)
{
    // *腿部关节初始化
    vmcInit(&leg_l, &servo4, &servo3);
    leg_l.theta1_bias = 0;
    leg_l.theta2_bias = -0.2;
    leg_l.theta1_lim[0] = PI_2;
    leg_l.theta1_lim[1] = PI3_2;
    leg_l.theta2_lim[0] = PI_2;
    leg_l.theta2_lim[1] = PI3_2;

    vmcInit(&leg_r, &servo1, &servo2);
    leg_r.theta1_bias = 0;
    leg_r.theta2_bias = 0;
    leg_r.theta1_lim[0] = PI_2;
    leg_r.theta1_lim[1] = PI3_2;
    leg_r.theta2_lim[0] = PI_2;
    leg_r.theta2_lim[1] = PI3_2;

    servoInit(&servo1, ATOM1_CH7_P20_11, 250, 1250, 180, 0, 0);
    servoInit(&servo2, ATOM1_CH5_P20_9, 250, 1220, 180, 0, 0);
    servoInit(&servo3, ATOM1_CH0_P15_6, 250, 1220, 180, 0, 0);
    servoInit(&servo4, ATOM1_CH6_P20_10, 250, 1250, 180, 0, 0);

    // wheelMotorInit(&motor_l, 0x144, MOTOR_DIRECTION_NORMAL);
    // wheelMotorInit(&motor_r, 0x141, MOTOR_DIRECTION_REVERSE);
}

void cc60_pit_ch0_isr_calllback()
{
    //*腿舵机控制
    vmcAntiSol(&leg_l, 0.1, 0);
    vmcAntiSol(&leg_r, 0.1, 0);
    vmcToPos(&leg_l);
    vmcToPos(&leg_r);
}

int core0_main(void)
{
    clock_init(); // 获取时钟频率<务必保留>

    system_delay_ms(2000);

    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);
    // uart_init(UART_3, 115200, UART3_TX_P21_7, UART3_RX_P21_6);

    BWRInit();

    gpio_init(P22_2, GPO, GPIO_HIGH, GPO_PUSH_PULL); // cs_high
    gpio_init(P22_1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // miso_low
    soft_iic_init(&icm20948_iic, ICM_I2C_ADDR_REVB, 100, P22_3, P23_1);
//    gpio_init(P15_7, GPO, GPIO_HIGH, GPO_PUSH_PULL); // cs_high
//    gpio_init(P20_12,  GPO, GPIO_HIGH, GPO_PUSH_PULL); // miso_low
//    soft_iic_init(&icm20948_iic, ICM_I2C_ADDR_REVB, 100, P20_11, P20_14);
    system_delay_ms(200);
    ICM20948_init(icmSettings);

    cpu_wait_event_ready(); // 等待所有核心初始化完毕
    system_delay_ms(200);
    while (TRUE)
    {
        ICM20948_task();
         LOGINFO("%f,%f,%f\n", angl_.yaw, angl_.pit, angl_.rol);
        // LOGINFO("%f,%f,%f\n", acce.x, acce.y, acce.z);
//         LOGINFO("%f,%f,%f\n", gyr_.x, gyr_.y, gyr_.z);
//        LOGINFO("%f,%f,%f,%f\n", quat.x, quat.y, quat.z, quat.w);
        system_delay_ms(100);

        //        gpio_set_level(P32_4,GPIO_HIGH);
        //        gpio_set_level(P23_1,GPIO_HIGH);
        //        system_delay_ms(100);
        //        gpio_set_level(P32_4,GPIO_LOW);
        //        gpio_set_level(P23_1,GPIO_LOW);
        ;
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
