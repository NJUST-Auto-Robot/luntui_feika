/*********************************************************************************************************************
 * TC377 Opensourec Library ����TC377 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� TC377 ��Դ���һ����
 *
 * TC377 ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          cpu0_main
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.10.2
 * ����ƽ̨          TC377TP
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2022-11-03       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"

#include "stm32_icm20948.h"
#include "bsp_log.h"
#include "servo.h"
#include "vmc.h"

#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// *�Ȳ�ʵ������
servo_s servo1;
servo_s servo2;
servo_s servo3;
servo_s servo4;

vmc_leg_s leg_l;
vmc_leg_s leg_r;

// **************************** �������� ****************************

void BWRInit(void)
{
    // *�Ȳ��ؽڳ�ʼ��
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
    //*�ȶ������
    vmcAntiSol(&leg_l, 0.1, 0);
    vmcAntiSol(&leg_r, 0.1, 0);
    vmcToPos(&leg_l);
    vmcToPos(&leg_r);
}

int core0_main(void)
{
    clock_init(); // ��ȡʱ��Ƶ��<��ر���>

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

    cpu_wait_event_ready(); // �ȴ����к��ĳ�ʼ�����
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
// **************************** �������� ****************************
