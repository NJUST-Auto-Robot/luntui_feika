#include "servo.h"

/**
 * @brief
 * @param servo
 * @param htim
 * @param TIM_CHANNEL
 * @param remap_deg_s 0°对应的duty值
 * @param remap_deg_e 90°/180°/270°对应的duty值
 */
void servoInit(servo_s *servo, pwm_channel_enum PWM_CHANNEL, float remap_deg_s, float remap_deg_e, int degrang, int degmode, int polority)
{
    memset(servo, 0, sizeof(servo_s));
    servo->PWM_CHANNEL = PWM_CHANNEL;
    servo->remap_deg_s = remap_deg_s;
    servo->remap_deg_e = remap_deg_e;
    servo->degmode = degmode;
    servo->degrang = degrang;
    servo->polarity = polority;

    pwm_init(servo->PWM_CHANNEL, 50, 0);
}

/**
 * @description:
 * @param {servo_s} *servo
 * @param {float} deg
 */
uint32_t servoSetDeg(servo_s *servo, float deg)
{
    if (deg < 0)
    {
        deg = 0;
        servo->deg = 0;
    }

    if (servo->degmode == 1)
    {
        if (deg > servo->degrang)
        {
            deg = servo->degrang;
        }
        servo->deg = deg;

        deg = servo->polarity == 0 ? deg : servo->degrang - deg;
        deg = (int)(deg * 100) % 36000;
    }
    else
    {
        if (deg > servo->degrang * 6.28 / 360)
        {
            deg = servo->degrang * 6.28 / 360;
        }
        servo->deg = deg;

        deg = servo->polarity == 0 ? deg : servo->degrang * 6.28 / 360 - deg;
        deg = (int)(deg * 1000) % 6280;
    }

    return deg;
}

/**
 * @description:
 * @param {servo_s} *servo
 * @param {float} deg
 */
void servoToDeg(servo_s *servo, float deg)
{
    float duty = 0;
    uint32_t deg_t = servoSetDeg(servo, deg);
    if (servo->degmode == 1)
        duty = remap(0, servo->degrang * 100, servo->remap_deg_s, servo->remap_deg_e, deg_t);
    else
        duty = remap(0, servo->degrang * 6.28 / 360 * 1000, servo->remap_deg_s, servo->remap_deg_e, deg_t);

    pwm_set_duty(servo->PWM_CHANNEL, duty);
}
