/*
 * @Author: Ptisak
 * @Date: 2023-07-09 15:17:41
 * @LastEditors: Ptisak
 * @LastEditTime: 2023-07-15 16:13:22
 * @Version: Do not edit
 */
#ifndef _STM32_ICM20948_H_
#define _STM32_ICM20948_H_

/*************************************************************************
  Defines
*************************************************************************/
#include "zf_common_headfile.h"

#include <stdbool.h>
#include <stdio.h>
#include "zf_driver_soft_iic.h"

#define ICM20948_UART
#define ICM20948_I2C hi2c1

#define AK0991x_DEFAULT_I2C_ADDR 0x0C   /* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR 0x0E /* The secondary I2C address for AK0991x Magnetometers */

#define ICM_I2C_ADDR_REVA 0x68 /* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB 0x69 /* I2C slave address for INV device on Rev B board */

// soft_spi_info_struct icm20948_spi = {{}};
extern soft_iic_info_struct icm20948_iic;

typedef struct
{
  int mode;
  bool enable_gyroscope;
  bool enable_accelerometer;
  bool enable_magnetometer;
  bool enable_quaternion;
  int gyroscope_frequency;
  int accelerometer_frequency;
  int magnetometer_frequency;
  int quaternion_frequency;

} STM32ICM20948Settings;
extern STM32ICM20948Settings icmSettings;
typedef struct
{
  float x;
  float y;
  float z;
} xyz_data_t;
typedef struct
{
  float w;
  float x;
  float y;
  float z;
} quaternion_data_t;
typedef struct
{
  float rol;
  float pit;
  float yaw;
} angle_data_t;

extern quaternion_data_t quat;
extern angle_data_t angl_;
extern xyz_data_t mag_;
extern xyz_data_t acce;
extern xyz_data_t gyr_;

/*************************************************************************
  Class
*************************************************************************/

void ICM20948_init(STM32ICM20948Settings settings);
void ICM20948_task();

// bool ICM20948_gyroDataIsReady();
// bool ICM20948_accelDataIsReady();
// bool ICM20948_magDataIsReady();
// bool ICM20948_quatDataIsReady();

// void ICM20948_readGyroData(float *x, float *y, float *z);
// void ICM20948_readAccelData(float *x, float *y, float *z);
// void ICM20948_readMagData(float *x, float *y, float *z);
// void ICM20948_readQuatData(float *w, float *x, float *y, float *z);
// void quat2euler(float q0, float q1, float q2, float q3, float *r, float *p, float *y);
void _quat2euler(quaternion_data_t *q, angle_data_t *a);
// void quat2euler(float q0, float q1, float q2, float q3, float *r, float *p, float *y);

#endif /* _STM32_ICM20948_H_ */
