#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  MPU6050_ACCEL_FS_2G  = 0,  /* 16384 LSB/g */
  MPU6050_ACCEL_FS_4G  = 1,  /* 8192  LSB/g */
  MPU6050_ACCEL_FS_8G  = 2,  /* 4096  LSB/g */
  MPU6050_ACCEL_FS_16G = 3   /* 2048  LSB/g */
} MPU6050_AccelFS;

typedef enum {
  MPU6050_GYRO_FS_250DPS  = 0,  /* 131 LSB/°/s */
  MPU6050_GYRO_FS_500DPS  = 1,  /* 65.5 */
  MPU6050_GYRO_FS_1000DPS = 2,  /* 32.8 */
  MPU6050_GYRO_FS_2000DPS = 3   /* 16.4 */
} MPU6050_GyroFS;

typedef struct {
  I2C_HandleTypeDef *i2c;
  uint16_t address; /* 7-bit address left-shifted by 1 for HAL */
  float accel_g_per_lsb;
  float gyro_dps_per_lsb;
} MPU6050_Device;

HAL_StatusTypeDef mpu6050_detect(MPU6050_Device *dev, I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef mpu6050_wake(MPU6050_Device *dev);
HAL_StatusTypeDef mpu6050_read_raw(MPU6050_Device *dev, int16_t *ax, int16_t *ay, int16_t *az,
                                   int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz);

/* Optional: configure full-scale ranges and basic filtering/sampling. */
HAL_StatusTypeDef mpu6050_configure(MPU6050_Device *dev,
                                    MPU6050_AccelFS accel_fs,
                                    MPU6050_GyroFS gyro_fs,
                                    uint8_t dlpf_cfg,    /* 0..6, e.g. 3 */
                                    uint8_t smplrt_div); /* 0..255, sample = 1kHz/(1+div) */

/* Read and convert to physical units: accel in g, gyro in °/s, temp in °C */
HAL_StatusTypeDef mpu6050_read_f(MPU6050_Device *dev,
                                 float *ax_g, float *ay_g, float *az_g,
                                 float *temp_c,
                                 float *gx_dps, float *gy_dps, float *gz_dps);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
