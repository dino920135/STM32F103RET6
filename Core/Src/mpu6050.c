#include "mpu6050.h"

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

static HAL_StatusTypeDef read_u8(MPU6050_Device *dev, uint8_t reg, uint8_t *val)
{
  return HAL_I2C_Mem_Read(dev->i2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}

static HAL_StatusTypeDef write_u8(MPU6050_Device *dev, uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(dev->i2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static void set_scale_factors(MPU6050_Device *dev, MPU6050_AccelFS afs, MPU6050_GyroFS gfs)
{
  switch (afs) {
    case MPU6050_ACCEL_FS_2G:  dev->accel_g_per_lsb = 1.0f / 16384.0f; break;
    case MPU6050_ACCEL_FS_4G:  dev->accel_g_per_lsb = 1.0f / 8192.0f;  break;
    case MPU6050_ACCEL_FS_8G:  dev->accel_g_per_lsb = 1.0f / 4096.0f;  break;
    case MPU6050_ACCEL_FS_16G: dev->accel_g_per_lsb = 1.0f / 2048.0f;  break;
  }
  switch (gfs) {
    case MPU6050_GYRO_FS_250DPS:  dev->gyro_dps_per_lsb = 1.0f / 131.0f;   break;
    case MPU6050_GYRO_FS_500DPS:  dev->gyro_dps_per_lsb = 1.0f / 65.5f;   break;
    case MPU6050_GYRO_FS_1000DPS: dev->gyro_dps_per_lsb = 1.0f / 32.8f;   break;
    case MPU6050_GYRO_FS_2000DPS: dev->gyro_dps_per_lsb = 1.0f / 16.4f;   break;
  }
}

HAL_StatusTypeDef mpu6050_detect(MPU6050_Device *dev, I2C_HandleTypeDef *i2c)
{
  dev->i2c = i2c;
  uint8_t who = 0;

  dev->address = (0x68 << 1);
  if (read_u8(dev, MPU6050_REG_WHO_AM_I, &who) == HAL_OK && who == 0x68)
  {
    /* Default scale factors if user never calls configure */
    set_scale_factors(dev, MPU6050_ACCEL_FS_2G, MPU6050_GYRO_FS_250DPS);
    return HAL_OK;
  }

  dev->address = (0x69 << 1);
  if (read_u8(dev, MPU6050_REG_WHO_AM_I, &who) == HAL_OK && who == 0x68)
  {
    set_scale_factors(dev, MPU6050_ACCEL_FS_2G, MPU6050_GYRO_FS_250DPS);
    return HAL_OK;
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef mpu6050_wake(MPU6050_Device *dev)
{
  /* Use PLL with X axis gyroscope as clock source for better stability */
  return write_u8(dev, MPU6050_REG_PWR_MGMT_1, 0x01);
}

HAL_StatusTypeDef mpu6050_configure(MPU6050_Device *dev,
                                    MPU6050_AccelFS accel_fs,
                                    MPU6050_GyroFS gyro_fs,
                                    uint8_t dlpf_cfg,
                                    uint8_t smplrt_div)
{
  /* DLPF and sample rate */
  if (write_u8(dev, MPU6050_REG_CONFIG, (uint8_t)(dlpf_cfg & 0x07)) != HAL_OK) return HAL_ERROR;
  if (write_u8(dev, MPU6050_REG_SMPLRT_DIV, smplrt_div) != HAL_OK) return HAL_ERROR;

  /* Full-scale ranges: bits 4:3 for gyro and accel configs */
  uint8_t gcfg = (uint8_t)((gyro_fs & 0x03) << 3);
  uint8_t acfg = (uint8_t)((accel_fs & 0x03) << 3);
  if (write_u8(dev, MPU6050_REG_GYRO_CONFIG, gcfg) != HAL_OK) return HAL_ERROR;
  if (write_u8(dev, MPU6050_REG_ACCEL_CONFIG, acfg) != HAL_OK) return HAL_ERROR;

  set_scale_factors(dev, accel_fs, gyro_fs);
  return HAL_OK;
}

HAL_StatusTypeDef mpu6050_read_raw(MPU6050_Device *dev,
                                   int16_t *ax, int16_t *ay, int16_t *az,
                                   int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t raw[14] = {0};
  if (HAL_I2C_Mem_Read(dev->i2c, dev->address, MPU6050_REG_ACCEL_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 200) != HAL_OK)
  {
    return HAL_ERROR;
  }
  *ax = (int16_t)((raw[0] << 8) | raw[1]);
  *ay = (int16_t)((raw[2] << 8) | raw[3]);
  *az = (int16_t)((raw[4] << 8) | raw[5]);
  *temp = (int16_t)((raw[6] << 8) | raw[7]);
  *gx = (int16_t)((raw[8] << 8) | raw[9]);
  *gy = (int16_t)((raw[10] << 8) | raw[11]);
  *gz = (int16_t)((raw[12] << 8) | raw[13]);
  return HAL_OK;
}

HAL_StatusTypeDef mpu6050_read_f(MPU6050_Device *dev,
                                 float *ax_g, float *ay_g, float *az_g,
                                 float *temp_c,
                                 float *gx_dps, float *gy_dps, float *gz_dps)
{
  int16_t ax, ay, az, t, gx, gy, gz;
  if (mpu6050_read_raw(dev, &ax, &ay, &az, &t, &gx, &gy, &gz) != HAL_OK)
  {
    return HAL_ERROR;
  }
  *ax_g = ax * dev->accel_g_per_lsb;
  *ay_g = ay * dev->accel_g_per_lsb;
  *az_g = az * dev->accel_g_per_lsb;
  *temp_c = (float)t / 340.0f + 36.53f;
  *gx_dps = gx * dev->gyro_dps_per_lsb;
  *gy_dps = gy * dev->gyro_dps_per_lsb;
  *gz_dps = gz * dev->gyro_dps_per_lsb;
  return HAL_OK;
}
