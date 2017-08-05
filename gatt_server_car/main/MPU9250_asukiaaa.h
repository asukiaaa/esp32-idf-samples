#ifndef _MPU9250_ASUKIAAA_H_
#define _MPU9250_ASUKIAAA_H_
#include <stdio.h>
#include "driver/i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define MPU9250_ADDRESS_AD0_LOW  0x68
#define MPU9250_ADDRESS_AD0_HIGH 0x69
#define MPU9250_MAG_ADDRESS      0x0C

#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G       0x00
#define ACC_FULL_SCALE_4_G       0x08
#define ACC_FULL_SCALE_8_G       0x10
#define ACC_FULL_SCALE_16_G      0x18

#define AK8963_RA_HXL                   0x03
#define AK8963_RA_CNTL1                 0x0A
#define AK8963_RA_ASAX                  0x10

#define AK8963_MODE_POWERDOWN           0x0
#define AK8963_MODE_SINGLE              0x1
#define AK8963_MODE_CONTINUOUS_8HZ      0x2
#define AK8963_MODE_EXTERNAL            0x4
#define AK8963_MODE_CONTINUOUS_100HZ    0x6
#define AK8963_MODE_SELFTEST            0x8
#define AK8963_MODE_FUSEROM             0xF

typedef struct {
  int16_t magXOffset, magYOffset, magZOffset;
  uint8_t address;
  uint8_t accelBuf[14];
  uint8_t magBuf[7];
  uint8_t magXAdjust, magYAdjust, magZAdjust;
  uint8_t sdaPin, sclPin;
} mpu9250_t;

void mpu9250_begin(mpu9250_t *data);
void mpu9250_mag_begin(mpu9250_t *data);
void mpu9250_mag_update(mpu9250_t *data);
int16_t mpu9250_mag_x(mpu9250_t *data);
int16_t mpu9250_mag_y(mpu9250_t *data);
int16_t mpu9250_mag_z(mpu9250_t *data);

#endif
