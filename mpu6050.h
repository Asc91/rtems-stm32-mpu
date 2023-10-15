#ifndef __MPU6050_h
#define __MPU6050_h

#include <bsp.h>
#include <bsp/i2c.h>


#define I2C_MASTER_SCL_IO PB6       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO PB7       /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */

#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_CLOCK_PLL_XGYRO 0x01

#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_GYRO_FS_250 0x00

#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_FS_2 0x00

#define MPU6050_ADDR 0x68    /*!< slave address for mpu6050 sensor */
#define ACCE_START_ADDR 0x3B /*!< accelerometer start address */
#define GYRO_START_ADDR 0x43 /*!< gyroscope start address */

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

#define ALPHA 0.9834
#define RAD_TO_DEG 57.2957795
#define BUFF_SIZE 6

#define MPU_CALIBRATION_AVG_COUNT 1024 // *todo Random number for now


rtems_status_code read_reg(stm32f4_i2c_bus_entry *e, stm32f4_i2c_message *msg, uint8_t reg, uint8_t *buf);

rtems_status_code write_reg(stm32f4_i2c_bus_entry *e, stm32f4_i2c_message *msg, uint8_t reg, uint8_t *buf);

rtems_status_code mpu6050_init();

rtems_status_code mpu_read_gyro(uint8_t *buf);

#endif