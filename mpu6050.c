#include "mpu6050.h"

uint8_t *data;
uint8_t *last;

#define rtems_vector 1

static stm32f4_i2c_bus_entry e = {
    .regs = STM32F4_I2C1,
    .index = 0,
    .vector = rtems_vector,
    .mutex = 1,
    .task_id = 1,
    .data = &data,
    .last = &last,
    .len = 8,

};

static stm32f4_i2c_message msg = {
    .addr = MPU6050_ADDR,
    .len = 8,
};

rtems_status_code read_reg(stm32f4_i2c_bus_entry *e, stm32f4_i2c_message *msg, uint8_t reg, uint8_t *buf)
{
    msg->buf = reg;
    msg->read = false;
    stm32f4_i2c_process_message(&e, &msg);

    msg->buf = &buf;
    msg->read = true;
    stm32f4_i2c_process_message(&e, &msg);

    return RTEMS_SUCCESSFUL;
}

rtems_status_code write_reg(stm32f4_i2c_bus_entry *e, stm32f4_i2c_message *msg, uint8_t reg, uint8_t *buf)
{
    msg->buf = reg;
    msg->read = false;
    stm32f4_i2c_process_message(&e, &msg);

    msg->buf = &buf;
    msg->read = false;
    stm32f4_i2c_process_message(&e, &msg);

    return RTEMS_SUCCESSFUL;
}

/** @brief Initialise MPU */
rtems_status_code mpu6050_init()
{
    rtems_status_code sc = stm32f4_i2c_init(&e);
    

    // set clock source
    uint8_t value;
    read_reg(&e, &msg, MPU6050_RA_PWR_MGMT_1, &value);
    value = value & 0xf8;
    value = value | MPU6050_CLOCK_PLL_XGYRO;
    write_reg(&e, &msg, MPU6050_RA_PWR_MGMT_1, &value);

    // Set fullscale gyro range
    value = MPU6050_GYRO_FS_250;
    write_reg(&e, &msg, MPU6050_RA_GYRO_CONFIG, &value);

    // Set fullscale accel range
    value = MPU6050_ACCEL_FS_2;
    write_reg(&e, &msg, MPU6050_RA_ACCEL_CONFIG, &value);

    // Set Sleep Mode to false
    value = 0x00;
    read_reg(&e, &msg, MPU6050_RA_PWR_MGMT_1, &value);
    value = value & 0x9f; // also check 0xbf
    write_reg(&e, &msg, MPU6050_RA_PWR_MGMT_1, &value);

    return RTEMS_SUCCESSFUL;
}

rtems_status_code mpu_read_gyro(uint8_t *buf)
{
    return read_reg(&e, &msg, GYRO_START_ADDR, buf);
}
