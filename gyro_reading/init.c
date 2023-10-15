
#include <stdio.h>
#include <bsp.h>

#include "../mpu6050.h"

rtems_task Init(rtems_task_argument argument)
{
    puts("\n\n*** MPU-GYRO ***");

    mpu6050_init();

    uint8_t angle = 0;

    while (1)
    {
        (void)rtems_task_wake_after(10);
        mpu_read_gyro(&angle);

        printf("angle = %d", angle);
    }

    (void)rtems_task_delete(RTEMS_SELF);
}


/**************** START OF CONFIGURATION INFORMATION ****************/

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER


#define CONFIGURE_MAXIMUM_TASKS         2
#define CONFIGURE_MAXIMUM_TIMERS        2

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_EXTRA_TASK_STACKS         (RTEMS_MINIMUM_STACK_SIZE)

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
/****************  END OF CONFIGURATION INFORMATION  ****************/