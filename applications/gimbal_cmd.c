#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdio.h>
#include <stdlib.h>
#include <gimbal.h>

static gimbal_t gimbal;

extern float stof(const char *s);

static void gimbal_init(void)
{
    servo_t servo_x = servo_create("pwm1", 4, 200, RT_NULL, RT_NULL);
    servo_t servo_z = servo_create("pwm4", 1, 200, RT_NULL, RT_NULL);
    gimbal = gimbal_create(servo_x, servo_z);
    gimbal_enable(gimbal);
    rt_kprintf("gimbal enable\n");
}

static void gimbal_test(int argc, char *argv[])
{
    if (argc < 2)
    {
        return;
    }
    if (!(rt_strcmp(argv[1], "init")))
    {
        gimbal_init();
    }

    if (argc < 4)
    {
        return;
    }
    if (!(rt_strcmp(argv[1], "angle")))
    {
        gimbal_set_angle(gimbal, stof(argv[2]), stof(argv[3]));
        rt_kprintf("angle:%d %d\n", (int)stof(argv[2]), (int)stof(argv[3]));
    }
}
MSH_CMD_EXPORT(gimbal_test, gimbal test);
