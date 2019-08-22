#include "follow.h"
#include "gimbal.h"

#define     DBG_SECTION_NAME    "follow"
#define     DBG_LEVEL           DBG_LOG
#include <rtdbg.h>

#define MQ_MSG_SIZE     sizeof(struct pos_data)
#define MQ_MSG_MAX      16
rt_mq_t pos_mq = RT_NULL;

// Thread
#define THREAD_STACK_SIZE      512
#define THREAD_PRIORITY        ((RT_THREAD_PRIORITY_MAX / 3) + 4)
#define THREAD_TICK            10
static rt_thread_t tid_follow = RT_NULL;

#define POS_X_MIDDLE    500
#define POS_Y_MIDDLE    500

// PID
static float follow_phi_kp = 0.02f;
static float follow_theta_kp = 0.03f;

// Gimbal
#define GIMBAL_SERVO_ANGLE      200.0f
#define GIMBAL_DEV_PHI          "pwm1"
#define GIMBAL_DEV_PHI_CH       4
#define GIMBAL_DEV_THETA        "pwm4"
#define GIMBAL_DEV_THETA_CH     1
static gimbal_t gimbal;
static float angle_phi = 100;
static float angle_theta = 97;

extern float stof(const char *s);

void follow_thread_entry(void *param)
{
    struct pos_data pos;
    float bias_x;
    float bias_y;

    while (1)
    {
        rt_mq_recv(pos_mq, &pos, MQ_MSG_SIZE, RT_WAITING_FOREVER);
        bias_x = -follow_theta_kp * (POS_X_MIDDLE - pos.x);
        bias_y = -follow_phi_kp * (POS_Y_MIDDLE - pos.y);
        angle_phi += bias_y;
        angle_theta += bias_x;
        rt_kprintf("bias-x-y: %d %d\n", (int)(bias_x), (int)(bias_y));
        rt_kprintf("angle %d %d\n", (int)(angle_phi), (int)(angle_theta));
        gimbal_set_angle(gimbal, angle_theta, angle_phi);
    }
}

int follow_init(void)
{
    pos_mq = rt_mq_create("follow", MQ_MSG_SIZE, MQ_MSG_MAX, RT_IPC_FLAG_FIFO);
    if (pos_mq == RT_NULL)
    {
        LOG_E("Failed to create message queue");
        return RT_ERROR;
    }

    tid_follow = rt_thread_create("follow", follow_thread_entry, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TICK);
    if (tid_follow == RT_NULL)
    {
        LOG_E("Failed to create thread");
        return RT_ERROR;
    }

    servo_t servo_x = servo_create(GIMBAL_DEV_PHI, GIMBAL_DEV_PHI_CH, GIMBAL_SERVO_ANGLE, RT_NULL, RT_NULL);
    servo_t servo_z = servo_create(GIMBAL_DEV_THETA, GIMBAL_DEV_THETA_CH, GIMBAL_SERVO_ANGLE, RT_NULL, RT_NULL);
    gimbal = gimbal_create(servo_x, servo_z);
    if (gimbal == RT_NULL)
    {
        LOG_E("Failed to create gimbal");
        return RT_ERROR;
    }
    gimbal_enable(gimbal);
    gimbal_set_angle(gimbal, angle_theta, angle_phi);
    LOG_I("gimbal enable");

    rt_thread_startup(tid_follow);
    LOG_I("thread startup");
}

static void follow(int argc, char *argv[])
{
    if (argc < 2)
        return;

    if (!rt_strcmp("init", argv[1]))
    {
        follow_init();
    }

    if (argc < 4)
    {
        return;
    }
    if (!rt_strcmp("set_kp", argv[1]))
    {
        float theta_kp = stof(argv[2]);
        float phi_kp = stof(argv[3]);
        follow_theta_kp = theta_kp;
        follow_phi_kp = phi_kp;
        rt_kprintf("theta_kp:%d phi_kp:%d factor:%d\n", (int)(theta_kp*1000),  (int)(phi_kp*1000), 1000);
    }
    if (!(rt_strcmp(argv[1], "set_gimbal_angle")))
    {
        gimbal_set_angle(gimbal, stof(argv[2]), stof(argv[3]));
        rt_kprintf("angle:%d %d\n", (int)stof(argv[2]), (int)stof(argv[3]));
    }
}
MSH_CMD_EXPORT(follow, follow test);
