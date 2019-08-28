#include <rtthread.h>
#include <chassis.h>
#include <command.h>
#include <single_pwm_motor.h>
#include <ab_phase_encoder.h>
#include <inc_pid_controller.h>
#include <pos_pid_controller.h>
#include <ps2.h>
#include <ano.h>

#define DBG_SECTION_NAME  "car"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

// MOTOR
#define FL_MOTOR_PWM                "pwm2"
#define FL_MOTOR_PWM_CHANNEL        2
#define FL_MOTOR_CTRL_PIN           10  // GET_PIN(A, 10)
#define FR_MOTOR_PWM                "pwm2"
#define FR_MOTOR_PWM_CHANNEL        3
#define FR_MOTOR_CTRL_PIN           8   // GET_PIN(A, 8)
#define BL_MOTOR_PWM                "pwm3"
#define BL_MOTOR_PWM_CHANNEL        1
#define BL_MOTOR_CTRL_PIN           21  // GET_PIN(B, 5)
#define BR_MOTOR_PWM                "pwm3"
#define BR_MOTOR_PWM_CHANNEL        2
#define BR_MOTOR_CTRL_PIN           9   // GET_PIN(A, 9)

// ENCODER
#define FL_ENCODER_PIN_A            32
#define FL_ENCODER_PIN_B            33
#define FR_ENCODER_PIN_A            36
#define FR_ENCODER_PIN_B            36
#define BL_ENCODER_PIN_A            36
#define BL_ENCODER_PIN_B            36
#define BR_ENCODER_PIN_A            36
#define BR_ENCODER_PIN_B            36
#define ENCODER_SAMPLE_TIME         20

// CONTROLLER PID
#define PID_SAMPLE_TIME             20
#define PID_PARAM_KP                7.0f
#define PID_PARAM_KI                1.2f
#define PID_PARAM_KD                0.5f

// WHEEL
#define WHEEL_RADIUS                0.060
#define GEAR_RATIO                  30

#define PULSE_PER_REVOL             1320      // Real value 1320

// CAR
chassis_t chas;

#define WHEEL_DIST_X                0.165
#define WHEEL_DIST_Y                0.210

// PS2
#define PS2_CS_PIN                  42// C10
#define PS2_CLK_PIN                 43// C11
#define PS2_DI_PIN                  44// C12
#define PS2_DO_PIN                  50 // D2

// Car Thread
#define THREAD_PRIORITY             10
#define THREAD_STACK_SIZE           512
#define THREAD_TIMESLICE            5

static rt_thread_t tid_car = RT_NULL;

void car_thread(void *param)
{
    // TODO

    struct velocity target_velocity;

    target_velocity.linear_x = 0.00f;
    target_velocity.linear_y = 0;
    target_velocity.angular_z = 0;
    chassis_set_velocity(chas, target_velocity);

    // Open loop control
    controller_disable(chas->c_wheels[0]->w_controller);
    controller_disable(chas->c_wheels[1]->w_controller);
    controller_disable(chas->c_wheels[2]->w_controller);
    controller_disable(chas->c_wheels[3]->w_controller);

    // motor_disable(chas->c_wheels[1]->w_motor);
    // motor_disable(chas->c_wheels[2]->w_motor);
    // motor_disable(chas->c_wheels[3]->w_motor);

    // Sender
    command_sender_t ano_sender = ano_get_sender();
    struct cmd_rpy rpy;
    pos_pid_controller_t pos_pid = (pos_pid_controller_t)chas->c_wheels[0]->w_controller;

    while (1)
    {
        rt_thread_mdelay(20);
        chassis_update(chas);
        rpy.roll = chas->c_wheels[0]->rpm;
        rpy.pitch = chas->c_wheels[0]->w_controller->target;
        rpy.yaw = pos_pid->integral;
        command_send(ano_sender, COMMAND_SEND_RPY, &rpy, sizeof(struct cmd_rpy));
    }
//    chassis_destroy(chas);
}

void car_init(void *parameter)
{
    // 1. Initialize two wheels - left and right
    wheel_t* c_wheels = (wheel_t*) rt_malloc(sizeof(wheel_t) * 2);
    if (c_wheels == RT_NULL)
    {
        LOG_D("Failed to malloc memory for wheels");
    }

    // 1.1 Create four motors
    single_pwm_motor_t fl_motor = single_pwm_motor_create(FL_MOTOR_PWM, FL_MOTOR_PWM_CHANNEL, FL_MOTOR_CTRL_PIN, 0xFF);
    single_pwm_motor_t fr_motor = single_pwm_motor_create(FR_MOTOR_PWM, FR_MOTOR_PWM_CHANNEL, 0xFF, FR_MOTOR_CTRL_PIN);
    single_pwm_motor_t bl_motor = single_pwm_motor_create(BL_MOTOR_PWM, BL_MOTOR_PWM_CHANNEL, BL_MOTOR_CTRL_PIN, 0xFF);
    single_pwm_motor_t br_motor = single_pwm_motor_create(BR_MOTOR_PWM, BR_MOTOR_PWM_CHANNEL, 0xFF, BR_MOTOR_CTRL_PIN);

    // 1.2 Create four encoders
    ab_phase_encoder_t fl_encoder = ab_phase_encoder_create(FL_ENCODER_PIN_A, FL_ENCODER_PIN_B, PULSE_PER_REVOL, ENCODER_SAMPLE_TIME);
    ab_phase_encoder_t fr_encoder = ab_phase_encoder_create(FR_ENCODER_PIN_A, FR_ENCODER_PIN_B, PULSE_PER_REVOL, ENCODER_SAMPLE_TIME);
    ab_phase_encoder_t bl_encoder = ab_phase_encoder_create(BL_ENCODER_PIN_A, BL_ENCODER_PIN_B, PULSE_PER_REVOL, ENCODER_SAMPLE_TIME);
    ab_phase_encoder_t br_encoder = ab_phase_encoder_create(BR_ENCODER_PIN_A, BR_ENCODER_PIN_B, PULSE_PER_REVOL, ENCODER_SAMPLE_TIME);

    // 1.3 Create four pid contollers
    pos_pid_controller_t fl_pid = pos_pid_controller_create(PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD, PID_SAMPLE_TIME);
    pos_pid_controller_t fr_pid = pos_pid_controller_create(PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD, PID_SAMPLE_TIME);
    pos_pid_controller_t bl_pid = pos_pid_controller_create(PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD, PID_SAMPLE_TIME);
    pos_pid_controller_t br_pid = pos_pid_controller_create(PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD, PID_SAMPLE_TIME);

    // 1.4 Add four wheels
    c_wheels[0] = wheel_create((motor_t)fl_motor, (encoder_t)fl_encoder, (controller_t)fl_pid, WHEEL_RADIUS, GEAR_RATIO);
    c_wheels[1] = wheel_create((motor_t)fr_motor, (encoder_t)fr_encoder, (controller_t)fr_pid, WHEEL_RADIUS, GEAR_RATIO);
    c_wheels[2] = wheel_create((motor_t)bl_motor, (encoder_t)bl_encoder, (controller_t)bl_pid, WHEEL_RADIUS, GEAR_RATIO);
    c_wheels[3] = wheel_create((motor_t)br_motor, (encoder_t)br_encoder, (controller_t)br_pid, WHEEL_RADIUS, GEAR_RATIO);

    // 2. Iinialize Kinematics
    kinematics_t c_kinematics = kinematics_create(MECANUM, WHEEL_DIST_X, WHEEL_DIST_Y, WHEEL_RADIUS);

    // 3. Initialize Chassis
    chas = chassis_create(c_wheels, c_kinematics);

    // 4. Enable Chassis
    chassis_enable(chas);

    // Command
    command_init(chas);

    ps2_init(PS2_CS_PIN, PS2_CLK_PIN, PS2_DO_PIN, PS2_DI_PIN);

    // thread
    tid_car = rt_thread_create("tcar",
                              car_thread, RT_NULL,
                              THREAD_STACK_SIZE,
                              THREAD_PRIORITY, THREAD_TIMESLICE);

    if (tid_car != RT_NULL)
    {
        rt_thread_startup(tid_car);
    }
}
