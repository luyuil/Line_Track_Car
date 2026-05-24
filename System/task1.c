#include "headfile.h"

// 外部变量声明
extern PID_t line;          // 菜单可调，其 Kp/Ki/Kd 用于 yaw 增量式 PID
extern int32_t speed_l;     // 左轮编码器（Encoder1）
extern int32_t speed_r;     // 右轮编码器（Encoder2）

// ========== 速度环 PID 参数（两轮共用，需要根据实际编码器标定）==========
#define SPEED_KP            0.42f
#define SPEED_KI            0.33f
#define SPEED_KD            0.11f

// 目标速度：编码器每控制周期的期望计数值，跑起来看 OLED 上的典型值再填
#define SPEED_TARGET        20.0f

// 角度环 / 速度环输出限幅
#define YAW_OUT_MAX         50.0f
#define YAW_OUT_MIN        -50.0f
#define SPEED_OUT_MAX       90.0f
#define SPEED_OUT_MIN      -90.0f

// PWM 限幅
#define PWM_MAX             100.0f
#define PWM_MIN            -100.0f

// 文件级静态变量：重置标志
static uint8_t needs_reset = 1;

void line_running_reset(void)
{
    needs_reset = 1;
}

void line_running(void)
{
    // ========== 增量式 PID 静态状态（跨调用保持）==========
    // 角度环
    static float yaw_error0 = 0.0f, yaw_error1 = 0.0f, yaw_error2 = 0.0f;
    static float yaw_out    = 0.0f;

    // 左轮速度环（跟踪 speed_l）
    static float L_error0 = 0.0f, L_error1 = 0.0f, L_error2 = 0.0f;
    static float L_speed_out = 0.0f;

    // 右轮速度环（跟踪 speed_r）
    static float R_error0 = 0.0f, R_error1 = 0.0f, R_error2 = 0.0f;
    static float R_speed_out = 0.0f;

    // ========== 任务重启时清空所有 PID 状态 ==========
    if (needs_reset)
    {
        yaw_error0 = 0.0f; yaw_error1 = 0.0f; yaw_error2 = 0.0f;
        yaw_out    = 0.0f;

        L_error0 = 0.0f; L_error1 = 0.0f; L_error2 = 0.0f;
        L_speed_out = 0.0f;

        R_error0 = 0.0f; R_error1 = 0.0f; R_error2 = 0.0f;
        R_speed_out = 0.0f;

        needs_reset = 0;
    }

    // ===== 1. Yaw 角增量式 PID（保持 yaw = 0）=====
    // 增益从菜单可调的 line 结构体读取
    float yaw_Kp = line.Kp;
    float yaw_Ki = line.Ki;
    float yaw_Kd = line.Kd;

    yaw_error2 = yaw_error1;
    yaw_error1 = yaw_error0;
    yaw_error0 = 0.0f - imu_angle.yaw;    // Target(0) - Actual(yaw)

    yaw_out += yaw_Kp * (yaw_error0 - yaw_error1)
             + yaw_Ki * yaw_error0
             + yaw_Kd * (yaw_error0 - 2.0f * yaw_error1 + yaw_error2);

    if (yaw_out > YAW_OUT_MAX)  yaw_out = YAW_OUT_MAX;
    if (yaw_out < YAW_OUT_MIN)  yaw_out = YAW_OUT_MIN;

    // ===== 2. 左轮速度增量式 PID（跟踪 SPEED_TARGET）=====
    L_error2 = L_error1;
    L_error1 = L_error0;
    L_error0 = SPEED_TARGET - (float)speed_l;

    L_speed_out += SPEED_KP * (L_error0 - L_error1)
                + SPEED_KI * L_error0
                + SPEED_KD * (L_error0 - 2.0f * L_error1 + L_error2);

    if (L_speed_out > SPEED_OUT_MAX)  L_speed_out = SPEED_OUT_MAX;
    if (L_speed_out < SPEED_OUT_MIN)  L_speed_out = SPEED_OUT_MIN;

    // ===== 3. 右轮速度增量式 PID（跟踪 SPEED_TARGET）=====
    R_error2 = R_error1;
    R_error1 = R_error0;
    R_error0 = SPEED_TARGET - (float)speed_r;

    R_speed_out += SPEED_KP * (R_error0 - R_error1)
                + SPEED_KI * R_error0
                + SPEED_KD * (R_error0 - 2.0f * R_error1 + R_error2);

    if (R_speed_out > SPEED_OUT_MAX)  R_speed_out = SPEED_OUT_MAX;
    if (R_speed_out < SPEED_OUT_MIN)  R_speed_out = SPEED_OUT_MIN;

    // ===== 4. 合成 PWM（你的原始架构）=====
    // Motor1(左轮) = 左速度环输出 + yaw修正
    // Motor2(右轮) = 右速度环输出 - yaw修正
    // yaw > 0(右偏) → yaw_error0 < 0 → yaw_out 减小(负方向)
    //   → motor1 = L_out + (负) = 左轮减速
    //   → motor2 = R_out - (负) = 右轮加速 → 车左转纠正
    // 【实测如果越跑越偏，互换下面两行的 ±yaw_out】
    float motor1_pwm = L_speed_out + yaw_out;
    float motor2_pwm = R_speed_out - yaw_out;

    // 最终输出限幅
    if (motor1_pwm >  PWM_MAX) motor1_pwm =  PWM_MAX;
    if (motor1_pwm <  PWM_MIN) motor1_pwm =  PWM_MIN;
    if (motor2_pwm >  PWM_MAX) motor2_pwm =  PWM_MAX;
    if (motor2_pwm <  PWM_MIN) motor2_pwm =  PWM_MIN;

    Motor1_SetPWM((int8_t)motor1_pwm);
    Motor2_SetPWM((int8_t)motor2_pwm);
}
