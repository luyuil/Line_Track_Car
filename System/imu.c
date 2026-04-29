#include "headfile.h"
#include <math.h>
#include <stdbool.h>

//-------------------------------------------------------------------------------------------------------------------
//  宏定义
//-------------------------------------------------------------------------------------------------------------------
float CYCLE_T                       =0.001f;                              // 三轴四元数采样周期
#define DELTA_T                     0.0010055f                              // 六轴四元数采样时间
#define IMU_ALPHA                   0.3f                                    // 加速度低通滤波系数
#define GYRO_STATIC_THRESHOLD       30.0f                                   // 原始角速度静止阈值
#define GYRO_STATIC_CHECK_COUNT      100                                    // 采集前静止判定次数
#define GYRO_OFFSET_SAMPLE_COUNT     2000                                   // 零漂采集次数

//-------------------------------------------------------------------------------------------------------------------
//  外部变量定义
//-------------------------------------------------------------------------------------------------------------------
IMU_Angle_typedef imu_angle;                                                // 各轴角度数据
icm_param_t       icm_data_t;                                               // ICM原始及处理后的数据
quater_param_t    Q_info = {1.0f, 0.0f, 0.0f, 0.0f};                        // 六轴位姿四元数

float             gyro_drift_x = 97.23f;                                     // 陀螺仪 X 轴零漂
float             gyro_drift_y = 91.23f;                                    // 陀螺仪 Y 轴零漂
float             gyro_drift_z = 9.23f;                                     // 陀螺仪 Z 轴零漂

float             param_Kp = 0.0001f;                                       // 加速度计收敛速率比例增益
float             param_Ki = -0.0000324f;                                   // 陀螺仪收敛速率积分增益

uint8_t             gyro_calibrate_flag = 1;                                  // 陀螺仪零漂采集触发标志
uint8_t             gyro_offset_finished = 0;                                 // 零漂采集完成标志

// mpu6050六轴数据
extern int16_t gx,gy,gz,ax,ay,az;

//-------------------------------------------------------------------------------------------------------------------
//  内部变量定义
//-------------------------------------------------------------------------------------------------------------------
static float      q_3dof[4] = {1.0f, 0.0f, 0.0f, 0.0f};                     // 初始单位四元数（三轴解算专用）
static float      I_ex = 0.0f, I_ey = 0.0f, I_ez = 0.0f;                    // 误差积分项
static float      vx = 0.0f, vy = 0.0f, vz = 0.0f;                          // 机体坐标系上的重力单位向量

int16_t             gyroscopeOffset[3] = {0};
static float        gyro_x_offset_sum = 0.0f;
static float        gyro_y_offset_sum = 0.0f;
static float        gyro_z_offset_sum = 0.0f;
static uint16_t     gyro_offset_count = 0;
static uint16_t     gyro_static_check_count = 0;
static uint8_t      gyro_offset_collecting = 0;

static float      last_raw_yaw = 0.0f;                                      // 上一次的原始 Yaw 角
static float      accumulated_yaw_drift = 0.0f;                             // 累积的死区误差
static bool       yaw_first_run = true;                                     // 第一次运行标志

bool              imu_flag = false;                                         // IMU 数据就绪标志

//-------------------------------------------------------------------------------------------------------------------
//  函数申明
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 MPU6050 加速度计数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的加速度计数据
// 返回参数     void
// 使用示例     float data = mpu6050_acc_transition(mpu6050_acc_x);                // 单位为 g(m/s^2)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float mpu6050_acc_transition (int16_t acc_value)
{
    float acc_data = 0;
    switch(MPU6050_ACC_SAMPLE)
    {
        case 0x00: acc_data = (float)acc_value / 16384; break;                  // 0x00 加速度计量程为:±2 g    获取到的加速度计数据 除以 16384      可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
        case 0x08: acc_data = (float)acc_value / 8192;  break;                  // 0x08 加速度计量程为:±4 g    获取到的加速度计数据 除以 8192       可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
        case 0x10: acc_data = (float)acc_value / 4096;  break;                  // 0x10 加速度计量程为:±8 g    获取到的加速度计数据 除以 4096       可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
        case 0x18: acc_data = (float)acc_value / 2048;  break;                  // 0x18 加速度计量程为:±16g    获取到的加速度计数据 除以 2048       可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
        default: break;
    }
    return acc_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 MPU6050 陀螺仪数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的陀螺仪数据
// 返回参数     void
// 使用示例     float data = mpu6050_gyro_transition(mpu6050_gyro_x);           // 单位为°/s
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float mpu6050_gyro_transition (int16_t gyro_value)
{
    float gyro_data = 0;
    switch(MPU6050_GYR_SAMPLE)
    {
        case 0x00: gyro_data = (float)gyro_value / 131.0f;  break;              // 0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以 131           可以转化为带物理单位的数据，单位为：°/s
        case 0x08: gyro_data = (float)gyro_value / 65.5f;   break;              // 0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以 65.5          可以转化为带物理单位的数据，单位为：°/s
        case 0x10: gyro_data = (float)gyro_value / 32.8f;   break;              // 0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以 32.8          可以转化为带物理单位的数据，单位为：°/s
        case 0x18: gyro_data = (float)gyro_value / 16.4f;   break;              // 0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以 16.4          可以转化为带物理单位的数据，单位为：°/s
        default: break;
    }
    return gyro_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     采集陀螺仪零漂并更新偏置
// 参数说明     void
// 返回参数     void
// 使用示例     gyro_offset_get();
// 备注信息     在 1ms 周期中断中触发，求取陀螺仪静止时的平均值作为零漂，并更新全局变量 gyro_drift_x/y/z
//-------------------------------------------------------------------------------------------------------------------
void gyro_offset_get(void)
{
    if (gyro_calibrate_flag)  
    {
        if (!gyro_offset_collecting)
        {
            if (fabsf((float)gx) < GYRO_STATIC_THRESHOLD
             && fabsf((float)gy) < GYRO_STATIC_THRESHOLD
             && fabsf((float)gz) < GYRO_STATIC_THRESHOLD)
            {
                gyro_static_check_count++;

                if (gyro_static_check_count >= GYRO_STATIC_CHECK_COUNT)
                {
                    gyro_offset_collecting = 1;
                    gyro_offset_finished = 0;
                    gyro_x_offset_sum = 0.0f;
                    gyro_y_offset_sum = 0.0f;
                    gyro_z_offset_sum = 0.0f;
                    gyro_offset_count = 0;
                }
            }
            else
            {
                gyro_calibrate_flag = 0;
                gyro_offset_finished = 1;
                gyro_static_check_count = 0;
                gyro_offset_collecting = 0;
            }

            return;
        }

        gyro_x_offset_sum += (float)gx;
        gyro_y_offset_sum += (float)gy;
        gyro_z_offset_sum += (float)gz;
        gyro_offset_count++;

        if (gyro_offset_count >= GYRO_OFFSET_SAMPLE_COUNT)
        {
            gyro_drift_x = gyro_x_offset_sum / (float)GYRO_OFFSET_SAMPLE_COUNT;
            gyro_drift_y = gyro_y_offset_sum / (float)GYRO_OFFSET_SAMPLE_COUNT;
            gyro_drift_z = gyro_z_offset_sum / (float)GYRO_OFFSET_SAMPLE_COUNT;

            gyroscopeOffset[0] = (int16_t)gyro_drift_x;
            gyroscopeOffset[1] = (int16_t)gyro_drift_y;
            gyroscopeOffset[2] = (int16_t)gyro_drift_z;

            gyro_x_offset_sum = 0.0f;
            gyro_y_offset_sum = 0.0f;
            gyro_z_offset_sum = 0.0f;
            gyro_offset_count = 0;
            gyro_static_check_count = 0;
            gyro_offset_collecting = 0;

            gyro_calibrate_flag = 0;
            gyro_offset_finished = 1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速平方根倒数
// 参数说明     x               输入值
// 返回参数     float           输出值 (1/sqrt(x))
// 使用示例     inv_sqrt = fast_sqrt(norm);
// 备注信息     Quake III 经典算法
//-------------------------------------------------------------------------------------------------------------------
float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取并预处理 ICM 传感器的原始数据
// 参数说明     void
// 返回参数     void
// 使用示例     ICM_getValues();
// 备注信息     包含加速度低通滤波及角速度弧度转换
//-------------------------------------------------------------------------------------------------------------------
void ICM_getValues(void)
{
    // 一阶低通滤波（加速度），数量级转换使用 imu660rb_acc_transition
    icm_data_t.acc_x = (-mpu6050_acc_transition((float)ax)) * IMU_ALPHA + icm_data_t.acc_x * (1.0f - IMU_ALPHA);
    icm_data_t.acc_y = ( mpu6050_acc_transition((float)ay)) * IMU_ALPHA + icm_data_t.acc_y * (1.0f - IMU_ALPHA);
    icm_data_t.acc_z = (-mpu6050_acc_transition((float)az)) * IMU_ALPHA + icm_data_t.acc_z * (1.0f - IMU_ALPHA);

    float gx_temp = (float)gx - gyro_drift_x;
    float gy_temp = (float)gy - gyro_drift_y;
    float gz_temp = (float)gz - gyro_drift_z;

    // 角速度死区处理
    if (gx_temp < GYRO_DEADZONE && gx_temp > -GYRO_DEADZONE) gx_temp = 0.0f;
    if (gy_temp < GYRO_DEADZONE && gy_temp > -GYRO_DEADZONE) gy_temp = 0.0f;
    if (gz_temp < GYRO_DEADZONE && gz_temp > -GYRO_DEADZONE) gz_temp = 0.0f;

    // 陀螺仪角速度转弧度，数量级转换使用 imu660rb_gyro_transition
    icm_data_t.gyro_x = (-mpu6050_gyro_transition(gx_temp)) * PI / 180.0f;
    icm_data_t.gyro_y = ( mpu6050_gyro_transition(gy_temp)) * PI / 180.0f;
    icm_data_t.gyro_z = (-mpu6050_gyro_transition(gz_temp)) * PI / 180.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     互补滤波 AHRS 姿态更新算法
// 参数说明     gx, gy, gz      陀螺仪角速度 (rad/s)
// 参数说明     ax, ay, az      加速度计数据 (m/s^2)
// 返回参数     void
// 使用示例     ICM_AHRSupdate(gx, gy, gz, ax, ay, az);
// 备注信息     更新全局四元数 Q_info
//-------------------------------------------------------------------------------------------------------------------
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfT = 0.5f * DELTA_T;
    float ex, ey, ez;
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;

    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取欧拉角 (基于六轴互补滤波)
// 参数说明     void
// 返回参数     void
// 使用示例     ICM_getEulerianAngles();
// 备注信息     包含 Yaw 角死区滤波处理
//-------------------------------------------------------------------------------------------------------------------
void ICM_getEulerianAngles(void)
{
    MPU6050_GetData(&ax,&ay,&az,&gx,&gy,&gz);

    ICM_getValues();
    ICM_AHRSupdate(icm_data_t.gyro_x, icm_data_t.gyro_y, icm_data_t.gyro_z, icm_data_t.acc_x, icm_data_t.acc_y, icm_data_t.acc_z);

    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;

    // 计算当前的原始Yaw角
    float raw_yaw = atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3, -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / PI;

    if (yaw_first_run)
    {
        last_raw_yaw = raw_yaw;
        yaw_first_run = false;
    }

    // 计算两次Yaw角之间的变化值
    float diff = raw_yaw - last_raw_yaw;

    // 处理角度越界跳变
    if (diff > 180.0f)       diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;

    // 如果变化值小于死区阈值，认为是噪声或漂移，进行累积以便后续减除
    if (fabsf(diff) < YAW_DEADZONE_THRESHOLD)
    {
        accumulated_yaw_drift += diff;
    }

    last_raw_yaw = raw_yaw;

    // 减去累积的死区误差
    float Daty_Z = raw_yaw - accumulated_yaw_drift;

    // 归一化到 [-180, 180]
    while (Daty_Z > 180.0f)  Daty_Z -= 360.0f;
    while (Daty_Z < -180.0f) Daty_Z += 360.0f;

    icm_data_t.yaw = Daty_Z;
    imu_angle.yaw = -Daty_Z;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     基于纯角速度积分计算 Yaw (三轴解算方式)
// 参数说明     void
// 返回参数     void
// 使用示例     imu_transform_gyro();
// 备注信息     仅用于特殊调试或对比测试
//-------------------------------------------------------------------------------------------------------------------
void imu_transform_gyro(void)
{
    MPU6050_GetData(&ax,&ay,&az,&gx,&gy,&gz);

    float gx_temp = (float)gx - gyro_drift_x;
    float gy_temp = (float)gy - gyro_drift_y;
    float gz_temp = (float)gz - gyro_drift_z;

    // 角速度死区处理
    if (gx_temp < GYRO_DEADZONE && gx_temp > -GYRO_DEADZONE) gx_temp = 0.0f;
    if (gy_temp < GYRO_DEADZONE && gy_temp > -GYRO_DEADZONE) gy_temp = 0.0f;
    if (gz_temp < GYRO_DEADZONE && gz_temp > -GYRO_DEADZONE) gz_temp = 0.0f;

    float gx = mpu6050_gyro_transition(gx_temp/10*10) * PI / 180.0f;
    float gy = mpu6050_gyro_transition(gy_temp/10*10) * PI / 180.0f;
    float gz = mpu6050_gyro_transition(gz_temp/10*10) * PI / 180.0f;

    // 暂存当前三轴四元数
    float q0 = q_3dof[0];
    float q1 = q_3dof[1];
    float q2 = q_3dof[2];
    float q3 = q_3dof[3];

    // 三轴四元数微分方程积分
    float dt = CYCLE_T;
    q_3dof[0] += (-q1 * gx - q2 * gy - q3 * gz) * 0.5f * dt;
    q_3dof[1] += ( q0 * gx + q2 * gz - q3 * gy) * 0.5f * dt;
    q_3dof[2] += ( q0 * gy - q1 * gz + q3 * gx) * 0.5f * dt;
    q_3dof[3] += ( q0 * gz + q1 * gy - q2 * gx) * 0.5f * dt;

    // 三轴四元数归一化
    float norm = sqrtf(q_3dof[0] * q_3dof[0] + q_3dof[1] * q_3dof[1] + q_3dof[2] * q_3dof[2] + q_3dof[3] * q_3dof[3]);
    if (norm > 0.0f)
    {
        norm = 1.0f / norm;
        q_3dof[0] *= norm;
        q_3dof[1] *= norm;
        q_3dof[2] *= norm;
        q_3dof[3] *= norm;
    }

    // 计算角度（基于三轴四元数）
    imu_angle.yaw = atan2f(2.0f * q_3dof[1] * q_3dof[2] + 2.0f * q_3dof[0] * q_3dof[3], -2.0f * q_3dof[2] * q_3dof[2] - 2.0f * q_3dof[3] * q_3dof[3] + 1.0f) * 57.2957f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU 获取角度顶层接口
// 参数说明     void
// 返回参数     void
// 使用示例     imu_get_angle();
// 备注信息     默认调用六轴互补滤波算法
//-------------------------------------------------------------------------------------------------------------------
void imu_get_angle(void)
{
    // 三轴四元数
    imu_transform_gyro();

    // 六轴互补四元数
    //ICM_getEulerianAngles();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将当前三轴解算的 Yaw 归零 (不改变俯仰/横滚相对姿态)
// 参数说明     void
// 返回参数     void
// 使用示例     imu_zero_yaw();
// 备注信息     通过四元数逆旋转实现
//-------------------------------------------------------------------------------------------------------------------
void imu_zero_yaw(void)
{
    float q0 = q_3dof[0];
    float q1 = q_3dof[1];
    float q2 = q_3dof[2];
    float q3 = q_3dof[3];

    float yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), -2.0f * (q2 * q2 + q3 * q3) + 1.0f);

    // 构造逆 yaw 旋转并左乘到当前四元数
    float half = -0.5f * yaw;
    float cz = cosf(half);
    float sz = sinf(half);

    float nq0 = cz * q0 - sz * q3;
    float nq1 = cz * q1 + sz * q2;
    float nq2 = cz * q2 - sz * q1;
    float nq3 = cz * q3 + sz * q0;

    // 归一化
    float norm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    if (norm > 0.0f)
    {
        norm = 1.0f / norm;
        q_3dof[0] = nq0 * norm;
        q_3dof[1] = nq1 * norm;
        q_3dof[2] = nq2 * norm;
        q_3dof[3] = nq3 * norm;
    }

    imu_angle.yaw = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将当前 Yaw 设为指定偏移量 (用于纠正发车角度)
// 参数说明     offset_deg      初始偏角(度)
// 返回参数     void
// 使用示例     imu_start_with_offset(5.0f); // 假设赛道向左偏5度
// 备注信息     通过四元数旋转实现
//-------------------------------------------------------------------------------------------------------------------
void imu_start_with_offset(float offset_deg)
{
    // 先归零
    imu_zero_yaw();

    // 重新获取归零后的四元数
    float q0 = q_3dof[0];
    float q1 = q_3dof[1];
    float q2 = q_3dof[2];
    float q3 = q_3dof[3];

    // 注入 offset 旋转 (将角度转为弧度)
    float half = 0.5f * (offset_deg * 3.14159265f / 180.0f);
    float cz = cosf(half);
    float sz = sinf(half);

    // 计算旋转后的四元数
    float nq0 = cz * q0 - sz * q3;
    float nq1 = cz * q1 + sz * q2;
    float nq2 = cz * q2 - sz * q1;
    float nq3 = cz * q3 + sz * q0;

    // 归一化并写回
    float norm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    if (norm > 0.0f)
    {
        norm = 1.0f / norm;
        q_3dof[0] = nq0 * norm;
        q_3dof[1] = nq1 * norm;
        q_3dof[2] = nq2 * norm;
        q_3dof[3] = nq3 * norm;
    }
    
    // 更新欧拉角缓存
    imu_angle.yaw = offset_deg;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将当前六轴解算的 Yaw 归零 (与三轴效果一致)
// 参数说明     void
// 返回参数     void
// 使用示例     ICM_zero_yaw();
// 备注信息     包含死区滤波变量重置
//-------------------------------------------------------------------------------------------------------------------
void ICM_zero_yaw(void)
{
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;

    float yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), -2.0f * (q2 * q2 + q3 * q3) + 1.0f);

    float half = -0.5f * yaw;
    float cz = cosf(half);
    float sz = sinf(half);

    float nq0 = cz * q0 - sz * q3;
    float nq1 = cz * q1 + sz * q2;
    float nq2 = cz * q2 - sz * q1;
    float nq3 = cz * q3 + sz * q0;

    float norm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    if (norm > 0.0f)
    {
        norm = 1.0f / norm;
        Q_info.q0 = nq0 * norm;
        Q_info.q1 = nq1 * norm;
        Q_info.q2 = nq2 * norm;
        Q_info.q3 = nq3 * norm;
    }

    icm_data_t.yaw = 0.0f;
    imu_angle.yaw = 0.0f;

    // 重置死区滤波相关的变量
    last_raw_yaw = 0.0f;
    accumulated_yaw_drift = 0.0f;
    yaw_first_run = true;
}
