#include "stm32f10x.h"                  // Device header
#include "quaternion.h"
#include "MPU6050.h"
#include <stdio.h>

// 全局变量声明（在main.c中定义）
extern int16_t AX, AY, AZ, GX, GY, GZ;

// 静态全局四元数算法实例
static QuaternionAlgorithm quat_algo;

// 辅助函数：陀螺仪数据处理
static float ProcessGyroData(int16_t raw_gyro, int16_t offset, float scale_factor)
{
    // 应用零漂校准
    int16_t calibrated = raw_gyro + offset;
    
    // 你的数据处理方式：(GX / 10 * 10) 相当于取整到十位数
    // 例如：123 -> 120, -45 -> -40
    int16_t processed = (calibrated / 100) * 100;
    
    // 转换为度/秒
    return (float)processed / scale_factor;
}

// 初始化四元数算法
void Quaternion_Init(void)
{
    // 默认采样时间 1ms
    quat_algo.sample_time = 0.001f;
    
    // 默认互补滤波系数
    quat_algo.beta = 0.01f;
    
    // 默认陀螺仪比例因子 (16.4对应2000dps)
    quat_algo.gyro_scale = 16.4f;
    
    // 默认零漂校准值
    quat_algo.offset.gx_offset = 92;
    quat_algo.offset.gy_offset = 7;
    quat_algo.offset.gz_offset = 11;
    
    // 默认滤波阈值
    quat_algo.filter_threshold = 0.08f;
    
    // 默认启用加速度计校正
    quat_algo.use_acc_correction = 1;
    
    // 初始四元数 (无旋转)
    quat_algo.quat.q0 = 1.0f;
    quat_algo.quat.q1 = 0.0f;
    quat_algo.quat.q2 = 0.0f;
    quat_algo.quat.q3 = 0.0f;
    
    // 初始欧拉角
    quat_algo.euler.roll = 0.0f;
    quat_algo.euler.pitch = 0.0f;
    quat_algo.euler.yaw = 0.0f;
    
    quat_algo.euler_filtered.roll = 0.0f;
    quat_algo.euler_filtered.pitch = 0.0f;
    quat_algo.euler_filtered.yaw = 0.0f;
    
    quat_algo.initialized = 1;
}

// 更新四元数（结合你的算法）
void Quaternion_Update(void)
{
    static float gx_deg, gy_deg, gz_deg;    // 陀螺仪角速度 (deg/s)
    static float gx_rad, gy_rad, gz_rad;    // 陀螺仪角速度 (rad/s)
    static float q0, q1, q2, q3;           // 四元数临时变量
    static float norm;                      // 归一化因子
    static float halfT;                     // 半采样时间
    static float acc_roll, acc_pitch;       // 加速度计计算的姿态角
    
    // 如果未初始化，直接返回
    if (!quat_algo.initialized) {
        return;
    }
    
    // 使用你的陀螺仪数据处理方式
    gx_deg = ProcessGyroData(GX, quat_algo.offset.gx_offset, quat_algo.gyro_scale);
    gy_deg = ProcessGyroData(GY, quat_algo.offset.gy_offset, quat_algo.gyro_scale);
    gz_deg = ProcessGyroData(GZ, quat_algo.offset.gz_offset, quat_algo.gyro_scale);
    
    // 转换为 rad/s（用于四元数更新）
    gx_rad = gx_deg * (M_PI / 180.0f);
    gy_rad = gy_deg * (M_PI / 180.0f);
    gz_rad = gz_deg * (M_PI / 180.0f);
    
    // ============ 四元数更新部分 ============
    // 当前四元数值
    q0 = quat_algo.quat.q0;
    q1 = quat_algo.quat.q1;
    q2 = quat_algo.quat.q2;
    q3 = quat_algo.quat.q3;
    
    // 半采样时间
    halfT = quat_algo.sample_time * 0.5f;
    
    // 四元数微分方程（一阶龙格库塔法）
    // dq/dt = 0.5 * q ⊗ ω
    float q0_dot = (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad) * halfT;
    float q1_dot = (q0 * gx_rad + q2 * gz_rad - q3 * gy_rad) * halfT;
    float q2_dot = (q0 * gy_rad - q1 * gz_rad + q3 * gx_rad) * halfT;
    float q3_dot = (q0 * gz_rad + q1 * gy_rad - q2 * gx_rad) * halfT;
    
    // 更新四元数
    q0 += q0_dot;
    q1 += q1_dot;
    q2 += q2_dot;
    q3 += q3_dot;
    
    // 四元数归一化
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0001f) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
    
    // 更新全局四元数
    quat_algo.quat.q0 = q0;
    quat_algo.quat.q1 = q1;
    quat_algo.quat.q2 = q2;
    quat_algo.quat.q3 = q3;
    
    // 四元数转换为欧拉角
    // Roll (X-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    quat_algo.euler.roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // Pitch (Y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        // 使用 90 度，如果超出范围
        quat_algo.euler.pitch = (sinp > 0 ? 1.0f : -1.0f) * 90.0f;
    } else {
        quat_algo.euler.pitch = asinf(sinp) * 180.0f / M_PI;
    }
    
    // Yaw (Z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    quat_algo.euler.yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
    
    // ============ 互补滤波部分（你的算法） ============
    if (quat_algo.use_acc_correction) {
        // 使用加速度计计算姿态角（用于互补滤波）
        acc_roll = atan2f((float)AY, (float)AZ) * 180.0f / M_PI;
        acc_pitch = -atan2f((float)AX, (float)AZ) * 180.0f / M_PI;
        
        // 使用互补滤波融合四元数角度和加速度计角度
        quat_algo.euler_filtered.roll = quat_algo.beta * acc_roll + 
                                       (1.0f - quat_algo.beta) * quat_algo.euler.roll;
        quat_algo.euler_filtered.pitch = quat_algo.beta * acc_pitch + 
                                        (1.0f - quat_algo.beta) * quat_algo.euler.pitch;
        
        // 偏航角直接使用四元数的值（无加速度计校正）
        quat_algo.euler_filtered.yaw = quat_algo.euler.yaw;
    } else {
        // 不使用加速度计校正
        quat_algo.euler_filtered = quat_algo.euler;
    }
    
    // ============ 输出滤波（你的算法） ============
    // 应用阈值滤波，减少噪声
    if (fabsf(quat_algo.euler_filtered.roll - quat_algo.euler.roll) > quat_algo.filter_threshold) {
        quat_algo.euler_filtered.roll = quat_algo.euler.roll;
    }
    if (fabsf(quat_algo.euler_filtered.pitch - quat_algo.euler.pitch) > quat_algo.filter_threshold) {
        quat_algo.euler_filtered.pitch = quat_algo.euler.pitch;
    }
}

// 获取原始欧拉角（四元数直接转换）
EulerAngle* Quaternion_GetEulerAngles(void)
{
    return &quat_algo.euler;
}

// 获取滤波后的欧拉角（结合了你的算法）
EulerAngle* Quaternion_GetFilteredEulerAngles(void)
{
    return &quat_algo.euler_filtered;
}

// 获取四元数
Quaternion* Quaternion_GetQuaternion(void)
{
    return &quat_algo.quat;
}

// 设置零漂校准值
void Quaternion_SetOffsets(int16_t gx, int16_t gy, int16_t gz)
{
    quat_algo.offset.gx_offset = gx;
    quat_algo.offset.gy_offset = gy;
    quat_algo.offset.gz_offset = gz;
}

// 设置互补滤波系数
void Quaternion_SetBeta(float beta)
{
    if (beta >= 0.0f && beta <= 1.0f) {
        quat_algo.beta = beta;
    }
}

// 设置陀螺仪比例因子
void Quaternion_SetGyroScale(float scale)
{
    if (scale > 0.0f) {
        quat_algo.gyro_scale = scale;
    }
}

// 启用/禁用加速度计校正
void Quaternion_EnableAccCorrection(uint8_t enable)
{
    quat_algo.use_acc_correction = (enable != 0);
}

// 获取滤波阈值
float Quaternion_GetFilterThreshold(void)
{
    return quat_algo.filter_threshold;
}

// 设置滤波阈值
void Quaternion_SetFilterThreshold(float threshold)
{
    if (threshold >= 0.0f) {
        quat_algo.filter_threshold = threshold;
    }
}
