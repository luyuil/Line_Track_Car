#ifndef __QUATERNION_H
#define __QUATERNION_H

#include <stdint.h>
#include <math.h>

// 定义π常量（Keil中M_PI未定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 四元数结构体
typedef struct {
    float q0;   // w
    float q1;   // x
    float q2;   // y
    float q3;   // z
} Quaternion;

// 欧拉角结构体
typedef struct {
    float roll;     // 横滚角 (X轴)
    float pitch;    // 俯仰角 (Y轴)
    float yaw;      // 偏航角 (Z轴)
} EulerAngle;

// 零漂校准结构体
typedef struct {
    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;
} GyroOffset;

// 四元数算法控制结构体
typedef struct {
    Quaternion quat;            // 当前四元数
    EulerAngle euler;           // 当前欧拉角
    EulerAngle euler_filtered;  // 滤波后的欧拉角
    GyroOffset offset;          // 陀螺仪零漂
    float sample_time;          // 采样时间 (秒)
    float beta;                 // 互补滤波系数 (0-1)
    float gyro_scale;           // 陀螺仪比例因子 (16.4对应2000dps)
    float filter_threshold;     // 滤波阈值
    uint8_t use_acc_correction; // 是否使用加速度计校正
    uint8_t initialized;        // 初始化标志
} QuaternionAlgorithm;

// 函数声明
void Quaternion_Init(void);
void Quaternion_Update(void);
EulerAngle* Quaternion_GetEulerAngles(void);
EulerAngle* Quaternion_GetFilteredEulerAngles(void);
Quaternion* Quaternion_GetQuaternion(void);
void Quaternion_SetOffsets(int16_t gx, int16_t gy, int16_t gz);
void Quaternion_SetBeta(float beta);
void Quaternion_SetGyroScale(float scale);
void Quaternion_EnableAccCorrection(uint8_t enable);
float Quaternion_GetFilterThreshold(void);
void Quaternion_SetFilterThreshold(float threshold);

#endif
