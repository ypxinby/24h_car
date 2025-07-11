#include "track.h"
#include "motor.h"
#include "gpio.h"
#include "speed_encoder.h"
#include "jy61p.h"  // 姿态传感器头文件
#include <math.h>

// 布尔类型定义
#ifndef __cplusplus
typedef enum { false = 0, true = 1 } bool;
#endif

// 传感器配置
#define SENSOR_COUNT 8
const uint16_t SENSOR_PINS[SENSOR_COUNT] = {
    GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5,
    GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_10, GPIO_PIN_11
};
const GPIO_TypeDef* SENSOR_PORTS[SENSOR_COUNT] = {
    GPIOA, GPIOA, GPIOA, GPIOA,
    GPIOA, GPIOA, GPIOB, GPIOB
};

// 传感器权重配置 - 用于计算线位置
const float weight[SENSOR_COUNT] = {-12.0f, -8.0f, -4.0f, -2.0f, 2.0f, 4.0f, 8.0f, 12.0f};

// PID参数结构体 - 用于存储比例、积分、微分参数
typedef struct {
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数
} PIDParams;

// 基础PID参数 - 用于直线跟踪,平衡控制
const PIDParams PID_BASE = {8.0f, 0.02f, 0.8f};
// 转向PID参数(大误差时)
const PIDParams PID_TURN = {12.0f, 0.0f, 0.4f};

// 速度控制参数 - 整体降低速度
const float BASE_SPEED = 75.0f;      // 基础速度
const float MAX_SPEED = 105.0f;      // 最大速度限制
const float MIN_SPEED = 45.0f;       // 最小速度限制
const float SPEED_SCALE = 0.45f;     // 速度缩放因子

// 平衡补偿参数
const float BALANCE_OFFSET = 0.0f;   // 左右电机平衡补偿值

// 转向增强参数
const float TURN_ENHANCEMENT = 1.5f;  // 转向增强系数
const float MAX_CORRECTION = 80.0f;   // 最大转向修正值限制

// 滤波参数
const float DERIVATIVE_FILTER = 0.7f;  // 微分滤波系数
const float OUTPUT_SMOOTHING = 0.3f;   // 输出平滑系数

// 陀螺仪PID参数
const PIDParams GYRO_PID = {1.7f, 0.0f, 0.1f};

// 积分分离阈值
const float INTEGRAL_SEPARATION_THRESHOLD = 3.0f;

// 融合控制权重
const float GYRO_DIRECTION_WEIGHT = 0.3f;  // 陀螺仪方向权重
const float LINE_POSITION_WEIGHT = 0.7f;   // 线位置权重

// PID控制器状态结构体
typedef struct {
    float error;                  // 当前误差值
    float integral;               // 积分项
    float derivative;             // 微分项
    float last_error;             // 上一次误差值
    float last_derivative;        // 上一次微分项
    float last_correction;        // 上一次修正值
    float output;                 // 原始输出值
    float smoothed_output;        // 平滑后的输出值
    bool line_lost;               // 线丢失标志
    
    // 陀螺仪相关变量
    float gyro_error;             // 陀螺仪角度误差
    float gyro_integral;          // 陀螺仪积分项
    float gyro_derivative;        // 陀螺仪微分项
    float last_gyro_error;        // 上一次陀螺仪误差
    float target_yaw;             // 目标偏航角
    bool use_gyro;                // 是否使用陀螺仪标志
} PIDController;

static PIDController pid = {0};

// 读取指定通道的传感器值
uint8_t read_sensor(uint8_t ch)
{
    if (ch >= 1 && ch <= SENSOR_COUNT) {
        return HAL_GPIO_ReadPin((GPIO_TypeDef*)SENSOR_PORTS[ch-1], SENSOR_PINS[ch-1]);
    }
    return 0;
}

// 获取线位置误差
float get_error(void)
{
    float sum = 0.0f;
    uint8_t count = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (!read_sensor(i + 1)) {  // 假设检测到线时为低电平(0)
            sum += weight[i];
            count++;
        }
    }

    if (count > 0) {
        pid.line_lost = false;
        return sum / count;
    } else {
        pid.line_lost = true;
        return pid.last_error;  // 线丢失时返回上一次误差值
    }
}

// 动态调整PID参数
static void adjust_pid_params(PIDParams* params)
{
    float error_abs = fabs(pid.error);
    
    // 根据误差大小动态调整PID参数(平滑过渡)
    float factor = 1.0f / (1.0f + exp(-2.0f * (error_abs - 3.0f)));
    
    params->Kp = PID_BASE.Kp + factor * (PID_TURN.Kp - PID_BASE.Kp);
    params->Ki = PID_BASE.Ki + factor * (PID_TURN.Ki - PID_BASE.Ki);
    params->Kd = PID_BASE.Kd + factor * (PID_TURN.Kd - PID_BASE.Kd);
}

// 应用PID控制算法
static float apply_pid(void)
{
    PIDParams params;
    adjust_pid_params(&params);
    
    // 积分分离 - 大误差时不进行积分积累
    if (fabs(pid.error) < INTEGRAL_SEPARATION_THRESHOLD) {
        pid.integral += pid.error;
    } else {
        // 大误差时,清除积分项
        pid.integral = 0.0f;
    }
    
    // 积分限幅,防止积分饱和
    if (pid.integral > 30.0f) pid.integral = 30.0f;
    if (pid.integral < -30.0f) pid.integral = -30.0f;
    
    // 计算微分项,并应用低通滤波
    pid.derivative = pid.error - pid.last_error;
    pid.derivative = (pid.derivative * (1.0f - DERIVATIVE_FILTER)) + 
                    (pid.last_derivative * DERIVATIVE_FILTER);
    pid.last_derivative = pid.derivative;
    
    // 计算陀螺仪误差 - 目标角度与当前角度的差值
    pid.gyro_error = pid.target_yaw - Yaw;
    
    // 将Yaw误差归一化到 (-180~180度)
    if (pid.gyro_error > 180.0f) pid.gyro_error -= 360.0f;
    if (pid.gyro_error < -180.0f) pid.gyro_error += 360.0f;
    
    // 陀螺仪积分和微分计算
    pid.gyro_integral += pid.gyro_error;
    pid.gyro_derivative = pid.gyro_error - pid.last_gyro_error;
    pid.last_gyro_error = pid.gyro_error;
    
    // 陀螺仪积分限幅
    if (pid.gyro_integral > 50.0f) pid.gyro_integral = 50.0f;
    if (pid.gyro_integral < -50.0f) pid.gyro_integral = -50.0f;
    
    // 计算陀螺仪PID输出
    float gyro_output = GYRO_PID.Kp * pid.gyro_error + 
                        GYRO_PID.Ki * pid.gyro_integral + 
                        GYRO_PID.Kd * pid.gyro_derivative;
    
    // 计算位置PID输出
    float position_output = params.Kp * pid.error + 
                           params.Ki * pid.integral + 
                           params.Kd * pid.derivative;
    
    // 融合两种控制输出
    float combined_output;
    if (pid.line_lost) {
        // 线丢失时,主要依靠陀螺仪保持方向
        combined_output = gyro_output;
    } else {
        // 线可见时,融合线位置和陀螺仪方向
        combined_output = (position_output * LINE_POSITION_WEIGHT) + 
                         (gyro_output * GYRO_DIRECTION_WEIGHT);
    }
    
    // 转向增强(大角度时)
    if (fabs(combined_output) > 15.0f) {
        combined_output *= TURN_ENHANCEMENT;
    }
    
    // 输出限幅
    if (combined_output > MAX_CORRECTION) combined_output = MAX_CORRECTION;
    if (combined_output < -MAX_CORRECTION) combined_output = -MAX_CORRECTION;
    
    // 输出平滑处理
    pid.smoothed_output = (combined_output * (1.0f - OUTPUT_SMOOTHING)) + 
                         (pid.last_correction * OUTPUT_SMOOTHING);
    
    // 保存当前状态用于下次计算
    pid.last_error = pid.error;
    pid.last_correction = pid.smoothed_output;
    
    return pid.smoothed_output;
}

// 计算目标速度 - 核心优化：转向时动态减速
static float calculate_speed(void)
{
    float error_abs = fabs(pid.error);
    float speed_factor;
    
    // 基础速度因子计算（原逻辑保留）
    if (pid.line_lost) {
        speed_factor = 0.3f;  // 线丢失时降低速度
    } else {
        speed_factor = 1.0f - (1.0f / (1.0f + exp(-2.0f * (5.0f - error_abs))));
        speed_factor = 0.5f + speed_factor * 0.3f;  // 限制速度范围在50%-80%
    }
    
    // 【转向减速核心优化】根据转向幅度动态计算减速因子
    float correction_abs = fabs(pid.smoothed_output);  // 转向修正幅度（越大转向越急）
    float turn_decel_factor = 1.0f;  // 减速因子（1.0=不减速，<1=减速）
    
    // 1. 小转向（轻微修正）：不减速或轻微减速
    if (correction_abs > 5.0f && correction_abs <= 20.0f) {
        turn_decel_factor = 1.0f - 0.2f * (correction_abs / 20.0f);  // 最多减速20%
    }
    // 2. 中转向（中等修正）：适度减速
    else if (correction_abs > 20.0f && correction_abs <= 50.0f) {
        turn_decel_factor = 0.8f - 0.3f * ((correction_abs - 20.0f) / 30.0f);  // 减速20%-50%
    }
    // 3. 大转向（急弯）：大幅减速
    else if (correction_abs > 50.0f) {
        turn_decel_factor = 0.5f - 0.2f * ((correction_abs - 50.0f) / 30.0f);  // 减速50%-70%
        if (turn_decel_factor < 0.3f) turn_decel_factor = 0.3f;  // 最低保留30%速度
    }
    
    // 陀螺仪变化率减速（原逻辑保留，与转向减速叠加）
    float gyro_change_rate = fabs(pid.gyro_derivative);
    float gyro_speed_factor = 1.0f - (gyro_change_rate / 40.0f);
    if (gyro_speed_factor < 0.6f) gyro_speed_factor = 0.6f;
    
    // 最终速度 = 基础速度 × 所有因子乘积
    float target_speed = BASE_SPEED * speed_factor * turn_decel_factor * gyro_speed_factor;
    
    // 速度限幅（原逻辑保留）
    if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;
    if (target_speed < MIN_SPEED) target_speed = MIN_SPEED;
    
    return target_speed;
}

// 计算左右电机PWM值
static void calculate_motor_speeds(float correction, float target_speed, int* left_pwm, int* right_pwm)
{
    // 根据转向修正值计算速度减少因子
    float turn_factor = fabs(correction) / MAX_CORRECTION;
    float speed_reduction = 0.6f + 0.4f * turn_factor;  // 速度减少因子范围0.6-1.0
    
    float left_speed = target_speed;
    float right_speed = target_speed;
    
    if (correction > 0) {  // 右转
        right_speed *= speed_reduction;
        left_speed = target_speed * (2.0f - speed_reduction);
    } else {  // 左转
        left_speed *= speed_reduction;
        right_speed = target_speed * (2.0f - speed_reduction);
    }
    
    // 平衡补偿
    left_speed += BALANCE_OFFSET;
    
    // 转换为PWM值
    *left_pwm = (int)left_speed;
    *right_pwm = (int)right_speed;
    
    // PWM限幅
    if (*left_pwm < 0) *left_pwm = 0;
    if (*left_pwm > 1000) *left_pwm = 1000;
    if (*right_pwm < 0) *right_pwm = 0;
    if (*right_pwm > 1000) *right_pwm = 1000;
}

// 设置目标偏航角 - 用于陀螺仪辅助导航
void set_target_yaw(float yaw)
{
    pid.target_yaw = yaw;
}

// 线跟踪主函数
void line_tracking(void)
{
    // 读取当前线位置误差
    pid.error = get_error();
    
    // 应用PID控制算法计算转向修正值
    float correction = apply_pid();
    
    // 计算目标速度
    float target_speed = calculate_speed();
    
    // 计算左右电机PWM值
    int left_pwm, right_pwm;
    calculate_motor_speeds(correction, target_speed, &left_pwm, &right_pwm);
    
    // 设置电机PWM值
    Set_left_pwm(left_pwm);
    Set_right_pwm(right_pwm);
}

// 线跟踪初始化
void track_init(void)
{
    // 初始化PID控制器状态
    pid.integral = 0.0f;
    pid.last_error = 0.0f;
    pid.last_derivative = 0.0f;
    pid.last_correction = 0.0f;
    pid.smoothed_output = 0.0f;
    pid.line_lost = false;
    
    // 初始化陀螺仪相关状态
    pid.gyro_integral = 0.0f;
    pid.last_gyro_error = 0.0f;
    pid.target_yaw = Yaw;  // 初始目标角度设为当前角度
    pid.use_gyro = true;   // 启用陀螺仪辅助控制
}

// 获取线颜色(预留函数)
uint8_t get_line_color(void)
{
    return 0;  // 默认返回0
}