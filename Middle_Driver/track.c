#include "track.h"
#include "motor.h"
#include "gpio.h"
#include "speed_encoder.h"
#include "jy61p.h"  // ��̬������ͷ�ļ�
#include <math.h>

// �������Ͷ���
#ifndef __cplusplus
typedef enum { false = 0, true = 1 } bool;
#endif

// ����������
#define SENSOR_COUNT 8
const uint16_t SENSOR_PINS[SENSOR_COUNT] = {
    GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5,
    GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_10, GPIO_PIN_11
};
const GPIO_TypeDef* SENSOR_PORTS[SENSOR_COUNT] = {
    GPIOA, GPIOA, GPIOA, GPIOA,
    GPIOA, GPIOA, GPIOB, GPIOB
};

// ������Ȩ������ - ���ڼ�����λ��
const float weight[SENSOR_COUNT] = {-12.0f, -8.0f, -4.0f, -2.0f, 2.0f, 4.0f, 8.0f, 12.0f};

// PID�����ṹ�� - ���ڴ洢���������֡�΢�ֲ���
typedef struct {
    float Kp;  // ����ϵ��
    float Ki;  // ����ϵ��
    float Kd;  // ΢��ϵ��
} PIDParams;

// ����PID���� - ����ֱ�߸���,ƽ�����
const PIDParams PID_BASE = {8.0f, 0.02f, 0.8f};
// ת��PID����(�����ʱ)
const PIDParams PID_TURN = {12.0f, 0.0f, 0.4f};

// �ٶȿ��Ʋ��� - ���彵���ٶ�
const float BASE_SPEED = 75.0f;      // �����ٶ�
const float MAX_SPEED = 105.0f;      // ����ٶ�����
const float MIN_SPEED = 45.0f;       // ��С�ٶ�����
const float SPEED_SCALE = 0.45f;     // �ٶ���������

// ƽ�ⲹ������
const float BALANCE_OFFSET = 0.0f;   // ���ҵ��ƽ�ⲹ��ֵ

// ת����ǿ����
const float TURN_ENHANCEMENT = 1.5f;  // ת����ǿϵ��
const float MAX_CORRECTION = 80.0f;   // ���ת������ֵ����

// �˲�����
const float DERIVATIVE_FILTER = 0.7f;  // ΢���˲�ϵ��
const float OUTPUT_SMOOTHING = 0.3f;   // ���ƽ��ϵ��

// ������PID����
const PIDParams GYRO_PID = {1.7f, 0.0f, 0.1f};

// ���ַ�����ֵ
const float INTEGRAL_SEPARATION_THRESHOLD = 3.0f;

// �ںϿ���Ȩ��
const float GYRO_DIRECTION_WEIGHT = 0.3f;  // �����Ƿ���Ȩ��
const float LINE_POSITION_WEIGHT = 0.7f;   // ��λ��Ȩ��

// PID������״̬�ṹ��
typedef struct {
    float error;                  // ��ǰ���ֵ
    float integral;               // ������
    float derivative;             // ΢����
    float last_error;             // ��һ�����ֵ
    float last_derivative;        // ��һ��΢����
    float last_correction;        // ��һ������ֵ
    float output;                 // ԭʼ���ֵ
    float smoothed_output;        // ƽ��������ֵ
    bool line_lost;               // �߶�ʧ��־
    
    // ��������ر���
    float gyro_error;             // �����ǽǶ����
    float gyro_integral;          // �����ǻ�����
    float gyro_derivative;        // ������΢����
    float last_gyro_error;        // ��һ�����������
    float target_yaw;             // Ŀ��ƫ����
    bool use_gyro;                // �Ƿ�ʹ�������Ǳ�־
} PIDController;

static PIDController pid = {0};

// ��ȡָ��ͨ���Ĵ�����ֵ
uint8_t read_sensor(uint8_t ch)
{
    if (ch >= 1 && ch <= SENSOR_COUNT) {
        return HAL_GPIO_ReadPin((GPIO_TypeDef*)SENSOR_PORTS[ch-1], SENSOR_PINS[ch-1]);
    }
    return 0;
}

// ��ȡ��λ�����
float get_error(void)
{
    float sum = 0.0f;
    uint8_t count = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (!read_sensor(i + 1)) {  // �����⵽��ʱΪ�͵�ƽ(0)
            sum += weight[i];
            count++;
        }
    }

    if (count > 0) {
        pid.line_lost = false;
        return sum / count;
    } else {
        pid.line_lost = true;
        return pid.last_error;  // �߶�ʧʱ������һ�����ֵ
    }
}

// ��̬����PID����
static void adjust_pid_params(PIDParams* params)
{
    float error_abs = fabs(pid.error);
    
    // ��������С��̬����PID����(ƽ������)
    float factor = 1.0f / (1.0f + exp(-2.0f * (error_abs - 3.0f)));
    
    params->Kp = PID_BASE.Kp + factor * (PID_TURN.Kp - PID_BASE.Kp);
    params->Ki = PID_BASE.Ki + factor * (PID_TURN.Ki - PID_BASE.Ki);
    params->Kd = PID_BASE.Kd + factor * (PID_TURN.Kd - PID_BASE.Kd);
}

// Ӧ��PID�����㷨
static float apply_pid(void)
{
    PIDParams params;
    adjust_pid_params(&params);
    
    // ���ַ��� - �����ʱ�����л��ֻ���
    if (fabs(pid.error) < INTEGRAL_SEPARATION_THRESHOLD) {
        pid.integral += pid.error;
    } else {
        // �����ʱ,���������
        pid.integral = 0.0f;
    }
    
    // �����޷�,��ֹ���ֱ���
    if (pid.integral > 30.0f) pid.integral = 30.0f;
    if (pid.integral < -30.0f) pid.integral = -30.0f;
    
    // ����΢����,��Ӧ�õ�ͨ�˲�
    pid.derivative = pid.error - pid.last_error;
    pid.derivative = (pid.derivative * (1.0f - DERIVATIVE_FILTER)) + 
                    (pid.last_derivative * DERIVATIVE_FILTER);
    pid.last_derivative = pid.derivative;
    
    // ������������� - Ŀ��Ƕ��뵱ǰ�ǶȵĲ�ֵ
    pid.gyro_error = pid.target_yaw - Yaw;
    
    // ��Yaw����һ���� (-180~180��)
    if (pid.gyro_error > 180.0f) pid.gyro_error -= 360.0f;
    if (pid.gyro_error < -180.0f) pid.gyro_error += 360.0f;
    
    // �����ǻ��ֺ�΢�ּ���
    pid.gyro_integral += pid.gyro_error;
    pid.gyro_derivative = pid.gyro_error - pid.last_gyro_error;
    pid.last_gyro_error = pid.gyro_error;
    
    // �����ǻ����޷�
    if (pid.gyro_integral > 50.0f) pid.gyro_integral = 50.0f;
    if (pid.gyro_integral < -50.0f) pid.gyro_integral = -50.0f;
    
    // ����������PID���
    float gyro_output = GYRO_PID.Kp * pid.gyro_error + 
                        GYRO_PID.Ki * pid.gyro_integral + 
                        GYRO_PID.Kd * pid.gyro_derivative;
    
    // ����λ��PID���
    float position_output = params.Kp * pid.error + 
                           params.Ki * pid.integral + 
                           params.Kd * pid.derivative;
    
    // �ں����ֿ������
    float combined_output;
    if (pid.line_lost) {
        // �߶�ʧʱ,��Ҫ���������Ǳ��ַ���
        combined_output = gyro_output;
    } else {
        // �߿ɼ�ʱ,�ں���λ�ú������Ƿ���
        combined_output = (position_output * LINE_POSITION_WEIGHT) + 
                         (gyro_output * GYRO_DIRECTION_WEIGHT);
    }
    
    // ת����ǿ(��Ƕ�ʱ)
    if (fabs(combined_output) > 15.0f) {
        combined_output *= TURN_ENHANCEMENT;
    }
    
    // ����޷�
    if (combined_output > MAX_CORRECTION) combined_output = MAX_CORRECTION;
    if (combined_output < -MAX_CORRECTION) combined_output = -MAX_CORRECTION;
    
    // ���ƽ������
    pid.smoothed_output = (combined_output * (1.0f - OUTPUT_SMOOTHING)) + 
                         (pid.last_correction * OUTPUT_SMOOTHING);
    
    // ���浱ǰ״̬�����´μ���
    pid.last_error = pid.error;
    pid.last_correction = pid.smoothed_output;
    
    return pid.smoothed_output;
}

// ����Ŀ���ٶ� - �����Ż���ת��ʱ��̬����
static float calculate_speed(void)
{
    float error_abs = fabs(pid.error);
    float speed_factor;
    
    // �����ٶ����Ӽ��㣨ԭ�߼�������
    if (pid.line_lost) {
        speed_factor = 0.3f;  // �߶�ʧʱ�����ٶ�
    } else {
        speed_factor = 1.0f - (1.0f / (1.0f + exp(-2.0f * (5.0f - error_abs))));
        speed_factor = 0.5f + speed_factor * 0.3f;  // �����ٶȷ�Χ��50%-80%
    }
    
    // ��ת����ٺ����Ż�������ת����ȶ�̬�����������
    float correction_abs = fabs(pid.smoothed_output);  // ת���������ȣ�Խ��ת��Խ����
    float turn_decel_factor = 1.0f;  // �������ӣ�1.0=�����٣�<1=���٣�
    
    // 1. Сת����΢�������������ٻ���΢����
    if (correction_abs > 5.0f && correction_abs <= 20.0f) {
        turn_decel_factor = 1.0f - 0.2f * (correction_abs / 20.0f);  // ������20%
    }
    // 2. ��ת���е����������ʶȼ���
    else if (correction_abs > 20.0f && correction_abs <= 50.0f) {
        turn_decel_factor = 0.8f - 0.3f * ((correction_abs - 20.0f) / 30.0f);  // ����20%-50%
    }
    // 3. ��ת�򣨼��䣩���������
    else if (correction_abs > 50.0f) {
        turn_decel_factor = 0.5f - 0.2f * ((correction_abs - 50.0f) / 30.0f);  // ����50%-70%
        if (turn_decel_factor < 0.3f) turn_decel_factor = 0.3f;  // ��ͱ���30%�ٶ�
    }
    
    // �����Ǳ仯�ʼ��٣�ԭ�߼���������ת����ٵ��ӣ�
    float gyro_change_rate = fabs(pid.gyro_derivative);
    float gyro_speed_factor = 1.0f - (gyro_change_rate / 40.0f);
    if (gyro_speed_factor < 0.6f) gyro_speed_factor = 0.6f;
    
    // �����ٶ� = �����ٶ� �� �������ӳ˻�
    float target_speed = BASE_SPEED * speed_factor * turn_decel_factor * gyro_speed_factor;
    
    // �ٶ��޷���ԭ�߼�������
    if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;
    if (target_speed < MIN_SPEED) target_speed = MIN_SPEED;
    
    return target_speed;
}

// �������ҵ��PWMֵ
static void calculate_motor_speeds(float correction, float target_speed, int* left_pwm, int* right_pwm)
{
    // ����ת������ֵ�����ٶȼ�������
    float turn_factor = fabs(correction) / MAX_CORRECTION;
    float speed_reduction = 0.6f + 0.4f * turn_factor;  // �ٶȼ������ӷ�Χ0.6-1.0
    
    float left_speed = target_speed;
    float right_speed = target_speed;
    
    if (correction > 0) {  // ��ת
        right_speed *= speed_reduction;
        left_speed = target_speed * (2.0f - speed_reduction);
    } else {  // ��ת
        left_speed *= speed_reduction;
        right_speed = target_speed * (2.0f - speed_reduction);
    }
    
    // ƽ�ⲹ��
    left_speed += BALANCE_OFFSET;
    
    // ת��ΪPWMֵ
    *left_pwm = (int)left_speed;
    *right_pwm = (int)right_speed;
    
    // PWM�޷�
    if (*left_pwm < 0) *left_pwm = 0;
    if (*left_pwm > 1000) *left_pwm = 1000;
    if (*right_pwm < 0) *right_pwm = 0;
    if (*right_pwm > 1000) *right_pwm = 1000;
}

// ����Ŀ��ƫ���� - ���������Ǹ�������
void set_target_yaw(float yaw)
{
    pid.target_yaw = yaw;
}

// �߸���������
void line_tracking(void)
{
    // ��ȡ��ǰ��λ�����
    pid.error = get_error();
    
    // Ӧ��PID�����㷨����ת������ֵ
    float correction = apply_pid();
    
    // ����Ŀ���ٶ�
    float target_speed = calculate_speed();
    
    // �������ҵ��PWMֵ
    int left_pwm, right_pwm;
    calculate_motor_speeds(correction, target_speed, &left_pwm, &right_pwm);
    
    // ���õ��PWMֵ
    Set_left_pwm(left_pwm);
    Set_right_pwm(right_pwm);
}

// �߸��ٳ�ʼ��
void track_init(void)
{
    // ��ʼ��PID������״̬
    pid.integral = 0.0f;
    pid.last_error = 0.0f;
    pid.last_derivative = 0.0f;
    pid.last_correction = 0.0f;
    pid.smoothed_output = 0.0f;
    pid.line_lost = false;
    
    // ��ʼ�����������״̬
    pid.gyro_integral = 0.0f;
    pid.last_gyro_error = 0.0f;
    pid.target_yaw = Yaw;  // ��ʼĿ��Ƕ���Ϊ��ǰ�Ƕ�
    pid.use_gyro = true;   // ���������Ǹ�������
}

// ��ȡ����ɫ(Ԥ������)
uint8_t get_line_color(void)
{
    return 0;  // Ĭ�Ϸ���0
}