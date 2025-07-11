#include "motor.h"
#include "gpio.h"  // ??GPIO????

/* ?????PWM??? */
void Set_left_pwm(uint16_t pwm)
{
    // ??PWM??(0 ~ TIM3??????,?????1000)
    if (pwm > 1000) pwm = 1000;
    if (pwm < 0) pwm = 0;
    // ??TIM3_CH3??PWM(PB0??)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);
}

/* ?????PWM??? */
void Set_right_pwm(uint16_t pwm)
{
    // ??PWM??
    if (pwm > 1000) pwm = 1000;
    if (pwm < 0) pwm = 0;
    // ??TIM3_CH4??PWM(PB1??)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);
}

/* ?????(??GPIO??????) */
void Set_left_forward(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // AIN1 = 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // AIN2 = 0
}

/* ????? */
void Set_left_backward(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // AIN1 = 0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // AIN2 = 1
}

/* ????? */
void Set_right_forward(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // BIN1 = 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // BIN2 = 0
}

/* ????? */
void Set_right_backward(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // BIN1 = 0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // BIN2 = 1
}