#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f1xx_hal.h"  // 引入 HAL 库头文件，用于 STM32F1 系列的外设控制

/* ========= 轨迹检测模块函数声明 ========= */

/**
 * @brief 初始化循迹模块（传感器、IO 口等）
 */
void track_init(void);           

/**
 * @brief 读取单个循迹传感器的状态
 * @param ch 传感器通道编号（1~N）
 * @retval 状态值（0 或 1）
 */
uint8_t read_sensor(uint8_t ch); 

/**
 * @brief 获取当前循迹线的颜色
 * @retval 0 = 黑线，1 = 白线 （或根据实际定义）
 */
uint8_t get_line_color(void);    

/**
 * @brief 计算循迹误差（例如偏移量）
 * @retval 返回误差值（如用于 PID 控制）
 */
float get_error(void);           

/**
 * @brief 循迹控制主函数（执行循迹算法、更新 PWM）
 */
void line_tracking(void);        

#endif /* __TRACK_H */
