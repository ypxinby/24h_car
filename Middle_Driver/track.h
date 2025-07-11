#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f1xx_hal.h"  // ���� HAL ��ͷ�ļ������� STM32F1 ϵ�е��������

/* ========= �켣���ģ�麯������ ========= */

/**
 * @brief ��ʼ��ѭ��ģ�飨��������IO �ڵȣ�
 */
void track_init(void);           

/**
 * @brief ��ȡ����ѭ����������״̬
 * @param ch ������ͨ����ţ�1~N��
 * @retval ״ֵ̬��0 �� 1��
 */
uint8_t read_sensor(uint8_t ch); 

/**
 * @brief ��ȡ��ǰѭ���ߵ���ɫ
 * @retval 0 = ���ߣ�1 = ���� �������ʵ�ʶ��壩
 */
uint8_t get_line_color(void);    

/**
 * @brief ����ѭ��������ƫ������
 * @retval �������ֵ�������� PID ���ƣ�
 */
float get_error(void);           

/**
 * @brief ѭ��������������ִ��ѭ���㷨������ PWM��
 */
void line_tracking(void);        

#endif /* __TRACK_H */
