#include "jy61p.h"

static uint8_t RxBuffer[11];/*������������*/
static volatile uint8_t RxState = 0;/*����״̬��־λ*/
static uint8_t RxIndex = 0;/*������������*/
float Roll,Pitch,Yaw;/*�Ƕ���Ϣ�����ֻ��Ҫ�������Ը�Ϊ��������*/


/**
 * @brief       ���ݰ�������
 * @param       ���ڽ��յ�����RxData
 * @retval      ��
 */
void jy61p_ReceiveData(uint8_t RxData)
{
	uint8_t i,sum=0;
	
	if (RxState == 0)	//�ȴ���ͷ
	{
		if (RxData == 0x55)	//�յ���ͷ
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 1;
			RxIndex = 1; //������һ״̬
		}
	}
	
	else if (RxState == 1)
	{
		if (RxData == 0x53)	/*�ж��������ݣ��޸�������Ըı�Ҫ�����������ݣ�0x53Ϊ�Ƕ����*/
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 2;
			RxIndex = 2; //������һ״̬
		}
	}
	
	else if (RxState == 2)	//��������
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//�������
		{
			for(i=0;i<10;i++)
			{
				sum = sum + RxBuffer[i]; //����У���
			}
			if(sum == RxBuffer[10])		//У��ɹ�
			{
				/*�������ݣ�������������ѡ���Ӧ�ļ��㹫ʽ*/
				Roll = ((uint16_t) ((uint16_t) RxBuffer[3] << 8 | (uint16_t) RxBuffer[2])) / 32768.0f * 180.0f;
				Pitch = ((uint16_t) ((uint16_t) RxBuffer[5] << 8 | (uint16_t) RxBuffer[4])) / 32768.0f * 180.0f;
				Yaw = ((uint16_t) ((uint16_t) RxBuffer[7] << 8 | (uint16_t) RxBuffer[6])) / 32768.0f * 180.0f;
			}
			RxState = 0;
			RxIndex = 0; //��ȡ��ɣ��ص����״̬���ȴ���ͷ
		}
	}
}