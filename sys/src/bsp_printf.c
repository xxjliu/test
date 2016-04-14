/*
*********************************************************************************************************
*
*	ģ������ : printfģ��
*	�ļ����� : bsp_printf.c
*	��    �� : V2.0
*	˵    �� : ʵ��printf��scanf�����ض��򵽴���1����֧��printf��Ϣ��UART1
*				ʵ���ض���ֻ��Ҫ���2������:
*				int fputc(int ch, FILE *f);
*				int fgetc(FILE *f);
*
*				���cģ���޶�Ӧ��h�ļ���
*				�����ҪӦ�ó���֧�� printf ������ֻ�ý� bsp_printf.c ����ļ���ӵ����̼��ɡ�
*
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v1.0    2012-10-12 armfly  ST�̼���汾 V2.1.0
*
*	Copyright (C), 2012-2013, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "stm8s.h"
#include <stdio.h>

/*
	���ڲ�ͬ�ı����� putcha �� getchar �βκͷ���ֵ���в�ͬ��
	��˴˴����ú궨��ķ�ʽ����

	_RAISONANCE_ �� _COSMIC_ ��2�������ɱ������Զ���ӵ�Ԥ�����
*/
#ifdef _RAISONANCE_
	#define PUTCHAR_PROTOTYPE int putchar (char c)
	#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
	#define PUTCHAR_PROTOTYPE char putchar (char c)
	#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
	#define PUTCHAR_PROTOTYPE int putchar (int c)
	#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */

/*
*********************************************************************************************************
*	�� �� ��: putchar
*	����˵��: �ض��� putchar ������ ��������ʹ��printf�����Ӵ���1��ӡ���
*	��    ��: �ȴ����͵��ַ�
*	�� �� ֵ: �����ѷ��͵��ַ�
*********************************************************************************************************
*/
PUTCHAR_PROTOTYPE
{
	/* ����һ���ַ� c ��UART1 */
	UART1_SendData8(c);

	/* �ȴ�������� */
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);

	return (c);
}

/*
*********************************************************************************************************
*	�� �� ��: getchar
*	����˵��: �ض���C���е� getchar ����,��������ʹ��scanff�����Ӵ���1��������
*	��    ��: ��
*	�� �� ֵ: �����Ѷ������ַ�
*********************************************************************************************************
*/
GETCHAR_PROTOTYPE
{
	#ifdef _COSMIC_
		char c = 0;
	#else
		int c = 0;
	#endif

	/* �ȴ������ݵ���  */
	while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);

	/* ��ȡ���ݼĴ��� */
	c = UART1_ReceiveData8();

	return (c);
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
