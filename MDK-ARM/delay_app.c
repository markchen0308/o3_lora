

#include <stdint.h>
#include "main.h"

static uint32_t fac_us= 0 ; // us���ɭ�����
static uint32_t fac_ms= 0 ; // ms���ɭ�����

void delay_init(uint8_t SYSCLK)
{
	SysTick->CTRL&= 0xfffffffb ; // bit2�M��,��ܥ~������HCLK/8 
	fac_us=SYSCLK/ 8 ;     
	fac_ms=(uint32_t)fac_us* 1000 ;
}     



void delay_ms(uint32_t nms)
{        
 
  uint32_t temp;    
	SysTick->LOAD=(uint32_t)nms*fac_ms; //�ɶ��[��(SysTick->LOAD��24bit) 

	SysTick->VAL = 0x00 ;            //�M�ŭp�ƾ�

	SysTick->CTRL= 0x01 ;          //�}�l�˼�  

	do 
		{
 
			temp=SysTick-> CTRL;
 
		}while (temp& 0x01 &&!(temp&( 1 << 16 ))); //���ݮɶ���F   

		SysTick->CTRL= 0x00 ;        //�����p�ƾ�
 
		SysTick->VAL = 0X00 ;        //�M�ŭp�ƾ�        
  
}   






//����nus

// nus���n���ɪ�us��.          

void delay_us(uint32_t nus)
{
		uint32_t temp;       
		SysTick->LOAD=nus*fac_us; //�ɶ��[��     

		SysTick->VAL = 0x00 ;         //�M�ŭp�ƾ�
		SysTick->CTRL= 0x01 ;       //�}�l�˼�   

		do 
			{
  
				temp=SysTick-> CTRL;
			}while (temp& 0x01 &&!(temp&( 1 << 16 ))); //���ݮɶ���F   

		SysTick->CTRL= 0x00 ;        //�����p�ƾ�

		SysTick->VAL = 0X00 ;        //�M�ŭp�ƾ�  
}

