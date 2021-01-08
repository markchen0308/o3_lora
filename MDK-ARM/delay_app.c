

#include <stdint.h>
#include "main.h"

static uint32_t fac_us= 0 ; // us┑计
static uint32_t fac_ms= 0 ; // ms┑计

void delay_init(uint8_t SYSCLK)
{
	SysTick->CTRL&= 0xfffffffb ; // bit2睲,匡拒场牧HCLK/8 
	fac_us=SYSCLK/ 8 ;     
	fac_ms=(uint32_t)fac_us* 1000 ;
}     



void delay_ms(uint32_t nms)
{        
 
  uint32_t temp;    
	SysTick->LOAD=(uint32_t)nms*fac_ms; //丁更(SysTick->LOAD24bit) 

	SysTick->VAL = 0x00 ;            //睲璸计竟

	SysTick->CTRL= 0x01 ;          //秨﹍计  

	do 
		{
 
			temp=SysTick-> CTRL;
 
		}while (temp& 0x01 &&!(temp&( 1 << 16 ))); //单丁笷   

		SysTick->CTRL= 0x00 ;        //闽超璸计竟
 
		SysTick->VAL = 0X00 ;        //睲璸计竟        
  
}   






//┑nus

// nus璶┑us计.          

void delay_us(uint32_t nus)
{
		uint32_t temp;       
		SysTick->LOAD=nus*fac_us; //丁更     

		SysTick->VAL = 0x00 ;         //睲璸计竟
		SysTick->CTRL= 0x01 ;       //秨﹍计   

		do 
			{
  
				temp=SysTick-> CTRL;
			}while (temp& 0x01 &&!(temp&( 1 << 16 ))); //单丁笷   

		SysTick->CTRL= 0x00 ;        //闽超璸计竟

		SysTick->VAL = 0X00 ;        //睲璸计竟  
}

