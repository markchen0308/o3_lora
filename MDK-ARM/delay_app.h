


#ifndef __DELAY_APP_H
#define __DELAY_APP_H
/* --------------------------------------------------------- include ---------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"
/* --------------------------------------------------------- function announce ---------------------------------------------------------*/




/*-----------------------------------------------------------define---------------------------------------------------------------------*/
void delay_init(uint32_t SYSCLK);

void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);


#endif  
