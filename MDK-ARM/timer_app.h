#ifndef __TIMER_APP_H
#define __TIMER_APP_H
/* --------------------------------------------------------- include ---------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
/* --------------------------------------------------------- function announce ---------------------------------------------------------*/
void reload_timer01_1msec(uint16_t count_1ms);
void start_timer01_1msec(void);
void stop_timer01_1msec(void);
bool is_timer01_1msec_over(void);
void clear_timer01_1msec(void);

void reload_timer02_1msec(uint32_t count_1ms);
void start_timer02_1msec(void);
void stop_timer02_1msec(void);
bool is_timer02_1msec_over(void);
void clear_timer02_1msec(void);



void reload_timer03_1msec(uint16_t count_1ms);
void start_timer03_1msec(void);
void stop_timer03_1msec(void);
bool is_timer03_1msec_over(void);
void clear_timer03_1msec(void);
#endif  
