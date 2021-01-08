/* --------------------------------------------------------- include ---------------------------------------------------------*/

#include "timer_app.h"
#include "uart_app.h"


/* --------------------------------------------------------- define---------------------------------------------------------*/


	
		
#define TIMER01_1MSEC 			htim1  //100Hz interrupt for long sync 
#define TIMER02_1MSEC 			htim2  //100Hz interrupt for slot sync 
#define TIMER03_1MSEC       htim3

//#define 

/* --------------------------------------------------------- variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef  TIMER01_1MSEC;//sync timer timer ,
extern TIM_HandleTypeDef  TIMER02_1MSEC;//sync period timer
extern TIM_HandleTypeDef  TIMER03_1MSEC;//slot timer

bool flag_timer01_1_msec_over=false;//timer interrupt flag
bool flag_timer02_1_msec_over=false;//timer interrupt flag
bool flag_timer03_1_msec_over=false;//timer interrupt flag
/* --------------------------------------------------------- function ---------------------------------------------------------*/
/**
  * @brief  timer ISR
	* @param  htim: timer handle
  * @retval Nothing
  */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIMER01_1MSEC.Instance)
		{
			    flag_timer01_1_msec_over=true;
			    __HAL_TIM_CLEAR_FLAG(&TIMER01_1MSEC, TIM_FLAG_CC1);//clear timer interrupt
			    HAL_TIM_Base_Stop_IT(&TIMER01_1MSEC);//stop timer's counting
		}
		else if(htim->Instance == TIMER02_1MSEC.Instance)
		{
			    flag_timer02_1_msec_over=true;
			    __HAL_TIM_CLEAR_FLAG(&TIMER02_1MSEC, TIM_FLAG_CC1);//clear timer interrupt
			    HAL_TIM_Base_Stop_IT(&TIMER02_1MSEC);//stop timer's counting
		}
		else if(htim->Instance == TIMER03_1MSEC.Instance)
		{
			    flag_timer03_1_msec_over=true;
			    __HAL_TIM_CLEAR_FLAG(&TIMER03_1MSEC, TIM_FLAG_CC1);//clear timer interrupt
			    HAL_TIM_Base_Stop_IT(&TIMER03_1MSEC);//stop timer's counting
		}
		
}



void reload_timer01_1msec(uint16_t count_1ms)
{
	__HAL_TIM_SET_AUTORELOAD(&TIMER01_1MSEC,count_1ms *10-1);//100-1 =>10ms, 200-1=>20ms
}

/**
  * @brief  start one second timer  
	* @param  Nothing
  * @retval Nothing
  */
void start_timer01_1msec(void)
{
	flag_timer01_1_msec_over=false;
	HAL_TIM_Base_Start_IT(&TIMER01_1MSEC);
}


/**
  * @brief  stop one second timer   
	* @param  Nothing
  * @retval Nothing
  */
void stop_timer01_1msec(void)
{
	 __HAL_TIM_CLEAR_FLAG(&TIMER01_1MSEC, TIM_FLAG_CC1);//clear timer interrupt
	 HAL_TIM_Base_Stop_IT(&TIMER01_1MSEC);
	flag_timer01_1_msec_over=false;
}


/**
  * @brief  check 10m-sec timer over  
	* @param  Nothing
  * @retval boolean
  */
bool is_timer01_1msec_over(void)
{
	return flag_timer01_1_msec_over;
}

/**
  * @brief  clear sampling over  
	* @param  Nothing
  * @retval Nothing
  */
void clear_timer01_1msec(void)
{
	flag_timer01_1_msec_over=false;
}





void reload_timer02_1msec(uint32_t count_1ms)
{
	//	flag_timer01_10_msec_over=false;
	__HAL_TIM_SET_AUTORELOAD(&TIMER02_1MSEC,count_1ms*10-1);//100-1 =>10ms, 200-1=>20ms
	//HAL_TIM_Base_Start_IT(&TIMER01_10MSEC);
}
/**
  * @brief  start one second timer  
	* @param  Nothing
  * @retval Nothing
  */
void start_timer02_1msec(void)
{
	flag_timer02_1_msec_over=false;
	HAL_TIM_Base_Start_IT(&TIMER02_1MSEC);
}


/**
  * @brief  stop one second timer   
	* @param  Nothing
  * @retval Nothing
  */
void stop_timer02_1msec(void)
{
	 __HAL_TIM_CLEAR_FLAG(&TIMER02_1MSEC, TIM_FLAG_CC1);//clear timer interrupt
	 HAL_TIM_Base_Stop_IT(&TIMER02_1MSEC);
	flag_timer02_1_msec_over=false;
}


/**
  * @brief  check 10m-sec timer over  
	* @param  Nothing
  * @retval boolean
  */
bool is_timer02_1msec_over(void)
{
	return flag_timer02_1_msec_over;
}

/**
  * @brief  clear sampling over  
	* @param  Nothing
  * @retval Nothing
  */
void clear_timer02_1msec(void)
{
	flag_timer02_1_msec_over=false;
}



void reload_timer03_1msec(uint16_t count_1ms)
{
	//	flag_timer01_10_msec_over=false;
	__HAL_TIM_SET_AUTORELOAD(&TIMER03_1MSEC,count_1ms*10-1);//100-1 =>10ms, 200-1=>20ms
	//HAL_TIM_Base_Start_IT(&TIMER01_10MSEC);
}
/**
  * @brief  start one second timer  
	* @param  Nothing
  * @retval Nothing
  */
void start_timer03_1msec(void)
{
	flag_timer03_1_msec_over=false;
	HAL_TIM_Base_Start_IT(&TIMER03_1MSEC);
}


/**
  * @brief  stop one second timer   
	* @param  Nothing
  * @retval Nothing
  */
void stop_timer03_1msec(void)
{
	 __HAL_TIM_CLEAR_FLAG(&TIMER03_1MSEC, TIM_FLAG_CC1);//clear timer interrupt
	 HAL_TIM_Base_Stop_IT(&TIMER03_1MSEC);
	flag_timer03_1_msec_over=false;
}


/**
  * @brief  check 10m-sec timer over  
	* @param  Nothing
  * @retval boolean
  */
bool is_timer03_1msec_over(void)
{
	return flag_timer03_1_msec_over;
}

/**
  * @brief  clear sampling over  
	* @param  Nothing
  * @retval Nothing
  */
void clear_timer03_1msec(void)
{
	flag_timer03_1_msec_over=false;
}

