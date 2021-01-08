/* --------------------------------------------------------- include ---------------------------------------------------------*/

#include "uart_app.h"
#include <string.h>

/* --------------------------------------------------------- define ---------------------------------------------------------*/

//#define MAX_UART_BUF		256
#define UART_AT		huart1
#define UART_DBG	huart3
/* --------------------------------------------------------- variables ---------------------------------------------------------*/
extern UART_HandleTypeDef UART_DBG;
extern UART_HandleTypeDef UART_AT;


//uint16_t uart_modbus_rx_ind = 0;
//uint8_t  uart_modbus_data;


/* 串口接收數據長度 */


at_rec_str_st_t at_rec_str_st;
at_tx_str_st_t  at_tx_str_st;

//char  uart_debug_rxbuf[MAX_LEN_UART_BUF];
//uint16_t dbg_rx_len;
//bool flag_dbg_recv = false;

//uint8_t Uart3RX_Data;
//uint8_t Uart3_Rx_Cnt = 0;		//接收緩衝計數


/* --------------------------------------------------------- function ---------------------------------------------------------*/

/**
  * @brief  point printf to huart3
	* @param  ch :char
  * @param  FILE :
  * @retval Nothing
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&UART_DBG, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}


/**
  * @brief  enable uart receive interrupt
  * @param  huart : uart event
  * @retval nothing
  */


void enable_at_receive(void)
{ 
	__HAL_UART_ENABLE_IT(&UART_AT,UART_IT_IDLE);//enable modbus interrupt
}

void start_at_dma_receive(void)
{
  HAL_UART_Receive_DMA(&UART_AT,(uint8_t*)at_rec_str_st.uart_at_rxbuf,MAX_LEN_UART_BUF);
}


/**
  * @brief  uart receive interrupt
  * @param  huart : uart event
  * @retval nothing
  */
void uart_app_init(void)
{
	enable_at_receive();
	at_rec_str_st.flag_at_recv_uart1=false;
}


void write_at_bytes(uint8_t *tx_msg, uint16_t len)
{
	HAL_UART_Transmit(&UART_AT,tx_msg, len, 0xffff ); 
}
