#ifndef __UART_APP_H
#define __UART_APP_H
/* --------------------------------------------------------- include ---------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"
/*-----------------------------------------------------------define---------------------------------------------------------------------*/


//#define DEBUG

#ifdef DEBUG
    #define dbg_printf(fmt, args...) printf(fmt,##args)
#else
    #define dbg_printf(fmt, ...)
#endif

#define MAX_LEN_UART_BUF   72


typedef struct 
{
		char  uart_at_rxbuf[MAX_LEN_UART_BUF];
	  uint16_t at_rx_len_uart1;
	  uint16_t at_rx_len_uart3;
	  bool flag_at_recv_uart1 ;
		bool flag_at_recv_uart3 ;
}at_rec_str_st_t;

typedef struct 
{
		uint8_t  uart_at_txbuf[MAX_LEN_UART_BUF];
	  uint16_t at_tx_len;
	 // bool flag_at_recv ;
}at_tx_str_st_t;

/* --------------------------------------------------------- function announce ---------------------------------------------------------*/

int fputc(int ch, FILE *f);

void uart_app_init(void);

void start_at_dma_receive(void);
void write_at_bytes(uint8_t *tx_msg, uint16_t len);

#endif  
