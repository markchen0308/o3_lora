#ifndef __LORA_APP_H
#define __LORA_APP_H
/* --------------------------------------------------------- include ---------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "radio.h"
#include "at_cmd.h"
/*-----------------------------------------------------------define---------------------------------------------------------------------*/





#define LORA_ROLE_NODE																1
#define LORA_ROLE_GATEWAY															2
#define LORA_BROADCAST_ID															0

#define MAX_COUNT_NODE_SEND														25

#define MODE_NORMAL																	 	1
#define MODE_PKG_LOSS_TEST														2
/*-----------------------------------------------------------modifiable  define---------------------------------------------------------------------*/

#define TX_OUTPUT_POWER                               22        // dBm
#define NUM_NODES                                    	11    										//lora node count
#define LORA_NETWORK_ID																1   											//1~7
#define LORA_ROLE																		 	LORA_ROLE_NODE						//LORA_ROLE_NODE or  LORA_ROLE_GATEWAY
#define LORA_GW_ID																		1                       	//gateway id 1~254
#define LORA_NODE_ID												          1                       	//node id 1~ NUM_NODES
#define MODE_WORK																			MODE_NORMAL    		//MODE_NORMAL , MODE_PKG_LOSS_TEST


/*-----------------------------------------------------------define---------------------------------------------------------------------*/


#define RF_MAIN_FREQUENCY                  					920000000 // Hz

#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define LORA_MAX_BUF_LEN                            128 // Define the payload size here
#define LORA_MAX_TX_BUF_LEN													128
#define LORA_MAX_RX_BUF_LEN													128
#define LORA_MAX_RX_DATA_BUF_LEN										LORA_MAX_RX_BUF_LEN
#define LORA_ROLE_NODE															1



#define LORA_MAX_SYNC_COUNT                         5

#define LORA_SYN_TX_TIME														10   //10ms tx sync time



#define LORA_SYNC_SLOT_TIME													LORA_SYN_TX_TIME+20   //20msec sync slot




#define LORA_SYNC_SLOT_TIMER_PERIOD									(10*LORA_SYNC_SLOT_TIME -1) //assign timer1    

#if LORA_ROLE == LORA_ROLE_NODE //for node
    #define LORA_SYNC_PERIOD														500//-LORA_SYNC_SLOT_TIMER_PERIOD//uint:m sec
							
#else          //for gateway
    #define LORA_SYNC_PERIOD														500 //uint:m sec
						
#endif
#define LORA_SYNC_PERIOD_TIMER_PERIOD								(10*LORA_SYNC_PERIOD -1) //assign timer2


#define LORA_NODE_TX_SLOT_TIME											(50)
#define LORA_GW_RX_SLOT_TIME												LORA_NODE_TX_SLOT_TIME   //20msec sync slot
#define LORA_GW_TX_RX_SLOT_TIME											LORA_GW_RX_SLOT_TIME +50 //>50



#if LORA_ROLE == LORA_ROLE_NODE //for node
    #define LORA_TX_RX_SLOT_TIMER_PERIOD								(10*LORA_GW_TX_RX_SLOT_TIME -1)  //assign timer3 
							
#else          //for gateway
    #define LORA_TX_RX_SLOT_TIMER_PERIOD								(10*LORA_GW_TX_RX_SLOT_TIME -1)  //assign timer3 
		#define LORA_GW_TOTAL_RX_TIME_WINDOW								((NUM_NODES)*MAX_COUNT_NODE_SEND*LORA_GW_TX_RX_SLOT_TIME*10-1)			
					
#endif





#define LORA_NODE_SYNC_OFFSET										(-1*(LORA_SYNC_SLOT_TIME+25))//	   (-1*(LORA_SYNC_SLOT_TIME+25))



typedef enum
{
	  //gateway state machine
	  GW_WAIT_SYNC,
    GW_SYNC,
		GW_WAIT_RX,
	  GW_WAIT_RX_STOP,
	  
	  //node state machine
	  NODE_WAIT_SYNC,
	  NODE_WAIT_TX,
	  NODE_TX_GW_SLOT,
	  NODE_WAIT_TX_STOP,
	
}States_t;


typedef struct
{

	 uint32_t working_freq;
	 RadioEvents_t radio_events;
	 States_t state;
	
	 uint16_t lora_tx_len;
   uint8_t lora_tx_buf[LORA_MAX_TX_BUF_LEN];
	
	 uint16_t lora_rx_len;
   uint8_t lora_rx_buf[LORA_MAX_RX_BUF_LEN];
	 	
	 uint16_t rx_timeout;
	 uint16_t tx_timeout;
	 int8_t rssi ;
	 int8_t snr ;
	 

	 uint8_t rx_id;
	 at_cmd_type_t rx_cmd_type;
	 uint8_t rx_cmd_val;
	 uint8_t rx_data_len;
	 uint8_t lora_rx_data_buf[LORA_MAX_RX_DATA_BUF_LEN];
	 bool flag_init_sync;
	 bool flag_lora_rx_parse;
	
}lora_package_st_t;
																	


/*-----------------------------------------------------------function---------------------------------------------------------------------*/

uint32_t get_lora_working_freq(uint32_t net_id);
void init_lora(void);





#endif  
