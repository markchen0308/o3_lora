

/* -------------------------------Includes ----------------------------------------*/
#include "lora_app.h"
#include "uart_app.h"
#include <string.h>
#include "application.h"

/* --------------------------------------------------------- define ---------------------------------------------------------*/


/* --------------------------------------------------------- variables ---------------------------------------------------------*/
extern const struct Radio_s Radio;

RadioEvents_t radio_events;
lora_package_st_t lora_package_st;

//lora_role_t lora_role=node;// node or gateway
/* --------------------------------------------------------- function ---------------------------------------------------------*/





uint32_t get_lora_working_freq(uint32_t net_id)
{
	uint32_t freq=((net_id-1) % 9) *1000000+RF_MAIN_FREQUENCY;
	return freq;
}

void init_lora(void)
{

	lora_package_st.working_freq=get_lora_working_freq(LORA_NETWORK_ID);
//	lora_package_st.state=LOWPOWER;
	lora_package_st.flag_init_sync=false;
	
	  radio_events.TxDone = OnTxDone;
    radio_events.RxDone = OnRxDone;
    radio_events.TxTimeout = OnTxTimeout;
    radio_events.RxTimeout = OnRxTimeout;
    radio_events.RxError = OnRxError;
    
    Radio.Init( &radio_events );
    Radio.SetChannel( lora_package_st.working_freq );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.SetMaxPayloadLength( MODEM_LORA, LORA_MAX_BUF_LEN );
	  
	  lora_package_st.rx_timeout=1000;//rx for 1000 ms
    //Radio.Rx( lora_package_st.rx_timeout );
		
		dbg_printf("Network ID : %d\n",LORA_NETWORK_ID);

#if LORA_ROLE == LORA_ROLE_NODE
		dbg_printf("Role:Node , Node ID:%d\n",LORA_NODE_ID);

#else
		dbg_printf("Role:Gateway , Gateway ID:%d\n",LORA_GW_ID);
		dbg_printf("Input 'AT+SYNC' to start network.\n");
		
		
#endif
	  
		

		
}









