
/* -------------------------------Includes ----------------------------------------*/
#include "uart_app.h"
#include "at_cmd.h"
#include "lora_app.h"
#include "timer_app.h"
#include "application.h"
#include <string.h>
#include <stdlib.h>
/* -------------------------------define----------------------------------------*/


/* -------------------------------variable ----------------------------------------*/
extern const struct Radio_s Radio;
extern at_rec_str_st_t at_rec_str_st;

extern lora_package_st_t lora_package_st;
bool flag_overcome_dma_bug=false;
uint16_t syn_count;
#if LORA_ROLE == LORA_ROLE_NODE  //for node
			uint16_t count_node_send=0;
      uint16_t sync_count=0; 		
			uint16_t node_time;
			uint32_t clear_count=0;
			#if MODE_WORK == MODE_PKG_LOSS_TEST
				uint8_t pkg_test_data[]="1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21";
				uint8_t pkg_data[64];
				uint32_t sync_num=0;
			#endif
#else  //for gateway
	  int16_t count_gw_rx=0;
		bool flag_gw_is_listen_node_slot=false;
		uint16_t gw_stop_time;
		uint32_t gw_got_data_times=0;
		uint32_t gw_total_slot_count=NUM_NODES*MAX_COUNT_NODE_SEND;
		bool flag_gw_sync=false;
		//uint16_t 
#endif

/* -------------------------------fucntion ----------------------------------------*/


void OnTxDone( void )
{
	  
    Radio.Sleep( );
	//	lora_package_st.state=TX_DONE;
	 // dbg_printf("OnTxDone\n");
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
		
#if LORA_ROLE == LORA_ROLE_NODE
	Radio.Sleep( );
#else
		if(flag_gw_is_listen_node_slot)
		{
			Radio.Rx(0);
		}
		else
		{
			Radio.Sleep( );
		}
#endif	

		
	  lora_package_st.lora_rx_len=size;
    memcpy(lora_package_st.lora_rx_buf, payload, lora_package_st.lora_rx_len);
	  lora_package_st.rssi=rssi;
	  lora_package_st.snr=snr;
	 
	  
	  //dbg_printf("received:");
		//for(uint16_t i=0;i<lora_package_st.lora_rx_len;i++)
		//{
			//dbg_printf("%02x ",lora_package_st.lora_rx_buf[i]);
		//}
		//dbg_printf("\n");
		
		lora_package_st.flag_lora_rx_parse=true;
	  

}

void OnTxTimeout( void )
{
    Radio.Sleep( );
	 // lora_package_st.state=TX_TIMEOUT;
	  dbg_printf("OnTxTimeout\n");
}

void OnRxTimeout( void )
{

	dbg_printf("OnRxTimeout\n");
	
#if LORA_ROLE == LORA_ROLE_NODE
	if(lora_package_st.state==NODE_WAIT_SYNC)
	{
		Radio.Rx(0);
	}
	else
	{
		Radio.Sleep( );
	}

#else
	Radio.Sleep( );
#endif	
	
	
}

void OnRxError( void )
{
    Radio.Sleep( );
    //lora_package_st.state= RX_ERROR;
	  dbg_printf("OnRxError\n");
}
//---------------------------------------------------------------------------------------------






//--------------------------------------------------------------------------------------------
uint16_t get_tx_head_and_value(	uint8_t *tx_buf, 
																uint8_t nw_id,
																uint8_t des_role,
																uint8_t des_id,
																uint8_t src_role, 
																uint8_t src_id,
																uint8_t cmd_type,
																uint8_t cmd_val,
																uint8_t data_len ,
																uint8_t *val_buf)
{
	tx_buf[POS_NETWORK_ID]=nw_id;
	tx_buf[POS_DES_ROLE]=des_role;
	tx_buf[POS_DES_ID]=des_id;
	tx_buf[POS_SRC_ROLE]=src_role;
	tx_buf[POS_SRC_ID]=src_id;
	tx_buf[POS_CMD_TYPE]=cmd_type;
	tx_buf[POS_CMD_VALUE]=cmd_val;
	tx_buf[POS_DATA_LEN]=data_len;
	memcpy(tx_buf+POS_DATA_START , val_buf,data_len); //copy data

	return POS_DATA_LEN+1+data_len;
}

//--------------------------------------------------------------------------------------------
bool lora_rx_parse(uint8_t *rx_buf,uint8_t *data_buf)
{
	if(rx_buf[POS_NETWORK_ID] != LORA_NETWORK_ID)
	{
		//dbg_printf("err 0\n");
		return false;
	}
#if LORA_ROLE == LORA_ROLE_NODE
	
	if(rx_buf[POS_DES_ROLE] != LORA_ROLE_NODE)
	{
		//dbg_printf("err 1\n");
		return false;
	}
	if((rx_buf[POS_DES_ID] != LORA_NODE_ID) && (rx_buf[POS_DES_ID] != 0))
	{	
		//dbg_printf("err 2\n");
		return false;
	}
	//is from gateway and gateway id is same as LORA_GW_ID
	if((rx_buf[POS_SRC_ROLE] != LORA_ROLE_GATEWAY) && (rx_buf[POS_SRC_ID] != LORA_GW_ID))
	{	
		//dbg_printf("err 3\n");
		return false;
	}
	lora_package_st.rx_id=rx_buf[POS_SRC_ID];
	lora_package_st.rx_cmd_type=rx_buf[POS_CMD_TYPE];
	lora_package_st.rx_cmd_val=rx_buf[POS_CMD_VALUE];
	lora_package_st.rx_data_len=rx_buf[POS_DATA_LEN];
	memcpy(data_buf , rx_buf+POS_DATA_START,lora_package_st.rx_data_len); //copy data
	//for(uint16_t i=0;i<lora_package_st.rx_data_len;i++)
	//{
	//	data_buf[i]=rx_buf[i+POS_DATA_START];
	//}
	return true;
#else
	if(rx_buf[POS_DES_ROLE] != LORA_ROLE_GATEWAY)
	{
		dbg_printf("err 1\n");
		return false;
	}
	if((rx_buf[POS_DES_ID] != LORA_GW_ID))
	{	
		dbg_printf("err 2\n");
		return false;
	}
	
	if(rx_buf[POS_SRC_ROLE] != LORA_ROLE_NODE)
	{	
		dbg_printf("err 3\n");
		return false;
	}
	
	lora_package_st.rx_id=rx_buf[POS_SRC_ID];
	lora_package_st.rx_cmd_type=rx_buf[POS_CMD_TYPE];
	lora_package_st.rx_cmd_val=rx_buf[POS_CMD_VALUE];
	lora_package_st.rx_data_len=rx_buf[POS_DATA_LEN];
	
	memcpy(data_buf , rx_buf+POS_DATA_START,lora_package_st.rx_data_len); //copy data
	//for(uint16_t i=0;i<lora_package_st.rx_data_len;i++)
	//{
	//	data_buf[i]=rx_buf[i+POS_DATA_START];
//	}
	

	return true;
#endif
	

}


//--------------------------------------------------------------------------------------------
#if LORA_ROLE == LORA_ROLE_NODE
void check_cmd_from_gw(void)
{
	
	switch(lora_package_st.rx_cmd_type)
	{
		case at_sync://sync cmd from gateway
		  
			node_time=(LORA_SYNC_PERIOD - (lora_package_st.rx_cmd_val-1)*LORA_SYNC_SLOT_TIME) + (LORA_NODE_ID)*LORA_NODE_TX_SLOT_TIME + LORA_NODE_SYNC_OFFSET;
		// dbg_printf("sync %d\n",start_node_tx);
		  count_node_send=0;
			sync_count=0;
		  reload_timer03_1msec(node_time);
      start_timer03_1msec();
			lora_package_st.state=NODE_WAIT_TX;
		  dbg_printf("Got GW cmd:sync#%d\n",lora_package_st.rx_cmd_val);
			break;
		
		default:
			break;
	}
}
#endif
//--------------------------------------------------------------------------------------------
#if LORA_ROLE == LORA_ROLE_GATEWAY
void check_cmd_from_node(void)
{

	switch(lora_package_st.rx_cmd_type)
	{
		case at_set_tx://get node tx cmd
			send_at_rx_cmd(lora_package_st.lora_rx_data_buf,lora_package_st.rx_id,lora_package_st.rx_data_len);//send data to uart
		  //send_at_rx_cmd(&lora_package_st.rx_cmd_val,lora_package_st.rx_id,lora_package_st.rx_data_len);//send data to uart
		
		  gw_got_data_times++;
			break;
		
		default:
			break;
	}
}

#endif

//--------------------------------------------------------------------------------------------
void run_process(void)
{
	uint16_t n_char;
//	bool flag_overcome_dma_bug=false;



	at_cmd_st_t at_cmd_st;
	
	dbg_printf("----------------System Runnung----------------\n");
	
	init_at_cmd(&at_cmd_st);//initialize at cmd
	init_lora();//init lora
//	Radio.Rx(0);
	HAL_Delay(1);
#if LORA_ROLE == LORA_ROLE_NODE  //for node
	    uint8_t len_temp;
			Radio.Rx(0);//node start to listen gateway syn signaling
			lora_package_st.state=NODE_WAIT_SYNC;//init state machine to wait time sync
#else  //for gateway
			lora_package_st.state=GW_WAIT_SYNC;
#endif
	
	
	lora_package_st.flag_lora_rx_parse=false;
  start_at_dma_receive(); //enable to receive AT cmd by DMA
	while(true)
	{
			//start_at_dma_receive(); //enable to receive AT cmd by DMA
			
		
		//get uart dma  
		if(at_rec_str_st.flag_at_recv_uart1)//get at-cmd
			{
					at_rec_str_st.flag_at_recv_uart1=false;
				  start_at_dma_receive(); //enable to receive AT cmd by DMA
					if(flag_overcome_dma_bug!=false)
					{
						parse_rx_cmd(at_rec_str_st.uart_at_rxbuf,&at_cmd_st);//sent at-cmd to at-cmd parser
              
					}
					else
					{
						flag_overcome_dma_bug=true;
					}
			}
			
			
			
			
			//  for at cmd
			
			if(at_cmd_st.flag_at_cmd_ready)//at cmd ready
			{
					at_cmd_st.flag_at_cmd_ready=false;
					switch(at_cmd_st.at_cmd_type)
					{
						case at_error:
							write_at_bytes((uint8_t *)"ERROR\r\n",7);
							break;
						
						case at_sync: //gateway start to sync 
							
#if LORA_ROLE == LORA_ROLE_NODE //for node
							write_at_bytes((uint8_t *)"ERROR\r\n",7);//this cmd is not for node
#else          //for gateway
							write_at_bytes((uint8_t *)"OK\r\n",4);//reply at cmd ok

						  //syn_count=0;

						//  
						  
							//start_timer03_1msec();//start  sync timer
						  //flag_gw_sync=true;
							//start_timer01_1msec();//start  timer
						  
						  //dbg_printf("syn_count 1=%d\n",syn_count);
							
							lora_package_st.state=GW_WAIT_RX_STOP;
#endif		
									break;
						
						case at_set_tx:
				
							write_at_bytes((uint8_t *)"OK\r\n",4);
							
							//save at data and wait for tx
							lora_package_st.lora_tx_len= get_tx_head_and_value(lora_package_st.lora_tx_buf,
																								LORA_NETWORK_ID,
																								LORA_ROLE_GATEWAY,
																								lora_package_st.rx_id,
																								LORA_ROLE_NODE, 
																								LORA_NODE_ID,at_set_tx,0,at_cmd_st.at_val_len ,(uint8_t *)at_cmd_st.at_val_buf);		
							
							break;
						
						
						case at_stop:
#if LORA_ROLE == LORA_ROLE_NODE //for node
							write_at_bytes((uint8_t *)"ERROR\r\n",7);//this cmd is not for node
#else          //for gateway
							write_at_bytes((uint8_t *)"OK\r\n",4);//reply at cmd ok
              
							stop_timer01_1msec();//start  sync timer
						  stop_timer02_1msec();//start  sync timer
							stop_timer03_1msec();//start  sync timer
							Radio.Sleep();
							lora_package_st.state=GW_WAIT_SYNC;
#endif								
							break;
						

						
						default:
							break;
					}
			}
			
			
			
		// get lora rx	
			
		if( lora_package_st.flag_lora_rx_parse)//received lora cmd
		{
				lora_package_st.flag_lora_rx_parse=false;
			
#if LORA_ROLE == LORA_ROLE_NODE  //for node
				if(lora_rx_parse(lora_package_st.lora_rx_buf,lora_package_st.lora_rx_data_buf))
				{
					//dbg_printf("rx from gw\n");
					check_cmd_from_gw();
				}
				else
				{
					dbg_printf("gw cmd error h!\n");
				}
#else    //for gateway
				if(lora_rx_parse(lora_package_st.lora_rx_buf,lora_package_st.lora_rx_data_buf))
				{
					//dbg_printf("rx_cmd from node\n");
					check_cmd_from_node();
				}
				else
				{
					//#if MODE_WORK == MODE_PKG_LOSS_TEST
							dbg_printf("rx_cmd fail\n");
					//#endif
				}
#endif			
		}			
			
			
		switch(lora_package_st.state)
		{

			
#if LORA_ROLE == LORA_ROLE_GATEWAY
			case GW_WAIT_SYNC:
				break;
			
			case GW_SYNC://only for gateway ,init sync process
				
						if( is_timer01_1msec_over())  
						{
								clear_timer01_1msec();
							  
								if( syn_count<LORA_MAX_SYNC_COUNT)
								{
									 
									  start_timer01_1msec();//start  timer
										
										n_char= get_tx_head_and_value(lora_package_st.lora_tx_buf,
																									LORA_NETWORK_ID,
																									LORA_ROLE_NODE,
																									LORA_BROADCAST_ID,
																									LORA_ROLE_GATEWAY, 
																									LORA_GW_ID,at_sync,syn_count++,0 ,NULL);								  
									  Radio.Send(lora_package_st.lora_tx_buf,n_char);
									 
								}
								else
								{
									//#if MODE_WORK == MODE_PKG_LOSS_TEST
										dbg_printf("GW sync. is over. Wait for node data.\n");
									//#endif
									syn_count=0;
									count_gw_rx=0;
								  lora_package_st.state=GW_WAIT_RX;//GW wait for signal from node
								}
						}
						else if(flag_gw_sync)
						{
							flag_gw_sync=false;
							start_timer01_1msec();//start  timer
										
							n_char= get_tx_head_and_value(lora_package_st.lora_tx_buf,
																									LORA_NETWORK_ID,
																									LORA_ROLE_NODE,
																									LORA_BROADCAST_ID,
																									LORA_ROLE_GATEWAY, 
																									LORA_GW_ID,at_sync,syn_count++,0 ,NULL);								  
							Radio.Send(lora_package_st.lora_tx_buf,n_char);
						 
						}

				break;
				
			case GW_WAIT_RX://only for GW to wait for rx 

					if(is_timer02_1msec_over())
					{
						  clear_timer02_1msec();
							flag_gw_is_listen_node_slot=false;
						  Radio.Sleep();
						  for(uint8_t i=0;i<NUM_NODES;i++)
						  {
								HAL_Delay(LORA_GW_TX_RX_SLOT_TIME);
							}
              
              #if MODE_WORK == MODE_PKG_LOSS_TEST
									dbg_printf("GW got %d times of node data. Wait next sync.\n",gw_got_data_times);
							#endif	
									count_gw_rx=0;
							    gw_got_data_times=0;
			            syn_count=0;   
			            flag_gw_sync=true;
						  lora_package_st.state=GW_WAIT_RX_STOP;//GW wait for signal from node
					}
					else 	if( is_timer03_1msec_over())
					{
						  clear_timer03_1msec();
							start_timer02_1msec();//start  timer
						  flag_gw_is_listen_node_slot=true;
							Radio.Rx(LORA_GW_TOTAL_RX_TIME_WINDOW+10);
						  count_gw_rx++;
					}
				break;
					
					
					
			
					
					
					
					
			case GW_WAIT_RX_STOP:
				
							    count_gw_rx=0;
							    gw_got_data_times=0;
			            syn_count=0;   
			            flag_gw_sync=true;
							    start_timer03_1msec();//start  sync timer
									flag_gw_sync=true;
			            
							    lora_package_st.state=GW_SYNC;//GW sync
							    
							
										
				break;
						
				   
#endif						
			
#if LORA_ROLE == LORA_ROLE_NODE

			case NODE_WAIT_SYNC:
				#if MODE_WORK == MODE_PKG_LOSS_TEST
			/*
					if(clear_count <40000000)
					{
						clear_count++;
					}
					else
					{
						sync_num=0;
						clear_count=0;
					}
			*/
				#endif
				break;
						
			case NODE_WAIT_TX://only for node 
				
			      if(is_timer02_1msec_over())
						{
							clear_timer02_1msec();
							if(count_node_send< (MAX_COUNT_NODE_SEND))
							 { 
										
										start_timer02_1msec();//start  timer


										
										#if MODE_WORK == MODE_PKG_LOSS_TEST
								          
													len_temp=sprintf((char *)pkg_data,"%d,",sync_num++);
													memcpy(pkg_data+len_temp , pkg_test_data,53); //copy data
								          count_node_send++;
													n_char= get_tx_head_and_value(lora_package_st.lora_tx_buf,
																											LORA_NETWORK_ID,
																											LORA_ROLE_GATEWAY,
																											lora_package_st.rx_id,
																											LORA_ROLE_NODE, 
																											LORA_NODE_ID,at_set_tx,0,53+len_temp ,pkg_data);		
													Radio.Send(lora_package_st.lora_tx_buf,n_char);//send empty package
										
										#else			
													if(lora_package_st.lora_tx_len >0)
													{
														Radio.Send(lora_package_st.lora_tx_buf,lora_package_st.lora_tx_len);
														lora_package_st.lora_tx_len=0;
													}
													count_node_send++;
										
										#endif												
								 
							 }
							 else
							 {
		                 dbg_printf("Node sent %d times data. wait GW sync cmd.\n",count_node_send);

           
								     for(uint8_t i=0;i<(NUM_NODES-LORA_NODE_ID);i++)
											{
												HAL_Delay(LORA_GW_TX_RX_SLOT_TIME);
											}
										  lora_package_st.state=NODE_WAIT_TX_STOP;//GW wait for signal from node

							 }		
							
							
							
							
							
						}
						else if( is_timer03_1msec_over())
						{
							    
									clear_timer03_1msec();
									start_timer02_1msec();
							 
									#if MODE_WORK == MODE_PKG_LOSS_TEST
												  len_temp=sprintf((char *)pkg_data,"%d,",sync_num++);
													memcpy(pkg_data+len_temp , pkg_test_data,53); //copy data
								          count_node_send++;
													n_char= get_tx_head_and_value(lora_package_st.lora_tx_buf,
																											LORA_NETWORK_ID,
																											LORA_ROLE_GATEWAY,
																											lora_package_st.rx_id,
																											LORA_ROLE_NODE, 
																											LORA_NODE_ID,at_set_tx,0,53+len_temp ,pkg_data);		
													Radio.Send(lora_package_st.lora_tx_buf,n_char);//send empty package
										
									#else			
													if(lora_package_st.lora_tx_len >0)
													{
														Radio.Send(lora_package_st.lora_tx_buf,lora_package_st.lora_tx_len);
														lora_package_st.lora_tx_len=0;
													}
													count_node_send++;
										
									#endif			
						}
				
					break;
						
			case NODE_WAIT_TX_STOP:
									
							    count_node_send=0;
			            sync_count=0;   
									Radio.Rx(0);//node start to listen gateway syn signaling
							    lora_package_st.state=NODE_WAIT_SYNC;//GW wait for signal from node
						
				break;
						

						

#endif						
						
			default:
				break;
		}			
		
		
		
	}
}




