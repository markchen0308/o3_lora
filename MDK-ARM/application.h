#ifndef __APPLICATION_H
#define __APPLICATION_H
/* --------------------------------------------------------- include ---------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
/*-----------------------------------------------------------define---------------------------------------------------------------------*/
		
#define POS_NETWORK_ID	0
#define POS_DES_ROLE	  1
#define POS_DES_ID			2
#define POS_SRC_ROLE		3
#define POS_SRC_ID			4
#define POS_CMD_TYPE		5
#define POS_CMD_VALUE		6
#define POS_DATA_LEN		7
#define POS_DATA_START	POS_DATA_LEN+1



//----------------------------------------------------------------------------------------------------------------------------------------
void run_process(void);

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

uint16_t get_tx_head_and_value(uint8_t *tx_buf, uint8_t nw_id,uint8_t des_role,uint8_t des_id,uint8_t src_role, uint8_t src_id,uint8_t cmd_type,uint8_t cmd_val,uint8_t data_len ,uint8_t *val_buf);
//void check_cmd_from_gw(void);
//void check_cmd_from_node(void);

#endif  

