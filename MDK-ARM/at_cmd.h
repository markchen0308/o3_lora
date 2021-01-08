#ifndef __AT_CMD_H
#define __AT_CMD_H
/* --------------------------------------------------------- include ---------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/*-----------------------------------------------------------define---------------------------------------------------------------------*/
#define CMD_MAX_LEN 10
#define CMD_VALVE_MAX_LEN 200

#define AT_CMD_HEAD         "AT"
#define AT_CMD_SET          "="
#define AT_CMD_SET_CHAR     '='
#define AT_CMD_QUERY        "?"
#define AT_CMD_QUERY_CHAR   '?'
#define RETURN_CHAR         '\r'
#define RETURN_STR          "\r"

#define RETURN_NEWLINE      "\r\n"


typedef enum
{
    at_error,
    at_set_tx,
	  at_sync,
    at_query_tx,
    at_query_rx,
	  at_stop
}at_cmd_type_t;


typedef struct 
{
    at_cmd_type_t at_cmd_type;
    char at_cmd_buf[CMD_MAX_LEN];
    char at_val_buf[CMD_VALVE_MAX_LEN];
    uint16_t at_val_len;
    bool flag_at_cmd_ready;

}at_cmd_st_t;

/*-----------------------------------------------------------function---------------------------------------------------------------------*/
void init_at_cmd(at_cmd_st_t * cmd_st);
void parse_rx_cmd(char * rx_string,at_cmd_st_t * cmd_st);
void check_set_cmd(at_cmd_st_t * cmd_st);
void check_query_cmd(at_cmd_st_t * cmd_st);
void send_at_rx_cmd(uint8_t * data_buf,uint8_t node_id ,uint16_t len);
//void send_at_rx_cmd(uint8_t * data_buf,uint8_t node_id,uint8_t cmd_val ,uint16_t len);
#endif  

