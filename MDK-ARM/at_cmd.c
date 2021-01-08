
/* -------------------------------Includes ----------------------------------------*/
#include "at_cmd.h"
#include <uart_app.h>






//at_cmd_st_t at_cmd_st;
extern at_tx_str_st_t  at_tx_str_st;

void init_at_cmd(at_cmd_st_t * cmd_st)
{
	 cmd_st->flag_at_cmd_ready=false;
}

void check_set_cmd(at_cmd_st_t * cmd_st)
{
    if(strcmp(cmd_st->at_cmd_buf,"AT+TX")==0)
    {
        cmd_st->at_cmd_type=at_set_tx;
        cmd_st->flag_at_cmd_ready=true;
        dbg_printf("set cmd type=%d\n",cmd_st->at_cmd_type);
    }

    else
    {
        cmd_st->at_cmd_type=at_error;
        cmd_st->flag_at_cmd_ready=true;
    }
}

void check_act_cmd(at_cmd_st_t * cmd_st)
{
		if(strcmp(cmd_st->at_cmd_buf,"AT+SYNC")==0)
    {
        cmd_st->at_cmd_type=at_sync;
        cmd_st->flag_at_cmd_ready=true;
			  dbg_printf("Get at cmd: AT+SYNC\n");
    }
		else if(strcmp(cmd_st->at_cmd_buf,"AT+STOP")==0)
		{
			  cmd_st->at_cmd_type=at_stop;
        cmd_st->flag_at_cmd_ready=true;
			  dbg_printf("Get at cmd: AT+STOP\n");
        //dbg_printf("act cmd type=%d\n",cmd_st->at_cmd_type);
		}
    else
    {
        cmd_st->at_cmd_type=at_error;
        cmd_st->flag_at_cmd_ready=true;
    }
}


void check_query_cmd(at_cmd_st_t * cmd_st)
{
    if(strcmp(cmd_st->at_cmd_buf,"AT+TX")==0)
    {
        cmd_st->at_cmd_type=at_query_tx;
        cmd_st->flag_at_cmd_ready=true;
        dbg_printf("query cmd type=%d\n",cmd_st->at_cmd_type);
    }
    else
    {
        cmd_st->at_cmd_type=at_error;
        cmd_st->flag_at_cmd_ready=true;
    }
}

void parse_rx_cmd(char * rx_string,at_cmd_st_t * cmd_st)
{
    uint16_t len;
	  printf("%s \n",rx_string);
	if(strstr(rx_string,AT_CMD_HEAD) && strstr(rx_string,RETURN_NEWLINE) )
	{
		  if(strstr(rx_string, AT_CMD_SET) && (len = (strchr(rx_string,AT_CMD_SET_CHAR) - rx_string)) > 0 && len < CMD_MAX_LEN)//setting
			{
          char * char_pos_start=strchr(rx_string, AT_CMD_SET_CHAR);
          char * char_pos_end=strchr(rx_string, RETURN_CHAR);
          uint16_t offset_val=char_pos_start-rx_string+1;
  
                    
					strncpy(cmd_st->at_cmd_buf , rx_string, char_pos_start - rx_string); 
					cmd_st->at_val_buf[len] = 0; // null-terminate the string

                    cmd_st->at_val_len=char_pos_end-char_pos_start-1;
					strncpy(cmd_st->at_val_buf, rx_string+offset_val, cmd_st->at_val_len); 
					cmd_st->at_val_buf[cmd_st->at_val_len] = 0; // null-terminate the string

          dbg_printf("set cmd: %s\n",cmd_st->at_cmd_buf);
          dbg_printf("value #bytes:%d\n",cmd_st->at_val_len);
          for(uint16_t i=0;i<cmd_st->at_val_len;i++)
          {
              printf("%02x ",cmd_st->at_val_buf[i]);
          }
          dbg_printf("\n");
          check_set_cmd(cmd_st);
                    
			}
			else if(strstr(rx_string, AT_CMD_QUERY) && (len = (strchr(rx_string, AT_CMD_QUERY_CHAR) - rx_string)) > 0 && len < CMD_MAX_LEN)//query
			{
					strncpy(cmd_st->at_cmd_buf, rx_string, strchr(rx_string, AT_CMD_QUERY_CHAR) - rx_string); 
					cmd_st->at_cmd_buf[len] = 0; // null-terminate the string
          cmd_st->at_val_len=0;
          dbg_printf("query cmd: %s\n",cmd_st->at_cmd_buf);
          check_query_cmd(cmd_st);
			}
			else if(strstr(rx_string, RETURN_STR) && (len = (strchr(rx_string, RETURN_CHAR) - rx_string)) > 0 && len < CMD_MAX_LEN)//act
			{
					strncpy(cmd_st->at_cmd_buf, rx_string, strchr(rx_string, RETURN_CHAR) - rx_string); 
					cmd_st->at_cmd_buf[len] = 0; // null-terminate the string
          cmd_st->at_val_len=0;
          dbg_printf("act cmd: %s\n",cmd_st->at_cmd_buf);
          check_act_cmd(cmd_st);
			}
			else
			{
				
				 dbg_printf("cmd error\n");
         cmd_st->at_cmd_type=at_error;
         cmd_st->flag_at_cmd_ready=true;
			}
	}
  else
  {
         dbg_printf("cmd error\n");
         cmd_st->at_cmd_type=at_error;
         cmd_st->flag_at_cmd_ready=true;
  }
}


void send_at_rx_cmd(uint8_t * data_buf,uint8_t node_id ,uint16_t len)
{
	char str[10];
	uint8_t len_temp;
	if(len > 0)
	{
		len_temp=sprintf(str,"AT+RX=%d,",node_id);
		memcpy(at_tx_str_st.uart_at_txbuf , str,len_temp); //copy at cmmd head
		memcpy(at_tx_str_st.uart_at_txbuf+len_temp, data_buf, len); //copy data
		len_temp+=len;
	}
	else
	{
		len_temp=sprintf(str,"AT+RX=%d",node_id);
		memcpy(at_tx_str_st.uart_at_txbuf , str,len_temp); //copy at cmmd head
	}
	memcpy(at_tx_str_st.uart_at_txbuf+len_temp, RETURN_NEWLINE,2); // copy '/r/n'
	len_temp+=2;
	write_at_bytes(at_tx_str_st.uart_at_txbuf,len_temp);


}

