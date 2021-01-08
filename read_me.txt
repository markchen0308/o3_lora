// o3_lora comment

1.Open file lora_01.uvprojx in ./MDK-ARM/ with keil IDE "uVison 5.xx".

2.Change following paramter in lora_app.h for gateway or node 

/*-----------------------------------------------------------modifiable  define---------------------------------------------------------------------*/

#define TX_OUTPUT_POWER                               22                        // tx powerr dBm

#define NUM_NODES                                    	11    										//lora node count

#define LORA_NETWORK_ID																1   											//1~7

#define LORA_ROLE																		 	LORA_ROLE_NODE						//LORA_ROLE_NODE or  LORA_ROLE_GATEWAY

#define LORA_GW_ID																		1                       	//gateway id 1~254

#define LORA_NODE_ID												          1                       	//node id 1~ NUM_NODES

#define MODE_WORK																			MODE_NORMAL    		        //MODE_NORMAL , MODE_PKG_LOSS_TEST


3.Compile code and download img to mcu flash.


PS: remember to install arm package "STM32L4xx_DFP.2.5.0" before running step 3
