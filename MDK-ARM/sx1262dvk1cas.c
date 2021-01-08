/*!
 * \file      sx1262dvk1cas-board.c
 *
 * \brief     Target board SX1261DVK1BAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
/**
  ******************************************************************************
  * @file    sx1262dvk1cas.c
  * @author  MCD Application Team
  * @brief   driver sx1262dvk1cas board
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "hw.h"
#include "radio.h"
#include "sx126x_board.h"
#include "main.h"
#include "uart_app.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BOARD_TCXO_WAKEUP_TIME  0 // no TCXO


#ifndef USE_SX126X_DVK
#error "please define USE_SX126X_DVK"
#endif

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void SX126xIoInit( void )
{
  GPIO_InitTypeDef initStruct={0};

//   Set DioIrqHandler port in IT_RISING mode
  initStruct.Mode = GPIO_MODE_IT_RISING;
  initStruct.Pull = GPIO_NOPULL;
  initStruct.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( RADIO_DIO_1_GPIO_Port, RADIO_DIO_1_Pin, &initStruct);

  initStruct.Mode = GPIO_MODE_INPUT;
  initStruct.Pull = GPIO_NOPULL;
  ///HW_GPIO_Init( DEVICE_SEL_PORT, DEVICE_SEL_PIN, &initStruct);
  HW_GPIO_Init( BUSY_GPIO_Port, BUSY_Pin, &initStruct);
  
  initStruct.Pull = GPIO_NOPULL;
  initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HW_GPIO_Init( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin,  &initStruct );
  HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin,  1 );

 // HW_GPIO_Init( RADIO_LEDTX_PORT, RADIO_LEDTX_PIN,  &initStruct );
 // HW_GPIO_Write( RADIO_LEDTX_PORT, RADIO_LEDTX_PIN,  0 );
  
 // HW_GPIO_Init( RADIO_LEDRX_PORT, RADIO_LEDRX_PIN,  &initStruct );
 // HW_GPIO_Write( RADIO_LEDRX_PORT, RADIO_LEDRX_PIN,  0 );
  
  HW_GPIO_Init( RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, &initStruct); 
  HW_GPIO_Write( RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, 0);
}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
  HW_GPIO_SetIrq( RADIO_DIO_1_GPIO_Port, RADIO_DIO_1_Pin, IRQ_HIGH_PRIORITY, dioIrq );
}

void SX126xIoDeInit( void )
{
}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX126xReset( void )
{
  GPIO_InitTypeDef initStruct = { 0 };

  initStruct.Mode =GPIO_MODE_OUTPUT_PP;
  initStruct.Pull = GPIO_NOPULL;
  initStruct.Speed = GPIO_SPEED_HIGH;

  // Wait 10 ms
	//dbg_printf("rSX126xReset 1\n");
   HAL_Delay( 10 );
	//dbg_printf("SX126xReset 1\n");
  
  // Set RESET pin to 0
  HW_GPIO_Init( RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, &initStruct );
  HW_GPIO_Write( RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, 0 );
  
  // Wait 20 ms
  HAL_Delay( 20 );
  // Configure RESET as input
  initStruct.Mode = GPIO_MODE_ANALOG;
  HW_GPIO_Init( RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, &initStruct );  
  HW_GPIO_Write( RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, 1); // internal pull-up
  
  // Wait 10 ms
  HAL_Delay( 10 );
}

void SX126xWaitOnBusy( void )
{
    while( HW_GPIO_Read(BUSY_GPIO_Port, BUSY_Pin) == 1 );
	  //dbg_printf("not busy\n");
}

void SX126xWakeup( void )
{
    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 0 );
	  //HAL_Delay(1);
    
    HW_SPI_InOut(RADIO_GET_STATUS);
    HW_SPI_InOut(0);
    //HAL_Delay(1);
    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 1 );
    
	  //dbg_printf("SX126xWakeup\n");
    // Wait for chip to be ready.
    SX126xWaitOnBusy( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 0 );

    HW_SPI_InOut(command);

    for( uint16_t i = 0; i < size; i++ )
    {
        HW_SPI_InOut( buffer[i] );
    }
    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 1 );

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

uint8_t SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint8_t status = 0;

    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 0 );

    HW_SPI_InOut(command);
    status = HW_SPI_InOut(0x00 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = HW_SPI_InOut( 0 );
    }

    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 1 );

    SX126xWaitOnBusy( );

    return status;
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 0 );
    //HAL_Delay(1);
    HW_SPI_InOut(RADIO_WRITE_REGISTER );
    HW_SPI_InOut(( address & 0xFF00 ) >> 8 );
    HW_SPI_InOut(address & 0x00FF );
    
    for( uint16_t i = 0; i < size; i++ )
    {
        HW_SPI_InOut(buffer[i] );
    }

    //NSS = 1;
		//HAL_Delay(1);
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 1 );

    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 0 );
   // HAL_Delay(1);
    HW_SPI_InOut(RADIO_READ_REGISTER);
    HW_SPI_InOut(( address & 0xFF00) >> 8 );
    HW_SPI_InOut(address & 0x00FF);
    HW_SPI_InOut(0);
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = HW_SPI_InOut( 0 );
    }
    //NSS = 1;
		//HAL_Delay(1);
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 1 );

    SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 0 );
//HAL_Delay(1);
    HW_SPI_InOut( RADIO_WRITE_BUFFER );
    HW_SPI_InOut( offset );
    for( uint16_t i = 0; i < size; i++ )
    {
        HW_SPI_InOut( buffer[i] );
    }
    //NSS = 1;
	//	HAL_Delay(1);
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 1 );

    SX126xWaitOnBusy( );
}


void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 0 );

    HW_SPI_InOut( RADIO_READ_BUFFER );
    HW_SPI_InOut( offset );
    HW_SPI_InOut( 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = HW_SPI_InOut( 0 );
    }
    
    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 1 );

    SX126xWaitOnBusy( );
}
void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}



uint8_t SX126xGetPaSelect( uint32_t channel )
{
   // if( HW_GPIO_Read( DEVICE_SEL_PORT, DEVICE_SEL_PIN) ==  1 )
   // {
   //     return SX1261;
  //  }
  //  else
 //   {
        return SX1262;
 //   }
}

void SX126xAntSwOn( void )
{
  HW_GPIO_Write( RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, 1);
}

void SX126xAntSwOff( void )
{
  HW_GPIO_Write( RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, 0);
}



bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

bool SX126xBoardIsTcxoPresent(void)
{ /*no TCXO on board*/
  return false;
}

void SX126xBoardSetLedTx( bool state )
{
  if (state==false)
  {
//    HW_GPIO_Write( RADIO_LEDTX_PORT, RADIO_LEDTX_PIN,  0 );
  }
  else
  {
   // HW_GPIO_Write( RADIO_LEDTX_PORT, RADIO_LEDTX_PIN, 1 );
  }
}

void SX126xBoardSetLedRx( bool state )
{
  if (state==false)
  {
   // HW_GPIO_Write( RADIO_LEDRX_PORT, RADIO_LEDRX_PIN, 0 );
  }
  else
  {
   // HW_GPIO_Write( RADIO_LEDRX_PORT, RADIO_LEDRX_PIN, 1 );
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
