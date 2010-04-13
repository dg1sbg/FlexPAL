/*
 *  FlexPALDevice.cpp
 *  flexpal
 *
 *  Created by Frank Goenninger on 18.03.10.
 *  Copyright 2010 Consequor Consulting AG. All rights reserved.
 *
 */

#include <iostream>
#include <errno.h>

#include "FlexPALPriv.h"

FlexPALDevice::~FlexPALDevice()
{
  unmount(); /* Unmount from bus */
}

void FlexPALDevice::update_user( DEVICE_NOTIFICATION nNotification )
{
  FlexPALController *pInstance = FlexPALController::getInstance();

  if( pInstance->debugLevel() > FLEXPAL_DEBUG_LEVEL_NO_DEBUG )
  {
    fprintf( stderr, "*** %s: DEBUG: %s (%s, %d): Notification = 0x%X\n", 
             FlexPALController::getInstance()->name(),
             __FUNCTION__, __FILE__ , __LINE__, 
             nNotification ); 
  }
  
  /* Notify All ? If yes then do ignore ! */
  if( nNotification == 0xffff0000 )
  {
    return;
  }
  
  /* Read Cmd ? */
  if( nNotification & DD_NOTIFY_USER4 )
  {
    if( pInstance->pmReadCmdNotifySem != NULL )
      pInstance->pmReadCmdNotifySem->post(); 
  }
    
  /* Async Cmd ? */
  if( nNotification & DD_NOTIFY_USER5 )
  {
    if( pInstance->pmAsyncCmdNotifySem != NULL )
      pInstance->pmAsyncCmdNotifySem->post(); 
  }
    
  /* WriteCmd ? */
  if( nNotification & DD_NOTIFY_USER6 )
  {
    if( pInstance->pmWriteCmdNotifySem != NULL )
      pInstance->pmWriteCmdNotifySem->post(); 
  }
  
  return;
}

void FlexPALDevice::mount()
{
  /* Do nothing */
}

tFlexPALrc FlexPALDevice::waitForCommandCompletion()
{
  bool                bResult;
  FlexPALController * pmInstance = FlexPALController::getInstance();
  
	bResult = pmInstance->pmAtomicCmdSem->wait( 100 );

  if( bResult == false )
    return FLEXPAL_ERROR;
  else
    return FLEXPAL_SUCCESS;
}

tFlexPALrc FlexPALDevice::signalCommandCompletion()
{
  bool                bResult;
  FlexPALController * pmInstance = FlexPALController::getInstance();
  
	bResult = pmInstance->pmAtomicCmdSem->post();
  
  if( bResult == false )
    return FLEXPAL_ERROR;
  else
    return FLEXPAL_SUCCESS;
}

tFlexPALrc FlexPALDevice::waitForCommandQueueEmpty()
{
	uint32 nReady = 0;

  // TODO: Temporarily disabled: lock lock(this);
  
	do 
	{
    this->hDevice->ohci_read( 0xE0001000, &nReady, 1 );
	}
	while( nReady & 0x80000000 );
  
  return FLEXPAL_SUCCESS;
}

tFlexPALrc FlexPALDevice::activate( void )
{
  tFlexPALrc nRC = FLEXPAL_SUCCESS;
  
  return nRC;
}

tFlexPALrc FlexPALDevice::writeOp( uint32 nOpCode,
                                   uint32 nData1,
                                   uint32 nData2 )
{
  uint32 anBuffer[ 3 ] = { nOpCode | 0x80000000, nData1, nData2 };
  
  this->hDevice->ohci_write( 0xE0001000, &anBuffer[ 0 ], 3);
  
  return FLEXPAL_SUCCESS;
}

tFlexPALrc FlexPALDevice::readOp( uint32 nOpCode, 
                                  uint32 nData1,
                                  uint32 nData2,
                                  uint32 *pnResult )
{
  tFlexPALrc nRC           = FLEXPAL_SUCCESS;
  int        nRetryCount   = 0;
  uint32     nReady        = 0;
  
  uint32     anBuffer[ 3 ] = { nOpCode, nData1, nData2 };
  
  nRC = waitForCommandCompletion();
  if( nRC != FLEXPAL_SUCCESS )
    return nRC;

  nRC = waitForCommandQueueEmpty();
  if( nRC != FLEXPAL_SUCCESS )
    return nRC;
  
  /* Execute Read Operation */

  /* Send command */
    
	this->hDevice->ohci_write( 0xE0001000, &anBuffer[ 0 ], 3);
  
  /* Read result */

  do 
  {
    this->hDevice->ohci_read( 0XE0001000, &nReady, 1 );
    
    if( nReady & 0x80000000 )
    {
      pal::delay( 10 );
      nRetryCount++;
    }
    
  } while (( nReady & 0x80000000) && ( nRetryCount < (FLEXPAL_MAX_READ_OP_RETRIES * 10)));
  
  this->hDevice->ohci_read( 0xE0001004, pnResult, 1);
	
  signalCommandCompletion();
  
  if( nRetryCount >= (FLEXPAL_MAX_READ_OP_RETRIES * 10))
  {
    nRC = FLEXPAL_ERROR_READ_OP_FAILED;
    // TODO: Set ErrorInfo
  }
  
  return nRC;
}

tFlexPALrc FlexPALDevice::deviceInfo( uint32 *pnModel, uint32 *pnSerialNr )
{
  tFlexPALrc nRC = FLEXPAL_SUCCESS;
  
  nRC = readOp( RDAL_OP_GET_MODEL, 0, 0, &mModel );
  if( nRC != FLEXPAL_SUCCESS )
  {
    // TODO: Set Error Info
    
    goto DEVICE_INFO_EXIT;
  }
  *pnModel = mModel;
  
  nRC = readOp( RDAL_OP_GET_SERIAL_NUM, 0, 0, &mSerialNr );
  if( nRC != FLEXPAL_SUCCESS )
  {
    // TODO: Set Error Info
    goto DEVICE_INFO_EXIT;
  }
  *pnSerialNr = this->mSerialNr;
   
DEVICE_INFO_EXIT:
  
  return nRC;
}

tFlexPALrc FlexPALDevice::setBufferSize( uint32 nSize )
{
  tFlexPALrc nRC = FLEXPAL_SUCCESS;
  
  // TODO: Find correct call for io_size()
  // FlexPALController::getInstance()->mPALSystem->at( this->mBusID )->io_size( nSize );
  
  return nRC;
}




/* ========================================================================= */
/* T E S T I N G  /  D E B U G G I N G                                       */
/* ========================================================================= */
