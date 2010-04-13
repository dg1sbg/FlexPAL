/*
 *  FlexPAL.cpp
 *  FlexPAL
 *
 *  Created by Frank Goenninger on 14.03.10.
 *  Copyright 2010 Consequor Consulting AG. All rights reserved.
 *
 *  $Header$
 *
 */

#include "FlexPALPriv.h"

/* =========================================================================== */
/* GLOBAL VARS                                                                 */
/* =========================================================================== */

FlexPALController * FlexPALController::smInstance;
// LockObject        * pgFlexPALGlobalLockObject;

/* =========================================================================== */
/* PRIVATE UTILITY FUNCTIONS                                                   */
/* =========================================================================== */

tFlexPALrc nCheckFPCPtr( void *ptr )
{
  if( ptr != (void *) FlexPALController::getInstance() )
    return FLEXPAL_ERROR;
  else 
    return FLEXPAL_SUCCESS;
}

static bool convertUint32ToBool( uint32 nValue )
{
  return (nValue != 0 );
}

static bool convertBoolToUint32( bool bValue )
{
  return (bValue == true ) ? 1 : 0;
}

/* =========================================================================== */
/* PRIVATE FUNCTIONS                                                           */
/* =========================================================================== */

DWORD WINAPI Notify(LPVOID lpParam) 
{
  FlexPALController *pInstance  = FlexPALController::getInstance();
  NotifyCBPtr        pfNotifyCB = NULL;
  
  if( pInstance != NULL )
  {
    while( true )
    {
      if( pInstance->pmAsyncCmdNotifySem->wait() == true )
      {
        uint32          nData           = 0;
        FlexPALDevice * pSelectedDevice = (FlexPALDevice *) (pInstance->selectedDevice());
          
        pSelectedDevice->ohci_read( 0xE0001008, &nData, 1);
       
        if( pInstance->debugLevel() > FLEXPAL_DEBUG_LEVEL_NO_DEBUG )
        {
          fprintf( stderr, "*** %s: DEBUG: %s (%s, %d): Notify ! Data = %d\n", 
                  FlexPALController::getInstance()->name(),
                  __FUNCTION__, __FILE__, __LINE__, 
                  (int) nData ); 
        }
               
        pfNotifyCB = (NotifyCBPtr)FlexPALController::getInstance()->pfNotifyCB;

        if( pfNotifyCB != NULL )
        {
          if( pInstance->debugLevel() > FLEXPAL_DEBUG_LEVEL_NO_DEBUG )
          {
            fprintf( stderr, "*** %s: DEBUG: %s (%s, %d): Notify ! Calling Notify Callback ...\n", 
                    FlexPALController::getInstance()->name(),
                    __FUNCTION__, __FILE__, __LINE__ ); 
          }
          pfNotifyCB( nData );
        }
      }
    }
  }
  
  return 0;
}  

void NotificationThread::run( void )
{
  Notify(this);
}

/* =========================================================================== */
/* PRIVATE METHODS                                                             */
/* =========================================================================== */

void FlexPALController::findDevices()
{
  register int nNrBusses  = 0;
  
  tFlexPALSystemRef hSystem = FlexPALController::getInstance()->mPALSystem;
   
  nNrBusses = hSystem->size();
  
  for( register int nCurrBus = 0; nCurrBus < nNrBusses; nCurrBus++ )
  {
    tFlexPALBusRef hBus = hSystem->at( nCurrBus );
    
    for( register int nCurrDevice = 0; nCurrDevice < hBus->size(); nCurrDevice++ )
    {
      tFlexPALDeviceRef hDevice;

      { /* New scope */
        tcat::dice::lock lock( hBus );
        
        hDevice = hBus->at( nCurrDevice );
      
        if( hDevice ) /* We found a valid device ... */
        {
          { /* New scope */
            
            tcat::dice::lock lock( hDevice );
          
            FlexPALDevice *pNewDevice = new FlexPALDevice();
        
            if( pNewDevice != NULL )
            {
              vector<FlexPALDevice *>::iterator Iterator = FlexPALController::getInstance()->mDevicePtrs.begin();
              
              pNewDevice->hDevice   = hDevice; /* Save reference */
              pNewDevice->mBusID    = nCurrBus;
              pNewDevice->mDeviceID = nCurrDevice;
              pNewDevice->mGUID     = hDevice->guid64();
              
              FlexPALController::getInstance()->mDevicePtrs.insert( Iterator, 1, pNewDevice ); /* Add device to device list */  
              
              selectDevice( pNewDevice );
            }
          }
        }
      }
    }
  }
}

void FlexPALController::unmountPreviouslySelectedDevice()
{
  if( mSelectedDevice )
    mSelectedDevice->unmount();
}

void FlexPALController::setErrorInfo( int nErrno, const char * pcFile, 
                                     int nLine, const char * pcFunction,
                                     time_t nTimestamp, const char * pcMessage )
{
  tsFlexPALErrorInfo * psErrorInfo = getInstance()->psErrorInfo;
  
  psErrorInfo->nErrno     = nErrno;
  psErrorInfo->pcFile     = pcFile;
  psErrorInfo->nLine      = nLine;
  psErrorInfo->pcFn       = pcFunction;
  psErrorInfo->nTimestamp = nTimestamp;
  psErrorInfo->pcMessage  = pcMessage;
}

/* =========================================================================== */
/* FLEXPAL CONTROLLER PUBLIC METHODS                                           */
/* =========================================================================== */

FlexPALController* FlexPALController::getInstance( void )
{
  if( smInstance == NULL )
  { 
    // TODO: Check if Instance is locked
    
    if( smInstance == NULL )
    {  
      smInstance              = new FlexPALController();
      smInstance->psErrorInfo = new tsFlexPALErrorInfo();
      
      /* Create thread for asynchronous notifications */
      
      smInstance->pNotificationThread = new NotificationThread();
      
      /* Initialize TCAT PAL */
      
      smInstance->init();
    }
  }
  
  return smInstance;
}

/* =========================================================================== */
/* LOW-LEVEL PUBLIC METHODS                                                    */
/* =========================================================================== */

void * FlexPALController::selectedDevice( void )
{
  if( mSelectedDeviceID != FLEXPAL_NO_DEVICE )
  {  
    return (void *) FlexPALController::getInstance()->mDevicePtrs[ mSelectedDeviceID ];
  }
  else
  {
    // TODO: Set Error Info
    return NULL;
  }
}

const int FlexPALController::numberOfDevices()
{
  return FlexPALController::getInstance()->mDevicePtrs.size();
}

const tsFlexPALErrorInfo * FlexPALController::errorInfo( void )
{
  return psErrorInfo;
}

tFlexPALrc FlexPALController::reset( void )
{
  tFlexPALrc nRC = FLEXPAL_SUCCESS;
  
  if( FlexPALController::getInstance()->mPALSystem != NULL ) // Initialized ? If not do nothing !
  {
    if( FlexPALController::getInstance()->mPALSystem->size() > 0 ) // Any busses there ?
    {              
      lock lock( FlexPALController::getInstance()->mPALSystem );
      
      findDevices();
    }
  }
  
  return nRC;
}

tFlexPALrc FlexPALController::init( void )
{
  tFlexPALrc nRC = FLEXPAL_SUCCESS;
  
  pmReadCmdNotifySem  = new BSemaphore( 0 );
  pmWriteCmdNotifySem = new BSemaphore( 0 );
  pmAsyncCmdNotifySem = new BSemaphore( 0 );
  pmAtomicCmdSem      = new BSemaphore( 1 );
  
  setDebugLevel( FLEXPAL_DEBUG_LEVEL_DEFAULT );
   
  // Instantiate PAL System
  
  try 
  {
    mPALSystem = sys::static_create< sys >();
  }
  catch(tcat::dice::xptn_driver_version xptn) 
  {
    tcat::exception::base * pException = dynamic_cast<tcat::exception::base *> (&xptn); 
    const char *pcException = pException->usr_msg().c_str(); 
    
    setErrorInfo( FLEXPAL_ERROR_DRIVER_MISMATCH,
                  __FILE__, __LINE__, __FUNCTION__, time(0),
                  pcException );
    
    return FLEXPAL_ERROR;
  }
  catch (tcat::exception::base xptn) 
  {
    tcat::exception::base * pException = dynamic_cast<tcat::exception::base *> (&xptn); 
    const char *pcException = pException->usr_msg().c_str();
    
    setErrorInfo( FLEXPAL_ERROR_CANNOT_CREATE_SYSTEM,
                 __FILE__, __LINE__, __FUNCTION__, time(0),
                  pcException );
    
    return FLEXPAL_ERROR;
  }
  
  // Find devices
  
  FlexPALController::getInstance()->mSelectedDeviceID = FLEXPAL_NO_DEVICE;
  
  nRC = reset();
  
  // Start Notification Catching Thread
  
  pNotificationThread->start();
  
  // Hurrey ...
  
  return nRC;
}

// TODO: Close() function for shutting down FlexPALController.

void FlexPALController::setNotifyCB( void *pfNotifyCB )
{
  FlexPALController::smInstance->pfNotifyCB = pfNotifyCB;  
}

void FlexPALController::selectDevice( FlexPALDevice *pDevice )
{
  unmountPreviouslySelectedDevice();
  
  FlexPALController::getInstance()->mSelectedDevice   = pDevice;
  FlexPALController::getInstance()->mSelectedDeviceID = pDevice->mDeviceID;
  
  pDevice->activate();
  
}

tFlexPALrc FlexPALController::selectDeviceByID( int nID )
{
  if(( nID >= 0 ) && ( nID < FlexPALController::getInstance()->mDevicePtrs.size() ))
  { 
    FlexPALDevice *pDevice = FlexPALController::getInstance()->mDevicePtrs[ nID ];
   
    FlexPALController::getInstance()->selectDevice( pDevice );
    
    return FLEXPAL_SUCCESS;
  }
  else
  {
    FlexPALController::getInstance()->setErrorInfo( FLEXPAL_ERROR_INVALID_DEVICE_ID,
                 __FILE__, __LINE__, __FUNCTION__, time(0),
                 NULL );
    return FLEXPAL_ERROR_INVALID_DEVICE_ID;
  }
}

tFlexPALrc FlexPALController::deviceInfo( uint32 *pnModel, uint32 *pnSerialNr )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->deviceInfo( pnModel, pnSerialNr );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeOp( uint32 nOpCode, uint32 nData1, uint32 nData2 )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( nOpCode, nData1, nData2 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readOp( uint32 nOpCode, uint32 nData1, uint32 nData2, uint32 *pnResult )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( nOpCode, nData1, nData2, pnResult );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setBufferSize( uint32 nSize )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->setBufferSize( nSize );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

/* =========================================================================== */
/* HIGH-LEVEL PUBLIC METHODS                                                   */
/* =========================================================================== */

tFlexPALrc FlexPALController::setRegister( uint32 nAddress, uint32 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_REG_SET, nAddress, nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getRegister( uint32 nAddress, uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_REG_GET, nAddress, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::gpioWrite( uint32 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_GPIO, 0, nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::gpioRead( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_READ_GPIO, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::gpioDDRWrite( uint32 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_GPIO_DDR, 0, nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::gpioDDRRead( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_READ_GPIO_DDR, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::i2cWriteValue( uint16 nAddress, uint8 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_I2C_WRITE_VALUE, (uint32) nAddress, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::i2cWrite2Value( uint16 nAddress, uint8 nValue1, uint8 nValue2 )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_I2C_WRITE_2_VALUE, (uint32) nAddress, (uint32) ((nValue1 << 8) + nValue2));
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::i2cReadValue( uint16 nAddress, unsigned int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_I2C_READ_VALUE, (uint32) nAddress, 0, (uint32 *)pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getSerialNum( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_SERIAL_NUM, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getModel( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_MODEL, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getFirmwareRev( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_FIRMWARE_REV, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::initialize( void )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_INITIALIZE, 0, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getTRXOK( bool *bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC    = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_GET_TRX_OK, 0, 0, &nValue );
    
    *bValue = convertUint32ToBool( nValue );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getTRXRev( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_TRX_REV, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getTRXSN( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_TRX_SN, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getPAOK( bool *bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC    = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_GET_PA_OK, 0, 0, &nValue );
    
    *bValue = convertUint32ToBool( nValue );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getPARev( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_PA_REV, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getPASN( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_PA_SN, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getRFIOOK( bool *bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_GET_RFIO_OK, 0, 0, &nValue );
    
    *bValue = convertUint32ToBool( nValue );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getRFIORev( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_RFIO_REV, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getRFIOSN( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_RFIO_SN, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getATUOK( bool *bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_GET_ATU_OK, 0, 0, &nValue );
    
    *bValue = convertUint32ToBool( nValue );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getATURev( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_ATU_REV, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getATUSN( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_ATU_SN, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getRX2OK( bool *bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_GET_RX2_OK, 0, 0, &nValue );
    
    *bValue = convertUint32ToBool( nValue );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getRX2Rev( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_RX2_REV, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getRX2SN( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_RX2_SN, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getVUOK( bool *bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_GET_VU_OK, 0, 0, &nValue );
    
    *bValue = convertUint32ToBool( nValue );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getVURev( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_VU_REV, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getVUSN( uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->readOp( RDAL_OP_GET_VU_SN, 0, 0, pnValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readClockReg( int nIndex, int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_CLOCK_REG, (uint32) nIndex, 0, &nValue );
    
    *pnValue = (int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeClockReg( int nIndex, int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_CLOCK_REG, (uint32) nIndex, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readTRXDDSReg( int nChannel, int nIndex, unsigned int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_TRX_DDS_REG, (uint32) nChannel, (uint32) nIndex, &nValue );
    
    *pnValue = (unsigned int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeTRXDDSReg( int nChannel, int nIndex, unsigned int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_TRX_DDS_REG, (uint32) ((nChannel << 8) + nIndex), (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readRX2DDSReg( int nChannel, int nIndex, unsigned int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_RX2_DDS_REG, (uint32) nChannel, (uint32) nIndex, &nValue );
    
    *pnValue = (unsigned int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeRX2DDSReg( int nChannel, int nIndex, unsigned int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_RX2_DDS_REG, (uint32) ((nChannel << 8) + nIndex), (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readPIOReg( int nIndex, int nRegister, unsigned int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PIO_REG, (uint32) nIndex, (uint32) nRegister, &nValue );
    
    *pnValue = (unsigned int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writePIOReg( int nIndex, int nRegister, unsigned int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_PIO_REG, (uint32) ((nIndex << 8) + nRegister), (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readTRXEEPROMByte( unsigned int nOffset, uint8 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_TRX_EEPROM_UINT8, (uint32) nOffset, 0, &nValue );
    
    *pnValue = (uint8) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readTRXEEPROMUshort( unsigned int nOffset, uint16 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_TRX_EEPROM_UINT16, (uint32) nOffset, 0, &nValue );
    
    *pnValue = (uint16) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readTRXEEPROMUint( unsigned int nOffset, uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_TRX_EEPROM_UINT32, (uint32) nOffset, 0, &nValue );
    
    *pnValue = (uint32) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readTRXEEPROMFloat( unsigned int nOffset, float *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_TRX_EEPROM_UINT32, (uint32) nOffset, 0, &nValue );
    
    *pnValue = (float) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeTRXEEPROMByte( unsigned int nOffset, uint8 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_TRX_EEPROM_UINT8, (uint32) nOffset, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeTRXEEPROMUshort( unsigned int nOffset, uint16 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_TRX_EEPROM_UINT16, (uint32) nOffset, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeTRXEEPROMUint( unsigned int nOffset, uint32 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_TRX_EEPROM_UINT32, (uint32) nOffset, nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeTRXEEPROMFloat( unsigned int nOffset, float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_TRX_EEPROM_UINT32, (uint32) nOffset, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readRX2EEPROMByte( unsigned int nOffset, uint8 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_RX2_EEPROM_UINT8, (uint32) nOffset, 0, &nValue );
    
    *pnValue = (uint8) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readRX2EEPROMUshort( unsigned int nOffset, uint16 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_RX2_EEPROM_UINT16, (uint32) nOffset, 0, &nValue );
    
    *pnValue = (uint16) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readRX2EEPROMUint( unsigned int nOffset, uint32 *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_RX2_EEPROM_UINT32, (uint32) nOffset, 0, &nValue );
    
    *pnValue = (uint32) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readRX2EEPROMFloat( unsigned int nOffset, float *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_RX2_EEPROM_UINT32, (uint32) nOffset, 0, &nValue );
    
    *pnValue = (float) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeRX2EEPROMByte( unsigned int nOffset, uint8 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_RX2_EEPROM_UINT8, (uint32) nOffset, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeRX2EEPROMUshort( unsigned int nOffset, uint16 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_RX2_EEPROM_UINT16, (uint32) nOffset, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeRX2EEPROMUint( unsigned int nOffset, uint32 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_RX2_EEPROM_UINT32, (uint32) nOffset, nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeRX2EEPROMFloat( unsigned int nOffset, float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_RX2_EEPROM_UINT32, (uint32) nOffset, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTRXPot( unsigned int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TRX_POT, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::trxPotSetRDAC( int nIndex, int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_TRX_POT_SET_RDAC, (uint32) nIndex, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::trxPotGetRDAC( unsigned int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_TRX_POT_GET_RDAC, 0, 0, &nValue );
    
    *pnValue = (unsigned int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::paPotSetRDAC( int nIndex, int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_PA_POT_SET_RDAC, (uint32) nIndex, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::paPotGetRDAC( unsigned int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_PA_POT_GET_RDAC, 0, 0, &nValue );
    
    *pnValue = (unsigned int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setMux( int nChannel )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_MUX, (uint32) nChannel, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::writeCodecReg( int nRegister, int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_WRITE_CODEC_REG, (uint32) nRegister, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readCodecReg( int nRegister, int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_CODEC_REG, (uint32) nRegister, 0, &nValue );
    
    *pnValue = (int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX1Freq( float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX1_FREQ, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX1FreqTW( unsigned int nTW, float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX1_FREQ_TW, (uint32) nTW, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX2Freq( float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX2_FREQ, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX2FreqTW( unsigned int nTW, float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX2_FREQ_TW, (uint32) nTW, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTXFreq( float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TX_FREQ, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTXFreqTW( unsigned int nTW, float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TX_FREQ_TW, (uint32) nTW, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTRXPreamp( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TRX_PREAMP, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTest( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TEST, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setGen( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_GEN, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setSig( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_SIG, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setImpulse( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_IMPULSE, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setXVEN( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_XVEN, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setXVTXEN( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_XVTXEN, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setQSD( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_QSD, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setQSE( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_QSE, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setXREF( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_XREF, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX1Filter( float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX1_FILTER, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX2Filter( float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX2_FILTER, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTXFilter( float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TX_FILTER, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setIntSpkr( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_INT_SPKR, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX1Tap( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX1_TAP, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::bypassRX1Filter( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_BYPASS_RX1_FILTER, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::bypassRX2Filter( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_BYPASS_RX2_FILTER, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::bypassTXFilter( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_BYPASS_TX_FILTER, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::bypassPAFilter( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_BYPASS_PA_FILTER, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readPTT( bool *pbDot, bool *pbDash, bool *pbRCAPTT, bool *pbMicPTT )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PTT, 0, 0, &nValue );

    *pbDot    = (( nValue & 0x01 ) == 1 );
    *pbDash   = (( nValue & 0x02 ) == 2 );
    *pbRCAPTT = (( nValue & 0x04 ) == 4 );
    *pbMicPTT = (( nValue & 0x08 ) == 8 );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readDot( bool *pbValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PTT, 0, 0, &nValue );
    
    *pbValue  = (( nValue & 0x01 ) == 1 );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readDash( bool *pbValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PTT, 0, 0, &nValue );
    
    *pbValue  = (( nValue & 0x02 ) == 2 );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readRCAPTT( bool *pbValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PTT, 0, 0, &nValue );
    
    *pbValue  = (( nValue & 0x04 ) == 4 );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readMicPTT( bool *pbValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PTT, 0, 0, &nValue );
    
    *pbValue  = (( nValue & 0x08 ) == 8 );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readMicDown( bool *pbValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PTT, 0, 0, &nValue );
    
    *pbValue  = (( nValue & 0x10 ) == 0x10 );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readMicUp( bool *pbValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PTT, 0, 0, &nValue );
    
    *pbValue  = (( nValue & 0x20 ) == 0x20 );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::readMicFast( bool *pbValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PTT, 0, 0, &nValue );
    
    *pbValue  = (( nValue & 0x40 ) == 0x40 );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setHeadphone( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_HEADPHONE, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setPLL( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_PLL, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRCATX1( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RCA_TX1, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRCATX2( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RCA_TX2, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRCATX3( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RCA_TX3, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setFan( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    // return pDevice->writeOp( RDAL_OP_SET_FAN, convertBoolToUint32( bValue ), 0 );
    return FLEXPAL_SUCCESS;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getPLLStatus2( bool *pbValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PTT, 0, 0, &nValue );
    
    *pbValue  = ( nValue != 0 );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}
                 
tFlexPALrc FlexPALController::readPAADC( int nChannel, int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC     = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_READ_PA_ADC, (uint32) nChannel, 0, &nValue );
    
    *pnValue  = (int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX1Loop( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX1_LOOP, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTR( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TR, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setAnt( int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_ANT, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX1Ant( int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX1_ANT, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX2Ant( int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX2_ANT, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTXAnt( int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TX_ANT, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTXMon( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TXMON, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setXVCOM( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_XVCOM, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setEN2M( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_EN_2M, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setKey2M( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_KEY_2M, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setXVTR( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_XVTR, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setPABias( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_PA_BIAS, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setPowerOff( void )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_POWER_OFF, 0, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setFPLED( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_FPLED, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setCTS( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_CTS, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRTS( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RTS, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setPCReset( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_PC_RESET, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setPCPWRBT( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_PC_PWRBT, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setMox( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_MOX, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setIntLED( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_INT_LED, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::atuSendCmd( uint8 nValue1, uint8 nValue2, uint8 nValue3 )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_ATU_SEND_CMD, (uint32) (( nValue1 << 8 ) + nValue2 ), (uint32) nValue3  );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::atuGetResult( uint8 *pnCmd, uint8 *pnValue2, uint8 *pnValue3, uint8 *pnValue4, unsigned int nTimeoutMS )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC    = FLEXPAL_SUCCESS;
    uint32     nValue = 0;

    nRC = pDevice->readOp( RDAL_OP_ATU_GET_RESULT, (uint32) nTimeoutMS, 0, &nValue );
    
    *pnCmd    = (uint8) nValue;
    *pnValue2 = (uint8) (nValue >> 8);
    *pnValue3 = (uint8) (nValue >> 16);
    *pnValue4 = (uint8) (nValue >> 24);
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setFullDuplex( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_FULL_DUPLEX, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTXDAC( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TX_DAC, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setAmpTX1( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_AMP_TX1, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setAmpTX2( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_AMP_TX2, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setAmpTX3( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_AMP_TX3, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setXVTRActive( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_XVTR_ACTIVE, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setXVTRSplit( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_XVTR_SPLIT, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX1Out( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX1OUT, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::flexwireWriteValue( uint8 nAddress, uint8 nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_FLEXWIRE_WRITE_VALUE, (uint32) nAddress, (uint32) nValue );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::flexwireWrite2Value( uint8 nAddress, uint8 nValue1, uint8 nValue2 )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_FLEXWIRE_WRITE_2_VALUE, (uint32) nAddress, (uint32) (( nValue1 << 8 ) + nValue2 ));
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::flexwireReadValue( uint16 nAddress, unsigned int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC    = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_FLEXWIRE_READ_VALUE, (uint32) nAddress, 0, &nValue );
    
    *pnValue = (unsigned int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::flexwireRead2Value( uint16 nAddress, unsigned int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC    = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_FLEXWIRE_READ_2_VALUE, (uint32) nAddress, 0, &nValue );
    
    *pnValue = (unsigned int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX1DSPMode( int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX1_DSP_MODE, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX2DSPMode( int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX2_DSP_MODE, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX2On( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX2_ON, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setStandby( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_STANDBY, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setAmpTX1DelayEnable( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_AMP_TX1_DELAY_ENABLE, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setAmpTX2DelayEnable( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_AMP_TX2_DELAY_ENABLE, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setAmpTX3DelayEnable( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_AMP_TX3_DELAY_ENABLE, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setAmpTX1Delay( unsigned int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_AMP_TX1_DELAY, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setAmpTX2Delay( unsigned int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_AMP_TX2_DELAY, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setAmpTX3Delay( unsigned int nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_AMP_TX3_DELAY, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::resetRX2DDS( void )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_RESET_RX2_DDS, 0, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRX2Preamp( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX2_PREAMP, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setIambic( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_IAMBIC, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setBreakIn( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_BREAK_IN, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setManualRX1Filter( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_MANUAL_RX1_FILTER, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setManualRX2Filter( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_MANUAL_RX2_FILTER, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setHiZ( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_HIZ, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setPDrvMon( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_PDRVMON, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setATUEnable( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_ENABLE_ATU, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setATUAttn( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_ATU_ATTN, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setRXAttn( bool bValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_RX_ATTN, convertBoolToUint32( bValue ), 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setFanPWM( int nOn, int nOff )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_FAN_PWM, (uint32) nOn, (uint32) nOff );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setFanSpeed( float nValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_FAN_SPEED, (uint32) nValue, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getRegion( unsigned int *pnValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC    = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_GET_REGION, 0, 0, &nValue );
    
    *pnValue = (unsigned int) nValue;
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTXDSPFilter( int nLow, int nHigh )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TX_DSP_FILTER, nLow, nHigh );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTXOffset( int nOffset )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TX_OFFSET, nOffset, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setTXDSPMode( int nMode )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_TX_DSP_MODE, (uint32) nMode, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::setCWPitch( unsigned int nPitch )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    return pDevice->writeOp( RDAL_OP_SET_CW_PITCH, (uint32) nPitch, 0 );
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}

tFlexPALrc FlexPALController::getStatus( bool *pbValue )
{
  FlexPALDevice *pDevice = (FlexPALDevice *) FlexPALController::getInstance()->selectedDevice();
  
  if( pDevice != NULL )
  {
    tFlexPALrc nRC    = FLEXPAL_SUCCESS;
    uint32     nValue = 0;
    
    nRC = pDevice->readOp( RDAL_OP_GET_STATUS, 0, 0, &nValue );
    
    *pbValue = ( nValue != 0 );
    
    return nRC;
  }
  else
  {
    // TODO: Set Error Info
    return FLEXPAL_ERROR_INVALID_DEVICE;
  }
}




/* =========================================================================== */
/* LOW-LEVEL C INTERFACE FUNCTIONS                                             */
/* =========================================================================== */

FLEXPAL_EXPORT const void * FlexPALGetInstance( void )
{
  return (void *)FlexPALController::getInstance();
}

FLEXPAL_EXPORT const tsFlexPALErrorInfo * FlexPALGetErrorInfo( void *pController )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return NULL;
  else
    return ((FlexPALController *)pController)->errorInfo();
}

FLEXPAL_EXPORT void FlexPALSetDebugLevel( void *pController, int nLevel )
{
  if( nCheckFPCPtr( pController ) == FLEXPAL_SUCCESS )
    ((FlexPALController *)pController)->setDebugLevel( nLevel );
}

FLEXPAL_EXPORT void FlexPALSetNotifyCB( void *pController, void *pfNotifyCB )
{
  if( nCheckFPCPtr( pController ) == FLEXPAL_SUCCESS )
    ((FlexPALController *)pController)->setNotifyCB( pfNotifyCB );
}

FLEXPAL_EXPORT int FlexPALGetNumberOfDevices( void *pController )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->numberOfDevices();
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSelectDeviceByID( void *pController, int nID )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->selectDeviceByID( nID );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetDeviceInfo( void *pController, uint32 *pnModel, uint32 *pnSerialNr )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->deviceInfo( pnModel, pnSerialNr );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetBufferSize( void *pController, uint32 nSize )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setBufferSize( nSize );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteOp( void *pController, uint32 nOpCode, uint32 nData1, uint32 nData2 )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeOp( nOpCode, nData1, nData2 );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadOp( void *pController, uint32 nOpCode, uint32 nData1, uint32 nData2, uint32 *pnResult )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readOp( nOpCode, nData1, nData2, pnResult );
}

/* =========================================================================== */
/* HIGH-LEVEL C INTERFACE FUNCTIONS                                            */
/* =========================================================================== */

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRegister( void *pController, uint32 nAddress, uint32 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRegister( nAddress, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetRegister( void *pController, uint32 nAddress, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getRegister( nAddress, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGPIOWrite( void *pController, uint32 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->gpioWrite( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGPIORead( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->gpioRead( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALDDRGPIOWrite( void *pController, uint32 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->gpioDDRWrite( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGPIODDRRead( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->gpioDDRRead( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALI2CWriteValue( void *pController, uint16 nAddress, uint8 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->i2cWriteValue( nAddress, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALI2CWrite2Value( void *pController, uint16 nAddress, uint8 nValue1, uint8 nValue2 )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->i2cWrite2Value( nAddress, nValue1, nValue2 );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALI2CReadValue( void *pController, uint16 nAddress, unsigned int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->i2cReadValue( nAddress, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetSerialNum( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getSerialNum( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetModel( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getModel( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetFirmwareRev( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getFirmwareRev( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALInitialize( void *pController )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->initialize();
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetTRXOK( void *pController, bool *bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getTRXOK( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetTRXRev( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getTRXRev( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetTRXSN( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getTRXSN( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetPAOK( void *pController, bool *bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getPAOK( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetPARev( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getPARev( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetPASN( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getPASN( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetRFIOOK( void *pController, bool *bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getRFIOOK( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetRFIORev( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getRFIORev( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetRFIOSN( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getRFIOSN( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetATUOK( void *pController, bool *bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getATUOK( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetATURev( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getATURev( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetATUSN( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getATUSN( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetRX2OK( void *pController, bool *bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getRX2OK( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetRX2Rev( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getRX2Rev( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetRX2SN( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getRX2SN( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetVUOK( void *pController, bool *bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getVUOK( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetVURev( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getVURev( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetVUSN( void *pController, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getVUSN( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadClockReg( void *pController, int nIndex, int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readClockReg( nIndex, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteClockReg( void *pController, int nIndex, int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeClockReg( nIndex, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadTRXDDSReg( void *pController, int nChannel, int nIndex, unsigned int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readTRXDDSReg( nChannel, nIndex, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteTRXDDSReg( void *pController, int nChannel, int nIndex, unsigned int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeTRXDDSReg( nChannel, nIndex, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadRX2DDSReg( void *pController, int nChannel, int nIndex, unsigned int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readRX2DDSReg( nChannel, nIndex, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteRX2DDSReg( void *pController, int nChannel, int nIndex, unsigned int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeRX2DDSReg( nChannel, nIndex, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadPIOReg( void *pController, int nIndex, int nRegister, unsigned int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readPIOReg( nIndex, nRegister, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWritePIOReg( void *pController, int nIndex, int nRegister, unsigned int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writePIOReg( nIndex, nRegister, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadTRXEEPROMByte( void *pController, unsigned int nOffset, uint8 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readTRXEEPROMByte( nOffset, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadTRXEEPROMUshort( void *pController, unsigned int nOffset, uint16 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readTRXEEPROMUshort( nOffset, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadTRXEEPROMUint( void *pController, unsigned int nOffset, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readTRXEEPROMUint( nOffset, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadTRXEEPROMFloat( void *pController, unsigned int nOffset, float *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readTRXEEPROMFloat( nOffset, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteTRXEEPROMByte( void *pController, unsigned int nOffset, uint8 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeTRXEEPROMByte( nOffset, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteTRXEEPROMUshort( void *pController, unsigned int nOffset, uint16 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeTRXEEPROMUshort( nOffset, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteTRXEEPROMUint( void *pController, unsigned int nOffset, uint32 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeTRXEEPROMUint( nOffset, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteTRXEEPROMFloat( void *pController, unsigned int nOffset, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeTRXEEPROMFloat( nOffset, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadRX2EEPROMByte( void *pController, unsigned int nOffset, uint8 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readRX2EEPROMByte( nOffset, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadRX2EEPROMUshort( void *pController, unsigned int nOffset, uint16 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readRX2EEPROMUshort( nOffset, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadRX2EEPROMUint( void *pController, unsigned int nOffset, uint32 *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readRX2EEPROMUint( nOffset, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadRX2EEPROMFloat( void *pController, unsigned int nOffset, float *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readRX2EEPROMFloat( nOffset, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteRX2EEPROMByte( void *pController, unsigned int nOffset, uint8 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeRX2EEPROMByte( nOffset, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteRX2EEPROMUshort( void *pController, unsigned int nOffset, uint16 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeRX2EEPROMUshort( nOffset, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteRX2EEPROMUint( void *pController, unsigned int nOffset, uint32 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeRX2EEPROMUint( nOffset, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteRX2EEPROMFloat( void *pController, unsigned int nOffset, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeRX2EEPROMFloat( nOffset, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTRXPot( void *pController, unsigned int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTRXPot( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALTRXPotSetRDAC( void *pController, int nIndex, int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->trxPotSetRDAC( nIndex, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALTRXPotGetRDAC( void *pController, unsigned int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->trxPotGetRDAC( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALPAPotSetRDAC( void *pController, int nIndex, int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->paPotSetRDAC( nIndex, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALPAPotGetRDAC( void *pController, unsigned int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->paPotGetRDAC( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetMux( void *pController, int nChannel )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setMux( nChannel );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALWriteCodecReg( void *pController, int nRegister, int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->writeCodecReg( nRegister, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadCodecReg( void *pController, int nRegister, int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readCodecReg( nRegister, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX1Freq( void *pController, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX1Freq( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX1FreqTW( void *pController, unsigned int nTW, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX1FreqTW( nTW, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX2Freq( void *pController, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX2Freq( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX2FreqTW( void *pController, unsigned int nTW, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX2FreqTW( nTW, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTXFreq( void *pController, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTXFreq( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTXFreqTW( void *pController, unsigned int nTW, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTXFreqTW( nTW, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTRXPreamp( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTRXPreamp( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTest( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTest( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetGen( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setGen( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetSig( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setSig( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetImpulse( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setImpulse( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetXVEN( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setXVEN( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetXVTXEN( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setXVTXEN( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetQSD( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setQSD( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetQSE( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setQSE( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetXREF( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setXREF( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX1Filter( void *pController, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX1Filter( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX2Filter( void *pController, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX2Filter( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTXFilter( void *pController, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTXFilter( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetIntSpkr( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setIntSpkr( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX1Tap( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX1Tap( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALBypassRX2Filter( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->bypassRX2Filter( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALBypassTXFilter( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->bypassTXFilter( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALBypassPAFilter( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->bypassPAFilter( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadPTT( void *pController, bool *pbDot, bool *pbDash, bool *pbRCAPTT, bool *pbMicPTT )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readPTT( pbDot, pbDash, pbRCAPTT, pbMicPTT );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadDot( void *pController, bool *pbValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readDot( pbValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadDash( void *pController, bool *pbValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readDash( pbValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadRCAPTT( void *pController, bool *pbValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readRCAPTT( pbValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadMicPTT( void *pController, bool *pbValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readMicPTT( pbValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadMicDown( void *pController, bool *pbValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readMicDown( pbValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadMicUp( void *pController, bool *pbValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readMicUp( pbValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALReadMicFast( void *pController, bool *pbValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readMicFast( pbValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetHeadphone( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setHeadphone( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetPLL( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setPLL( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRCATX1( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRCATX1( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRCATX2( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRCATX2( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRCATX3( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRCATX3( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetFan( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setFan( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetPLLStatus2( void *pController, bool *pbValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getPLLStatus2( pbValue );
}
                 
FLEXPAL_EXPORT tFlexPALrc FlexPALReadPAADC( void *pController, int nChannel, int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->readPAADC( nChannel, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX1Loop( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX1Loop( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTR( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTR( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetAnt( void *pController, int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setAnt( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX1Ant( void *pController, int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX1Ant( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX2Ant( void *pController, int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX2Ant( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTXAnt( void *pController, int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTXAnt( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTXMon( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTXMon( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetXVCOM( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setXVCOM( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetEN2M( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setEN2M( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetKey2M( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setKey2M( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetXVTR( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setXVTR( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetPABias( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setPABias( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetPowerOff( void *pController )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setPowerOff();
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetFPLED( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setFPLED( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetCTS( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setCTS( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRTS( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setCTS( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetPCReset( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setPCReset( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetPCPWRBT( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setPCPWRBT( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetMox( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setMox( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetIntLED( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setIntLED( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALATUSendCmd( void *pController, uint8 nValue1, uint8 nValue2, uint8 nValue3 )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->atuSendCmd( nValue1, nValue2, nValue3 );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALATUGetResult( void *pController, uint8 *pnCmd, uint8 *pnValue2, uint8 *pnValue3, uint8 *pnValue4, unsigned int nTimeoutMS )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->atuGetResult( pnCmd, pnValue2, pnValue3, pnValue4, nTimeoutMS );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetFullDuplex( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setFullDuplex( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTXDAC( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTXDAC( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetAmpTX1( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setAmpTX1( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetAmpTX2( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setAmpTX2( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetAmpTX3( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setAmpTX3( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetXVTRActive( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setXVTRActive( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetXVTRSplit( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setXVTRSplit( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX1Out( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX1Out( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALFlexWireWriteValue( void *pController, uint8 nAddress, uint8 nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->flexwireWriteValue( nAddress, nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALFlexWireWrite2Value( void *pController, uint8 nAddress, uint8 nValue1, uint8 nValue2 )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->flexwireWrite2Value( nAddress, nValue1, nValue2 );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALFlexWireReadValue( void *pController, uint16 nAddress, unsigned int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->flexwireReadValue( nAddress, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALFlexWireRead2Value( void *pController, uint16 nAddress, unsigned int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->flexwireRead2Value( nAddress, pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX1DSPMode( void *pController, int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX1DSPMode( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX2DSPMode( void *pController, int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX2DSPMode( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX2On( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX2On( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetStandby( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setStandby( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetAmpTX1DelayEnable( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setAmpTX1DelayEnable( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetAmpTX2DelayEnable( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setAmpTX2DelayEnable( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetAmpTX3DelayEnable( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setAmpTX3DelayEnable( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetAmpTX1Delay( void *pController, unsigned int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setAmpTX1Delay( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetAmpTX2Delay( void *pController, unsigned int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setAmpTX2Delay( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetAmpTX3Delay( void *pController, unsigned int nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setAmpTX3Delay( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALResetRX2DDS( void *pController )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->resetRX2DDS();
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRX2Preamp( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRX2Preamp( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetIambic( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setIambic( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetBreakIn( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setBreakIn( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetManualRX1Filter( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setManualRX1Filter( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetManualRX2Filter( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setManualRX2Filter( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetHiZ( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setHiZ( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetPDrvMon( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setPDrvMon( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetATUEnable( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setATUEnable( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetATUAttn( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setATUAttn( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetRXAttn( void *pController, bool bValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setRXAttn( bValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetFanPWM( void *pController, int nOn, int nOff )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setFanPWM( nOn, nOff );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetFanSpeed( void *pController, float nValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setFanSpeed( nValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetRegion( void *pController, unsigned int *pnValue )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getRegion( pnValue );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTXDSPFilter( void *pController, int nLow, int nHigh )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTXDSPFilter( nLow, nHigh );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTXOffset( void *pController, int nOffset )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTXOffset( nOffset );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetTXDSPMode( void *pController, int nMode )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setTXDSPMode( nMode );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALSetCWPitch( void *pController, unsigned int nPitch )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->setCWPitch( nPitch );
}

FLEXPAL_EXPORT tFlexPALrc FlexPALGetStatus( void *pController, bool *pnStatus )
{
  if( nCheckFPCPtr( pController ) != FLEXPAL_SUCCESS )
    return FLEXPAL_ERROR_INVALID_FPC_INSTANCE;
  else
    return ((FlexPALController *)pController)->getStatus( pnStatus );
}


