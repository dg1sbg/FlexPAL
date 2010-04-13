/*
 *  FlexPAL.h
 *  FlexPAL
 *
 *  Created by Frank Goenninger on 14.03.10.
 *  Copyright 2010 Consequor Consulting AG. All rights reserved.
 *
 *  $Header$
 */

#ifndef __FLEXPAL_H__
#define __FLEXPAL_H__

#include <FlexPAL/FlexPALPriv.h>

// GENERAL DEFINES AND DECLARATIONS

#ifdef WIN32
typedef void (__stdcall *NotifyCBPtr)(uint32_t nBitmap);
#else
typedef void (*NotifyCBPtr)( uint32_t nBitmap );  
#endif

// FLEXPAL DEFINES

#define FLEXPAL_MAX_RETRIES           10
#define FLEXPAL_MAX_READ_OP_RETRIES   FLEXPAL_MAX_RETRIES
#define FLEXPAL_MAX_WRITE_OP_RETRIES  FLEXPAL_MAX_RETRIES

#define FLEXPAL_DEBUG_LEVEL_NO_DEBUG   0
#define FLEXPAL_DEBUG_LEVEL_1          1
#define FLEXPAL_DEBUG_LEVEL_2          2

#define FLEXPAL_DEBUG_LEVEL_DEFAULT    FLEXPAL_DEBUG_LEVEL_2


// DICE DEFINES

#define DD_NOTIFY_USER4 0x01000000
#define DD_NOTIFY_USER5 0x02000000
#define DD_NOTIFY_USER6 0x04000000

// CLASSES

class LockObject : public tcat::dice::base
{
public:
  char *pcName;
};

class NotificationThread : public BThread
{
  public:
  void run();
};

class FlexPALController
{
  friend class FlexPALDevice;
  
	public:
		
  // FlexPALController PUBLIC METHODS
    
  static FlexPALController * getInstance( void );
  
  // Debug Aids
  
  char                     * name( void ) { return (char *)"FlexPALController"; };
  int                        debugLevel( void ) { return mDebugLevel; };
  void                       setDebugLevel( int nLevel ) { mDebugLevel = nLevel; };
  
  // FlexPALController functionality
  
  const tsFlexPALErrorInfo * errorInfo( void );
  
  // LOW-LEVEL PUBLIC METHODS
  
  void          * selectedDevice( void );
  const int       numberOfDevices( void );
  tFlexPALrc      reset( void );
  tFlexPALrc      init( void );
  void            setNotifyCB( void * );
  tFlexPALrc      selectDeviceByID( const int );
  void            selectDevice( FlexPALDevice * );
  const int       selectedDeviceID( void );
  tFlexPALrc      deviceInfo( uint32 *, uint32 * );  
  tFlexPALrc      writeOp( uint32, uint32, uint32 );
  tFlexPALrc      readOp( uint32, uint32, uint32, uint32 * );
  tFlexPALrc      setBufferSize( uint32 );
  
  // HIGH-LEVEL PUBLIC METHODS
  
  tFlexPALrc      setRegister( uint32, uint32 );
  tFlexPALrc      getRegister( uint32, uint32 * );
  tFlexPALrc      gpioWrite( uint32 );
  tFlexPALrc      gpioRead( uint32 * );
  tFlexPALrc      gpioDDRWrite( uint32 );             
  tFlexPALrc      gpioDDRRead( uint32 * );
  tFlexPALrc      i2cWriteValue( uint16, uint8 );
  tFlexPALrc      i2cWrite2Value( uint16, uint8, uint8 );
  tFlexPALrc      i2cReadValue( uint16, unsigned int * );
  tFlexPALrc      getSerialNum( uint32 * );
  tFlexPALrc      getModel( uint32 * );
  tFlexPALrc      getFirmwareRev( uint32 * );
  tFlexPALrc      initialize( void );
  tFlexPALrc      getTRXOK( bool * );
  tFlexPALrc      getTRXRev( uint32 * );
  tFlexPALrc      getTRXSN( uint32 * );
  tFlexPALrc      getPAOK( bool * );
  tFlexPALrc      getPARev( uint32 * );
  tFlexPALrc      getPASN( uint32 * );
  tFlexPALrc      getRFIOOK( bool * );
  tFlexPALrc      getRFIORev( uint32 * );
  tFlexPALrc      getRFIOSN( uint32 * );
  tFlexPALrc      getATUOK( bool * );
  tFlexPALrc      getATURev( uint32 * );
  tFlexPALrc      getATUSN( uint32 * );
  tFlexPALrc      getRX2OK( bool * );
  tFlexPALrc      getRX2Rev( uint32 * );
  tFlexPALrc      getRX2SN( uint32 * );
  tFlexPALrc      getVUOK( bool * );
  tFlexPALrc      getVURev( uint32 * );
  tFlexPALrc      getVUSN( uint32 * );
  tFlexPALrc      readClockReg( int, int * );
  tFlexPALrc      writeClockReg( int, int );
  tFlexPALrc      readTRXDDSReg( int, int, unsigned int * );
  tFlexPALrc      writeTRXDDSReg( int, int, unsigned int );
  tFlexPALrc      readRX2DDSReg( int, int, unsigned int * );
  tFlexPALrc      writeRX2DDSReg( int, int, unsigned int );
  tFlexPALrc      readPIOReg( int, int, unsigned int * );
  tFlexPALrc      writePIOReg( int, int, unsigned int );
  tFlexPALrc      readTRXEEPROMByte( unsigned int, uint8 * );
  tFlexPALrc      readTRXEEPROMUshort( unsigned int, uint16 * );
  tFlexPALrc      readTRXEEPROMUint( unsigned int, uint32 * );
  tFlexPALrc      readTRXEEPROMFloat( unsigned int, float * );
  tFlexPALrc      writeTRXEEPROMByte( unsigned int, uint8 );
  tFlexPALrc      writeTRXEEPROMUshort( unsigned int, uint16 );
  tFlexPALrc      writeTRXEEPROMUint( unsigned int, uint32 );
  tFlexPALrc      writeTRXEEPROMFloat( unsigned int, float );
  tFlexPALrc      readRX2EEPROMByte( unsigned int, uint8 * );
  tFlexPALrc      readRX2EEPROMUshort( unsigned int, uint16 * );
  tFlexPALrc      readRX2EEPROMUint( unsigned int, uint32 * );
  tFlexPALrc      readRX2EEPROMFloat( unsigned int, float * );
  tFlexPALrc      writeRX2EEPROMByte( unsigned int, uint8 );
  tFlexPALrc      writeRX2EEPROMUshort( unsigned int, uint16 );
  tFlexPALrc      writeRX2EEPROMUint( unsigned int, uint32 );
  tFlexPALrc      writeRX2EEPROMFloat( unsigned int, float );
  tFlexPALrc      setTRXPot( unsigned int );
  tFlexPALrc      trxPotSetRDAC( int, int );
  tFlexPALrc      trxPotGetRDAC( unsigned int * );
  tFlexPALrc      paPotSetRDAC( int, int );
  tFlexPALrc      paPotGetRDAC( unsigned int * );
  tFlexPALrc      setMux( int );
  tFlexPALrc      writeCodecReg( int, int );
  tFlexPALrc      readCodecReg( int, int * );
  tFlexPALrc      setRX1Freq( float );
  tFlexPALrc      setRX1FreqTW( uint32, float );
  tFlexPALrc      setRX2Freq( float );
  tFlexPALrc      setRX2FreqTW( uint32, float );
  tFlexPALrc      setTXFreq( float );
  tFlexPALrc      setTXFreqTW( uint32, float );
  tFlexPALrc      setTRXPreamp( bool );
  tFlexPALrc      setTest( bool );
  tFlexPALrc      setGen( bool );
  tFlexPALrc      setSig( bool );
  tFlexPALrc      setImpulse( bool );
  tFlexPALrc      setXVEN( bool );
  tFlexPALrc      setXVTXEN( bool );
  tFlexPALrc      setQSD( bool );
  tFlexPALrc      setQSE( bool );
  tFlexPALrc      setXREF( bool );
  tFlexPALrc      setRX1Filter( float );
  tFlexPALrc      setRX2Filter( float );
  tFlexPALrc      setTXFilter( float );
  tFlexPALrc      setPAFilter( float );
  tFlexPALrc      setIntSpkr( bool );
  tFlexPALrc      setRX1Tap( bool );
  /* -------------------------------- */
  tFlexPALrc      bypassRX1Filter( bool );
  tFlexPALrc      bypassRX2Filter( bool );
  tFlexPALrc      bypassTXFilter( bool );
  tFlexPALrc      bypassPAFilter( bool );
  tFlexPALrc      readPTT( bool *, bool *, bool *, bool * );
  tFlexPALrc      readDot( bool * );
  tFlexPALrc      readDash( bool * );
  tFlexPALrc      readRCAPTT( bool * );
  tFlexPALrc      readMicPTT( bool * );
  tFlexPALrc      readMicDown( bool * );
  tFlexPALrc      readMicUp( bool * );
  tFlexPALrc      readMicFast( bool * );
  tFlexPALrc      setHeadphone( bool );
  tFlexPALrc      setPLL( bool );
  tFlexPALrc      setRCATX1( bool );
  tFlexPALrc      setRCATX2( bool );
  tFlexPALrc      setRCATX3( bool );
  tFlexPALrc      setFan( bool );
  tFlexPALrc      getPLLStatus2( bool * );
  tFlexPALrc      readPAADC( int, int * );
  tFlexPALrc      setRX1Loop( bool );
  tFlexPALrc      setTR( bool );
  tFlexPALrc      setAnt( int );
  tFlexPALrc      setRX1Ant( int );
  tFlexPALrc      setRX2Ant( int );
  tFlexPALrc      setTXAnt( int);
  tFlexPALrc      setTXMon( bool );
  tFlexPALrc      setXVCOM( bool );
  tFlexPALrc      setEN2M( bool );
  tFlexPALrc      setKey2M( bool );
  tFlexPALrc      setXVTR( bool );
  tFlexPALrc      setPABias( bool );
  tFlexPALrc      setPowerOff( void );
  tFlexPALrc      setFPLED( bool );
  tFlexPALrc      setCTS( bool );
  tFlexPALrc      setRTS( bool );
  tFlexPALrc      setPCReset( bool );
  tFlexPALrc      setPCPWRBT( bool );
  tFlexPALrc      setMox( bool );
  tFlexPALrc      setIntLED( bool );
  tFlexPALrc      atuSendCmd( uint8, uint8, uint8 );
  tFlexPALrc      atuGetResult( uint8 *, uint8 *, uint8 *, uint8 *, uint32 );
  tFlexPALrc      setFullDuplex( bool );
  tFlexPALrc      setTXDAC( bool );
  tFlexPALrc      setAmpTX1( bool );
  tFlexPALrc      setAmpTX2( bool );
  tFlexPALrc      setAmpTX3( bool );
  tFlexPALrc      setXVTRActive( bool );
  tFlexPALrc      setXVTRSplit( bool );
  tFlexPALrc      setRX1Out( bool );
  tFlexPALrc      flexwireWriteValue( uint8, uint8 );
  tFlexPALrc      flexwireWrite2Value( uint8, uint8, uint8 );
  tFlexPALrc      flexwireReadValue(uint16, unsigned int * );
  tFlexPALrc      flexwireRead2Value( uint16, unsigned int * );
  tFlexPALrc      setRX1DSPMode( int );
  tFlexPALrc      setRX2DSPMode( int );
  tFlexPALrc      setRX2On( bool );
  tFlexPALrc      setStandby( bool );
  tFlexPALrc      setAmpTX1DelayEnable( bool );
  tFlexPALrc      setAmpTX2DelayEnable( bool );
  tFlexPALrc      setAmpTX3DelayEnable( bool );
  tFlexPALrc      setAmpTX1Delay( uint32 );
  tFlexPALrc      setAmpTX2Delay( uint32 );
  tFlexPALrc      setAmpTX3Delay( uint32 );
  tFlexPALrc      resetRX2DDS( void );
  tFlexPALrc      setRX2Preamp( bool );
  tFlexPALrc      setIambic( bool );
  tFlexPALrc      setBreakIn( bool );
  tFlexPALrc      setManualRX1Filter( bool );
  tFlexPALrc      setManualRX2Filter( bool );
  tFlexPALrc      setHiZ( bool );
  tFlexPALrc      setPDrvMon( bool );
  tFlexPALrc      setATUEnable( bool );
  tFlexPALrc      setATUAttn( bool );
  tFlexPALrc      setRXAttn( bool );
  tFlexPALrc      setFanPWM( int, int );
  tFlexPALrc      setFanSpeed( float );
  tFlexPALrc      getRegion( uint32 * );
  tFlexPALrc      setTXDSPFilter( int, int );
  tFlexPALrc      setTXOffset( int );
  tFlexPALrc      setTXDSPMode( int );
  tFlexPALrc      setCWPitch( uint32 );
  tFlexPALrc      setPALCallback( void * );
  tFlexPALrc      getStatus( bool * );
  tFlexPALrc      setVU_FanHigh( bool );
  tFlexPALrc      setVU_KeyV( bool );
  tFlexPALrc      setVU_KeyU( bool );
  tFlexPALrc      setVU_K15( bool );
  tFlexPALrc      setVU_K13( bool );
  tFlexPALrc      setVU_K18( bool );
  tFlexPALrc      setVU_K16( bool );
  tFlexPALrc      setVU_K12( bool );
  tFlexPALrc      setVU_KeyVU( bool );
  tFlexPALrc      setVU_DrvU( bool );
  tFlexPALrc      setVU_DrvV( bool );
  tFlexPALrc      setVU_LPwrU( bool );
  tFlexPALrc      setVU_LPwrV( bool );
  tFlexPALrc      setVU_RX2U( bool );
  tFlexPALrc      setVU_TXIFU( bool );
  tFlexPALrc      setVU_RXIFU( bool );
  tFlexPALrc      setVU_UIFHG1( bool );
  tFlexPALrc      setVU_VIFHG1( bool );
  tFlexPALrc      setVU_UIFHG2( bool );
  tFlexPALrc      setVU_RXURX2( bool );
  tFlexPALrc      setVU_VIFHG2( bool );
  tFlexPALrc      setVU_RX2V( bool );
  tFlexPALrc      setVU_TXU( bool );
  tFlexPALrc      setVU_TXV( bool );
                               
  // PUBLIC MEMBER VARS
  
  BSemaphore               * pmAsyncCmdNotifySem;
  void                     * pfNotifyCB;


  private:
    
  // Singleton control
    
  FlexPALController() {}
  ~FlexPALController() {}
    
  static FlexPALController * smInstance;
  
  // FlexPAL private data
  
  char                     * mName;
  int                        mDebugLevel;
  
  tsFlexPALErrorInfo       * psErrorInfo;
  NotificationThread       * pNotificationThread;
  
  tFlexPALSystemRef          mPALSystem;
  
  std::vector<FlexPALDevice *> mDevicePtrs;
  int                          mSelectedDeviceID;
  FlexPALDevice              * mSelectedDevice;
  
  BSemaphore               * pmReadCmdNotifySem;
  BSemaphore               * pmWriteCmdNotifySem;
  BSemaphore               * pmAtomicCmdSem;
  
  /* FlexPAL private methods */

  void                       setErrorInfo( int, const char *, 
                                           int, const char *, 
                                           time_t, const char * );
  void                       findDevices( void );
  void                       unmountPreviouslySelectedDevice( void );
  
  
  
};

#endif
