/*
 *  FlexPALCInterface.h
 *  flexpal
 *
 *  Created by Frank Goenninger on 26.03.10.
 *  Copyright 2010 Consequor Consulting AG. All rights reserved.
 *
 */

#ifndef __FLEXPAL_C_INTERFACE_H__
#define __FLEXPAL_C_INTERFACE_H__

#include <FlexPAL/FlexPALtypes.h>
#include <tcat.h>

void               * FlexPALGetInstance( void );

/* LOW-LEVEL C INTERFACE */

tsFlexPALErrorInfo * FlexPALGetErrorInfo( void * );
void                 FlexPALSetDebugLevel( void *, int );
void                 FlexPALSetNotifyCB( void *, void * );

int                  FlexPALGetNumberOfDevices( void * );
tFlexPALrc           FlexPALSelectDeviceByID( void *, int );

tFlexPALrc           FlexPALGetDeviceInfo( void *, uint32 *, uint32 * );
tFlexPALrc           FlexPALReadOp( void *, uint32, uint32, uint32, uint32 * );
tFlexPALrc           FlexPALWriteOp( void *, uint32, uint32, uint32 );
tFlexPALrc           FlexPALSetBufferSize( void *, uint32 );

/* HIGH-LEVEL C INTERFACE */

tFlexPALrc           FlexPALSetRegister( void *, uint32, uint32 );
tFlexPALrc           FlexPALGetRegister( void *, uint32, uint32 * );
tFlexPALrc           FlexPALGPIOWrite( void *, uint32 );
tFlexPALrc           FlexPALGPIORead( void *, uint32 * );
tFlexPALrc           FlexPALDDRGPIOWrite( void *, uint32 );
tFlexPALrc           FlexPALGPIODDRRead( void *, uint32 * );
tFlexPALrc           FlexPALI2CWriteValue( void *, uint8 , uint8 );
tFlexPALrc           FlexPALI2CWrite2Value( void *, uint8 , uint8, uint8 );
tFlexPALrc           FlexPALI2CReadValue( void *, uint8, uint8 * );
tFlexPALrc           FlexPALGetSerialNum( void *, uint32 * );
tFlexPALrc           FlexPALGetModel( void *, uint32 * );
tFlexPALrc           FlexPALGetFirmwareRev( void *, uint32 * );
tFlexPALrc           FlexPALInitialize( void * );
tFlexPALrc           FlexPALGetTRXOK( void *, bool * );
tFlexPALrc           FlexPALGetTRXRev( void *, uint32 * );
tFlexPALrc           FlexPALGetTRXSN( void *, uint32 * );
tFlexPALrc           FlexPALGetPAOK( void *, bool * );
tFlexPALrc           FlexPALGetPARev( void *, uint32 * );
tFlexPALrc           FlexPALGetPASN( void *, uint32 * );
tFlexPALrc           FlexPALGetRFIOOK( void *, bool * );
tFlexPALrc           FlexPALGetRFIORev( void *, uint32 * );
tFlexPALrc           FlexPALGetRFIOSN( void *, uint32 * );
tFlexPALrc           FlexPALGetATUOK( void *, bool * );
tFlexPALrc           FlexPALGetATURev( void *, uint32 * );
tFlexPALrc           FlexPALGetATUSN( void *, uint32 * );
tFlexPALrc           FlexPALGetRX2OK( void *, bool * );
tFlexPALrc           FlexPALGetRX2Rev( void *, uint32 * );
tFlexPALrc           FlexPALGetRX2SN( void *, uint32 * );
tFlexPALrc           FlexPALGetVUOK( void *, bool * );
tFlexPALrc           FlexPALGetVURev( void *, uint32 * );
tFlexPALrc           FlexPALGetVUSN( void *, uint32 * );
tFlexPALrc           FlexPALReadClockReg( void *, int, int * );
tFlexPALrc           FlexPALWriteClockReg( void *, int, int );
tFlexPALrc           FlexPALReadTRXDDSReg( void *, int, int, unsigned int * );
tFlexPALrc           FlexPALWriteTRXDDSReg( void *, int, int, unsigned int );
tFlexPALrc           FlexPALReadRX2DDSReg( void *, int, int, unsigned int * );




#endif
