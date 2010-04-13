/*
 *  FlexPALDevice.h
 *  FlexPAL
 *
 *  Created by Frank Goenninger on 14.03.10.
 *  Copyright 2010 Consequor Consulting AG. All rights reserved.
 *
 */

#ifndef __FLEXPALDEVICE_H__
#define __FLEXPALDEVICE_H__

class FlexPALDevice : public tcat::dice::device
{
  friend class FlexPALController;
  
  public:

  ~FlexPALDevice();
  
  void                mount();
  
  tFlexPALrc          readOp( uint32, uint32, uint32, uint32 * );
  tFlexPALrc          writeOp( uint32, uint32, uint32 );
  
  tFlexPALrc          deviceInfo( uint32 *, uint32 * );
  tFlexPALrc          setBufferSize( uint32 );
  
  private:
  
	void                update_user( tcat::dice::DEVICE_NOTIFICATION );
  tFlexPALrc          waitForCommandCompletion();
  tFlexPALrc          signalCommandCompletion();
  tFlexPALrc          waitForCommandQueueEmpty();
  tFlexPALrc          activate();

  tFlexPALDeviceRef   hDevice;
  
  int                 mBusID;    /* TCAT PAL BUS ID of bus of device           */
  int                 mDeviceID; /* TCAT PAL Device ID of dvice on current bus */
  
  tcat::uint64        mGUID;      
  tcat::uint32        mModel;
  tcat::uint32        mSerialNr;
};

#endif
