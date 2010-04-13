/*
 *  FlexPALTypes.h
 *  flexpal
 *
 *  Created by Frank Goenninger on 26.03.10.
 *  Copyright 2010 Consequor Consulting AG. All rights reserved.
 *
 */

#ifndef __FLEXPAL_TYPES_H__
#define __FLEXPAL_TYPES_H__

// ERROR HANDLING

#define FLEXPAL_SUCCESS  0
#define FLEXPAL_ERROR   -1

typedef int tFlexPALrc;       // Return code for FlexPAL functions

typedef struct _tsFlexPALErrorInfo
{
  int            nErrno;      // Error number
  const char   * pcFile;      // Source file
  int            nLine;       // Line nr in source file 
  const char   * pcFn;        // Name of function in which error occurred
  time_t         nTimestamp;  // Timestamp of error
  const char   * pcMessage;   // Error Message
  
} tsFlexPALErrorInfo;

#define FLEXPAL_ERROR_DRIVER_MISMATCH            1001
#define FLEXPAL_ERROR_CANNOT_CREATE_SYSTEM       1002
#define FLEXPAL_ERROR_INVALID_FPC_INSTANCE       1003
#define FLEXPAL_ERROR_INVALID_DEVICE_ID          1004
#define FLEXPAL_ERROR_READ_OP_FAILED             1005
#define FLEXPAL_ERROR_WRITE_OP_FAILED            1006
#define FLEXPAL_ERROR_INVALID_DEVICE             1007

#define FLEXPAL_NO_DEVICE                          -1


#endif