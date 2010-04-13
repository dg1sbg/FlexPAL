/*
 *  FlexPALPriv.h
 *  FlexPAL
 *
 *  Created by Frank Goenninger on 14.03.10.
 *  Copyright 2010 Consequor Consulting AG. All rights reserved.
 *
 */

#ifndef __FLEXPALPRIV_H__
#define __FLEXPALPRIV_H__

#ifdef WIN32
#define FLEXPAL_EXPORT extern "C" __declspec(dllexport)
#else
#define FLEXPAL_EXPORT extern "C"
#endif

#ifdef WIN32
  #define WINDOWS
#else
  #if defined( __MACH__ )
    #undef POSIX
  #else
    #define POSIX
    #if defined( __APPLE__ )
      #error "#define not correct: This is an Apple machine but does not support MACH defines ..."
    #endif
  #endif
#endif

#ifndef WIN32
#define DWORD int
#define WINAPI
#define LPVOID void *
#endif

#include <iostream>
#include <errno.h>
#include <string>
#include <map>
#include <vector>

#include "tcat_dice_pal_system.h"
#include "tcat_dice_pal.h"
#include "tcat.h"

using namespace std;
using namespace tcat;
using namespace tcat::dice;

#include "FlexPALTypes.h"
#include "FlexPALOpCodes.h"

class FlexPALDevice;
class FlexPALController;
class BSemaphore;

#include "BSemaphore.h"
#include "BThread.h"

typedef tcat::dice::reference<FlexPALDevice> tFlexPALDeviceRef;
typedef tcat::dice::reference<tcat::dice::bus<FlexPALDevice> > tFlexPALBusRef;
typedef tcat::dice::reference<tcat::dice::system<tcat::dice::bus<FlexPALDevice> > > tFlexPALSystemRef;
typedef tcat::dice::system<tcat::dice::bus<FlexPALDevice> > sys;

#include "FlexPALDevice.h"
#include "FlexPAL.h"

#endif // __FLEXPALPRIV_H__