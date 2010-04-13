#ifndef BSLEEP_H
#define BSLEEP_H


// Use Win or Posix
#ifdef WINDOWS
	#include <windows.h>
#else
	#ifndef POSIX
		#warning POSIX will be used (but you did not define it)
	#endif
	#include <unistd.h>
#endif


/**
* @author Berenger
* @version 0.5
* @date February 15 2010
* @file BSleep.hpp
* @package Package-OS specific (POSS)
* @brief Cross plateform sleep functions
*
* @example BSleep::sleep(50); // Sleep 50 ms
*
* @must You may have to change this class if you are not on Windows
* @must or Posix OS
*
* All methods may be inlined by the compiler
* @copyright Brainable.Net
*/
namespace BSleep{
	/**
	* @brief sleep in milli seconds
	* @param in_mseconds the number of milli seconds
	*/
	void msleep( unsigned int in_mseconds ){
	#ifdef WINDOWS
		Sleep( in_mseconds );
	#else
		usleep( in_mseconds * 1000 );
	#endif
	}
	/**
	* @brief sleep in seconds
	* @param in_mseconds the number of seconds
	*/
	void ssleep( unsigned int in_seconds ){
	#ifdef WINDOWS
		Sleep(in_seconds * 1000);
	#else
		sleep(in_seconds);
	#endif
	}
}


#endif


