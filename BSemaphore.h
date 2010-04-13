#ifndef BSEMAPHORE_H
#define BSEMAPHORE_H

// Use Win or MACH or Posix
#ifdef WINDOWS
	#include <windows.h>
	#define LMAXIMUMCOUNT 99999999 /**< Maximum semaphore value in Windows*/
#else
  #if defined( __MACH__ )
    #include <mach/mach_init.h>
    #include <mach/semaphore.h>
    #include <mach/task.h>
    #include <mach/mach_traps.h>
  #else
	  #ifndef POSIX
	  	#warning POSIX will be used (but you did not define it)
	  #endif
  	#include <semaphore.h>
  #endif
#endif

#include "FlexPALPriv.h"

/**
* @author Berenger
* @version 0.5
* @date February 15 2010
* @file BSemaphore.hpp
* @package Package-OS specific (POSS)
* @brief Semaphore
*
*
* This class represent a simple way to use semaphore
*
* @must You may have to change this class if you are not on Windows
* @must or Posix OS
*
* All methods may be inlined by the compiler
* @copyright Brainable.Net
*/

class BSemaphore{
protected:

#ifdef WINDOWS
	HANDLE _sem;	/**< Win semaphore*/
#else
  #if defined( __MACH__ )
    semaphore_t _sem; /**< MACH semaphore */
  #else
	  sem_t _sem;		/**< Posix semaphore*/
  #endif
#endif

public:
	/**
	* @brief Constructor
	* @param in_init original value
	*/
	BSemaphore( int in_init = 0 ){
	#ifdef WINDOWS
		_sem = CreateSemaphore(0,in_init,LMAXIMUMCOUNT,0); 
  #else
    #if defined( __MACH__ )
      kern_return_t nRC;
      nRC = semaphore_create(mach_task_self(), &_sem, SYNC_POLICY_FIFO, in_init);
      if( nRC != KERN_SUCCESS )
        abort();
    #else
	  	sem_init(&_sem,0,in_init);
    #endif
	#endif
	}
	
	/**
	* @brief Copy constructor
	* @param in_sem original semaphore
	*/
	BSemaphore(const BSemaphore &in_sem){
		int value = in_sem.value();
	#ifdef WINDOWS
		_sem = CreateSemaphore(0,value,LMAXIMUMCOUNT,0); 
	#else
    #if defined( __MACH__ )
      semaphore_create(mach_task_self(), &_sem, SYNC_POLICY_FIFO, value);
    #else
		  sem_init(&_sem,0,value); 
    #endif
	#endif
	}
	
	/**
	* @brief Copy method
	* @param in_sem original semaphore
	*/
	void operator=(const BSemaphore &in_sem){
		reset(in_sem.value());
	}
	
	
	/**
	* @brief destroy semaphore
	*/
	virtual ~BSemaphore(){ 
	#ifdef WINDOWS
		CloseHandle(_sem);
	#else
    #if defined( __MACH__ )
      semaphore_destroy(mach_task_self(), _sem);
    #else
		  sem_destroy(&_sem);
    #endif
	#endif
	}

	/**
	* @brief Wait until the semaphore is called by another thread
	* @return true if success or false if timeout or error
	*/
	bool wait() const { 
	#ifdef WINDOWS
		return WaitForSingleObject(static_cast< HANDLE >(_sem),INFINITE) == 0x00000000L;
	#else
    #if defined( __MACH__ )
      kern_return_t result = KERN_SUCCESS;
      result = semaphore_wait( _sem );
      return ( result == KERN_SUCCESS );
    #else
		  return sem_wait(const_cast<sem_t*>(&_sem)) == 0;
    #endif
  #endif
	}

	bool wait( unsigned int nMilliSeconds ) const { 
  #ifdef WINDOWS
		return WaitForSingleObject(static_cast< HANDLE >(_sem), nMilliSeconds) == 0x00000000L;
  #else
    #if defined( __MACH__ )
      mach_timespec_t sWaitTime = { (int) nMilliSeconds / 1000, ( nMilliSeconds % 1000 ) * 1000000 };
      return (semaphore_timedwait(_sem, sWaitTime) != KERN_OPERATION_TIMED_OUT);
    #else
      timespec_t sWaitTime = { (int) nMilliSeconds / 1000, ( nMilliSeconds % 1000 ) * 1000000 };
      return sem_timedwait(_sem, sWaitTime) == 0;
    #endif
  #endif
	}
	
	/**
	* @brief post a token
	* @return true if success or false if error
	*/
	bool post(){ 
	#ifdef WINDOWS
		return ReleaseSemaphore(static_cast< HANDLE >(_sem),1,0) != 0;
  #else
    #if defined( __MACH__ )
      return semaphore_signal( _sem ) == KERN_SUCCESS;
	  #else
		  return sem_post(&_sem) == 0;
    #endif
	#endif
	}
	
	/**
	* @brief get current value
	* @return value
	*/
	int value() const{ 
	#ifdef WINDOWS
		long value = -1;
		ReleaseSemaphore(static_cast<const HANDLE>(_sem),0,&value);
		return value;
  #else
    #if defined( __MACH__ )
      return 0;
	  #else
	  	int value = -1;
  		sem_getvalue(const_cast<sem_t*>(&_sem),&value);
	  	return value;
    #endif
	#endif
	}

	/**
	* @brief release current semaphore and create a new one
	* @param init the value after reset
	*/
	void reset( int in_init = 0 ){ 
	#ifdef WINDOWS
		CloseHandle(_sem);
		_sem = CreateSemaphore(0,in_init,LMAXIMUMCOUNT,0);
  #else
    #if defined( __MACH__ )
      semaphore_destroy(mach_task_self(), _sem);
      semaphore_create(mach_task_self(), &_sem, SYNC_POLICY_FIFO, in_init);
	  #else
	  	sem_destroy(&_sem);
	  	sem_init(&_sem,0,in_init);
    #endif
	#endif

	}
};


#endif
