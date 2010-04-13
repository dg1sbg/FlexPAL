#ifndef BTHREAD_H
#define BTHREAD_H

// Use Win or Posix
#ifdef WINDOWS
	#include <windows.h>
#else
  #if defined( __MACH__ )
    #include <pthread.h>
    #include <signal.h>
  #else
	  #ifndef POSIX
	  	#warning POSIX will be used (but you did not define it)
	  #endif
	  #include <pthread.h>
	  #include <signal.h>
  #endif
#endif


/**
* @author Berenger
* @version 0.5
* @date February 15 2010
* @file BThread.hpp
* @package Package-OS specific (POSS)
* @brief Thread
*
*
* This class represent a simple way to use thread. You must
* inherit and implement "run".
*
*
* @example BThread th;
* @example th.start();		// start thread
* @example ...
* @example th.isRunning();  // fast look up
* @example ...
* @example th.wait();		// wait thread end
* @example 
*
* Ressource : http://www.relisoft.com/Win32/active.html
*
* @must You may have to change this class if you are not on Windows
* @must or Posix OS
*
* All methods may be inlined by the compiler
* @copyright Brainable.Net
*/

class BThread{
private:
	/**
	* @brief Denied equality operator
	* @param none
	*/
	void operator=(const BThread &){}
	/**
	* @brief Denied copy constructor
	* @param none
	*/
	BThread(const BThread &){}

#ifdef WINDOWS
 	HANDLE _handle;
#else
	pthread_t _thread; /**< Posix Thread*/
#endif
	
	bool _isRunning;  /**< Fast bool lookup */


	/**
	* @brief Static starter function to execute posix thread
	* @brief This function set thread->isRunning to false
	*/
	#ifdef WINDOWS
 	static DWORD WINAPI Starter(LPVOID in_thread){
	#else
	static void* Starter(void* in_thread){
	#endif
		BThread * thread = static_cast< BThread * >(in_thread);
		thread->_isRunning = true;
		thread->run();
		thread->_isRunning = false;
		
		return 0x00;
	}

public:
	/**
	* @brief Constructor
	*/
	BThread(){
	#ifdef WINDOWS
 		_handle = 0x00;
	#else
	#endif
		_isRunning = false;
	}
	/**
	* @brief Destructor, Warning, it waits the end of the current thread
	*/
	virtual ~BThread(){
		if(!_isRunning) return;
        
        // if we destroy the thread until it has finished
        // there is a problem in your implementation algorithme
        // So we wait before destroying the thread!
        wait();
	#ifdef WINDOWS
		CloseHandle (_handle);
	#else
	#endif
	}

	/**
	* @brief start the thread
	* @return true if success else false
	*/
	bool start(){
		if(_isRunning) return false;		
	#ifdef WINDOWS
		_handle = CreateThread( 0x00, 0x00,BThread::Starter, static_cast< void* >(this), 0x00, 0x00);
		return _handle != NULL;
	#else
		return pthread_create(&_thread, NULL, BThread::Starter, static_cast< void* >(this)) == 0;
	#endif
	}

	/**
	* @brief Fast look up to know if a thread is running
	* @return true if running else false
	*/
	bool isRunning() const{
		return _isRunning;
	}

	/**
	* @brief Wait the end of a thread
	* @return false in case of error, true if all right
	*/
	bool wait() const{
 		if(!_isRunning) return false;
	#ifdef WINDOWS
		return WaitForSingleObject(_handle,INFINITE) == 0x00000000L;
	#else
		return pthread_join(_thread, NULL) == 0;
	#endif

	}

	/**
	* @brief the function is called when thread is starting
	* @must You must implement this methode!
	*/
	virtual void run() = 0;
	
	/**
	*
	*/
	bool kill(){
		if(!_isRunning) return false;	
		
        _isRunning = false;	
	#ifdef WINDOWS
		bool success = TerminateThread(_handle,1) && CloseHandle(_handle);
		_handle = 0x00;
 		return success;
	#else
 		return pthread_kill( _thread, SIGKILL) == 0;
	#endif
	}

};

#endif
