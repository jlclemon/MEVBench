/*
 * Copyright (c) 2006-2009 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include "ThreadManager.h"
#include <iostream>
#include <cstring>

Thread::Thread(void)
{
	#ifdef VERBOSE
	cout << "Thread Created" << endl;
	#endif

	threadFunction = NULL;
	threadParam = NULL;
	threadData = NULL;
	threadLogicalId = -1;
	barrierPtr = NULL;
	wakeupSet = false;
	int err = pthread_mutex_init(&wakeupLock, NULL);
	if(err != 0)
	{
		cout << "Error initializing mutex: " << strerror(err) << endl;
	}


	//int err = pthread_mutex_lock(&wakeupLock);
	//int err = pthread_mutex_unlock(&wakeupLock);

	err = pthread_cond_init(&wakeupSignalConditionVariable,NULL);
	if(err != 0)
	{
		cout << "Error initializing condition variable: " << strerror(err) << endl;
	}


	#ifdef VERBOSE
		cout << "Err in creation is " << err << endl;
	#endif
	//int err = pthread_mutex_lock(&wakeupLock);
	//int returnVale = pthread_cond_wait(&wakeupSignalConditionVariable,&wakeupLock);
	//wakeupSet = false;
	//err = pthread_mutex_unlock(&wakeupLock);
	//
	//int err = pthread_mutex_lock(&wakeupLock);
	//wakeupSet = true;
	//pthread_cond_signal(&wakeupSignalConditionVariable);
	//pthread_cond_broadcast(&wakeupSignalConditionVariable);
	//err = pthread_mutex_unlock(&wakeupLock);
}

Thread::Thread(const Thread & oldThread)
{
	#ifdef VERBOSE
	cout << "Thread Created" << endl;
	#endif


	this->threadData = oldThread.threadData;
	this->barrierPtr = oldThread.barrierPtr;
	this->threadFunction = oldThread.threadFunction;
	this->threadId = oldThread.threadId;
	this->threadLogicalId = oldThread.threadLogicalId;
	this->threadParam = oldThread.threadParam;

	wakeupSet = false;
	int err = pthread_mutex_init(&wakeupLock, NULL);
	if(err != 0)
	{
		cout << "Error initializing mutex: " << strerror(err) << endl;
	}


	//int err = pthread_mutex_lock(&wakeupLock);
	//int err = pthread_mutex_unlock(&wakeupLock);

	err = pthread_cond_init(&wakeupSignalConditionVariable,NULL);
	if(err != 0)
	{
		cout << "Error initializing condition variable: " << strerror(err) << endl;
	}

}


Thread::~Thread(void)
{
	#ifdef VERBOSE
	cout << "Thread Destroyed" << endl;
	#endif

	int err = pthread_mutex_destroy(&wakeupLock);
	if(err != 0)
	{
		cout << "Error destroying mutex: " << strerror(err) << endl;
	}



	pthread_cond_destroy(&wakeupSignalConditionVariable);
	if(err != 0)
	{
		cout << "Error destroying condition variable: " << strerror(err) << endl;
	}


}

Thread & Thread::operator=(const Thread & oldThread)
{
	#ifdef VERBOSE
	cout << "Thread Assigned" << endl;
	#endif


	this->threadData = oldThread.threadData;
	this->barrierPtr = oldThread.barrierPtr;
	this->threadFunction = oldThread.threadFunction;
	this->threadId = oldThread.threadId;
	this->threadLogicalId = oldThread.threadLogicalId;
	this->threadParam = oldThread.threadParam;



	return *this;
}

int Thread::wakeup()
{

	int err = pthread_mutex_lock(&wakeupLock);
	if(err != 0)
	{
		cout << "Error locking mutex: " << strerror(err) << endl;
	}


	if(!wakeupSet)
	{
		#ifdef VERBOSE
			cout << "Thread "<< this->threadLogicalId << " has been told to wake up" << endl;
		#endif
		wakeupSet = true;
		err = pthread_cond_signal(&wakeupSignalConditionVariable);
		if(err != 0)
		{
			cout << "Error signaling condition variable: " << strerror(err) << endl;
		}


		#ifdef VERBOSE
			cout << "Err on wakeup " << err << endl;
		#endif
		//pthread_cond_broadcast(&wakeupSignalConditionVariable);

	}
	err = pthread_mutex_unlock(&wakeupLock);
	if(err != 0)
	{
		cout << "Error unlocking mutex: " << strerror(err) << endl;
	}


	return err;
}

int Thread::waitForWakeup()
{
	int err = pthread_mutex_lock(&wakeupLock);
	if(err != 0)
	{
		cout << "Error locking mutex: " << strerror(err) << endl;
	}



	#ifdef VERBOSE
		cout << "Thread "<< this->threadLogicalId << " has gone to sleep." << endl;
	#endif
	while(!wakeupSet) 
	{
		err = pthread_cond_wait(&wakeupSignalConditionVariable,&wakeupLock);
		if(err != 0)
		{
			cout << "Error waiting on condition variable: " << strerror(err) << endl;
		}


		#ifdef VERBOSE
			cout << "Thread "<<this->threadLogicalId <<" Woke up for a second" << endl;
		#endif
	}
	wakeupSet = false;

	#ifdef VERBOSE
		cout << "Thread "<< this->threadLogicalId << " has woke up" << endl;
	#endif
	err = pthread_mutex_unlock(&wakeupLock);
	if(err != 0)
	{
		cout << "Error unlocking mutex: " << strerror(err) << endl;
	}


	return err;


}
int Thread::waitAtBarrier()
{

	int returnVal = pthread_barrier_wait(barrierPtr);
	if(!((returnVal == 0)|| (returnVal ==PTHREAD_BARRIER_SERIAL_THREAD)))
	{
		cout << "Error waiting at barrier: " << strerror(returnVal) << endl;
	}



	if(returnVal == PTHREAD_BARRIER_SERIAL_THREAD)
	{
		returnVal = 0;
	}
	return returnVal;
}

ThreadManager::ThreadManager(void)
{
	#ifdef VERBOSE
	cout << "Thread Manager Created" << endl;
	#endif
}

ThreadManager::ThreadManager(int numThreads)
{
	#ifdef VERBOSE
	cout << "Thread Manager Created" << endl;
	#endif
	this->threadList.resize(numThreads);
	this->setupBarrier();
}



ThreadManager::~ThreadManager(void)
{
	#ifdef VERBOSE
	cout << "Thread Manager Destroyed" << endl;
	#endif
}

int ThreadManager::wakeupThread(int threadId)
{

	return this->threadList[threadId].wakeup();


}

int ThreadManager::setupBarrier()
{

	
	for(int i = 0; i<threadList.size(); i++)
	{
		threadList[i].setBarrierPtr(&barrier);

	}
	return pthread_barrier_init(&barrier, NULL, threadList.size());


}

int ThreadManager::setupBarrier(int numberOfThreads)
{

	
	return pthread_barrier_init(&barrier, NULL, numberOfThreads);


}

void ThreadManager::setThreadBarrier(int threadId)
{

	threadList[threadId].setBarrierPtr(&barrier);
	


}
