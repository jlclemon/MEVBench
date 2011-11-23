
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

#pragma once



#ifndef THREAD_MANAGER_H
#define THREAD_MANAGER_H
#include <pthread.h>
#include <vector>

using namespace std;
struct ThreadStruct
{
	void * (*threadFunction)(void *);
	void * threadParam;
	void * threadData;
	pthread_t threadId;
	int threadLogicalId;
	pthread_cond_t wakeupSignalConditionVariable;
	pthread_mutex_t wakeupLock;
	bool wakeupSet;
	pthread_barrier_t * barrierPtr;




};


class Thread
{


	public:
		Thread();
		~Thread();
		Thread(const Thread & oldThread); 
		Thread & operator=(const Thread & p);

		pthread_t getThreadId( )
		{
			return this->threadId;

		}

		void setThreadId(pthread_t newThreadId)
		{

			this->threadId = newThreadId;

		}
		pthread_t * getThreadIdPtr()
		{

			return &(this->threadId);

		}



		int getThreadLogicalId()
		{
			return this->threadLogicalId;
		}

		void setThreadLogicalId(int newThreadLogicalId)
		{
			this->threadLogicalId = newThreadLogicalId;

		}


		//Return the function pointer
		void * (*getThreadFunction())(void *) 
		{

			return threadFunction;
		}

		void setThreadFunction( void * (*newThreadFunction)(void *) )
		{
				
			threadFunction = newThreadFunction;	
				
		}


		void * getThreadParam()
		{
			return this->threadParam;

		}

		void setThreadParam(void * newThreadParam)
		{
			this->threadParam = newThreadParam;

		}


		void setBarrierPtr(pthread_barrier_t * newBarrierPtr)
		{

			this->barrierPtr = newBarrierPtr;

		}

		pthread_barrier_t * getBarrierPtr()
		{

			return this->barrierPtr;

		}

		int waitAtBarrier();
		int wakeup();

		int waitForWakeup();


protected:
		void * (*threadFunction)(void *);
		void * threadParam;
		void * threadData;
		pthread_t threadId;
		int threadLogicalId;
		pthread_cond_t wakeupSignalConditionVariable;
		pthread_mutex_t wakeupLock;
		bool wakeupSet;
		pthread_barrier_t * barrierPtr;

};

class ThreadManager
{
public:
	ThreadManager(void);
	ThreadManager(int numThreads);
	~ThreadManager(void);
	Thread * getThread(int index)
	{
		return &threadList[index];

	}
	int addThread(Thread & newThread)
	{
		this->threadList.push_back(newThread);
		return this->threadList.size();

	}
	int removeThread(int index)
	{
		this->threadList.erase(this->threadList.begin()+index);		
		return	this->threadList.size();
	}



	int getNumberOfThreads()
	{
			
		return this->threadList.size();		
			
	};

	int wakeupThread(int threadId);
	int setupBarrier();
	int setupBarrier(int numberOfThreads);
	void setThreadBarrier(int threadId);
protected:
	vector<Thread> threadList;
	pthread_barrier_t barrier;

};



#endif
