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


//READ TIME STAMP COUNTER -- for simulator
//NOTE: This file is in C format .... and should remain that way. Use tsc_class.hpp in c++
//BY: ANDREW JONES

#ifndef RDTSC_H_
#define RDTSC_H_

#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <sched.h>
#include <cstdio>
#include <errno.h>
#include <string>
#include <cstring>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#ifndef _GNU_SOURCE
	#define _GNU_SOURCE
#endif
#include <sys/types.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <algorithm>
#include <sstream>
//#include "shared.h"

//#include "linux_system_info.h"

using namespace std;

//----------------CONTROL DEFINES------------------------
#define FORCE_RDTSC_INORDER

//IMPORTANT: to use auto-calibrate, this must point at python script: find_in_file_r.py

//------------------PROTOTYPES:---------------------------
int cmp_TSC_VAL(const void * elem1, const void * elem2);
static int cmp_TSC_VAL2(const void * elem1, const void * elem2);
inline void linux_auto_calib_tsc();

//--------------------DATA STRUCTS-----------------------
typedef union{       
	uint64_t tsc_full;
	struct {uint32_t lo, hi;} tsc_half;
} TSC_VAL;

//-----------------GLOBALS------------------------------
static TSC_VAL MIN_TIME_BW_RDTSCS;
//this is available to anything that wants to know the systems min time between rdtsc calls
//NOTE: must run AUTO_CALIBRATE_RDTSC() before you use it if you want a good value

//----------------PRIVATE INTERFACE----------------------
#define CPUID() __asm__ __volatile__ ( "cpuid" ::: "%rax", "%rbx", "%rcx", "%rdx")

#ifdef FORCE_RDTSC_INORDER
#define rdtsc_hl( lo, hi ) \
	CPUID(); \
	__asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
#else
#define rdtsc_hl( lo, hi ) \
	__asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
#endif

//--------------PUBLIC INTERFACE------------------------
#define READ_TIMESTAMP(full_TSC_VAL) \
	rdtsc_hl( full_TSC_VAL.tsc_half.lo, full_TSC_VAL.tsc_half.hi )

#define TIMESTAMP_TO_SECONDS(timestamp_val, PHYS_PROC_CLK_PERIOD) ((double)timestamp_val.tsc_full * PHYS_PROC_CLK_PERIOD);

#ifdef __WIN32
#define AUTO_CALIBRATE_RDTSC() assert(0) 
//FIXME: NOT IMPLIMENTED
#else
//#define AUTO_CALIBRATE_RDTSC() linux_auto_calib_tsc()
//BEST CALLED AT THE BEGINNING OF MAIN - automatically figures out your machines rdtsc timing - NO MSR used (portable)
#endif


//----------OPTIONAL AUTO-CALIBRATION -----------------------
#define RDTSC_CALIB_RUN_SIZE 100000

static int cmp_TSC_VAL2(const void * elem1, const void * elem2){
//The return value of this function should represent whether elem1 is considered less than, equal to, or greater than elem2 by returning, respectively, a negative value, zero or a positive value.
	return ( (*(TSC_VAL*)elem1).tsc_full - (*(TSC_VAL*)elem2).tsc_full );
}

/*inline void linux_auto_calib_tsc(){
	ofstream myfile;
 	myfile.open ("rdtsc_results.csv");

	TSC_VAL tsc_calib_array[RDTSC_CALIB_RUN_SIZE];//global tsc array
	TSC_VAL beg, end;
//	long int x, y;
	for(int i = 0 ; i < RDTSC_CALIB_RUN_SIZE ; i++){ 
		CPUID(); 
		READ_TIMESTAMP( beg );
		//x=y+x; //AUTO CALIB WITHOUT THIS
		READ_TIMESTAMP( end );
		tsc_calib_array[i].tsc_full = end.tsc_full - beg.tsc_full;
	}

	//CHECK FORCED CTXT S/W
	int nv_ctxt_sw = get_my_nonvoluntary_ctxt_switches();
	cout<<"\nnv_ctxt_sw="<<nv_ctxt_sw<<"\n";
	myfile<<"\nnv_ctxt_sw="<<nv_ctxt_sw<<"\n";

	//SORT THE ARRAY
	qsort( tsc_calib_array, RDTSC_CALIB_RUN_SIZE, sizeof(TSC_VAL), cmp_TSC_VAL2 ); //should sort low...high
	//for(int i = 0 ; i < RDTSC_CALIB_RUN_SIZE ; i++){
	//	cout<<tsc_calib_array[i].tsc_full<<"\t";
	//} //DEBUG PURPOSES

	//REMOVE THE BAD (CTXT SWITCHED VALUES):
	unsigned long int effective_array_size = RDTSC_CALIB_RUN_SIZE - nv_ctxt_sw;

	//STATISTICAL ANALYSIS
	TSC_VAL max, min, sum, avg;
	max.tsc_full = tsc_calib_array[effective_array_size-1].tsc_full;
	min.tsc_full = tsc_calib_array[0].tsc_full;
	sum.tsc_full = 0;
	avg.tsc_full = 0;
	for(unsigned int i = 0 ; i < effective_array_size ; i++){
		//log to file
		myfile<<tsc_calib_array[i].tsc_full<<"\n";

		sum.tsc_full += tsc_calib_array[i].tsc_full;

		//if( tsc_calib_array[i].tsc_full < min.tsc_full ) min.tsc_full = tsc_calib_array[i].tsc_full;
		//else if( tsc_calib_array[i].tsc_full > max.tsc_full ) max.tsc_full = tsc_calib_array[i].tsc_full;
	}
	avg.tsc_full = sum.tsc_full / effective_array_size;

	cout<<"\nmin="<<min.tsc_full<<"\tmax="<<max.tsc_full<<"\tavg="<<avg.tsc_full<<"\n";
	myfile<<"\nmin="<<min.tsc_full<<"\tmax="<<max.tsc_full<<"\tavg="<<avg.tsc_full<<"\n";

	//Save to global	
	//MIN_TIME_BW_RDTSCS.tsc_full = 0;
	MIN_TIME_BW_RDTSCS.tsc_full = min.tsc_full;

	//REMOVED C FILE CALL STUFF -- deprecated

	myfile.close();
	
	CPUID();
}

*/

#endif

