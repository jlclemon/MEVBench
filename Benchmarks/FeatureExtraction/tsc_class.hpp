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

// TSC CLASS INTERFACE FOR C++
//BY: ANDREW JONES

#ifndef TSC_HPP_
#define TSC_HPP_

#include <cstdlib>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>

#include "rdtsc.h"

#define __forceinline inline __attribute__((always_inline))

using namespace std;

class TSC_VAL_w {
	//THIS class provides a wrapper for the C level union -- all normal operators are included
public:
	TSC_VAL_w( uint64_t i ) { my_tsc_measure.tsc_full = i; } //ctor with default value
	TSC_VAL_w( time_t i ) { my_tsc_measure.tsc_full = i; }
	TSC_VAL_w( double d ) { my_tsc_measure.tsc_full = (uint64_t)(d*1000000000.0); } //ctor with default value
	TSC_VAL_w() { my_tsc_measure.tsc_full = 0; } //ctor withOUT default value

	//Operators
	operator const TSC_VAL& () const { return (const TSC_VAL&)(this->my_tsc_measure); }
	operator const TSC_VAL () const { return (const TSC_VAL)(this->my_tsc_measure); }
	operator TSC_VAL& () { return this->my_tsc_measure; }
	operator TSC_VAL () { return this->my_tsc_measure; }
	operator TSC_VAL () const { return this->my_tsc_measure; }
	operator TSC_VAL* () { return &(this->my_tsc_measure); }
	operator const TSC_VAL* () const { return &(this->my_tsc_measure); }
	operator TSC_VAL_w*(){ return static_cast<TSC_VAL_w*>(this); }
	operator const TSC_VAL_w(){ return static_cast<const TSC_VAL_w>(*this); }
	operator const TSC_VAL_w&() const { return static_cast<const TSC_VAL_w&>(*this); }
	operator double() { return static_cast<double>(this->my_tsc_measure.tsc_full); }
	operator const double() const { return static_cast<const double>(this->my_tsc_measure.tsc_full); }

	//Assignment/ref ops
	const TSC_VAL_w* operator&() const { return (this); }
	TSC_VAL_w* operator&() { return (this); }
	TSC_VAL_w& operator=(const TSC_VAL_w &rhs) {
		if (this == &rhs) return *this;
		this-> my_tsc_measure.tsc_full = rhs.my_tsc_measure.tsc_full; return *this;}

	//ALU operators
	const TSC_VAL_w operator+(const TSC_VAL_w& rv) const { return rv.my_tsc_measure.tsc_full + my_tsc_measure.tsc_full; }
	const TSC_VAL_w operator-(const TSC_VAL_w& rv) const { return my_tsc_measure.tsc_full - rv.my_tsc_measure.tsc_full; }
	TSC_VAL_w& operator+=(const TSC_VAL_w& rv) { //NOTE TO SELF: putting () const { ... means no member can be written
		my_tsc_measure.tsc_full = rv.my_tsc_measure.tsc_full + my_tsc_measure.tsc_full;
		return (TSC_VAL_w&)(*this); }
	TSC_VAL_w& operator-=(const TSC_VAL_w& rv) { //NOTE TO SELF: putting () const { ... means no member can be written
		my_tsc_measure.tsc_full = my_tsc_measure.tsc_full - rv.my_tsc_measure.tsc_full;
		return (TSC_VAL_w&)(*this); }
	const TSC_VAL_w operator*(const TSC_VAL_w& rv) const { return rv.my_tsc_measure.tsc_full * my_tsc_measure.tsc_full; }
	const double operator*(double rv) const { return (double)(rv * (double)my_tsc_measure.tsc_full); }

	//bool ops
	bool operator==(const TSC_VAL_w& other) const { return ( other.my_tsc_measure.tsc_full == my_tsc_measure.tsc_full ); }
	bool operator!=(const TSC_VAL_w& other) const { return ( other.my_tsc_measure.tsc_full != my_tsc_measure.tsc_full ); }
	bool operator>=(const TSC_VAL_w& other) const { return ( other.my_tsc_measure.tsc_full <= my_tsc_measure.tsc_full ); }
	bool operator<=(const TSC_VAL_w& other) const { return ( other.my_tsc_measure.tsc_full >= my_tsc_measure.tsc_full ); }
	bool operator>(const TSC_VAL_w& other) const { return !(*this <= other); }
	bool operator<(const TSC_VAL_w& other) const { return !(*this >= other); }

//private:
	TSC_VAL my_tsc_measure;
};
__forceinline ostream &operator<<( ostream &os, const TSC_VAL &L ) { os << L.tsc_full;  return os;  }
__forceinline ostream &operator<<( ostream &os, const TSC_VAL_w &L ) {return os << (L.my_tsc_measure.tsc_full); } // goes with TSC_VAL_w class


//-----------------------MACRO INTERFACE-------------------
#define READ_TIMESTAMP_WITH_WRAPPER( wrapper ) READ_TIMESTAMP(wrapper.my_tsc_measure) 

#endif /* TSC_HPP_ */
