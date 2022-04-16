#include "perf_cnt.h"

//The addr of specific perf_cnt:
	//perf_cnt_0  ->  0x60010000
	//perf_cnt_1  ->  0x60010008
	//perf_cnt_2  ->  0x60011000
	//...


//The following 10 func is side-by-side
unsigned long _uptime() {
  // TODO [COD]
  //   You can use this function to access performance counter related with time or cycle.
	unsigned long *msec_addr;
	msec_addr = (unsigned long *)0x60010000;
	return *msec_addr;
}

unsigned long _inst_freq() {
	unsigned long *inst_cnt_addr;
	inst_cnt_addr = (unsigned long *)0x60010008;
	return *inst_cnt_addr;
}

unsigned long _ex_freq() {
	unsigned long *ex_cnt_addr;
	ex_cnt_addr = (unsigned long *)0x60011000;
	return *ex_cnt_addr;
}

unsigned long _mem_freq() {
	unsigned long *mem_cnt_addr;
	mem_cnt_addr = (unsigned long *)0x60011008;
	return *mem_cnt_addr;
}

unsigned long _mem_delay_freq() {
	unsigned long *mem_delay_cnt_addr;
	mem_delay_cnt_addr = (unsigned long *)0x60012000;
	return *mem_delay_cnt_addr;
}

unsigned long _br_j_v_freq() {
	unsigned long *br_j_v_cnt_addr;
	br_j_v_cnt_addr = (unsigned long *)0x60012008;
	return *br_j_v_cnt_addr;
}

unsigned long _br_j_f_freq() {
	unsigned long *br_j_f_cnt_addr;
	br_j_f_cnt_addr = (unsigned long *)0x60013000;
	return *br_j_f_cnt_addr;
}

unsigned long _branch_v_freq() {
	unsigned long *branch_v_cnt_addr;
	branch_v_cnt_addr = (unsigned long *)0x60013008;
	return *branch_v_cnt_addr;
}

unsigned long _branch_f_freq() {
	unsigned long *branch_f_cnt_addr;
	branch_f_cnt_addr = (unsigned long *)0x60014000;
	return *branch_f_cnt_addr;
}

unsigned long _jump_freq() {
	unsigned long *jump_cnt_addr;
	jump_cnt_addr = (unsigned long *)0x60014008;
	return *jump_cnt_addr;
}



void bench_prepare(Result *res) {
  // TODO [COD]
  //   Add preprocess code, record performance counters' initial states.
  //   You can communicate between bench_prepare() and bench_done() through
  //   static variables or add additional fields in `struct Result`
  res->msec 		= _uptime();
  res->inst_cnt		= _inst_freq();
  res->ex_cnt		= _ex_freq();
  res->mem_cnt		= _mem_freq();
  res->mem_delay_cnt	= _mem_delay_freq();
  res->br_j_v_cnt	= _br_j_v_freq();
  res->br_j_f_cnt	= _br_j_f_freq();
  res->branch_v_cnt	= _branch_v_freq();
  res->branch_f_cnt	= _branch_f_freq();
  res->jump_cnt		= _jump_freq();
}

void bench_done(Result *res) {
  // TODO [COD]
  //  Add postprocess code, record performance counters' current states.
  res->msec 		= _uptime() 		- res->msec;
  res->inst_cnt		= _inst_freq()		- res->inst_cnt;
  res->ex_cnt		= _ex_freq()		- res->ex_cnt;
  res->mem_cnt		= _mem_freq()		- res->mem_cnt;
  res->mem_delay_cnt	= _mem_delay_freq()	- res->mem_delay_cnt;
  res->br_j_v_cnt	= _br_j_v_freq()	- res->br_j_v_cnt;	
  res->br_j_f_cnt	= _br_j_f_freq()	- res->br_j_f_cnt;
  res->branch_v_cnt	= _branch_v_freq()	- res->branch_v_cnt;
  res->branch_f_cnt	= _branch_f_freq()	- res->branch_f_cnt;
  res->jump_cnt		= _jump_freq()		- res->jump_cnt;
}

