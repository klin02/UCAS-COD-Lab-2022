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

unsigned long _Inst_freq() {
	unsigned long *Inst_cnt_addr;
	Inst_cnt_addr = (unsigned long *)0x60010008;
	return *Inst_cnt_addr;
}

unsigned long _Branch_freq() {
	unsigned long *Branch_cnt_addr;
	Branch_cnt_addr = (unsigned long *)0x60011000;
	return *Branch_cnt_addr;
}

unsigned long _Jump_freq() {
	unsigned long *Jump_cnt_addr;
	Jump_cnt_addr = (unsigned long *)0x60011008;
	return *Jump_cnt_addr;
}

unsigned long _Prdt_go_freq() {
	unsigned long *Prdt_go_cnt_addr;
	Prdt_go_cnt_addr = (unsigned long *)0x60012000;
	return *Prdt_go_cnt_addr;
}

unsigned long _BorJ_v_freq() {
	unsigned long *BorJ_v_cnt_addr;
	BorJ_v_cnt_addr = (unsigned long *)0x60012008;
	return *BorJ_v_cnt_addr;
}

unsigned long _Cancel_freq() {
	unsigned long *Cancel_cnt_addr;
	Cancel_cnt_addr = (unsigned long *)0x60013000;
	return *Cancel_cnt_addr;
}

unsigned long _MEM_visit_freq() {
	unsigned long *MEM_visit_cnt_addr;
	MEM_visit_cnt_addr = (unsigned long *)0x60013008;
	return *MEM_visit_cnt_addr;
}

unsigned long _MEM_delay_freq() {
	unsigned long *MEM_delay_cnt_addr;
	MEM_delay_cnt_addr = (unsigned long *)0x60014000;
	return *MEM_delay_cnt_addr;
}

unsigned long _Valid_Inst_freq() {
	unsigned long *Valid_Inst_cnt_addr;
	Valid_Inst_cnt_addr = (unsigned long *)0x60014008;
	return *Valid_Inst_cnt_addr;
}



void bench_prepare(Result *res) {
  // TODO [COD]
  //   Add preprocess code, record performance counters' initial states.
  //   You can communicate between bench_prepare() and bench_done() through
  //   static variables or add additional fields in `struct Result`
  res->msec 		= _uptime();
  res->Inst_cnt		= _Inst_freq();
  res->Branch_cnt	= _Branch_freq();
  res->Jump_cnt		= _Jump_freq();
  res->Prdt_go_cnt	= _Prdt_go_freq();
  res->BorJ_v_cnt	= _BorJ_v_freq();
  res->Cancel_cnt	= _Cancel_freq();
  res->MEM_visit_cnt	= _MEM_visit_freq();
  res->MEM_delay_cnt	= _MEM_delay_freq();
  res->Valid_Inst_cnt	= _Valid_Inst_freq();
}

void bench_done(Result *res) {
  // TODO [COD]
  //  Add postprocess code, record performance counters' current states.
  res->msec 		= _uptime() 		- res->msec;
  res->Inst_cnt		= _Inst_freq()		- res->Inst_cnt;
  res->Branch_cnt	= _Branch_freq()	- res->Branch_cnt;
  res->Jump_cnt		= _Jump_freq()		- res->Jump_cnt;
  res->Prdt_go_cnt	= _Prdt_go_freq()	- res->Prdt_go_cnt;
  res->BorJ_v_cnt	= _BorJ_v_freq()	- res->BorJ_v_cnt;	
  res->Cancel_cnt	= _Cancel_freq()	- res->Cancel_cnt;
  res->MEM_visit_cnt	= _MEM_visit_freq()	- res->MEM_visit_cnt;
  res->MEM_delay_cnt	= _MEM_delay_freq()	- res->MEM_delay_cnt;
  res->Valid_Inst_cnt	= _Valid_Inst_freq()	- res->Valid_Inst_cnt;
}

