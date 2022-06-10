
#ifndef __PERF_CNT__
#define __PERF_CNT__

#ifdef __cplusplus
extern "C" {
#endif


typedef struct Result {
	int pass;
	unsigned long msec;		//cycle_cnt
	unsigned long Inst_cnt;
	unsigned long Branch_cnt;
	unsigned long Jump_cnt;
	unsigned long Prdt_go_cnt;
	unsigned long BorJ_v_cnt;
	unsigned long Cancel_cnt;
	unsigned long MEM_visit_cnt;
	unsigned long MEM_delay_cnt;
	unsigned long Valid_Inst_cnt;
	//unsigned long decode_cnt;
} Result;

void bench_prepare(Result *res);
void bench_done(Result *res);

#endif