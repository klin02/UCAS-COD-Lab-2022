
#ifndef __PERF_CNT__
#define __PERF_CNT__

#ifdef __cplusplus
extern "C" {
#endif


typedef struct Result {
	int pass;
	unsigned long msec;		//cycle_cnt
	unsigned long inst_cnt;
	unsigned long ex_cnt;
	unsigned long mem_cnt;
	unsigned long mem_delay_cnt;
	unsigned long br_j_v_cnt;
	unsigned long br_j_f_cnt;
	unsigned long branch_v_cnt;
	unsigned long branch_f_cnt;
	unsigned long jump_cnt;
} Result;

void bench_prepare(Result *res);
void bench_done(Result *res);

#endif