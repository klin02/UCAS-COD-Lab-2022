#include <am.h>
#include <benchmark.h>
#include <trap.h>
#include <limits.h>
#include "perf_cnt.h"

Benchmark *current;
Setting *setting;

static char *start;

#define ARR_SIZE(a) (sizeof((a)) / sizeof((a)[0]))

// The benchmark list

#define ENTRY(_name, _sname, _s1, _s2, _desc) \
  { .prepare = bench_##_name##_prepare, \
    .run = bench_##_name##_run, \
    .validate = bench_##_name##_validate, \
    .name = _sname, \
    .desc = _desc, \
    .settings = {_s1, _s2}, },

Benchmark benchmarks[] = {
  BENCHMARK_LIST(ENTRY)
};

extern char _heap_start[];
extern char _heap_end[];
_Area _heap = {
  .start = _heap_start,
  .end = _heap_end,
};

static const char *bench_check(Benchmark *bench) {
  unsigned long freesp = (unsigned long)_heap.end - (unsigned long)_heap.start;
  if (freesp < setting->mlim) {
    return "(insufficient memory)";
  }
  return NULL;
}

void run_once(Benchmark *b, Result *res) {
  bench_reset();       // reset malloc state
  current->prepare();  // call bechmark's prepare function
  bench_prepare(res);  // clean everything, start timer
  current->run();      // run it
  bench_done(res);     // collect results
  res->pass = current->validate();
}

int main() {
  int pass = 1;

  _Static_assert(ARR_SIZE(benchmarks) > 0, "non benchmark");

  for (int i = 0; i < ARR_SIZE(benchmarks); i ++) {
    Benchmark *bench = &benchmarks[i];
    current = bench;
    setting = &bench->settings[SETTING];
    const char *msg = bench_check(bench);
    printk("[%s] %s: ", bench->name, bench->desc);
    if (msg != NULL) {
      printk("Ignored %s\n", msg);
    } else {
	unsigned long msec 			    = ULONG_MAX;
	unsigned long Inst_cnt		  = ULONG_MAX;
	unsigned long Branch_cnt		= ULONG_MAX;
	unsigned long Jump_cnt			= ULONG_MAX;
	unsigned long Prdt_go_cnt	  = ULONG_MAX;
	unsigned long BorJ_v_cnt		= ULONG_MAX;
	unsigned long Cancel_cnt		= ULONG_MAX;
	unsigned long MEM_visit_cnt	= ULONG_MAX;
	unsigned long MEM_delay_cnt	= ULONG_MAX;
	unsigned long Valid_Inst_cnt= ULONG_MAX;
	int succ = 1;
      for (int i = 0; i < REPEAT; i ++) {
        Result res;
        run_once(bench, &res);
        printk(res.pass ? "*" : "X");
        succ &= res.pass;
        if (res.msec 		        < msec) 	         msec 		      = res.msec;
	      if (res.Inst_cnt 	      < Inst_cnt) 	    Inst_cnt 	      = res.Inst_cnt;
	      if (res.Branch_cnt 		  < Branch_cnt) 	  Branch_cnt 		  = res.Branch_cnt;
	      if (res.Jump_cnt 	      < Jump_cnt) 	    Jump_cnt 	      = res.Jump_cnt;
	      if (res.Prdt_go_cnt 	  < Prdt_go_cnt)    Prdt_go_cnt	    = res.Prdt_go_cnt;
	      if (res.BorJ_v_cnt 	    < BorJ_v_cnt) 	  BorJ_v_cnt 	    = res.BorJ_v_cnt;
	      if (res.Cancel_cnt 	    < Cancel_cnt) 	  Cancel_cnt 	    = res.Cancel_cnt;
	      if (res.MEM_visit_cnt 	< MEM_visit_cnt)  MEM_visit_cnt 	= res.MEM_visit_cnt;
	      if (res.MEM_delay_cnt 	< MEM_delay_cnt)  MEM_delay_cnt 	= res.MEM_delay_cnt;
	      if (res.Valid_Inst_cnt 	< Valid_Inst_cnt) Valid_Inst_cnt	= res.Valid_Inst_cnt;
      }

      if (succ) printk(" Passed.\n");
      else printk(" Failed.\n");

      pass &= succ;

      // TODO [COD]
      //   A benchmark is finished here, you can use printk to output some information.
      //   `msec' is intended indicate the time (or cycle),
      //   you can ignore according to your performance counters semantics.
	  printk ("The num of clock : %u\n" , msec);
	  printk ("The num of obtained instruction : %u\n" , Inst_cnt);
	  printk ("The num of Branch instruction : %u\n" , Branch_cnt);
	  printk ("The num of Jump instruction : %u\n" , Jump_cnt);
	  printk ("The num of PC jump or branch by predictor : %u\n" , Prdt_go_cnt);
	  printk ("The num of successful branch or jump : %u\n" , BorJ_v_cnt);
	  printk ("The num of prediction fail : %u\n" , Cancel_cnt);
	  printk ("The num of mem visit : %u\n" , MEM_visit_cnt);
	  printk ("The num of mem delay : %u\n" , MEM_delay_cnt);
	  printk ("The num of retire valid instruction : %u\n" , Valid_Inst_cnt);

    }
  }

  printk("benchmark finished\n");

  if(pass)
	  hit_good_trap();
  else
	  nemu_assert(0);

  return 0;
}

// Library


void* bench_alloc(size_t size) {
  if ((uintptr_t)start % 16 != 0) {
    start = start + 16 - ((uintptr_t)start % 16);
  }
  char *old = start;
  start += size;
  assert((uintptr_t)_heap.start <= (uintptr_t)start && (uintptr_t)start < (uintptr_t)_heap.end);
  for (char *p = old; p != start; p ++) *p = '\0';
  assert((uintptr_t)start - (uintptr_t)_heap.start <= setting->mlim);
  return old;
}

void bench_free(void *ptr) {
}

void bench_reset() {
  start = (char*)_heap.start;
}

static int32_t seed = 1;

void bench_srand(int32_t _seed) {
  seed = _seed & 0x7fff;
}

int32_t bench_rand() {
  seed = (mmul_u(seed , (int32_t)214013L) + (int32_t)2531011L);
  return (seed >> 16) & 0x7fff;
}

// FNV hash
uint32_t checksum(void *start, void *end) {
  const int32_t x = 16777619;
  int32_t hash = 2166136261u;
  for (uint8_t *p = (uint8_t*)start; p + 4 < (uint8_t*)end; p += 4) {
    int32_t h1 = hash;
    for (int i = 0; i < 4; i ++) {
      h1 = mmul_u((h1 ^ p[i]) , x);
    }
    hash = h1;
  }
  hash += hash << 13;
  hash ^= hash >> 7;
  hash += hash << 3;
  hash ^= hash >> 17;
  hash += hash << 5;
  return hash;
}

