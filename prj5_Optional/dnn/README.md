## 代码复用
#### hardware
复用prj2中ALU、Reg_file和Shifter  

#### software
复用prj3中perf_cnt.h perf_cnt.c printf.c  

----

## 版本说明   
dnn_cpu_simple.v基于prj3中modify版本修改，且MUL直接用*实现  
dnn_cpu_booth.v支持booth乘法，需要例化mul.v文件  
为在master分支同时支持dma和dnn，将其合并为dnn_dma_mips_cpu.v