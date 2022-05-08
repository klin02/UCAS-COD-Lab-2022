## 代码复用
#### hardware
复用prj2中ALU、Reg_file和Shifter

#### software
复用prj3中bench.c perf_cnt.h perf_cnt.c printf.c

----

## 版本说明  
1. riscv_cpu为初始版本  
2. riscv_cpu_nodelay为去除延迟槽版本，及不提前做PC+4操作，只更新一次  
3. riscv_cpu_modify为初始版本添加alu和shifter寄存器   
4. riscv_cpu_save为modify版本节约PC+B_imm加法器版本  
5. riscv_cpu_final为save版本将三目运算层次大于2的替换为与非门以降低延迟，同时更改后缀_reg为_tmp以避免歧义