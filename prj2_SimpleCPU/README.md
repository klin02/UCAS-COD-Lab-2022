## 代码复用
1. 复用prj1中reg_file.v  
2. 改进prj1中alu.v，并新增shifter.v   

----

## 版本说明
+ 单周期：  
simple_cpu_single为最终版本
+ 多周期：  
  1. simple_cpu_multi为简易版本，可通过验收  
  2. modify_multi为验收后改进版本，添加了ALU和Shifter输入输出的寄存器，以缩短组合逻辑链，降低延迟  