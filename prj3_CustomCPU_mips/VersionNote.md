## 版本说明  
Hardware：  
custom_cpu为初始简易版本，可通过验收   
custom_cpu_modify为修改版本，添加了ALU和Shifter的输入输出寄存器  
mips_custom_save为最终版本，减少了PC+br_extension的加法器  
  
Software：  
printf_backup.c为初始版本  
printf.c为最终版本，节约了内层循环和移位操作  

通过全部阶段测试  