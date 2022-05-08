## 代码复用  
复用prj2中ALU、Shifter和Reg_file  

---

## 版本说明  
### Hardware：  
1. custom_cpu为初始简易版本，可通过验收   
2. custom_cpu_modify为修改版本，添加了ALU和Shifter的输入输出寄存器  
3. mips_custom_save为modify版本减少了PC+br_extension的加法器  
4. mips_custom_fianl为save版本将三目运算层次大于2的替换为与非门以降低延迟，同时更改后缀_reg为_tmp以避免歧义
  
### Software：  
1. printf_backup.c为初始版本  
2. printf.c为最终版本，节约了内层循环和移位操作  

通过全部阶段测试  