## 代码复用
#### hardware
复用prj2中ALU、Reg_file和Shifter    
文件夹对应不同cache容量的修改，axb表示a路组，每路组中有b个数据块     
修改CACHE容量：只需按照文件开头注释修改即可   

处理器部分可使用如下：  
mips：custom_CPU   
rv32：custom_CPU turbo(pipeline)  

#### software
复用prj3中bench.c perf_cnt.h perf_cnt.c printf.c

#### framework
backup文件夹中data_array.v tag_array.v为框架代码,不做改动，在cache内部实现相应功能   
为方便进行cache容量，在其余以数字命名的文件夹中，均例化了darray和tarray  

#### performance  
8路组情况下只支持每路组16个cache块，4路组情形可支持每路组128个cache块  
无cache流水线在300hz下bit-gen中WNS均大于0，加上4x128的i/dcache后在280hz可保证bit-gen中WNS均大于0