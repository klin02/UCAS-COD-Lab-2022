## 代码复用
#### hardware
复用prj2中ALU、Reg_file和Shifter  

#### software
在框架代码data_mover.c中添加了性能计数

#### framework  
除mips_dma_cpu.v, engine_core.v, intr_handler.S外均为框架代码   
为在master分支同时支持dma和dnn，将其合并为dnn_dma_mips_cpu.v