#include "dma.h"
#include "printf.h"
#include "trap.h"

void dma_setup()
{
	unsigned int reg_val;

	//set base address of source and destination buffer respectively
	*(dma_mmio + (DMA_SRC_BASE >> 2)) = (unsigned int)src_buf;	//ctrl[0]
	*(dma_mmio + (DMA_DEST_BASE >> 2)) = (unsigned int)dest_buf; 	//ctrl[1]
	//右移2，除以4,因为内存中按4字节对齐通过该操作得到目的和源缓冲区地址
	//基地址不变，指针加1将偏移4个字节；
	
	//set size (number of bytes) of DMA transferring
	*(dma_mmio + (DMA_SIZE_REG >> 2)) = DMA_SIZE;			//ctrl[4]

	//clear DMA work queue
	*(dma_mmio + (DMA_TAIL_PTR >> 2)) = 0;				//ctrl[2]
	*(dma_mmio + (DMA_HEAD_PTR >> 2)) = 0;				//ctrl[3]
	
	//enable DMA engine
	reg_val = *(dma_mmio + (DMA_CTRL_STAT >> 2));			//ctrl[5]
	reg_val |= DMA_EN;
	*(dma_mmio + (DMA_CTRL_STAT >> 2)) = reg_val;			
	//将ctrl寄存器的值取出来改变
}

void generate_data(unsigned int *buf)
{
	unsigned int *addr = buf;
	
	for(int i = 0; i < (DMA_SIZE / sizeof(int)); i++)
		*(addr + i) = i;
	//对一个buffer中的子缓冲区生成数据
}

void memcpy()
{
	unsigned int *src = (unsigned int *)src_buf;
	unsigned int *dest = (unsigned int *)dest_buf;

	for(int i = 0; i < (BUF_SIZE / sizeof(int)); i++)
		*dest++ = *src++;
	//数据搬移，和DMA互为替代
}

void setup_buf()
{
	volatile extern int dma_buf_stat;
	//dma未处理的子缓冲区数量 

	int sub_buf_num = (BUF_SIZE / DMA_SIZE);
	//子缓冲区数量

	unsigned char *buf = src_buf;

#ifdef USE_DMA
	unsigned int reg_val;
#endif
	printf("Bp1:setup_buf_begin\n");
	dma_buf_stat = 0;
	
	for(int i = 0; i < sub_buf_num; i++)
	{
		generate_data((unsigned int *)buf);
		//生成子缓冲区的数据，即处理器填充子缓冲区 
		printf("Bp2:Loop generate done: i=%d\n",i);
		//move buffer pointer to next sub region
		buf += DMA_SIZE;
		//移动buf指针到下一个子缓冲区

		dma_buf_stat++;
		//DMA未处理的子缓冲区数量增加

#ifdef USE_DMA
		//refresh head ptr in DMA engine
		reg_val = *(dma_mmio + (DMA_HEAD_PTR >> 2));
		reg_val += DMA_SIZE;
		*(dma_mmio + (DMA_HEAD_PTR >> 2)) = reg_val;
		printf("Bp3:Loop Dma change: i=%d\n",i);
		//更新DMA可见的头指针位置
#endif
	}
	printf("Bp4:Loop end\n");
#ifdef USE_DMA
	//waiting for all sub-region are processed by DMA engine
	while(dma_buf_stat);
	//等待数据搬移结束,因此intr_handler的“标记”只需处理dma_buf_stat
	printf("Bp5:Dma done\n");
#else
	memcpy();
	printf("Bp6:memcpy done\n");
#endif
}

int main()
{
	printf("Bp0: main begin\n");
#ifdef USE_DMA
	printf("Prepare DMA engine\n");
	
	//setup DMA engine
	dma_setup();
#endif

	//start buffer writing
	setup_buf();

	printf("Data mover done\n");
	return 0;
}

