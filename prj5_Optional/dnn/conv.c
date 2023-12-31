#include "printf.h"
#include "trap.h"
#include "mul.h"
#include "div.h"
#include "perf_cnt.h"
#define FRAC_BIT 10

#define RD_ADDR 135106448
#define RD_SIZE_D0 1
#define RD_SIZE_D1 1
#define RD_SIZE_D2 28
#define RD_SIZE_D3 28

#define WEIGHT_ADDR 134217728
#define WEIGHT_SIZE_D0 20
#define WEIGHT_SIZE_D1 1
#define WEIGHT_SIZE_D2 5
#define WEIGHT_SIZE_D3 5

#define WR_ADDR 135108240
#define WR_SIZE_D0 1
#define WR_SIZE_D1 20
#define WR_SIZE_D2 12
#define WR_SIZE_D3 12

#define KERN_ATTR_CONV_PAD 0
#define KERN_ATTR_CONV_STRIDE 1
#define KERN_ATTR_POOL_PAD 0
#define KERN_ATTR_POOL_KERN_SIZE 2
#define KERN_ATTR_POOL_STRIDE 2

// MMIO register address of DNN accelerator
#define GPIO_START_ADDR    0x60030000
#define GPIO_DONE_ADDR     0x60030008

struct size_vec4
{
	unsigned d0;
	unsigned d1;
	unsigned d2;
	unsigned d3;
};

struct mem_addr
{
	unsigned rd_addr;
	unsigned weight_addr;
	unsigned wr_addr;
};


int mul(short a, short b)
{
#ifndef USE_MUL
	int ans = mul_ll(a, b);
#else
	int ans = a * b;
#endif
	return ans;
}

struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR};
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3};
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3};
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3};

struct size_vec4 conv_size;

extern char _binary_data_result_bin_start[];
extern char _binary_data_result_bin_size[];

void convolution()
{
	short *in = (short *)addr.rd_addr;
	short *weight = (short *)addr.weight_addr;
	short *out = (short *)addr.wr_addr;

	// unsigned output_offset = 0;
	// unsigned input_offset = 0;

	unsigned input_fm_w = rd_size.d3;
	unsigned input_fm_h = rd_size.d2;

	unsigned pad = KERN_ATTR_CONV_PAD;
	unsigned pad_len = pad << 1;

	unsigned conv_out_w = rd_size.d3 - weight_size.d3 + pad_len;
	unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len;

	unsigned stride = KERN_ATTR_CONV_STRIDE;

	conv_out_w = div(conv_out_w, stride);
	conv_out_h = div(conv_out_h, stride);

	/* ow = (iw-kw+pad*2)/stride +1 */
	conv_out_w++;
	conv_out_h++;

	conv_size.d0 = wr_size.d0;
	conv_size.d1 = wr_size.d1;
	conv_size.d2 = conv_out_h;
	conv_size.d3 = conv_out_w;

	// TODO: Please add your implementation here
	/* Fixed value for all loop */
	short FilterSize; // Filter include bias
	short InputSize;

	/* Reduce multiplications to speed things up, store the repeated result */
	short stride_x, stride_y; // x*S, y*S
	short FilterOuterAddr;	// Addr of head of Filter Channel
	short FilterInnerAddr;	// Addr of head of Filter in the Channel
	short InputAddr;	// Addr of head of a input volume
	short LineAddrInFilter;	// Addr of head of the line in the Filter
	short LineAddrInInput;	// Addr of head of the line in the Input Volume

	/* Store the result of calculation temporarily, use int to avoid overflow */
	int temp; 

	/* Store the offset of output*/
	short offset = 0; 

	//Software will no care blank between vallue
	FilterSize = 1 + mul(weight_size.d2, weight_size.d3);
	InputSize = mul(input_fm_h, input_fm_w);	

	// y always ahead to x, because use line as main sequence
	// put ahead x,y to save times of multiplication
	for (short no = 0; no < conv_size.d1; no++)
	{ // num of output channel, each has a set of Filters
		FilterOuterAddr = mul(no, FilterSize); //ADDR of first item(bias) of filter
		for (short y = 0; y < conv_out_h; y++)
		{ // height of output graph
			stride_y = mul(y, stride); //num of raw
			for (short x = 0; x < conv_out_w; x++)
			{
				stride_x = mul(x, stride); //num of column
				temp = 0;
				out[offset] = weight[FilterOuterAddr]; // Add bias to out
				for (short ni = 0; ni < rd_size.d1; ni++)
				{
                                        FilterInnerAddr = 1; //Consider bias
					InputAddr = mul(ni, InputSize);
					for (short ky = 0; ky < weight_size.d2; ky++)
					{
						LineAddrInFilter = mul(ky, weight_size.d3); // width
						short ih = ky + stride_y - pad;
						LineAddrInInput = mul(ih, input_fm_w);//不考虑每行填充的8字节
						for (short kx = 0; kx < weight_size.d3; kx++)
						{
							short iw = kx + stride_x - pad;
							//(kx+stride_x,...)is postion considering pad
							//(iw,ih) is real position ignoring pad, consistent with memory
							// the real point of origin has position (pad,pad) considering pad
							if (iw < 0 || ih < 0 || iw >= input_fm_w || ih >= input_fm_h)
								continue; // not calculate padding
							temp += mul(in[InputAddr + LineAddrInInput + iw] , weight[FilterOuterAddr + FilterInnerAddr +  LineAddrInFilter + kx]);
						}
					}
				}
				out[offset++] += (short)(temp >> FRAC_BIT); 
				/*
				 *The temp bit is changed from 32 to 16
				 *Since two 16-digit numbers are multiplied to produce 32 digits,
				 *if the original decimal place is 10, the decimal place is 20
				 *Therefore, after moving 10 bits to the right, the lowest 15 bits are taken as the significant bits
				 *The sign bit of the original multiplication of two numbers is taken in the highest bit
				 */
			}
		}
	}
}

void pooling()
{
	short *out = (short *)addr.wr_addr;

	// unsigned output_offset = 0;
	// unsigned input_offset = 0;

	unsigned input_fm_w = conv_size.d3;
	unsigned input_fm_h = conv_size.d2;

	unsigned pad = KERN_ATTR_POOL_PAD;
	unsigned pad_len = pad << 1;

	unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE;
	unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE;

	unsigned pool_out_w = pad_w_test + pad_len;
	unsigned pool_out_h = pad_h_test + pad_len;

	unsigned stride = KERN_ATTR_POOL_STRIDE;

	unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride);
	unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride);

	pool_out_w = div(pool_out_w, stride);
	pool_out_h = div(pool_out_h, stride);
	pool_out_w++;
	pool_out_h++;

	if ((!pad) && (pad_w_test_remain || pad_h_test_remain))
	{
		pool_out_w++;
		pool_out_h++;
	}

	// TODO: Please add your implementation here
	/* Fixed value for all loop */
	short InputSize;

	/* Reduce multiplications to speed things up, store the repeated result */
	short stride_x, stride_y;
	short InputAddr;
	short LineAddrInInput;

	short offset = 0;
        //Treat Out by conv as input 
	InputSize = mul(input_fm_h, input_fm_w);

	for (short no = 0; no < conv_size.d1; no++) //Channel of output
	{
                InputAddr =mul(no, InputSize);
		for (short y = 0; y < pool_out_h; y++)
		{
			stride_y = mul(stride, y);
			for (short x = 0; x < pool_out_w; x++)
			{
				stride_x = mul(stride, x);
				//The minimum value of short type
                                short maxtmp = 0x8000; 
				for (short ky = 0; ky < KERN_ATTR_POOL_KERN_SIZE; ky++)
				{
					short ih = ky + stride_y - pad;
                                        //Address of head of line
                                        LineAddrInInput = mul(ih, input_fm_w); 
					for (short kx = 0; kx < KERN_ATTR_POOL_KERN_SIZE; kx++)
					{
						short iw = kx + stride_x - pad;
						short curtmp;
						if (iw < 0 || ih < 0 || iw >= input_fm_w || ih >= input_fm_h)
							curtmp = 0;
						else
							curtmp = *(out + InputAddr + LineAddrInInput + iw);
						if (curtmp > maxtmp)
							maxtmp = curtmp;
					}
				}
				out[offset++] = maxtmp;
			}
		}
	}
}
#ifdef USE_HW_ACCEL
void launch_hw_accel()
{
	volatile int* gpio_start = (void*)(GPIO_START_ADDR);
	volatile int* gpio_done = (void*)(GPIO_DONE_ADDR);

	//TODO: Please add your implementation here
	*gpio_start = 1;
	
	while(*(gpio_done) != 1)
                ;

        *gpio_start = 0;

}
#endif

int comparing()
{
	char *out = (char *)addr.wr_addr;
	char *result = (char *)_binary_data_result_bin_start;

#ifdef USE_HW_ACCEL
	int count = (int)_binary_data_result_bin_size + 
		    (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
	int count = (int)_binary_data_result_bin_size;
#endif

	for (int i = 0, j = 0; i < count; i++)
	{
#ifdef USE_HW_ACCEL
		int alignment = i & 0x0000001f;
		if (alignment >= (WR_SIZE_D3 << 1))
			continue;
#endif
		if (*(out + i) != *(result + j))
		{
			printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
			return 1;
		}
		j++;
	}

	printf("Passed!\n");
	return 0;
}


int main()
{
	
        Result res;
        res.msec = 0;

        bench_prepare(&res);
	
#ifdef USE_HW_ACCEL
	printf("Launching task...\n");
	launch_hw_accel();

#else
	printf("starting convolution\n");
	convolution();
	printf("starting pooling\n");
	pooling();
#endif

	int result = comparing();

	bench_done(&res);
	
        printf("Total cycle: %d\n", res.msec);

	printf("benchmark finished\n");

	if (result == 0)
	{
		hit_good_trap();
	}
	else
	{
		nemu_assert(0);
	}

	return 0;
}
