.data
last_tail_ptr:
        .word 0

.globl intr_handler
	.align  2
	.type intr_handler, @function
	.section .exception

intr_handler:
	# TODO: Please add your own interrupt handler for DMA engine 
 # Respond to DMA call
	la 	$k0, 0x60020000		# base address of DMA MMIO register set
	lw 	$k1, 0x14($k0) 		# get ctrl_Stat, GPR[k1] = MEM [offset + GPR[k0]] 
	li 	$k0, 0x7fffffff		# Store imm temporarily, because imm out of range of andi
	and 	$k1, $k1, $k0		# Set INTR(31bit) to 0
	la 	$k0, 0x60020000		# Recover k0, as base addr
	sw 	$k1, 0x14($k0) 		# Write back ctrl_stat
 # Mark sub buffer according to tail_ptr, last_tail_ptr and dma_size
  # count sub result between tail_ptr
	la 	$k0, 0x60020000
	lw 	$k0, 0x08($k0) 		# GPR[k0] = cur_tail_ptr
	la 	$k1, last_tail_ptr
	lw 	$k1, 0($k1) 		# GPR[k1] = last_tail_ptr
        beq	$k0, $k1, L1	        # if DMA_engine work succesfully(change tail),change stat
        nop
  # dma_buf_stat -= sub_buf_num
	la 	$k0, dma_buf_stat 
	lw 	$k1, 0($k0) 		# GPR[k0] = dma_buf_stat(origin)
	addi 	$k1, $k1, -1 		# GPR[k0] = dma_buf_stat(refreshed)
	sw 	$k1, 0($k0) 		# Write back dma_buf_stat
  # store cur_tail_ptr to last_tail_ptr for next call	
        la 	$k0, last_tail_ptr 	# k0 = addr of last_tail_ptr
	la 	$k1, 0x60020000
	lw 	$k1, 0x08($k1) 		# GPR[k1] = cur_tail_ptr
	sw 	$k1, 0($k0) 		# store cur to last
L1:        
 # Return from interrupt
	eret

	