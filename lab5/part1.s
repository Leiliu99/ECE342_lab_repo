main:
	addi a0, zero, ledr_addr
	lw a0, a0, 0
	addi a1, zero, sw_addr
	lw a1, a1, 0

loop:
	lw a2, a1, 0
	sw a0, a2, 0
	jal zero, loop

end:	jal, zero,end // idle here

sw_addr: 
	.dw 0x0000A010

ledr_addr: 
	.dw 0x0000A000
