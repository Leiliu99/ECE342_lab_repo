main:
	addi t0, zero, 0 // loop counter
	addi a0, zero, 0b1111 // loop max
	addi s2, zero, 0xff00 // addr
loop:
	bge t0, a0, end // if done
	lw t2, s2, 0x0 // lw dest, addr, offset
	andi t2, t2, 0xFF
	sw s2, t2, 0x0 // sw addr, data, offset
	addi t0, t0, 4
	jal, zero, loop // jump to top of loop
end:	jal, zero,end // idle here