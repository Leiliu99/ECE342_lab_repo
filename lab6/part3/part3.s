main:
	addi a0, zero, op_one_addr //0x0
	lw a0, a0, 0 //0x04

	addi a1, zero, op_two_addr //0x08
	lw a1, a1, 0//0x0c

	addi a2, zero, result_addr //0x10
	lw a2, a2, 0 //0x14

	addi a3, zero, fp_base_addr //0x18
	lw a3, a3, 0 //0x1c

	addi a4, zero, fp_second_addr //0x20
	lw a4, a4, 0 //0x24

	addi a5, zero, fp_status_addr //0x28
	lw a5, a5, 0 //0x2c

	addi a6, zero, fp_result_addr //0x30
	lw a6, a6, 0 //0x34

loop:
	lw a7, a0, 0 //0x38
	sw a3, a7, 0 //0x3c store op1 in 0xA000

	lw a7, a1, 0 //0x40
	sw a4, a7, 0 //0x44 store op2 in 0xA004
	
	addi a7, zero, 1 //0x48
	sw a5, a7, 0 //0x4c

	lw a7, a6, 0 //0x50
	sw a2, a7, 0 //0x54
	jal zero, loop //0x58

op_one_addr:
	.dw 0x00007ff4

op_two_addr:
	.dw 0x00007ff8

result_addr:
	.dw 0x00007ffc

fp_base_addr:
	.dw 0x0000a000

fp_second_addr:
	.dw 0x0000a004

fp_status_addr:
	.dw 0x0000a008

fp_result_addr:
	.dw 0x0000a00c
