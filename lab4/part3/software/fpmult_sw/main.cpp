

#include "system.h"

#define MEMORY_BASE (ONCHIP_MEMORY2_0_BASE + 0x3000)

int main()
{
volatile float* memory_op1 = (float*) MEMORY_BASE;
volatile float* memory_op2 = (float*) (MEMORY_BASE + 4);
volatile float* memory_out = (float*) (MEMORY_BASE + 8);
volatile float* memory_out2 = (float*) (MEMORY_BASE + 12);
volatile int* memory_start = (int*) (MEMORY_BASE + 16);
volatile float* acc_op1 = (float*) AVS_FP_MULT_0_BASE;
volatile float* acc_op2 = (float*) (AVS_FP_MULT_0_BASE + 4);
volatile float* acc_out = (float*) (AVS_FP_MULT_0_BASE + 12);
volatile int* acc_start = (int*) (AVS_FP_MULT_0_BASE + 8);

while (true) {
*memory_start = 1;
*memory_out = (*memory_op1) * (*memory_op2);
*acc_op1 = (*memory_op1);
*acc_op2 = (*memory_op2);
  *acc_start = 1;
  *memory_out2 = *acc_out;
}
  return 0;
}
