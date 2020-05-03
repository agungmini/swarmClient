#include <man_fpu_stm32f4.h>

void FPU_Init(){
	SCB->CPACR|= (3UL<<20U)|(3UL<<22);
}
