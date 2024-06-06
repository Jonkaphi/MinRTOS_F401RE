#include "stdint.h"
#include "kernel.h"
#include "cmsis_gcc.h"
#include "main.h"
#include "stm32f4xx_hal.h"



#define MAX_THREADS 32

OSThread* volatile OS_curr;//pointer to current thread
OSThread* volatile OS_next;//pointer to next thread

OSThread* OS_threads[32+1];//thread[0] is always associated with the idle thread

uint32_t OS_readySet;// bitmask contains the status for each thread, bit 0 coresponds to thread [1], bit n coresponds to thread [n+1]
uint32_t OS_delayedSet;//bitmask of all the delayed threads

#define LOG2(x) (32 - __CLZ(x))

void OSThread_Init ( OSThread *task_pointer, uint8_t prio, OSThreadHandler thread_Handler, void *stkSto, uint32_t stack_size){
  __disable_irq();
    
if((prio < MAX_THREADS) && (OS_threads[prio] == (OSThread *)0)){

    uint32_t *sp = (uint32_t *)((((uint32_t) stkSto + stack_size)/8)*8); //rounds down the poinmter to be byte orientated

    uint32_t *stk_limit;


    *(--sp) = (1U<<24); //xPSR
  *(--sp) = (uint32_t)thread_Handler;//The U macro jus makes sure the value is unsigned
  *(--sp) = 0x0000000EU; // LR
  *(--sp) = 0x0000000CU; // R12
  *(--sp) = 0x00000003U; // R3
  *(--sp) = 0x00000002U; // R2
  *(--sp) = 0x00000001U; // R1
  *(--sp) = 0x00000000U; // R0
  // Additionally, fake registers for R4-R11s
  *(--sp) = 0x0000000BU; // R11
  *(--sp) = 0x0000000AU; // R10
  *(--sp) = 0x00000009U; // R9
  *(--sp) = 0x00000008U; // R8
  *(--sp) = 0x00000007U; // R7
  *(--sp) = 0x00000006U; // R6
  *(--sp) = 0x00000005U; // R5
  *(--sp) = 0x00000004U; // R4

    task_pointer->sp = sp;//saves the top of the stack to the stack pointer struct for the thread

    stk_limit = (uint32_t *)(((((uint32_t)stkSto - 1U)/8)+1U)* 8);//  

    for (sp= sp - 1U; sp >= stk_limit; --sp)// populates the remaining stack to see the stack size
    {
        *sp = 0xDEADBEEFU;
    }

	
    OS_threads[prio] = task_pointer;//stores the newly initilised thread stack pointer
	task_pointer->prio = prio;
	if(prio > 0U){
		OS_readySet |= (1U << (prio - 1U));//set the ready set bit for the thread to on to inidcate it is ready for the context switch
	}


}
	__enable_irq();
}

//callback to handle idle condition
void OS_onIdle(void){//in this state this is thge place for a low power sleep mode
	__WFI();

}

uint32_t stack_idle_thread[40];
OSThread idle_sp;

void idle_thread (void){
  while (1)
  {
    OS_onIdle();
  }
}

void OS_sched(void){
   

	if(OS_readySet == 0U){
		OS_next = OS_threads[0];
	}else{
		OS_next = OS_threads[LOG2(OS_readySet)];
	}
	
    //OS swicthing logic

   if(OS_next != (OSThread *)0){
    if (OS_curr != OS_next){
        *(uint32_t  volatile*)0xE000ED04 = (1U << 28);// manualy setting PendSV bit in SYSPRI3 register, as a result calling PendSV handler
    }
} 

}
void OS_startUP(void *stkSto, uint32_t stack_size){

	OSThread_Init(&idle_sp, 0U ,&idle_thread, stkSto, stack_size );

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
  	HAL_InitTick(TICK_INT_PRIORITY);

  /* Init the low level hardware */
 	 HAL_MspInit();
}

void OS_run (void ){
	

	//setsup the interrupts for PendSV and the Systick handler
	__disable_irq();
	OS_sched();//forces a context switch as to start the first thread
	__enable_irq();
	//next line should not be executed since pendsv will trigger the contex switch

	Error_Handler();

}

void OS_tick(void){
	
	uint32_t workingset = OS_delayedSet;
	
	while (workingset != 0U){//goes thorugh all treads untill all delayed threads a ticked 
		OSThread *t = OS_threads[LOG2(workingset)];
		uint32_t bit;
		if((t != (OSThread *)0) && (t->timeout != 0U)){
			bit = (1U <<(t->prio-1U));
			--t->timeout;
			if (t->timeout == 0U){
				OS_readySet |= bit;
				OS_delayedSet &= ~bit;
			}
		}
		workingset &= ~(1U<<(t->prio-1U));//mask to indicate the the thread has been processed
	}

}

void OS_delay(uint32_t ticks){
	uint32_t bit;
	__disable_irq();

	if(OS_curr != OS_threads[0]){
	OS_curr->timeout = ticks;//might be wrong
	bit = (1U<<(OS_curr->prio - 1U));
	OS_readySet &= ~bit;//idicates the the current thread is not ready and is in a delay state
	OS_delayedSet |= bit;
	OS_sched();
	}
	__enable_irq();
}





    
__attribute__ ((naked))
void PendSV_Handler(void){
	//     08000aae:   cpsid   i
	// 08000ab0:   nop
	// 60                if(OS_curr != (OSThread*)0){
	// 08000ab2:   ldr     r3, [pc, #48]   @ (0x8000ae4 <PendSV_Handler+60>)
	// 08000ab4:   ldr     r3, [r3, #0]
	// 08000ab6:   cmp     r3, #0
	// 08000ab8:   beq.n   0x8000ac2 <PendSV_Handler+26>
	// 62                    OS_curr->sp = sp;
	// 08000aba:   ldr     r3, [pc, #40]   @ (0x8000ae4 <PendSV_Handler+60>)
	// 08000abc:   ldr     r3, [r3, #0]
	// 08000abe:   ldr     r2, [r7, #4]
	// 08000ac0:   str     r2, [r3, #0]
	// 64                sp = OS_next->sp;
	// 08000ac2:   ldr     r3, [pc, #36]   @ (0x8000ae8 <PendSV_Handler+64>)
	// 08000ac4:   ldr     r3, [r3, #0]
	// 08000ac6:   ldr     r3, [r3, #0]
	// 08000ac8:   str     r3, [r7, #4]
	// 65                OS_curr = OS_next;
	// 08000aca:   ldr     r3, [pc, #28]   @ (0x8000ae8 <PendSV_Handler+64>)
	// 08000acc:   ldr     r3, [r3, #0]
	// 08000ace:   ldr     r2, [pc, #20]   @ (0x8000ae4 <PendSV_Handler+60>)
	// 08000ad0:   str     r3, [r2, #0]
	//  951        __ASM volatile ("cpsie i" : : : "memory");
	// 08000ad2:   cpsie   i
	//  952      }

	// when done blanket the whole in a single __asm volatile

__asm volatile ("cpsid   i	\n"  );



	             //if(OS_curr != (OSThread*)0){
	//push registers r4-r11 on to stack
		__asm volatile ("ldr     r3, =OS_curr \n"); //;0x8000ae4
		__asm volatile ("ldr     r3, [r3, #0] \n"  );
		__asm volatile ("cbz     r3, PendSV_restore \n"  );
	//push registers r4-r11 on to stack
		__asm volatile ("push {r4-r11}	\n"  );
	           //  OS_curr->sp = sp;
		__asm volatile ("ldr     r3, =OS_curr \n"  );
		__asm volatile ("ldr     r3, [r3, #0]	\n"  );

		__asm volatile ("str     sp, [r3, #0]	\n"  );
		//stores stack pointer to OS_curr

                 // sp = OS_next->sp;
		__asm volatile ("PendSV_restore:         \n"  );
		__asm volatile ("ldr     r3, =OS_next  \n"  );
		__asm volatile ("ldr     r3, [r3, #0]	\n"  );
		__asm volatile ("ldr     sp, [r3, #0]	\n"  );


	             //  OS_curr = OS_next;
		__asm volatile ("ldr     r3, =OS_next 	\n"  );
		__asm volatile ("ldr     r3, [r3, #0]	\n"  );
		__asm volatile ("ldr     r2, =OS_curr	\n"  );
		__asm volatile ("str     r3, [r2, #0]	\n"  );
		__asm volatile ("pop {r4-r11}	\n");
							 //__enable_irq();
		__asm volatile ("cpsie   i	\n"  );
		__asm volatile ("bx 	lr	\n"  );


}
    
