#include "stdint.h"

#ifndef KERNEL_H
#define KERNEL_H

typedef struct 
{
    void *sp;//stack pointer to the task stack pointer for the thread
    uint32_t timeout;//timeout delay down-counter
    uint8_t prio;//priority orentation is the higher the number the higher the task priority 

} OSThread;

typedef void (*OSThreadHandler) ();

void OSThread_Init ( OSThread *task, uint8_t prio, OSThreadHandler thread_Handler, void *stkSto, uint32_t stack_size);
void OS_startUP(void *stkSto, uint32_t stack_size);
void OS_sched(void);

void OS_onIdle(void);
void OS_run (void );
void OS_delay(uint32_t ticks);
void OS_tick(void);

#endif 
