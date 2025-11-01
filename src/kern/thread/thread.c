/*
 * Copyright (c) 2022 
 * Computer Science and Engineering, University of Dhaka
 * Credit: CSE Batch 25 (starter) and Prof. Mosaddek Tushar
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <types.h>
#include <schedule.h>
#include <stm32f446xx.h>
#include <errno.h>
#include <cm4.h>

/* Task control blocks array */
TCB_TypeDef task_list[MAX_TASKS];

/* Current running task */
TCB_TypeDef *current_task = NULL;

/* Number of active tasks */
uint32_t num_tasks = 0;

/* Next task ID counter */
uint32_t next_task_id = 1000;

/* Task stack memory - statically allocated */
uint32_t task_stacks[MAX_TASKS][TASK_STACK_SIZE / 4] __attribute__((aligned(8)));

/*
 * Initialize the scheduler
 */
void scheduler_init(void)
{
    num_tasks = 0;
    next_task_id = 1000;
    current_task = NULL;
    
    /* Initialize all TCBs */
    for (int i = 0; i < MAX_TASKS; i++) {
        task_list[i].magic_number = 0;
        task_list[i].task_id = 0;
        task_list[i].psp = NULL;
        task_list[i].status = TASK_TERMINATED;
        task_list[i].priority = 0;
        task_list[i].parent_id = 0;
        task_list[i].execution_time = 0;
        task_list[i].waiting_time = 0;
        task_list[i].digital_signature = 0;
        task_list[i].heap_mem_start = NULL;
        task_list[i].heap_mem_size = 0;
        task_list[i].sem_waiting_count = 0;
        task_list[i].mutex_locked_count = 0;
        task_list[i].last_wakeup_time = 0;
        
        for (int j = 0; j < MAX_CHILD_TASKS; j++) {
            task_list[i].w_chld[j] = 0;
        }
        
        for (int j = 0; j < MAX_OPEN_RESOURCES; j++) {
            task_list[i].open_resources[j] = NULL;
        }
    }
    
    /* Create idle task */
    task_create(idle_task, 255);  // Lowest priority
}

/*
 * Initialize task stack frame
 * Creates a stack frame that looks like an exception frame
 */
void* init_task_stack(void *stack_top, void (*task_func)(void))
{
    uint32_t *stk = (uint32_t *)stack_top;
    
    /* ARM Cortex-M exception stack frame (stacked by hardware during exception) */
    *(--stk) = (1 << 24);           // xPSR (Thumb bit set)
    *(--stk) = (uint32_t)task_func; // PC (return address - task function)
    *(--stk) = 0xFFFFFFFD;          // LR (EXC_RETURN - return to thread mode with PSP)
    *(--stk) = 0x12121212;          // R12
    *(--stk) = 0x03030303;          // R3
    *(--stk) = 0x02020202;          // R2
    *(--stk) = 0x01010101;          // R1
    *(--stk) = 0x00000000;          // R0
    
    /* Remaining registers (saved manually by PendSV) */
    *(--stk) = 0x11111111;          // R11
    *(--stk) = 0x10101010;          // R10
    *(--stk) = 0x09090909;          // R9
    *(--stk) = 0x08080808;          // R8
    *(--stk) = 0x07070707;          // R7
    *(--stk) = 0x06060606;          // R6
    *(--stk) = 0x05050505;          // R5
    *(--stk) = 0x04040404;          // R4
    
    return (void *)stk;
}

/*
 * Create a new task
 */
int32_t task_create(void (*task_func)(void), uint8_t priority)
{
    if (num_tasks >= MAX_TASKS) {
        return -EAGAIN;  // No more task slots
    }
    
    /* Find free TCB */
    int idx = -1;
    for (int i = 0; i < MAX_TASKS; i++) {
        if (task_list[i].status == TASK_TERMINATED || task_list[i].magic_number == 0) {
            idx = i;
            break;
        }
    }
    
    if (idx == -1) {
        return -EAGAIN;
    }
    
    /* Initialize TCB */
    task_list[idx].magic_number = TCB_MAGIC_NUMBER;
    task_list[idx].task_id = next_task_id++;
    task_list[idx].status = TASK_READY;
    task_list[idx].priority = priority;
    task_list[idx].parent_id = (current_task != NULL) ? current_task->task_id : 0;
    task_list[idx].execution_time = 0;
    task_list[idx].waiting_time = 0;
    task_list[idx].digital_signature = TCB_SIGNATURE;
    task_list[idx].heap_mem_start = NULL;
    task_list[idx].heap_mem_size = 0;
    task_list[idx].sem_waiting_count = 0;
    task_list[idx].mutex_locked_count = 0;
    task_list[idx].last_wakeup_time = 0;
    
    /* Initialize child and resource arrays */
    for (int j = 0; j < MAX_CHILD_TASKS; j++) {
        task_list[idx].w_chld[j] = 0;
    }
    
    for (int j = 0; j < MAX_OPEN_RESOURCES; j++) {
        task_list[idx].open_resources[j] = NULL;
    }
    
    /* Initialize stack */
    void *stack_top = &task_stacks[idx][TASK_STACK_SIZE / 4];
    task_list[idx].psp = init_task_stack(stack_top, task_func);
    
    num_tasks++;
    
    /* If this is the first task, set it as current */
    if (current_task == NULL) {
        current_task = &task_list[idx];
        current_task->status = TASK_RUNNING;
    }
    
    return (int32_t)task_list[idx].task_id;
}

/*
 * Get current running task
 */
TCB_TypeDef* get_current_task(void)
{
    return current_task;
}

/*
 * Get next ready task (Round-Robin scheduling)
 */
TCB_TypeDef* get_next_task(void)
{
    if (current_task == NULL || num_tasks == 0) {
        return NULL;
    }
    
    /* Find current task index */
    int current_idx = -1;
    for (int i = 0; i < MAX_TASKS; i++) {
        if (&task_list[i] == current_task) {
            current_idx = i;
            break;
        }
    }
    
    /* Round-robin: search for next ready task */
    int search_count = 0;
    int next_idx = (current_idx + 1) % MAX_TASKS;
    
    while (search_count < MAX_TASKS) {
        if (task_list[next_idx].status == TASK_READY &&
            task_list[next_idx].magic_number == TCB_MAGIC_NUMBER) {
            return &task_list[next_idx];
        }
        next_idx = (next_idx + 1) % MAX_TASKS;
        search_count++;
    }
    
    /* If no ready task found, return current task if still runnable */
    if (current_task->status == TASK_RUNNING || current_task->status == TASK_READY) {
        return current_task;
    }
    
    /* Last resort: find any ready task */
    for (int i = 0; i < MAX_TASKS; i++) {
        if (task_list[i].status == TASK_READY &&
            task_list[i].magic_number == TCB_MAGIC_NUMBER) {
            return &task_list[i];
        }
    }
    
    return current_task;  // No ready task, return current
}

/*
 * Schedule next task (called by PendSV)
 */
void schedule(void)
{
    /* Save execution time of current task */
    if (current_task != NULL && current_task->status == TASK_RUNNING) {
        current_task->status = TASK_READY;
    }
    
    /* Get next task */
    TCB_TypeDef *next_task = get_next_task();
    
    if (next_task != NULL && next_task != current_task) {
        /* Update task states */
        if (current_task != NULL && current_task->status != TASK_TERMINATED &&
            current_task->status != TASK_BLOCKED) {
            current_task->status = TASK_READY;
        }
        
        next_task->status = TASK_RUNNING;
        current_task = next_task;
    }
}

/*
 * Trigger PendSV exception for context switch
 */
void trigger_pendsv(void)
{
    /* Set PendSV interrupt pending bit */
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

/*
 * Terminate a task
 */
void task_terminate(uint16_t task_id)
{
    TCB_TypeDef *task = get_task_by_id(task_id);
    if (task != NULL) {
        task->status = TASK_TERMINATED;
        num_tasks--;
        
        /* If terminating current task, trigger rescheduling */
        if (task == current_task) {
            trigger_pendsv();
        }
    }
}

/*
 * Block a task
 */
void task_block(uint16_t task_id)
{
    TCB_TypeDef *task = get_task_by_id(task_id);
    if (task != NULL) {
        task->status = TASK_BLOCKED;
        
        /* If blocking current task, trigger rescheduling */
        if (task == current_task) {
            trigger_pendsv();
        }
    }
}

/*
 * Unblock a task
 */
void task_unblock(uint16_t task_id)
{
    TCB_TypeDef *task = get_task_by_id(task_id);
    if (task != NULL && task->status == TASK_BLOCKED) {
        task->status = TASK_READY;
    }
}

/*
 * Get task by ID
 */
TCB_TypeDef* get_task_by_id(uint16_t task_id)
{
    for (int i = 0; i < MAX_TASKS; i++) {
        if (task_list[i].task_id == task_id &&
            task_list[i].magic_number == TCB_MAGIC_NUMBER) {
            return &task_list[i];
        }
    }
    return NULL;
}

/*
 * Idle task - runs when no other task is ready
 */
void idle_task(void)
{
    while (1) {
        /* Just wait for interrupts */
        __WFI();
    }
}
