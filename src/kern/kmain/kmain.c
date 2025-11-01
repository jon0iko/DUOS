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

#include <sys_init.h>
#include <cm4.h>
#include <kmain.h>
#include <stdint.h>
#include <sys_usart.h>
#include <kstdio.h>
#include <sys_rtc.h>
#include <kstring.h>
#include <schedule.h>
#include <kunistd.h>

#ifndef DEBUG
#define DEBUG 1
#endif

/* Task function prototypes */
void task1(void);
void task2(void);
void task3(void);

/* Task 1: Prints messages and tests getpid and time syscalls */
void task1(void)
{
    while (1)
    {
        int32_t pid = getpid();
        uint32_t time = getSysTickTime();
        
        duprintf("[Task 1] PID: %d, Time: %d ms\r\n", pid, time);
        duprintf("[Task 1] Testing SYS_write and SYS_read...\r\n");
        
        /* Delay using yield to allow other tasks to run */
        for (int i = 0; i < 5; i++) {
            yield();
        }
    }
}

/* Task 2: Tests string I/O */
void task2(void)
{
    uint8_t input_buffer[64];
    int count = 0;
    
    while (1)
    {
        int32_t pid = getpid();
        uint32_t time = getSysTickTime();
        
        duprintf("[Task 2] PID: %d, Time: %d ms\r\n", pid, time);
        duprintf("[Task 2] Count: %d\r\n", count++);
        
        if (count == 3) {
            duprintf("[Task 2] Enter your name: ");
            duscanf("%s", input_buffer);
            duprintf("\r\n[Task 2] Hello, %s!\r\n", input_buffer);
        }
        
        /* Delay using yield */
        for (int i = 0; i < 5; i++) {
            yield();
        }
    }
}

/* Task 3: Tests number I/O and yield */
void task3(void)
{
    uint32_t counter = 0;
    
    while (1)
    {
        int32_t pid = getpid();
        uint32_t time = getSysTickTime();
        
        duprintf("[Task 3] PID: %d, Time: %d ms, Counter: %d\r\n", pid, time, counter);
        counter++;
        
        if (counter == 10) {
            duprintf("[Task 3] Demonstrating voluntary yield...\r\n");
            yield();
            counter = 0;
        }
        
        /* Delay using yield */
        for (int i = 0; i < 5; i++) {
            yield();
        }
    }
}

void kmain(void)
{
    /* Initialize all STM32 board components */
    __sys_init();
    
    /* Initialize scheduler */
    scheduler_init();
    
    /* Print banner using duprintf */
    duprintf("\r\n");
    duprintf("===================================================\r\n");
    duprintf("    DUOS - Task Scheduling and Syscall Demo\r\n");
    duprintf("===================================================\r\n");
    duprintf("Testing Implementation:\r\n");
    duprintf("  - SYS_read and SYS_write\r\n");
    duprintf("  - SYS_getpid\r\n");
    duprintf("  - SYS_time\r\n");
    duprintf("  - SYS_yield\r\n");
    duprintf("  - SYS_exit\r\n");
    duprintf("  - SYS_reboot (use command to test)\r\n");
    duprintf("  - Task Scheduling (Round-Robin, 10ms time slice)\r\n");
    duprintf("  - Context Switching via PendSV\r\n");
    duprintf("===================================================\r\n");
    duprintf("\r\n");
    
    /* Create user tasks */
    duprintf("Creating tasks...\r\n");
    int32_t tid1 = task_create(task1, 1);
    duprintf("Task 1 created with ID: %d\r\n", tid1);
    
    int32_t tid2 = task_create(task2, 1);
    duprintf("Task 2 created with ID: %d\r\n", tid2);
    
    int32_t tid3 = task_create(task3, 1);
    duprintf("Task 3 created with ID: %d\r\n", tid3);
    
    duprintf("\r\n");
    duprintf("Starting scheduler...\r\n");
    duprintf("Tasks will switch every 10ms (SysTick)\r\n");
    duprintf("---------------------------------------------------\r\n");
    duprintf("\r\n");
    
    /* Small delay before starting task switching */
    ms_delay(2000);
    
    /* Set PSP to first task's stack pointer */
    TCB_TypeDef *first_task = get_current_task();
    if (first_task != NULL) {
        __asm volatile("MSR PSP, %0" : : "r"(first_task->psp));
        __asm volatile("MSR CONTROL, %0" : : "r"(0x03)); /* Use PSP, unprivileged mode */
        __asm volatile("ISB");
    }
    
    /* Jump to first task */
    duprintf("Jumping to first task...\r\n\r\n");
    
    /* Infinite loop - tasks will run via context switching */
    while (1)
    {
        /* Main loop - tasks are scheduled by PendSV */
        yield();
    }
}

