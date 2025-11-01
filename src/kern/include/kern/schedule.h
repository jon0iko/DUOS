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
 
#ifndef __SCHEDULE_H
#define __SCHEDULE_H

#include <stdint.h>
#include <types.h>

/* Maximum number of tasks */
#define MAX_TASKS 8

/* Task stack sizes */
#define TASK_STACK_SIZE 1024  // 1KB per task

/* Global task management */
extern TCB_TypeDef task_list[MAX_TASKS];
extern TCB_TypeDef *current_task;
extern uint32_t num_tasks;
extern uint32_t next_task_id;

/* Task stack memory (allocated statically) */
extern uint32_t task_stacks[MAX_TASKS][TASK_STACK_SIZE / 4];

/* Scheduler functions */
void scheduler_init(void);
void schedule(void);
TCB_TypeDef* get_current_task(void);
TCB_TypeDef* get_next_task(void);
void switch_context(void);
void trigger_pendsv(void);

/* Task management functions */
int32_t task_create(void (*task_func)(void), uint8_t priority);
void task_terminate(uint16_t task_id);
void task_block(uint16_t task_id);
void task_unblock(uint16_t task_id);
TCB_TypeDef* get_task_by_id(uint16_t task_id);

/* Stack initialization */
void* init_task_stack(void *stack_top, void (*task_func)(void));

/* Idle task */
void idle_task(void);

#endif


