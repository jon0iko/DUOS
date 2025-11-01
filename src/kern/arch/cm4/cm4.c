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
 
#include <cm4.h>
#include <sys_clock.h>
#include <syscall.h>
#include <sys_timer.h>

volatile uint32_t tick_counter = 0;

void __SysTick_init(uint32_t reload)
{
    SYSTICK->CTRL = 0;
    SYSTICK->LOAD = ((__AHB_CLK()*1000000)  / reload);
    SYSTICK->VAL = 0;
    SYSTICK->CTRL = (1 << 2) | (1 << 1) | (1 << 0);  // Clock source + Interrupt + Enable
    tick_counter = 0;
}

void SysTickIntDisable(void)
{
    SYSTICK->CTRL &= ~(1 << 1); 
}

void SysTickIntEnable(void)
{
    SYSTICK->CTRL |= (1 << 1);
}

void __SysTick_enable(void)
{
    SYSTICK->CTRL |= (1 << 0);
}

void __sysTick_disable(void)
{
    SYSTICK->CTRL &= ~(1 << 0);
}

uint32_t __getSysTickCount(void)
{
    return tick_counter;
}

void __updateSysTick(uint32_t count)
{
    tick_counter = count;
}

uint32_t __getTime(void)
{
    return tick_counter;
}

uint32_t __get__Second(void)
{
    return tick_counter / 1000;
}

uint32_t __get__Minute(void)
{
    return tick_counter / 60000;
}

uint32_t __get__Hour(void)
{
    return tick_counter / 3600000;
}

void SysTick_Handler(void)
{
    tick_counter++;
}

void __enable_fpu()
{
    SCB->CPACR |= ((0xFUL<<20));
}

uint8_t ms_delay(uint32_t delay)
{
    uint32_t start = tick_counter;
    while(((tick_counter - start) < delay)) {
        __WFI();
    };
    return 0;
}

uint32_t getmsTick(void)
{
    return tick_counter;
}

uint32_t wait_until(uint32_t delay)
{
    uint32_t start = tick_counter;
    while((tick_counter - start) < delay);
    return tick_counter;
}

void SYS_SLEEP_WFI(void)
{
    __WFI();
}
