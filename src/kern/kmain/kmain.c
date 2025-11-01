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
#ifndef DEBUG
#define DEBUG 1
#endif

void kmain(void)
{
    /* Initialize all STM32 board components */
    __sys_init();
    
    /* Test SYS_write using kprintf */
    kprintf("\r\n");
    kprintf("===================================\r\n");
    kprintf("DUOS - Syscall Test Program\r\n");
    kprintf("Testing SYS_read and SYS_write\r\n");
    kprintf("===================================\r\n");
    kprintf("\r\n");
    
    /* Main test loop */
    while (1)
    {
        uint8_t input_buffer[256];
        uint32_t number;
        
        /* Test 1: String input/output */
        kprintf("Test 1: String I/O\r\n");
        kprintf("Enter your name: ");
        kscanf("%s", input_buffer);
        kprintf("\r\nHello, %s!\r\n", input_buffer);
        kprintf("\r\n");
        
        ms_delay(1000);
        
        /* Test 2: Integer input/output */
        kprintf("Test 2: Integer I/O\r\n");
        kprintf("Enter a number: ");
        kscanf("%d", &number);
        kprintf("\r\nYou entered: %d\r\n", number);
        kprintf("In hex: 0x%x\r\n", number);
        kprintf("In octal: %o\r\n", number);
        kprintf("\r\n");
        
        ms_delay(1000);
        
        /* Test 3: Character input/output */
        kprintf("Test 3: Character I/O\r\n");
        kprintf("Enter a character: ");
        uint8_t ch;
        kscanf("%c", &ch);
        kprintf("\r\nYou entered: '%c' (ASCII: %d)\r\n", ch, ch);
        kprintf("\r\n");
        
        ms_delay(2000);
        
        /* Repeat tests */
        kprintf("-----------------------------------\r\n");
        kprintf("Repeating tests in 3 seconds...\r\n");
        kprintf("-----------------------------------\r\n");
        kprintf("\r\n");
        ms_delay(3000);
    }
}

