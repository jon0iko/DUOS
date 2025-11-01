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
#include <kunistd.h>
#include <syscall_def.h>
#include <sys_usart.h>
#include <system_config.h>
#include <UsartRingBuffer.h>
#include <errno.h>

/* 
 * read() - Application layer library function
 * Issues SVC with SYS_read syscall ID
 * Arguments: fd - file descriptor, buf - buffer pointer, size - size to read
 * Returns: number of bytes read, or -1 on error
 */
int32_t read(int fd, void *buf, size_t size)
{
    int32_t result;
    /* Trigger SVC instruction with syscall number and arguments */
    __asm volatile(
        "mov r0, %1\n"      /* syscall ID (SYS_read) */
        "mov r1, %2\n"      /* fd */
        "mov r2, %3\n"      /* buf */
        "mov r3, %4\n"      /* size */
        "svc #0\n"          /* Trigger SVC exception */
        "mov %0, r0\n"      /* Store return value */
        : "=r"(result)
        : "r"(SYS_read), "r"(fd), "r"(buf), "r"(size)
        : "r0", "r1", "r2", "r3"
    );
    return result;
}

/*
 * write() - Application layer library function
 * Issues SVC with SYS_write syscall ID
 * Arguments: fd - file descriptor, buf - buffer pointer, size - size to write
 * Returns: number of bytes written, or -1 on error
 */
int32_t write(int fd, const void *buf, size_t size)
{
    int32_t result;
    /* Trigger SVC instruction with syscall number and arguments */
    __asm volatile(
        "mov r0, %1\n"      /* syscall ID (SYS_write) */
        "mov r1, %2\n"      /* fd */
        "mov r2, %3\n"      /* buf */
        "mov r3, %4\n"      /* size */
        "svc #0\n"          /* Trigger SVC exception */
        "mov %0, r0\n"      /* Store return value */
        : "=r"(result)
        : "r"(SYS_write), "r"(fd), "r"(buf), "r"(size)
        : "r0", "r1", "r2", "r3"
    );
    return result;
}

/*
 * sys_read() - Kernel level read function
 * Reads from USART when fd is STDIN_FILENO
 * Arguments: fd - file descriptor, buf - buffer pointer, size - size to read
 * Returns: number of bytes read, or -1 on error
 */
int32_t sys_read(int fd, void *buf, size_t size)
{
    if (buf == NULL || size == 0) {
        return -EINVAL;
    }

    /* Handle STDIN_FILENO using USART */
    if (fd == STDIN_FILENO) {
        uint8_t *buffer = (uint8_t *)buf;
        size_t bytes_read = 0;
        size_t max_size = (size > MAX_READ_SIZE) ? MAX_READ_SIZE : size;

        /* Read characters until newline or max size */
        while (bytes_read < max_size) {
            /* Wait for data to be available */
            while (!IsDataAvailable(__CONSOLE));
            
            int c = Uart_read(__CONSOLE);
            if (c == -1) {
                break;
            }

            /* Check for termination character */
            if (c == '\n' || c == '\r') {
                buffer[bytes_read] = '\0';
                break;
            }

            buffer[bytes_read++] = (uint8_t)c;
        }

        /* Null-terminate if we reached max size */
        if (bytes_read >= max_size) {
            buffer[max_size - 1] = '\0';
            bytes_read = max_size;
        }

        return (int32_t)bytes_read;
    }

    /* Unsupported file descriptor */
    return -EBADF;
}

/*
 * sys_write() - Kernel level write function
 * Writes to USART when fd is STDOUT_FILENO or STDERR_FILENO
 * Arguments: fd - file descriptor, buf - buffer pointer, size - size to write
 * Returns: number of bytes written, or -1 on error
 */
int32_t sys_write(int fd, const void *buf, size_t size)
{
    if (buf == NULL || size == 0) {
        return -EINVAL;
    }

    /* Handle STDOUT_FILENO and STDERR_FILENO using USART */
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
        const uint8_t *buffer = (const uint8_t *)buf;
        
        for (size_t i = 0; i < size; i++) {
            Uart_write(buffer[i], __CONSOLE);
        }

        return (int32_t)size;
    }

    /* Unsupported file descriptor */
    return -EBADF;
}

#include <kunistd.h>
/* Add your functions here */

