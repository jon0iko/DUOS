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
 #include <stdarg.h>
#include <kstdio.h>
#include <sys_usart.h>
#include <UsartRingBuffer.h>
#include <kstring.h>
#include <float.h>
#include <system_config.h>
#include <kunistd.h>

/**
* first argument define the type of string to kprintf and kscanf, 
* %c for charater
* %s for string, 
* %d for integer
* %x hexadecimal
* %o octal number
* %f for floating point number
*/
// Simplified version of printf
void kprintf(char *format,...)
{
//write your code here
	char *tr;
	uint32_t i;
	uint8_t *str;
	va_list list;
	double dval;
	char output_buf[256];
	int buf_idx = 0;
	
	//uint32_t *intval;
	va_start(list,format);
	for(tr = format;*tr != '\0';tr++)
	{
		while(*tr != '%' && *tr!='\0')
		{
			/* Buffer the output character */
			output_buf[buf_idx++] = *tr;
			if (buf_idx >= sizeof(output_buf) - 1) {
				/* Flush buffer using write syscall */
				write(STDOUT_FILENO, output_buf, buf_idx);
				buf_idx = 0;
			}
			tr++;
		}
		if(*tr == '\0') break;
		tr++;
		switch (*tr)
		{
		case 'c': i = va_arg(list,int);
			output_buf[buf_idx++] = (char)i;
			break;
		case 'd': i = va_arg(list,int);
			if(i<0)
			{
				output_buf[buf_idx++] = '-';
				i=-i;				
			}
			{
				char *num_str = (char*)convert(i,10);
				while(*num_str) {
					output_buf[buf_idx++] = *num_str++;
					if (buf_idx >= sizeof(output_buf) - 1) {
						write(STDOUT_FILENO, output_buf, buf_idx);
						buf_idx = 0;
					}
				}
			}
			break;
		case 'o': i = va_arg(list,int);
			if(i<0)
			{
				output_buf[buf_idx++] = '-';
				i=-i;				
			}
			{
				char *num_str = (char*)convert(i,8);
				while(*num_str) {
					output_buf[buf_idx++] = *num_str++;
					if (buf_idx >= sizeof(output_buf) - 1) {
						write(STDOUT_FILENO, output_buf, buf_idx);
						buf_idx = 0;
					}
				}
			}
			break;
		case 'x': i = va_arg(list,int);
			{
				char *num_str = (char*)convertu32(i,16);
				while(*num_str) {
					output_buf[buf_idx++] = *num_str++;
					if (buf_idx >= sizeof(output_buf) - 1) {
						write(STDOUT_FILENO, output_buf, buf_idx);
						buf_idx = 0;
					}
				}
			}
			break;
		case 'u':	
		case 's': str = va_arg(list,uint8_t*);
			while(*str) {
				output_buf[buf_idx++] = *str++;
				if (buf_idx >= sizeof(output_buf) - 1) {
					write(STDOUT_FILENO, output_buf, buf_idx);
					buf_idx = 0;
				}
			}
			break;
		case 'f': 
			dval = va_arg(list,double);
			{
				char *num_str = (char*)float2str(dval);
				while(*num_str) {
					output_buf[buf_idx++] = *num_str++;
					if (buf_idx >= sizeof(output_buf) - 1) {
						write(STDOUT_FILENO, output_buf, buf_idx);
						buf_idx = 0;
					}
				}
			}
			break;	
		default:
			break;
		}
	}
	
	/* Flush remaining buffer contents */
	if (buf_idx > 0) {
		write(STDOUT_FILENO, output_buf, buf_idx);
	}
	
	va_end(list);
}

void putstr(const uint8_t *str,size_t size)
{
	for(uint32_t i=0;i<size;i++)
	{
		Uart_write(str[i],__CONSOLE);
	}
}

// Simplified version of scanf
void kscanf(char *format,...)
{
//write your code here
	va_list list;
	char *ptr;
	uint8_t buff[256];  /* Increased to MAX_READ_SIZE */
	uint8_t *str;
	int len;
	ptr=format;
	va_start(list,format);
	
	/* Read input string using syscall - reads until newline or MAX_READ_SIZE */
	int bytes_read = read(STDIN_FILENO, buff, sizeof(buff) - 1);
	if (bytes_read > 0) {
		buff[bytes_read] = '\0';  /* Ensure null termination */
	} else {
		buff[0] = '\0';
	}
	
	/* Parse the input buffer according to format */
	uint8_t *input_ptr = buff;
	while (*ptr)
	{
		if(*ptr == '%') //looking for format of an input
		{
			ptr++;
			switch (*ptr)
			{
			case 'c': //character
				if (*input_ptr != '\0') {
					*(uint8_t*)va_arg(list,uint8_t*) = *input_ptr++;
				}
				break;
			case 'd': //integer number 
				*(uint32_t*)va_arg(list,uint32_t*)=__str_to_num(buff,10);	
				break;
			case 's': //string without spaces
				str = va_arg(list,uint8_t*);
				len = __strlen(buff);
				for(int u = 0; u<=len; u++)	// copy from buff to user defined char pointer (i.e string)
					str[u] = buff[u];	
				break;
			case 'x': //hexadecimal number
				*(int*)va_arg(list,uint32_t*)=__str_to_num(buff,16);	
				break;	
			case 'o': //octal number
				*(uint32_t*)va_arg(list,uint32_t*)=__str_to_num(buff,8);	
				break;	
			case 'f': //floating point number
				//*(uint32_t*)va_arg(list,double*) = __str_to_num(buff,10);
				*(float*)va_arg(list,float*) = str2float(buff);	// Works for float but not for double !!!
				break;	
			default: //rest not recognized
				break;
			}
		}
		ptr++;
	}
	va_end(list);
}
