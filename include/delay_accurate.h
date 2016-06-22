#ifndef _delay_accurate_h
#define _delay_accurate_h

/*
	Delay loop with single cycle accuracy.

	The actual delay is __cycles + k cycles, with k being about 10.
	Does the single cycle fixup using a skip instruction combined with a single
	word NOP which needs two cycles to execute (adiw 0). We need to fix 0-3
	cycles because the main delay loop is 4 cycles per iteration.
*/
inline static void delay_cycles( uint16_t __cycles )
{
	__asm__ volatile (
		"1: sbiw %0,4" "\n"
		"brsh 1b" "\n"
		"sbrc %0,0" "\n"
		"adiw %0,0" "\n"
		"sbrc %0,1" "\n"
		"adiw %0,0" "\n"
		"sbrc %0,1" "\n"
		"adiw %0,0" "\n"
		: "=w" (__cycles)
		: "0" (__cycles)
	);
}

#endif
