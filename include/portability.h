#ifndef __portability_h
#define __portability_h

/*
	I'm using macros here because even inline static functions somehow add to
	code size.
*/
#if defined(__AVR_ATtiny13A__)

#	define watchdog_interrupt_enable() \
	do { WDTCR |= (1 << WDTIE); } while(0)

#	define watchdog_interrupt_disable() \
	do { WDTCR &= ~(1 << WDTIE); } while(0)

#else

#	define watchdog_interrupt_enable() \
	do { WDTCR |= (1 << WDIE); } while(0)

#	define watchdog_interrupt_disable() \
	do { WDTCR &= ~(1 << WDIE); } while(0)

#endif

#endif
