#ifndef __cell_levels_h
#define __cell_levels_h

/*
	Macros for the cell voltage readout with the ADC from a two resistor
	voltage divider. Resistor values should be specified with no more than 4
	digits to avoid overflow. So a 4.7k/19k pair can be specified as 47/190
	or as 470/1900.

	The diode drop before voltage divider should be specified in units of 10 mV.
*/

#include <stdint.h>

#if defined(DRIVER_BLFA6)
/* Driver in the BLF-A6 / astrolux S1. 65B / 223 resistors -> 4.64k / 22k */
#define VCC_DIV_R1 464ul
#define VCC_DIV_R2 2200ul
#define VCC_DIV_DIODE_DROP 0ul
#elif defined(DRIVER_MTN17DDm)
/*
	MTN-17DDm driver. Uses 4.7k / 19.1k. Has voltage divider after reverse
	polarity diode which drops about 140 mV (measured).
*/
#define VCC_DIV_R1 470ul
#define VCC_DIV_R2 1910ul
#define VCC_DIV_DIODE_DROP 14ul
#else
/* Generic values (4.7k/19k). */
#define VCC_DIV_R1 470ul
#define VCC_DIV_R2 1900ul
#define VCC_DIV_DIODE_DROP 0ul
#endif

/*
	8-bit ADC value from voltage, as a 3 digit value (ie. 385 for 3.85V).
*/
#define ADC8_FROM_CELL_V( vcc ) ((uint8_t)((uint32_t)(vcc) * VCC_DIV_R1 * 255ul / (110ul * (VCC_DIV_R1 + VCC_DIV_R2))))
#define ADC8_CELL_DIODE_DROP ADC8_FROM_CELL_V( VCC_DIV_DIODE_DROP )

#endif
