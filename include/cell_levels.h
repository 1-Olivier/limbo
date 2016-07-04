#ifndef __cell_levels_h
#define __cell_levels_h

/*
	Macros for the cell voltage readout with the ADC from a two resistor
	voltage divider. Resistor values should be specified with no more than 4
	digits to avoid overflow. So a 4.7k/19k pair can be specified as 47/190
	or as 470/1900.
*/

#include <stdint.h>

#if DRIVER == BLFA6
/* Driver in the BLF-A6 / astrolux S1. 65B / 223 resistors -> 4.64k / 22k */
#define VCC_DIV_R1 464ul
#define VCC_DIV_R2 2200ul
#else
/* Generic widely used values (4.7k/19k). */
#define VCC_DIV_R1 470ul
#define VCC_DIV_R2 1900ul
#endif

/*
	8-bit ADC value from voltage, as a 3 digit value (ie. 385 for 3.85V).
*/
#define ADC8_FROM_CELL_V( vcc ) ((uint8_t)((uint32_t)(vcc) * VCC_DIV_R1 * 255ul / (110ul * (VCC_DIV_R1 + VCC_DIV_R2))))

#endif
