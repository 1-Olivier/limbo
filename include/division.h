#ifndef __division_h
#define __division_h

/*
	div_u16_u8_sl8

	This means: divide 16-bit unsigned by 8-bit unsigned, shift left by 8

	In other words, (dividend << 8) / divisor but without having to make
	dividend more than 16-bit to avoid overflow. You can also think of it as
	A / (B / 256), which makes sense as fixed point arithmetic.

	It works for all sane cases, meaning no division by 0 and no overflow  (ie.
	divisor != 0 and dividend < divisor * 256).
*/
uint16_t div_u16_u8_sl8( uint16_t dividend, uint8_t divisor )
{
	/* Shifting by 8 is a register move so it takes one fewer instruction
	   to do << 8 and then >> 1 than to do << 7. Semi-stupid compiler. */
	uint16_t div = (((uint16_t)divisor) << 8) >> 1;
	uint16_t result = 0;
	uint8_t i = 16;
	while( 1 == 1 )
	{
		if( dividend >= div )
		{
			result |= 1;
			dividend -= div;
		}
		if( --i == 0 )
			break;
		result <<= 1;
		dividend <<= 1;
	}
	return result;
}

#endif
