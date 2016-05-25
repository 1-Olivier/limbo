/*

   NANJG 105C Diagram
             ---
           -|   |- VCC
       OTC -|   |- Voltage ADC
    Star 3 -|   |- PWM (FET)
       GND -|   |- PWM (1x7135)
             ---

             ---
       PB5 -|1 8|- VCC
       PB3 -|2 7|- PB2
       PB4 -|3 6|- PB1
       GND -|4 5|- PB0
             ---
*/

#define F_CPU 4800000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "../include/delay_accurate.h"

#define NOP __asm__ volatile( "clc" )

uint16_t user_set_level;
#define USER_LEVEL_MIN 60
#define USER_LEVEL_MAX 85

uint8_t state;
#define STATE_RAMP_UP 0
#define STATE_STEADY 1

/* Last ADC cell voltage readout. */
volatile uint8_t cell_level;
#define ADC_CELL_100 211 /* 4.2V theoritical value with 4.7/19.2 divider */

/* 0 = no, 1 = start, 2 = done */
uint8_t do_power_adc;

void empty_gate()
{
	/* enable output, off to empty gate  */
	DDRB |= (1 << DDB1);
	NOP;
	NOP;
	NOP;
	/* that should be more than enough */
	DDRB &= ~(1 << DDB1);
}

void charge_gate( uint16_t gate_charge )
{
	/* disable interruptions or counter overflow could add to the delay */
	cli();
	/* enable pull-up */
	PORTB |= (1 << PORTB1);
	/* wait */
	delay_cycles( gate_charge );
	/* disable pull-up */
	PORTB &= ~(1 << PORTB1);
	/* enable interruptions again */
	sei();
}

void set_output_level()
{
	/* FIXME: Two divides use far less space than a multiply + divide */
	uint8_t cell2 = (cell_level * cell_level) >> 8;
	uint16_t gate_level = (user_set_level << 8) / cell2;
	empty_gate();
	charge_gate( gate_level - 6u );
}

/* flash the 7135 channel to say we're running */
void flash()
{
	PORTB |= (1 << PORTB0);
	_delay_us( 2 );
	PORTB &= ~(1 << PORTB0);
}

/* flash an 8-bit value on the 7135 channel */
void flash_debug( uint8_t value )
{
	for( uint8_t i = 0; i < 8; ++i )
	{
		PORTB |= (1 << PORTB0);
		if( value & 1u )
		{
			_delay_us( 5 );
		}
		else
		{
			_delay_us( 1 );
		}
		PORTB &= ~(1 << PORTB0);
		value >>= 1u;
		_delay_ms( 500 );
	}
}

/* Counter overflow interrupt. */
uint8_t counter_high;
ISR( TIM0_OVF_vect )
{
	++counter_high;
}

/* Watchdog interrupt. */
uint8_t dog_count;
ISR( WDT_vect )
{
#if 0
	charge_gate( ((uint16_t)(TCNT0 + (uint8_t)10) >> 0) + (uint16_t)150 );
#else
	// This should be about 9600
	uint16_t cycles = ((uint16_t)counter_high << 8) | TCNT0;
	//charge_gate( (cycles & 0x1fff) - (uint16_t)(6144 + 95) ); 
#endif
	// FIXME: Should this be reset after we set output level, just before interrupts get reenabled?
	// Should be ok... won't have time to overflow before we return from here.
	TCNT0 = 0;
	counter_high = 0;

	if( state == STATE_RAMP_UP )
	{
		if( ++dog_count == 4 )
		{
			dog_count = 0;
			++user_set_level;
			if( user_set_level == USER_LEVEL_MAX )
			{
				state = STATE_STEADY;
			}
		}
	}

	/* Set new light value. */
	set_output_level();

	/* Start cell level ADC. */
	/*
		We could reasonably do this every 16th interrupt or more. Or just slow
		down the WDT. The only point is to save power in low power modes.
		Perhaps not that relevant as we should already be at roughly 0.1 mA.
		OTOH, if things work just as well with 1/2 or 1/4 WDT frequency...
	*/
	do_power_adc = 1;
}

int main(void)
{
	/* initialize state */
	user_set_level = USER_LEVEL_MIN;
	cell_level = ADC_CELL_100; /* assume full until we know better */

	/* drain anything left from before click */
	empty_gate();

	/* initialize ADC to read OTC */
	/* disable digital input */
	DIDR0 |= (1 << ADC3D);
	/* internal ref, left adjusted, ADC3 (PB3) */
	ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX1) | (1 << MUX0);
	/* enable, start conversion, prescaler = 1/64 */
	ADCSRA = (1 << ADEN) | (1 << ADSC) | 0x6;

	while( ADCSRA & (1 << ADSC) )
	{
	}
	uint8_t otc_value = ADCH;

	/* switch ADC to read cell level on ADC1 (PB2) */
	ADMUX &= ~(1 << MUX1);
	/* disable digital input */
	DIDR0 |= (1 << ADC1D);

	/* charge the OTC */
	DDRB |= (1 << DDB3);
	PORTB |= (1 << PORTB3);

	/* enable 7135 output (useful for debugging) */
	/* also to make sure it doesn't produce crap */
	DDRB |= (1 << DDB0);
#if 0
	/* flash the OTC value */
	for( uint8_t i = 0; i < otc_value; i += 16 )
	{
		flash();
		_delay_ms( 200 );
	}
#endif

	if( otc_value > 190 )
	{
		/* short press */
	}
	else if( otc_value > 54 )
	{
		/* long press */
	}
	else
	{
		/* longer time off (not a press) */
	}

	/* enable watchdog interrupt */
	WDTCR |= (1 << WDTIE);
	sei();

	//empty_gate();
	//flash();

	/* start counter with clk */
	//TCCR0B = (1 << CS00); // clk
	TCCR0B = (1 << CS01); // clk / 8 (appears to work when used directly)
	//TCCR0B = (1 << CS01) | (1 << CS00); // clk/64
	//TCCR0B = (1 << CS02); // clk / 256

	/* enable counter overflow interrupt */
	TIMSK0 |= (1 << TOIE0);

	set_sleep_mode( SLEEP_MODE_PWR_DOWN );

	/*
		Potential states to handle here:
		- Ramping up.
		- Ramping down.
		- Steady output.
		- Temperature configuration mode.
		- Range configuration mode?
		- Modelist configuration

		They should likely all do Vcc ADC.
		Perhaps just put it in WDT? Start it at the end, fetch result going in!
		Problem if we want to power down between interrupts. Perhaps just check
		do_power_adc and power down when none is running. Won't power down as
		fast though (one WDT cycle vs one ADC cycle).
	*/
	while( 1 == 1 )
	{
		// TODO: move this to function
		if( do_power_adc )
		{
			ADCSRA |= (1 << ADEN);
			/* Should we give it time to stabilize here? Probably not. */
			ADCSRA |= (1 << ADSC);
			while( ADCSRA & (1 << ADSC ) )
			{
			}
#if 0
			cell_level = ADCH;
#else
			/*
				Filter ADC output to avoid light level flicker. I do this by
				giving it a bit of wiggle room. It's not that the reading is
				very noisy but rather that 8 bits are not enough. A change in
				the LSB causes visible flicker so I'm not certain even the full
				10 bits would be safe. Besides, getting that requires ADC noise
				reduction mode. This is simpler but generates fat code.

				TODO: See if this can be put in a function and reused for
				temperature reading.
			*/
			uint8_t read_level = ADCH;
			uint8_t rp1 = read_level + 1u;
			uint8_t rm1 = read_level - 1u;
			uint8_t local_cell_level = cell_level;
			if( rp1 < local_cell_level )
				local_cell_level = rp1;
			if( rm1 > local_cell_level )
				local_cell_level = rm1;
			cell_level = local_cell_level;
#endif
#if 0
			/* That range is enough to cause a flicker! */
			if( cell_level < 164 )
				cell_level = 164;
			if( cell_level > 165 )
				cell_level = 165;
#endif
			ADCSRA &= ~(1 << ADEN);
			do_power_adc = 0;
		}
		/* Sleep until next interrupt. */
		sleep_mode();
	}
}
