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
#define USER_LEVEL_MIN 155
#define USER_LEVEL_MAX 235

uint8_t state;
#define STATE_RAMP_UP 0
#define STATE_STEADY 1

/* Last ADC cell voltage readout. */
uint8_t cell_level;
#define ADC_CELL_100 170

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
	empty_gate();
	charge_gate( user_set_level - 6u );
}

/* flash the 7135 channel to say we're running */
void flash()
{
	PORTB |= (1 << PORTB0);
	_delay_us( 2 );
	PORTB &= ~(1 << PORTB0);
}

/* Counter overflow interrupt. */
uint8_t counter_high;
ISR( TIM0_OVF_vect )
{
	++counter_high;
}

/* Watchdog interrupt. */
uint16_t dog_count;
ISR( WDT_vect )
{
#if 0
	charge_gate( ((uint16_t)(TCNT0 + (uint8_t)10) >> 0) + (uint16_t)150 );
#else
	// This should be about 9600
	uint16_t cycles = ((uint16_t)counter_high << 8) | TCNT0;
	//charge_gate( (cycles & 0x1fff) - (uint16_t)(6144 + 95) ); 
#endif
	TCNT0 = 0;
	counter_high = 0;

	if( state == STATE_RAMP_UP )
	{
		if( ++dog_count == 2 )
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

	/* flash the OTC value */
#if 0
	/* enable 7135 output (useful for debugging) */
	DDRB |= (1 << DDB0);
	for( uint8_t i = 0; i < otc_value; i += 16 )
	{
		flash();
		_delay_ms( 200 );
	}
#endif

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

	while( 1 == 1 )
	{
		if( do_power_adc )
		{
			ADCSRA |= (1 << ADEN);
			/* Should we give it time to stabilize here? Probably not. */
			ADCSRA |= (1 << ADSC);
			while( ADCSRA & (1 << ADSC ) )
			{
			}
			cell_level = ADCH;
			ADCSRA &= ~(1 << ADEN);
		}
		/* Sleep until next interrupt. */
		sleep_mode();
	}
}
