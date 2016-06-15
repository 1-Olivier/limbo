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

/*
	Potential config options:
	- memory
	- initial ramp direction (?)
	- mode list
	- limit temp

	Config should be stored separately from state to keep state to 2 bytes.
	Probably does not need wear leveling as it isn't changed often.
*/

#define F_CPU 4800000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "../include/delay_accurate.h"
#include "../include/division.h"

#define NOP __asm__ volatile( "clc" )

/* To check memory decay. */
uint8_t mem_check __attribute__ ((section (".noinit")));

uint16_t user_set_level __attribute__ ((section (".noinit")));
#define USER_LEVEL_MIN 120
#define USER_LEVEL_MAX 200

/*
	Uses only the high nibble. Important for eeprom read/write as this gets
	merged with the high byte of the level (of which only the low nibble is
	used).
*/
uint8_t state __attribute__ ((section (".noinit")));
#define STATE_RAMP_UP 0x00
#define STATE_STEADY 0x10
#define STATE_RAMP_DOWN 0x20

/* Last ADC cell voltage readout. */
volatile uint8_t cell_level;
#define ADC_CELL_100 211 /* 4.2V theoritical value with 4.7/19.2 divider */
#define ADC_CELL_LOWEST 137 /* 3.0V */

/* 0 = no, 1 = start, 2 = done */
uint8_t do_power_adc;

/* current eeprom address for state */
uint8_t eeprom_state_addr __attribute__ ((section (".noinit")));

void empty_gate()
{
	/* enable output, off to empty gate  */
	DDRB |= (1 << DDB1);
	// TODO: See how many are really needed to flush the gate at high power levels.
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
	/*
		Adjust charge time by dividing by the square of Vcc. This appears to
		make it fairly stable over the useful range of a cell. I actually use
		two divide instead of squaring Vcc as this requires less code.

		This adjustment can make the steps of gate_level easily 4 large for a
		change of one unit in user_set_level. This is why gate_level is divided
		by 2 before use and the user levels are made twice as large,
		accordingly. It helps keep a small granularity to the ramp.

		FIXME: The shift is two words. Could be avoided by changing the delay
		loop. It could be written with 1 to 3 bit shift built-in.
	*/
	uint8_t local_cell_level = cell_level;
#if 0
	/* This overflows if user level > 255 */
	uint16_t gate_level = (user_set_level << 8);
	gate_level /= local_cell_level;
	gate_level <<= 8;
	gate_level /= local_cell_level;
#else
	uint16_t gate_level = div_u16_u8_sl8( user_set_level, local_cell_level );
	gate_level = div_u16_u8_sl8( gate_level, local_cell_level );
#endif
	empty_gate();
	// FIXME: could this underflow with invalid (0) user_set_level?
	charge_gate( (gate_level >> 1) - 6u );
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

/*
	Helper to write to eeprom.
*/
void eeprom_program()
{
	/* The two bits for eeprom program need to be set within 4 cycles so we
	   can't afford an interrupt. */
	cli();
	EECR |= (1 << EEMPE); /* master program enable */
	EECR |= (1 << EEPE); /* progam enable */
	/* Wait before return so it is then safe to modify the eeprom registers for
	   the next write. */
	while( EECR & (1 << EEPE) ) {}
	/* Enable interrupts again. */
	sei();
}

/*
	TODO: see if the wait, MPE, PE sequence can be moved to a function. I don't
	remember of the address register is latched or not. Seems not, see FAQ
	about eeprom address 0 corruption at
	http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_eeprom_corruption.html

	TODO: Add interrupt disable in the above sequence, if relevent. Perhaps not
	as we can read state before interrupts are enabled and write it from within
	an interrupt.
		Update: Actually, I don't think we can write from within the WDT
		interrupt as we'll miss counter interrupts.

	Interrupts are assumed to be enabled when this is called.
*/
void save_state_to_eeprom()
{
	/* Write current state to next location in eeprom, for wear leveling. */
	uint8_t eep_addr = eeprom_state_addr;
	eep_addr += 2;
	eeprom_state_addr = eep_addr;
	/* Write first byte. */
	EEARL = eep_addr;
	EECR |= (1 << EEPM1); /* write only */
	EEDR = state | (user_set_level >> 8);
	eeprom_program();
	/* Write second byte. */
	EEARL |= 1;
	EEDR = user_set_level;
	eeprom_program();
	EECR &= ~(1 << EEPM1); /* back to atomic mode */

	/* Clear previous state. */
	EEARL = eep_addr - 2;
	EECR |= (1 << EEPM0); /* erase only */
	eeprom_program();
	/* second byte */
	EEARL |= 1;
	eeprom_program();
	EECR &= ~(1 << EEPM0); /* back to atomic mode */

}

/*
	We roll our own eeprom access to save some code space. We assume that there
	are no writes in progress. Loading the state should be the first eeprom
	operation we do and we should do it only once.

	Assumes interruptions are disabled.
*/
void load_state_from_eeprom()
{
	uint8_t eep_addr;
	for( eep_addr = 0; eep_addr < 64; eep_addr += 2 )
	{
#if 1
		EEARL = eep_addr;
		EECR |= (1 << EERE);
		uint8_t eep_data = EEDR;
#else
		uint8_t eep_data = eeprom_read_byte( (const uint8_t*)(uint16_t)eep_addr );
#endif
		if( eep_data != 0xff )
		{
			/* read second byte */
#if 1
			EEARL |= 1;
			EECR |= (1 << EERE);
			uint8_t eep_data2 = EEDR;
#else
			uint8_t eep_data2 = eeprom_read_byte( (const uint8_t*)(uint16_t)(eep_addr + 1) );
#endif
			/* put in state variables */
			state = eep_data & 0xf0;
			user_set_level = eep_data2 | ((eep_data & 0x0f) << 8);
			/* save address for later state save */
			eeprom_state_addr = eep_addr;
			break;
		}
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
	// FIXME: not true anymore if eeprom writes are in here
	TCNT0 = 0;
	counter_high = 0;

	if( ++dog_count == 2 )
	{
		dog_count = 0;
		if( state == STATE_RAMP_UP )
		{
			++user_set_level;
			if( user_set_level >= USER_LEVEL_MAX )
			{
				state = STATE_STEADY;
			}
		}
		if( state == STATE_RAMP_DOWN )
		{
			--user_set_level;
			if( user_set_level <= USER_LEVEL_MIN )
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
	for( uint8_t i = 15; i < otc_value; i += 16 )
	{
		flash();
		_delay_ms( 200 );
	}
#endif

#if 0
	flash_debug( state );
	state = 0x55;
	_delay_ms( 5000 );
	if( (MCUSR & (1 << BORF)) )
	{
		flash();
	}
	while( 1 == 1 );
#endif

	/* read previous state */
	//load_state_from_eeprom();

#if 0
	if( state != 0xff )
	{
		/* short press */
		if( state == STATE_RAMP_UP )
			state = STATE_STEADY;
		if( state == STATE_STEADY )
			state = STATE_RAMP_UP;
	}
	else
	{
		user_set_level = USER_LEVEL_MIN;
		state = STATE_RAMP_UP;
	}
#endif

	/* Safety. Should be removed eventually. */
	if( user_set_level >= USER_LEVEL_MAX )
		user_set_level = USER_LEVEL_MAX;

#if 1
	if( otc_value > 240 && mem_check == 0x55 )
	{
		/* short press */
		if( state == STATE_RAMP_UP || state == STATE_RAMP_DOWN )
			state = STATE_STEADY;
		else if( state == STATE_STEADY )
			state = STATE_RAMP_UP;
	}
	else
	{
		mem_check = 0x55;
		load_state_from_eeprom();

		if( otc_value > 180 )
		{
			/* long press */
			state = STATE_RAMP_DOWN;
		}
		else
		{
			/* longer time off (not a press) */
			user_set_level = USER_LEVEL_MIN;
			state = STATE_RAMP_UP;
		}
	}

	/* Save new state to eeprom. */
	/* FIXME: Should be in short press if above but it's 4 more words (!) */
	save_state_to_eeprom();
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
				TODO: Use one-way offset if a code word must be saved.
				TODO: See if this needs to be larger to avoid flicker.
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
		uint8_t prev_state = state;
		sleep_mode();

		/* If state has changed, save the new one to eeprom. */
		if( state != prev_state )
		{
			save_state_to_eeprom();
		}
	}
}
