/*
	Typical driver layout.

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

#if defined(__AVR_ATtiny13A__)
#	define F_CPU 4800000UL
#else
#	define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

/* Fuses */
#if defined(__AVR_ATtiny13A__)
/*
	low fuse:
	- Enable programming & download.
	- 4 ms startup delay.
	- 4.8 MHz clock.
	high fuse:
	- Enable Brown-Out Detection at 1.8V.
*/
FUSES =
{
	.low = FUSE_SPIEN & FUSE_SUT1 & FUSE_CKSEL1,
	.high = FUSE_BODLEVEL0
};
#elif defined(__AVR_ATtiny25__)
/*
	low fuse:
	- 4 ms startup delay.
	- 8 MHz clock
	high fuse:
	- Enable programming & download.
	- Enable Brown-Out Detection at 1.8V.
*/
FUSES =
{
	.low = FUSE_SUT1 & FUSE_CKSEL3 & FUSE_CKSEL2 & FUSE_CKSEL0,
	.high = FUSE_SPIEN & FUSE_BODLEVEL0,
	.extended = EFUSE_DEFAULT
};
#else
#error no fuse definitions
#endif

#include "../include/delay_accurate.h"
#include "../include/division.h"
#include "../include/cell_levels.h"

/* To check memory decay. */
uint8_t mem_check __attribute__ ((section (".noinit")));

/* Number of clicks without significant delay between them. */
uint8_t click_count __attribute__ ((section (".noinit")));

/* WDT interrupt counter since boot. */
uint8_t wdt_count;

/* Output level which the user selected. */
uint16_t user_set_level __attribute__ ((section (".noinit")));
#define USER_LEVEL_MIN 120
#define USER_LEVEL_MAX 450
#define USER_LEVEL_START (USER_LEVEL_MIN + 30)
/* Level above which temperature control is enabled. Undefine to disable. */
#define TEMPERATURE_THRESHOLD_LEVEL 180
/* When defined, USER_LEVEL_MAX is a turbo (full on). Comment this to use a
   lower max level without turbo (eg. on a light which can't handle turbo). */
#define MAX_LEVEL_IS_TURBO

/*
	Output level we're actually using, after taking into account low voltage
	protection, temperature control, etc.
*/
uint16_t output_level __attribute__ ((section (".noinit")));

/*
	Uses only the high nibble. Important for eeprom read/write as this gets
	merged with the high byte of the level (of which only the low nibble is
	used).
*/
uint8_t g_state __attribute__ ((section (".noinit")));
#define STATE_RAMP_UP 0
#define STATE_STEADY 1
#define STATE_RAMP_DOWN 2
#define STATE_THERMAL_CONFIG 3

/* Last ADC cell voltage readout. */
volatile uint8_t cell_level;
#define ADC_CELL_100 ADC8_FROM_CELL_V( 420 )
#define ADC_CELL_LOWEST ADC8_FROM_CELL_V( 300 )

/* 0 = no, 1 = start */
uint8_t do_power_adc;

volatile uint8_t g_temperature_limit __attribute__ ((section (".noinit")));
volatile uint8_t g_current_temperature __attribute__ ((section (".noinit")));
int8_t g_temperature_window_low;
int8_t g_temperature_window_high;
#define TEMPERATURE_WINDOW_ADJUST_DELAY 8
uint8_t g_seconds_until_window_adjust = 0;
#define TEMPERATURE_CONFIG_ADDRESS 1

static void empty_gate()
{
	/* enable output, off to empty gate  */
	DDRB |= (1 << DDB1);
	// TODO: See how many are really needed to flush the gate at high power levels.
	NOP2;
	/* that should be more than enough */
	DDRB &= ~(1 << DDB1);
}

inline static
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

inline static
void apply_output_level( uint16_t level )
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
	uint16_t gate_level = div_u16_u8_sl8( level, local_cell_level );
	gate_level = div_u16_u8_sl8( gate_level, local_cell_level );
#endif
	empty_gate();
	// FIXME: could this underflow with invalid (0) user_set_level?
	charge_gate( (gate_level >> 1) - 6u );
#ifdef MAX_LEVEL_IS_TURBO
	/* Quick hack to make sure max really is max. */
	if( level == USER_LEVEL_MAX )
	{
		PORTB |= (1 << PORTB1);
	}
#endif
}

/* flash the 7135 channel to say we're running */
static
void flash()
{
	PORTB |= (1 << PORTB0);
	_delay_us( 1 );
	PORTB &= ~(1 << PORTB0);
}

/* flash an 8-bit value on the 7135 channel */
static
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
		_delay_ms( 300 );
	}
}

/*
	Helper to write to eeprom.
*/
static
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

/* Watchdog interrupt. */
ISR( WDT_vect )
{
	uint8_t temperature = TCNT0;
	/*
		It's ok to reset this right now because charge_gate() won't take long
		enough for TCNT0 to overflow. It's also better to do it close to the
		start of the interrupt handler as that's what the watchdog timer
		measures.
	*/
	TCNT0 = 0;

	++wdt_count;

	/* Reset click count after 320 ms */
	if( wdt_count == 20 )
		click_count = 0;

	/* Let temperature window drift back to its normal value. */
	if( (wdt_count & 0x3f) == 0 )
	{
		if( --g_seconds_until_window_adjust == 0 )
		{
			if( g_temperature_window_low < -2 )
				++g_temperature_window_low;
			if( g_temperature_window_high > 2 )
				--g_temperature_window_high;
			g_seconds_until_window_adjust = TEMPERATURE_WINDOW_ADJUST_DELAY;
		}
	}

	/* This weird step is to speed up the ramp a little at the high end. */
	uint8_t level_step = (user_set_level >> 8) + 1u;
	uint8_t state = g_state;
	if( state == STATE_RAMP_UP )
	{
		user_set_level += level_step;
		if( user_set_level >= USER_LEVEL_MAX )
		{
			state = STATE_STEADY;
		}
	}
	/* Ramp down more slowly than we ramp up. */
	if( state == STATE_RAMP_DOWN && (wdt_count & 0x2) != 0 )
	{
		user_set_level -= level_step;
		if( user_set_level <= USER_LEVEL_MIN )
		{
			state = STATE_STEADY;
		}
	}
	g_state = state;

	/*
		Now figure out the actual output level from the requested level. It
		will go down if either:
		- The user requested a lower level.
		- The cell voltage is too low.
		- The temperature is too high.

		It will go up if none of the above conditions occur and the user
		requested a higher level.

		This fairly primitive control for low voltage protection works well
		enough because the cell responds quickly and we use a window around
		the ADC readout. This is enough to prevent oscillation.
	*/
	int8_t max_increase = cell_level - ADC_CELL_LOWEST;

	uint16_t local_output_level = output_level;

	/*
		If output level is high enough that we entered idle mode last time
		around, use the temperature readout.
	*/
#ifdef TEMPERATURE_THRESHOLD_LEVEL
	uint8_t temp_offset = 0;
	if( local_output_level >= TEMPERATURE_THRESHOLD_LEVEL )
	{
		/*
			Update current temp with a small window as this is a noisy. The
			slow update (+/- 1 per WDT interrupt) nicely handles any initial
			bad reading we might get.
		*/
		uint8_t current_temp = g_current_temperature;
		int8_t d_temp = temperature - current_temp;
		uint8_t temperature_limit = g_temperature_limit;
		if( d_temp > g_temperature_window_high )
		{
			++current_temp;
			/*
				Increase window temporarily to slow down the intensity decrease.
				Only do it once above temp limit or we'll have increased it by
				a huge amount from cold to hot. Also do not do it while in
				thermal config mode or the recorded reading will be wrong.
			*/
			if( state != STATE_THERMAL_CONFIG &&
			    (int8_t)(current_temp - temperature_limit) > 0 )
			{
				++g_temperature_window_high;
			}
			g_seconds_until_window_adjust = TEMPERATURE_WINDOW_ADJUST_DELAY;
		}
		else if( d_temp < g_temperature_window_low )
		{
			--current_temp;
			/* Increase window temporarily to slow down the intensity increase. */
			--g_temperature_window_low;
			g_seconds_until_window_adjust = TEMPERATURE_WINDOW_ADJUST_DELAY;
		}
		g_current_temperature = current_temp;

		if( state != STATE_THERMAL_CONFIG )
		{
			int8_t temp_overshoot = current_temp - temperature_limit;
			if( temp_overshoot > 0 )
			{
				uint8_t uover = temp_overshoot;
				if( uover > 25 )
					uover = 25;

				temp_offset = uover << 3;
				if( uover > 6 )
					uover = 6;
				temp_offset += uover << 3;
				/*
					Largest temp_offset value will be 248. Would need a 16-bit
					variable to go larger but there's no point with the current
					level range. USER_LEVEL_MAX - 248 is too low to generate a
					lot of heat.
				*/
			}
		}
	}
#endif

	local_output_level += max_increase;
	if( user_set_level < local_output_level )
	{
		local_output_level = user_set_level;
	}

	uint16_t max_level = (uint16_t)USER_LEVEL_MAX - (uint16_t)temp_offset;
	if( local_output_level > max_level )
		local_output_level = max_level;

	/*
		If we reach the lower level, just turn the light off. Our sleep
		mode is already set to power down and interrupts are already
		disabled because we're in an interrupt handler. So we just disable
		the watchdog and go to sleep.
	*/
	if( local_output_level < USER_LEVEL_MIN - 10 ) // FIXME: Avoid overshooting user range.
	{
		/* enable output, off to empty gate  */
		DDRB |= (1 << DDB1);
		/* disable watchdog timer */
		WDTCR &= ~(1 << WDTIE);
		/* power down MCU */
		sleep_mode();
	}

	/* Write back to memory. */
	output_level = local_output_level;

	/*
		At higher power levels, temperature control is enabled and we can only
		enter idle mode here because clk I/O must remain active to count
		cycles.

		At lower levels, we don't bother with the temperature so we can power
		down the MCU which could otherwise use more power than the light
		itself. This mode setting is also important to the low voltage
		protection's final shutdown of the light.

		TODO: optimize this. It probably wastes an insane amount of code
		space.
	*/
#ifdef TEMPERATURE_THRESHOLD_LEVEL
	if( local_output_level >= TEMPERATURE_THRESHOLD_LEVEL )
	{
		set_sleep_mode( SLEEP_MODE_IDLE );
	}
	else
#endif
	{
		set_sleep_mode( SLEEP_MODE_PWR_DOWN );
	}

	/* Set new light value. */
	apply_output_level( local_output_level );

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
	/*
		Assume largest possible value until we know better. This means the
		light will be less bright than it should be for the first WDT cycle,
		which is less visible than being too bright.
	*/
#define CELL_INIT_VALUE (ADC_CELL_LOWEST < 128 ? ADC_CELL_LOWEST + 127 : 255)
	cell_level = CELL_INIT_VALUE;

	/* Drain anything left in the gate from before click. Does not seem really
	   needed so far. */
	DDRB |= (1 << DDB1);

	/* initialize ADC to read cell level on ADC1 (PB2) */
	/* disable digital input */
	DIDR0 |= (1 << ADC1D);
	/* internal ref, left adjusted, ADC1 (PB2) */
	ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX0);
	/* prescaler is set later, when ADC is enabled. */

	/* Don't leave the OTC pin floating. Use pull-up. */
	PORTB |= (1 << PORTB3);

	/* enable 7135 output (useful for debugging) */
	/* also to make sure it doesn't produce crap */
	DDRB |= (1 << DDB0);

	/* Click starts from the actual output level (limited by temp or Vcc). */
	user_set_level = output_level;

	/* Safety. Should be removed eventually. */
	if( user_set_level >= USER_LEVEL_MAX )
		user_set_level = USER_LEVEL_MAX;

	uint8_t state = g_state;

	if( mem_check == 0x55 )
	{
		/* short press */
		++click_count;

		if( click_count == 1 )
		{
			if( state == STATE_RAMP_UP || state == STATE_RAMP_DOWN )
				state = STATE_STEADY;
			else if( state == STATE_STEADY )
				state = STATE_RAMP_UP;
			else if( state == STATE_THERMAL_CONFIG )
			{
				/* store current temperature as limit in eeprom */
				eeprom_write_byte(
					(uint8_t*)TEMPERATURE_CONFIG_ADDRESS,
					g_current_temperature );
			}
		}
		else if( click_count == 2 )
		{
			state = STATE_RAMP_DOWN;
		}
		else
		{
			state = STATE_STEADY;

			if( click_count > 10 )
			{
				state = STATE_THERMAL_CONFIG;
				user_set_level = USER_LEVEL_MAX;
			}
		}
	}
	else
	{
		/* cold start */
		mem_check = 0x55;
		user_set_level = USER_LEVEL_START;
		state = STATE_RAMP_UP;
		click_count = 0;
		/* read some config from eeprom */
		uint8_t temp_limit_config = eeprom_read_byte(
			(uint8_t*)TEMPERATURE_CONFIG_ADDRESS );
		g_temperature_limit = temp_limit_config;
		/* sane temperature value until we get a reading */
		g_current_temperature = temp_limit_config;
	}

	g_state = state;

	/* FIXME: too many load/stores here. */
	output_level = user_set_level;

	/* enable watchdog interrupt */
	WDTCR |= (1 << WDTIE);
	sei();

	/*
		start counter with clk / 64
		The frequency is so that the count between two watchdog interrupts fits
		in 16 bits, with room to spare. Yet it must be high enough that there
		is enough resolution to detect the desired changes.
	*/
	TCCR0B = (1 << CS01) | (1 << CS00);

	while( 1 == 1 )
	{
		// TODO: move this to function
		if( do_power_adc )
		{
			/*
				enable ADC, start conversion, prescaler = 1/64

				We should in theory give it a bit of time for the reference to
				settle but the extra cycles of the first conversion appear to
				be enough with the slow ADC clock used here. The result is very
				stable.
			*/
			ADCSRA = (1 << ADEN) | (1 << ADSC) | 0x6;
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

				When the reading is going lower, I only allow decrementing by
				one unit (per watchdog interrupt). This is still much faster
				than a cell can possibly drain itself (1 V/s). The reason is
				that when the power is cut (the switch it pressed), the MCU
				will keep running a little off the capacitor and read a value
				here which is much lower than the cell actually is. This messes
				with LVP code, causing the output to be dropped quickly and
				thus the click to appear to change output level. This is most
				visible at very low levels. This also appears to be the source
				of bright flashes on clicks, probably because of the voltage
				compensation when the output level is applied.

				The one exception to this rule is on startup where we want to
				use the first reading directly.

				This whole thing uses up 30 bytes :(

				TODO: Use one-way offset if a code word must be saved.
				TODO: See if this needs to be larger to avoid flicker.
			*/
			uint8_t read_level = ADCH;
			uint8_t rp1 = read_level + 1u;
			uint8_t rm1 = read_level - 1u;
			uint8_t local_cell_level = cell_level;
			if( local_cell_level == CELL_INIT_VALUE )
				local_cell_level = rp1;
			if( rp1 < local_cell_level )
				--local_cell_level;
			if( rm1 > local_cell_level )
				local_cell_level = rm1;
			cell_level = local_cell_level;
#endif
			ADCSRA &= ~(1 << ADEN);
			do_power_adc = 0;
		}

		/* Sleep or power down until next interrupt. */
		sleep_mode();
	}
}
