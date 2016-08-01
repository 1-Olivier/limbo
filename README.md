# limbo
This is flashlight firmware for attiny based driver with a FET.

## Requirements

### Driver hardware
* An attiny13a
* FET
* Voltage divider

OTC and 7135 may be present but are unused for now.

### Software
It builds on linux so far. Developed on Arch Linux but should build on any linux setup with avr-gcc. Untested on windows so there are no project files obviously.

## Building

To build and flash the firmware, just run 'make flash' and 'make flash_fuses'. Each driver is a little different so you will likely need to tweak the USER_LEVEL_MIN and USER_LEVEL_MAX defines in the code to set the low and high limit of the ramp. The low limit can be whatever you want but the high limit should be set such that you still see a visible step to turbo, or lower if turbo is disabled. Setting it higher will make thermal control not work well as part of the ramp will have no visible effect on light output. Setting it lower will just cause a large step from turbo but is not too harmful otherwise.

This was tested on the BLF A6 driver so will need to change some constants if you use a driver with a different voltage divider. See include/cell_levels.h.

## Features

* No PWM but there is a very short gap in the output every 16ms. Not visible as far as I can tell.
* Low voltage protection which is measured under load and fairly accurate. Set to 3.0V by default. See ADC_CELL_LOWEST in the source.
* Its lowest level can be set lower than you could possibly want it.
* Should be fairly power efficient but I don't have the equipment to measure it yet.

### Interface
The interface is fairly simple as there are few features. The light turns on at a set level, ramping toward the max level. A single click on the switch will stop any ramp at the current level. A single click while it is not ramping will resume the ramp toward the max level. A double click (two clicks within about 0.3s of each other) will make it ramp toward the lowest level, at half the speed.

The only soft configuration is thermal limit calibration. This is currently always active so it must be calibrated or the light might never go to high levels. Calibration is done by a series of quick clicks (8) until the light comes on at maximum output. Let it heat up until it is where you want the limit to be and give it a click. Then turn it off and wait several seconds.

### Known issues
* The output flickers a little. It's not visible for normal use but you might see it if you shine the light at a white wall or ceiling bounce it. This makes the firmware not that great as a reading light if you're sensitive to such things. I suspect either clock jitter or some randomness in the FET.
