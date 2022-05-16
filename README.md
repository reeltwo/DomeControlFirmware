# Dome Controller Interface Board

This is the sketch for the Dome Controller Interface Board. It depends on the Adafruit_SSD1306, Adafruit_GFX, and the Reeltwo library. You can download the latest release of the Reeltwo library here: https://github.com/reeltwo/Reeltwo/releases

[![Uppity Spinner](https://i.vimeocdn.com/video/1432876855-e1a8651d23573b8ac84be2ccedc1b2b981c0a1533f0eff7b665f34ed3dbf8920-d_500x800)](https://vimeo.com/708875980)

Configuration can happen through either the LCD display using the encoder wheel or via serial.

## Serial Configuration commands

`#DPZERO`:
Factory reset all settings in EEPROM
*Examples*:

    #DPZERO

`#DPCONFIG`:
List all settings in EEPROM
*Examples*:

    #DPCONFIG

`#DPL`:
List all stored sequences
*Examples*:

    #DPL

`#DPD`[number]:
Delete numbered sequence from EEPROM
*Examples*:

    #PD0 (delete sequence 0)

`#DPSPEED`[number]:
Set dome speed 0-100 (default 50)
*Examples*:

    #DPSPEED50 (set dome speed to 50)

`#DPHOMESPEED`[number]:
Set dome home mode speed 0-100 (default 40)
*Examples*:

    #DPHOMESPEED40 (set dome home speed to 40)

`#DPSEEKSPEED`[number]:
Set dome random mode seek speed 0-100 (default 30)
*Examples*:

    #DPSEEKSPEED35 (set dome seek speed to 30)

`#DPMINSPEED`[number]:
Set dome minimum speed 0-100 (default 15)
*Examples*:

    #DPMINSPEED15 (set dome minimum speed to 15)

`#DPSEEKLEFT`[number]:
Set dome maximum distance to seek left 0-180 (default 80)
*Examples*:

    #DPSEEKLEFT80 (set dome seek left distance to 80)

`#DPSEEKRIGHT`[number]:
Set dome maximum distance to seek right 0-180 (default 80)
*Examples*:

    #DPSEEKRIGHT80 (set dome seek left distance to 80)

`#DPSEEKMIN`[number]:
Set random seek mode minimum delay in seconds (default 6)
*Examples*:

    #DPSEEKMIN6 (set minimum random mode delay to 6 seconds)

`#DPSEEKMAX`[number]:
Set random seek mode maximum delay in seconds (default 8)
*Examples*:

    #DPSEEKMAX8 (set maximum random mode delay to 8 seconds)

`#DPFUDGE`[number]:
Set dome target position tolerance to 0-20 degrees (default 5)
*Examples*:

    #DPFUDGE5 (set dome position tolerance to within 5 degrees)

`#DPHOMEPOS`[number]:
Set new dome home position 0-359 degrees (default 0)
*Examples*:

    #DPHOMEPOS10 (set new dome home position)

`#DPHOME`[number]:
Enable/disable dome home mode 0 or 1 (default disabled 0)
*Examples*:

    #DPHOME0 (disable dome home mode)
    #DPHOME1 (enable dome home mode)

`#DPSEEK`[number]:
Enable/disable dome random seek mode 0 or 1 (default disabled 0)
*Examples*:

    #DPSEEK0 (disable dome random seek mode)
    #DPSEEK1 (enable dome random seek mode)

`#DPSCALE`[number]:
Enable/disable dome speed scaling/ramping (default disabled 0)
*Examples*:

    #DPSCALE0 (disable dome speed ramping mode)
    #DPSCALE1 (enable dome speed ramping mode)

`#DPASCALE`[number]:
Set dome speed acceleration scale to 0-255 (default 100). Lower value means accelerate faster.
*Examples*:

    #DPASCALE100 (set dome acceleration scale to 100)

`#DPDSCALE`[number]:
Set dome speed acceleration scale to 0-255 (default 100). Lower value means decelerate faster.
*Examples*:

    #DPDSCALE100 (set dome acceleration scale to 20)

`#DPINVERT`[number]:
Enable/disable dome motor inversion 0 or 1 (default disabled 0)
*Examples*:

    #DPINVERT0 (disable dome motor inversion)
    #DPINVERT1 (enable dome motor inversion)

`#DPSERIALIN`[number]:
Enable/disable packet serial input 0 or 1 (default enabled 1)
*Examples*:

    #DPSERIALIN0 (disable packet serial input)
    #DPSERIALIN1 (enable packet serial input)

`#DPSERIALOUT`[number]:
Enable/disable packet serial output 0 or 1 (default enabled 1)
*Examples*:

    #DPSERIALOUT0 (disable packet serial output)
    #DPSERIALOUT1 (enable packet serial output)

`#DPPWMIN`[number]:
Enable/disable PWM input 0 or 1 (default disabled 0)
*Examples*:

    #DPPWMIN0 (disable PWM input)
    #DPPWMIN1 (enable PWM input)

`#DPPWMOUT`[number]:
Enable/disable PWM output 0 or 1 (default disabled 0)
*Examples*:

    #DPPWMOUT0 (disable PWM output)
    #DPPWMOUT1 (enable PWM output)

`#DPMARCBAUD`[number]:
Specify Marcduino serial baud rate 2400,9600,19200,384000 (default 9600)
*Examples*:

    #DPMARCBAUD9600 (set Marcduino baud rate to 9600)

`#DPPIN`[pin][number]:
Specify default value (0,1) for pin (1-8)
*Examples*:

    #DPPIN10 (set pin 1 to 0)
    #DPPIN21 (set pin 2 to 1)

`#DPJOY`:
Enable VT100 serial emulation joystick for debugging. Press Home key to enable home mode. Press End to end emulation mode. Page Up increase speed. Page Down decrease speed. Press R or r to enable random mode. Arrow keys Left and Right will move the dome.

*Examples*:

    #DPJOY

`#DPS`[number]:seq
Store specified sequence (seq) as number in EEPROM.
*Examples*:

	#DPS1:H                       (home)
	#DPS2:A100                    (move dome to absolute 100 degrees)
	#DPS3:D50:W2:D-50             (move dome 50 degrees, wait 2 seconds, move -50 degrees)

Dome commands
=============

Dome commands starts with ':DP' followed by one of the these. Multiple commands can be issued seperated by a colon. Only the first command needs to be prefixed with 'DP':

	:DPA90:W2:H

Would turn the dome to 90 degrees absolute. Wait 2 seconds. Return dome to home position:

	:DPA90
	:DPW2
	:DPH

### Play stored sequence

`S`[number]:
Sequences can be stored using #DPS
*Examples*:

    #DPS0:H:A90:W2:A270:W2:H

### Wait number of seconds
`W`[R]seconds:
Wait for specified number of seconds. If 'R' is specified it will randomize the wait time in the range of 1..seconds. 'seconds' is a number between 1 and 600.
*Examples*:

	:DPW2    (wait 2 seconds before executing next sequence)
	:DPWR    (wait random 1-6 seconds)
	:DPWR10  (wait random 1-10 seconds)

### Home
`H`[speed]:
Rotates the dome into the home position.
*Examples*:

	:DPH     (go home)

### Rotate dome relative degrees
`D`[R]:
Rotate dome random relative degrees
*Examples*:

	:DPDR

`D`[degrees]:
Rotate dome relative degrees positive for counter clockwise and negative for clockwise
*Examples*:

	:DPD90       (rotate dome relative 90 degrees counter clockwise)
	:DPD-90      (rotate dome relative 90 degrees clockwise)

### Rotate dome absolute degrees
`A`[R][,speed][,maxspeed]:
Rotate dome random absolute degrees
*Examples*:

	:DPAR

`A`[degrees][,speed][,maxspeed]:
Rotate dome to absolute position degrees. Optionally specify the speed percentage and maximum speed percentage. By default the speed used is minimum rotary power (20%). The maximum speed defaults to the same value as speed if omitted.
*Examples*:

	:DPA90        (rotate dome to 90 degrees clockwise)
	:DPA270       (rotate dome to 270 degrees clockwise)
	:DPA-90       (rotate dome to 270 degrees clockwise)
	:DPA0         (rotate dome to home position)
	:DPA90,20,100 (rotate dome to 90 degrees starting at 20% speed increasing to 100%)

### Rotate dome continously
`R`[speed]:
Spin dome at specified speed percentage
*Examples*:

	:DPR30       (spin dome at 30% speed counter clockwise)
	:DPR-30      (spin dome at 30% speed clockwise)

`T`[pin]:
Toggle digital pin (1-8)
*Examples*:

	:DPT1        (toggle digital pin 1)
	:DPT6        (toggle digital pin 6)

`P`[pin][value]:
Set digital pin (1-8) to value (0-1)
*Examples*:

	:DPP10       (set digital pin 1 to LOW)
	:DPP21       (set digital pin 2 to HIGH)

`Z`:
Restore settings to default (stored in EEPROM). Useful for reseting digital pins to default position.
*Examples*:

	:DPZ         (restore settings to default)

