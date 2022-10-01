# Dome Controller Interface Board

This is the sketch for the Roam-A-Dome-Home (RDH) setup.

![Roam-a-dome-home](https://raw.githubusercontent.com/reeltwo/DomeControlFirmware/main/images/RDH-PCBs.jpg)

## Libraries Used

<ul>
<li>https://github.com/reeltwo/Reeltwo</li>
<li>https://github.com/adafruit/Adafruit_NeoPixel</li>
<li>https://github.com/adafruit/Adafruit-GFX-Library</li>
<li>https://github.com/adafruit/Adafruit_SSD1306</li>
<li>https://github.com/adafruit/Adafruit_BusIO</li>
<li>https://github.com/rimim/espsoftwareserial</li>
<li>https://github.com/reeltwo/PCF8574</li>
</ul>

## Build using Arduino IDE

The firmware should build with esp32 board package 1.0.6 or 2.0.5. For ESP32S2 support 2.0.5 is required.

## Build using command line tools (Mac)

    cd ~/Documents/Arduino
    curl -o Arduino.mk https://raw.githubusercontent.com/reeltwo/Arduino/master/Arduino.mk
    # Fetch repository
    git clone https://github.com/reeltwo/DomeControlFirmware
    # Enter directory
    cd DomeControlFirmware
    # Build firmware for version 3 compact PCB
    make
    # Build firmware for version 4 with LVGL graphics display
    make TARGET=ESP32S3
    # Build firmware for full size Arduino Mega PCB (LCD and rotatry dial)
    make TARGET=Mega
    # Build firwmare for full size ESP32 PCB (LCD and rotatry dial)
    make FULLSIZE=1
    # Build firmware for barebones ESP32 (no display and no rotary dial)
    make LILYGO=1


## Build using command line tools (Linux)

    cd ~/Arduino
    wget https://raw.githubusercontent.com/reeltwo/Arduino/master/Arduino.mk
    # Fetch repository
    git clone  https://github.com/reeltwo/DomeControlFirmware
    # Enter directory
    cd DomeControlFirmware
    # Build firmware for version 3 compact PCB
    make
    # Build firmware for version 4 with LVGL graphics display
    make TARGET=ESP32S3
    # Build firmware for full size Arduino Mega PCB (LCD and rotatry dial)
    make TARGET=Mega
    # Build firwmare for full size ESP32 PCB (LCD and rotatry dial)
    make FULLSIZE=1
    # Build firmware for barebones ESP32 (no display and no rotary dial)
    make LILYGO=1

## Sample wiring diagram for Shadow based systems

![Wiring diagram for Shadow](https://raw.githubusercontent.com/reeltwo/DomeControlFirmware/main/images/RDH-Shadow.png)

## Video of RDH in action

Configuration can happen through either the LCD display using the encoder wheel or via serial.

[![RDH](https://i.vimeocdn.com/video/1432876855-e1a8651d23573b8ac84be2ccedc1b2b981c0a1533f0eff7b665f34ed3dbf8920-d_500x800)](https://vimeo.com/708875980)

## Serial Configuration commands

`#DPZERO`:
Factory reset all settings in EEPROM
*Examples*:

    #DPZERO

`#DPSETUPVELOCITY`[number]:
Store the preferred maximum angular velocity for `#DPSETUP` command. Default value is 45 cm/s
*Examples*:

    #DPSETUPVELOCITY80 (set preferred maximum angular velocity to 80 cm/s)

`#DPSETUP`:
Inital setup. Will figure out minimum speed and if inversion is needed.
*Examples*:

    #DPSETUP

`#DPDEBUG`[1|0]:
**ESP32** Enables/disables debug output in dome positioning code.
*Examples*:

    #DPDEBUG1 (enable debug output)
    #DPDEBUG0 (disable debug output)

`#DPWIFI`[1|0]:
**ESP32** Enables/disables WiFi support.
*Examples*:

    #DPWIFI1 (enable WiFi support)
    #DPWIFI0 (disable WiFi support)

`#DPREMOTE`[1|0]:
**ESP32** Enables/disables droid remote support.
*Examples*:

    #DPREMOTE1 (enable droid remote support)
    #DPREMOTE0 (disable droid remote support)

`#DPRNAME`[string]:
**ESP32** Change the droid remote host name. The default name is "RoamADome".
*Examples*:

    #DPRNAMEMyDome (changes reported name for droid remote to "MyDome")

`#DPRSECRET`[string]:
**ESP32** Change the droid remote secret string. The default secret is "Astromech". If you plan on leaving the droid remote support enabled outside your workshop you should change the shared secret so it is not affected by other droids.
*Examples*:

    #DPRSECRETMySecret (changes shared secret for droid remote to "MySecret")

`#DPCONFIG`:
List all settings in EEPROM
*Examples*:

    #DPCONFIG

`#DPSTATUS`:
Display status information for the dome controller.
*Examples*:

    #DPSTATUS

`#DPRESTART`:
Reboot the controller.
*Examples*:

    #DPRESTART

`#DPL`:
List all stored sequences
*Examples*:

    #DPL

`#DPD`[number]:
Delete numbered sequence from EEPROM
*Examples*:

    #PD0 (delete sequence 0)

`#DPMAXSPEED`[number]:
Set dome max speed 0-100 (default 50)
*Examples*:

    #DPMAXSPEED50 (set dome speed to 50)

`#DPHOMESPEED`[number]:
Set dome home mode speed 0-100 (default 40)
*Examples*:

    #DPHOMESPEED40 (set dome home speed to 40)

`#DPAUTOSPEED`[number]:
Set dome random mode auto speed 0-100 (default 30)
*Examples*:

    #DPAUTOSPEED35 (set dome auto speed to 35)

`#DPTARGETSPEED`[number]:
Set dome target mode auto speed 0-100 (default 100)
*Examples*:

    #DPTARGETSPEED65 (set dome target speed to 65)

`#DPMINSPEED`[number]:
Set dome minimum speed 0-100 (default 15)
*Examples*:

    #DPMINSPEED15 (set dome minimum speed to 15)

`#DPAUTOLEFT`[number]:
Set dome maximum distance to auto left 0-180 (default 80)
*Examples*:

    #DPAUTOLEFT80 (set dome auto left distance to 80)

`#DPAUTORIGHT`[number]:
Set dome maximum distance to auto right 0-180 (default 80)
*Examples*:

    #DPAUTORIGHT80 (set dome auto left distance to 80)

`#DPAUTOMIN`[number]:
Set random auto mode minimum delay in seconds (default 6)
*Examples*:

    #DPAUTOMIN6 (set minimum random mode delay to 6 seconds)

`#DPAUTOMAX`[number]:
Set random auto mode maximum delay in seconds (default 8)
*Examples*:

    #DPAUTOMAX8 (set maximum random mode delay to 8 seconds)

`#DPHOMEMIN`[number]:
Set home mode minimum delay in seconds (default 6)
*Examples*:

    #DPHOMEMIN6 (set minimum home mode delay to 6 seconds)

`#DPHOMEMAX`[number]:
Set home mode maximum delay in seconds (default 8)
*Examples*:

    #DPHOMEMAX8 (set maximum home mode delay to 8 seconds)

`#DPTARGETMIN`[number]:
Set target mode minimum delay in seconds (default 0)
*Examples*:

    #DPTARGETMIN6 (set minimum target mode delay to 6 seconds)

`#DPTARGETMAX`[number]:
Set target mode maximum delay in seconds (default 1)
*Examples*:

    #DPTARGETMAX8 (set maximum random mode delay to 8 seconds)

`#DPFUDGE`[number]:
Set dome target position tolerance to 0-20 degrees (default 5)
*Examples*:

    #DPFUDGE5 (set dome position tolerance to within 5 degrees)

`#DPHOMEPOS`[number]:
Set new dome home position to the current position. Or optionally specify the home position in 0-359 degrees
*Examples*:

    #DPHOMEPOS (set new dome home position to the current position)
    #DPHOMEPOS10 (set new dome home position to 10)

`#DPHOME`[0|1]:
Enable/disable dome home mode 0 or 1 (default disabled 0)
*Examples*:

    #DPHOME0 (disable dome home mode)
    #DPHOME1 (enable dome home mode)

`#DPAUTO`[0|1]:
Enable/disable dome random auto mode 0 or 1 (default disabled 0)
*Examples*:

    #DPAUTO0 (disable dome random auto mode)
    #DPAUTO1 (enable dome random auto mode)

`#DPSCALE`[0|1]:
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

`#DPINVERT`[0|1]:
Enable/disable dome motor inversion 0 or 1 (default disabled 0)
*Examples*:

    #DPINVERT0 (disable dome motor inversion)
    #DPINVERT1 (enable dome motor inversion)

`#DPAUTOSAFETY`[0|1]:
Enable/disable automatic dome movement safety check 0 or 1 (default disabled 0)
*Examples*:

    #DPAUTOSAFETY0 (disable automatic dome movement safety check)
    #DPAUTOSAFETY1 (enable automatic dome movement safety check)

`#DPTIMEOUT`[number]:
Set movement timeout delay in seconds. If dome has not changed position before the timeout expires the movement is cancelled (default 5 seconds)
*Examples*:

    #DPTIMEOUT5 (set timeout value to 5 seconds)

`#DPSERIALIN`[0|1]:
Enable/disable packet serial input 0 or 1 (default enabled 1)
*Examples*:

    #DPSERIALIN0 (disable packet serial input)
    #DPSERIALIN1 (enable packet serial input)

`#DPSERIALOUT`[0|1]:
Enable/disable packet serial output 0 or 1 (default enabled 1)
*Examples*:

    #DPSERIALOUT0 (disable packet serial output)
    #DPSERIALOUT1 (enable packet serial output)

`#DPPWMIN`[0|1]:
Enable/disable PWM input 0 or 1 (default disabled 0)
*Examples*:

    #DPPWMIN0 (disable PWM input)
    #DPPWMIN1 (enable PWM input)

`#DPPWMOUT`[0|1]:
Enable/disable PWM output 0 or 1 (default disabled 0)
*Examples*:

    #DPPWMOUT0 (disable PWM output)
    #DPPWMOUT1 (enable PWM output)

`#DPSERIALBAUD`[number]:
Specify command serial baud rate 2400,9600,19200,384000 (default 9600)
*Examples*:

    #DPSERIALBAUD9600 (set command Serial baud rate to 9600)

`#DPSABERBAUD`[number]:
Specify command Syren/Sabertooth packet serial baud rate 2400,9600,19200,384000 (default 9600)
*Examples*:

    #DPSABERBAUD9600 (set Syren/Sabertooth baud rate to 9600)

`#DPSABERADDRIN`[number]:
Specify command Syren/Sabertooth packet serial input address (default 129)
*Examples*:

    #DPSABERADDRIN129 (set Syren/Sabertooth input address to 129)

`#DPSABERADDROUT`[number]:
Specify command Syren/Sabertooth packet serial output address (default 129)
*Examples*:

    #DPSABERADDROUT129 (set Syren/Sabertooth output address to 129)

`#DPSENSORBAUD`[number]:
Specify command serial baud rate 57600 or 115200 (default 115200)
*Examples*:

    #DPSENSORBAUD57600 (set command Serial baud rate to 57600)
    #DPSENSORBAUD115200 (set command Serial baud rate to 115200)

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
    #DPS4:H:R100:WR10,20:R-50:WR10,20:WR10,20:R-50:WR10,20:H  (rotate dome right for 10-20 seconds then left for 10-20 seconds then back home)
    #DPS5:H:D-90:D180:H           (look left then look right)

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

    :DPS1      (play sequence 1)
    :DPS2      (play sequence 2)

### Wait number of seconds
`W`[R]seconds:
Wait for specified number of seconds. If 'R' is specified it will randomize the wait time in the range of 1..seconds. 'seconds' is a number between 1 and 600.
*Examples*:

	:DPW2      (wait 2 seconds before executing next sequence)
	:DPWR      (wait random 1-6 seconds)
	:DPWR10    (wait random 1-10 seconds)
    :DPWR10,20 (wait random 10-20 seconds)

### Home
`H`[R][speed]:
Rotates the dome into the home position.
*Examples*:

	:DPH     (go home)

### Rotate dome relative degrees
`D`[R]:
Rotate dome random relative degrees
*Examples*:

	:DPDR

`A`[R][,speed][,maxspeed]:
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

