Example of using CC1101 module with Arduino Pro Micro ( ATMEGA32U4 ) for radio jamming

This device allows for programmable jamming of sub 1GHz frequency with 10mW signal and selected modulation.
Pseudo random payloads are used to feed CC1101 frames that will be send on particular frequency ( white noise ).
Frames are transmitted on multiple channels with frequencyu hopping.
The code has predefined settings for following base ISM  frequencies : 315 MHz, 433 MHz, 868 MHz..

Used Arduino Pro Micro board ( ATMEGA32U4 chip ) must support 3.3Volt VCC and 3.3V TTL logic because this is required by CC1101 board, otherwise you will fry CC1101 chip.

Connections to be made :

- ARDUINO PRO MICRO 3.3V / 8MHz <-> CC1101 BOARD
- DIGITAL PIN 3  ( PD0 / INT0 ) <-> CC1101 GDO0
- DIGITAL PIN 9  ( PB5 )        <-> CC1101 GDO2
- DIGITAL PIN 10 ( PB6 )        <-> CC1101 CSN / CS / SS
- DIGITAL PIN 16 ( PB2 / MOSI ) <-> CC1101 MOSI
- DIGITAL PIN 14 ( PB3 / MISO ) <-> CC1101 MISO
- DIGITAL PIN 15 ( PB1 / SCK )  <-> CC1101 SCLK / CLK 

Please use SMARTRFTM studio to fine tune CC1101 registers to particular : base frequency, modulation, output power etc..
https://www.ti.com/tool/SMARTRFTM-STUDIO

