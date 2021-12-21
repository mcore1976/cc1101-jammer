Example of using CC1101 module with Arduino Pro Micro ( ATMEGA32U4, 3.3V / 8MHz version ) for radio jamming

This device allows for programmable jamming of sub 1GHz frequency with 10mW signal and selected modulation type and payload.
In this example pseudo random payloads (1024 bytes) are used to feed CC1101 frames (64 bytes long)  that will be send on particular frequency ( white noise ).
Frames are transmitted on multiple channels with frequency hopping or you may configure the code to send only on single frequency for better jamming.
The code has predefined settings for following base ISM  frequencies : 315 MHz, 433 MHz, 868 MHz, 915 MHz..
Please take into account that jamming even ISM frequency may be illegal in your country !

Used Arduino Pro Micro board ( ATMEGA32U4 chip ) must support 3.3Volt VCC and 3.3V TTL logic because this is required by CC1101 board, otherwise you will fry CC1101 chip.
Please follow this guide to setup your Arduino environment for Arduino Pro Micro board : https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide/all
If you are having issues with uploading the code from Arduino IDE to the board, after pressing "Upload" in Arduino you have to immediatelly short GND+RST pins two times in few seconds. Then bootloader in Arduino Pro Micro will start (common issue) and upload will begin.

Connections to be made :

ARDUINO PRO MICRO 3.3V / 8MHz <-> CC1101 BOARD
- DIGITAL PIN 3  ( PD0 / INT0 ) <-> CC1101 GDO0
- DIGITAL PIN 9  ( PB5 )        <-> CC1101 GDO2
- DIGITAL PIN 10 ( PB6 )        <-> CC1101 CSN / CS / SS
- DIGITAL PIN 16 ( PB2 / MOSI ) <-> CC1101 MOSI
- DIGITAL PIN 14 ( PB3 / MISO ) <-> CC1101 MISO
- DIGITAL PIN 15 ( PB1 / SCK )  <-> CC1101 SCLK / CLK 

Please use SMARTRFTM studio to fine tune CC1101 registers to particular : base frequency, modulation, output power etc..
https://www.ti.com/tool/SMARTRFTM-STUDIO


The video showing how the jammer work is available here : https://www.youtube.com/watch?v=vZcGP-O2GvQ
