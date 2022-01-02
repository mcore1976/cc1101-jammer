Example of using CC1101 module with Arduino Pro Micro ( ATMEGA32U4, 3.3V / 8MHz version ) for radio jamming. Alternatively Arduino Pro Mini (ATMEGA328P 3.3V / 8MHz) may be used.

This device allows for programmable jamming of sub 1GHz frequency with 10mW signal of selected modulation type and payload.
In this example pseudo random payloads (1024 bytes) are used to feed CC1101 frames (64 bytes long)  that will be send on particular frequency ( white noise ).
Frames are transmitted on multiple channels with frequency hopping or you may configure the code to send only on single frequency for better jamming.
The code has predefined settings for following base ISM  frequencies : 315 MHz, 433 MHz, 868 MHz, 915 MHz.. The program also allows to select custom transmission frequency within 300-348 MHz, 387-464 MHz and 779-928 MHz ranges. Please use SMARTRFTM studio to fine tune CC1101 registers to your needs for base frequency, modulation, output power etc.. https://www.ti.com/tool/SMARTRFTM-STUDIO  
By adding USB serial port handling you may extend this software to support similar features to Yardstick One.

Attached 'version 2' of jammer code uses SmartRC library (modified Electrohouse library by Little_S@tan) which allows to customize ALL transmission parameters in human readable format without using SmartRF studio from TI. To use this version of INO script , ZIP library from following github link https://github.com/LSatan/SmartRC-CC1101-Driver-Lib  has to be attached to the script in Arduino IDE.

Please take into account that jamming even ISM frequency may be illegal in your country !

---

Arduino Pro Micro board ( ATMEGA32U4 chip ) must support 3.3Volt VCC and 3.3V TTL logic because this is required by CC1101 board, otherwise you will fry CC1101 chip.
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

---

OPTIONALLY you may use ARDUINO PRO MINI (ATMEGA328P  3.3V / 8 MHz version ). In that case you would have to use additional FTDI232 adapter to be able to program ARDUINO PRO MINI from Arduino IDE environment ( connected to TXD, RXD, GND pins )

ARDUINO PRO MINI 3.3V / 8MHz <-> CC1101 BOARD
- DIGITAL PIN 2  ( PD2 / INT0 ) <-> CC1101 GDO0
- DIGITAL PIN 9  ( PB1 )        <-> CC1101 GDO2
- DIGITAL PIN 10 ( PB2 )        <-> CC1101 CSN / CS / SS
- DIGITAL PIN 11 ( PB3 / MOSI ) <-> CC1101 MOSI
- DIGITAL PIN 12 ( PB4 / MISO ) <-> CC1101 MISO
- DIGITAL PIN 13 ( PB5 / SCK )  <-> CC1101 SCLK / CLK 

---

The video showing how the jammer work is available here : https://www.youtube.com/watch?v=vZcGP-O2GvQ
