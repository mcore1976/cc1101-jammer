//
// CC1101 universal jammer Version 2
//
// (C) Adam Loboda '2021
//
// based on great SmartRC library by Little_S@tan
// Please download ZIP from 
// https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
// and attach it as ZIP library for Arduino
// This code will work with Arduino Pro Micro 3.3V 8MHz
//

#include <avr/io.h>
#include <avr/boot.h>
#include <inttypes.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/crc16.h>
#include <inttypes.h>


#include <ELECHOUSE_CC1101_SRC_DRV.h>

// buffer for TC CC data 
static volatile uint8_t ccsendingbuffer[64] = "12345678901234567890123456789012345678901234567890123456789012345678901234567890123\x00";


const uint8_t  randomized[] PROGMEM = 
  {
66,29,77,60,51,149,88,4,199,190,201,150,191,202,97,140,36,99,253,15,26,182,
95,60,150,7,173,205,37,120,216,225,50,227,192,222,17,129,178,4,173,235,139,
79,254,58,129,244,173,122,1,89,39,218,254,39,97,95,70,124,23,182,90,79,187,
183,199,11,39,252,76,111,68,143,231,204,151,203,119,30,100,5,30,192,3,118,
78,74,24,180,189,118,135,63,30,43,197,132,56,169,196,246,237,32,244,28,22,
215,63,79,207,251,211,81,185,23,203,243,190,149,203,2,139,204,228,212,204,
222,184,140,182,21,237,75,203,84,40,204,130,221,252,213,182,138,147,207,76,
89,77,147,250,60,221,218,80,102,19,98,42,196,238,147,154,177,182,134,239,
123,85,156,21,166,32,150,242,33,96,189,63,159,229,211,85,239,182,141,206,
123,227,255,249,169,99,87,176,157,128,168,86,66,39,195,24,220,82,162,124,
165,103,70,6,205,63,175,252,245,94,129,71,8,213,44,9,15,63,239,119,235,115,
7,224,32,15,192,209,227,190,59,98,40,156,179,17,165,111,235,226,243,90,44,
43,140,107,110,44,76,193,35,8,103,170,2,165,254,143,96,98,207,81,55,198,33,
113,205,141,191,37,147,164,1,194,119,90,13,189,159,214,56,142,248,181,114,
229,225,44,149,93,239,138,120,196,195,28,253,109,217,83,197,248,130,58,196,
18,182,254,53,245,159,167,170,17,58,16,38,212,38,152,246,45,21,17,25,78,94,
54,236,126,106,185,32,238,129,205,115,45,8,89,238,30,25,79,141,225,159,201,
51,213,142,111,8,144,196,34,86,214,179,87,55,145,53,100,246,194,233,16,206,
238,248,244,69,116,195,135,253,73,82,97,98,135,44,214,35,67,167,101,119,209,
213,242,100,185,245,167,197,34,158,191,62,178,168,127,1,236,38,191,161,179,
105,17,119,23,124,160,236,228,80,89,222,63,185,68,106,248,10,45,89,230,243,
175,205,41,149,99,68,140,11,34,106,126,241,40,123,70,102,141,112,111,254,208,
72,113,103,9,5,164,72,219,90,174,1,22,62,147,81,226,112,253,205,115,184,44,
86,11,12,64,40,183,127,127,101,2,86,107,47,79,166,137,92,101,28,123,244,55,
171,48,12,65,5,145,116,191,19,110,202,161,104,140,43,125,60,14,3,16,223,17,
134,42,8,17,199,199,155,137,183,111,0,206,61,239,119,124,186,56,14,219,27,15,
45,186,76,9,88,89,148,93,6,209,37,118,204,111,56,231,201,124,52,121,180,24,
213,19,244,174,84,106,207,109,64,114,72,222,234,147,156,125,139,196,160,206,
35,252,8,195,42,185,24,187,249,232,243,236,16,254,255,15,225,19,47,39,42,71,
223,15,153,86,119,32,157,77,219,89,100,208,107,191,22,155,10,106,132,35,10,
130,101,76,110,7,163,15,124,142,135,214,197,132,0,166,179,154,24,208,105,57,
253,34,254,50,155,207,208,22,105,240,221,95,87,49,125,148,64,58,159,22,197,
96,240,61,165,67,53,32,196,50,149,87,130,179,113,91,19,124,170,103,232,115,
90,114,131,176,79,175,251,127,253,255,118,250,225,255,28,167,39,158,5,209,
155,19,106,61,238,55,225,143,83,24,79,15,252,127,130,218,132,145,33,233,66,
185,148,61,121,6,219,194,3,22,42,42,137,247,129,202,255,230,174,251,123,178,
50,186,125,31,38,214,148,65,133,214,229,164,44,211,182,29,175,94,92,50,143,
171,134,48,68,214,212,48,216,144,181,186,200,172,216,50,91,74,127,162,141,240,
105,61,76,18,27,2,74,38,120,143,105,229,72,52,36,146,86,106,221,202,121,132,
182,158,15,39,224,217,84,109,199,80,23,236,221,136,94,26,148,143,231,67,94,
165,58,53,178,139,249,99,241,38,196,127,21,44,215,92,42,23,3,113,90,149,187,
142,196,45,116,254,46,153,73,134,30,152,99,135,161,120,59,30,143,91,124,115,
205,37,15,136,5,127,24,38,201,32,191,93,144,123,82,232,158,123,72,184,43,7,
122,159,203,195,95,190,59,228,113,240,255,182,57,231,55,170,109,74,137,149,
110,69,16,189,33,177,194,163,83,254,1,175,176,148,90,205,180,57,1,181,157,51,
184,236,74,180,100,248,58,219,219,152,76,218,234,17,203,117,64,248,155,10,159,
217,85,134,152,94,65,132,218,222,117,10,167,17,88,189,95,73,4,123,193,22,128,
214,7,222,166,106,39,242,8,188,253,156,134,200,114,3,38,0,149,222,68,149,4,
110,8,160,11,136,125,170,52,243,
  };



void setup() {

     // Following section enables SmartRC CC1101 library 
     // to work with Arduino Pro Micro
     // if using different board, please remove it
     // defining PINs set for Arduino Pro Micro setup
     byte sck = 15;   
     byte miso = 14;
     byte mosi = 16;
     byte ss = 10;
     int gdo0 = 3;
     // initializing library with custom pins selected
     ELECHOUSE_cc1101.setSpiPin(sck, miso, mosi, ss);

    // Main part to tune CC1101 with proper frequency, modulation and encoding    
    ELECHOUSE_cc1101.Init();                // must be set to initialize the cc1101!
    ELECHOUSE_cc1101.setGDO0(gdo0);         // set lib internal gdo pin (gdo0). Gdo2 not use for this example.
    ELECHOUSE_cc1101.setCCMode(1);          // set config for internal transmission mode.
    ELECHOUSE_cc1101.setModulation(2);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
    ELECHOUSE_cc1101.setMHZ(433.92);        // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    ELECHOUSE_cc1101.setDeviation(47.60);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
    ELECHOUSE_cc1101.setChannel(0);         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
    ELECHOUSE_cc1101.setChsp(199.95);       // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
    ELECHOUSE_cc1101.setRxBW(812.50);       // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
    ELECHOUSE_cc1101.setDRate(99.97);       // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
    ELECHOUSE_cc1101.setPA(10);             // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
    ELECHOUSE_cc1101.setSyncMode(2);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
    ELECHOUSE_cc1101.setSyncWord(211, 145); // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)
    ELECHOUSE_cc1101.setAdrChk(0);          // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
    ELECHOUSE_cc1101.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
    ELECHOUSE_cc1101.setWhiteData(0);       // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
    ELECHOUSE_cc1101.setPktFormat(0);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
    ELECHOUSE_cc1101.setLengthConfig(1);    // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved
    ELECHOUSE_cc1101.setPacketLength(0);    // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
    ELECHOUSE_cc1101.setCrc(1);             // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
    ELECHOUSE_cc1101.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
    ELECHOUSE_cc1101.setDcFilterOff(0);     // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
    ELECHOUSE_cc1101.setManchester(0);      // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
    ELECHOUSE_cc1101.setFEC(0);             // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
    ELECHOUSE_cc1101.setPRE(0);             // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
    ELECHOUSE_cc1101.setPQT(0);             // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
    ELECHOUSE_cc1101.setAppendStatus(0);    // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.

}



void loop() {

     uint8_t index2;
     uint8_t index;
     uint8_t channelnumber;

        // send 16 noise frames 
       for(index=0; index<16; index++)
        {

        // read next 64 randomized values from PROGMEM and put to CC1101 sending buffer
           for(index2=0; index2<64; index2++)
               {
                ccsendingbuffer[index2] = pgm_read_byte(randomized + (index*16) + index2) ;
               };  // end of 'index2' loop
           //

        // this is to send on 16 different channels, may be removed for particular freq jamming 
        ELECHOUSE_cc1101.setChannel(index);
        // ELECHOUSE_cc1101.setChannel(0);
          
        // send these data to radio over CC1101
        ELECHOUSE_cc1101.SendData(ccsendingbuffer);
        
        }; // End of 'index' loop

}
