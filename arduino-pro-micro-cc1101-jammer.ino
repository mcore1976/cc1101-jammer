/* 
 *         SIMPLE CC1101  JAMMER
 *         
 * uses C1101 to send some data over radio frequency 
 * based on RFBOOT  code by Panagiotis Karagiannis
 * from https://github.com/pkarsy/rfboot
 * who done great job converting to pure AVR-GCC
 * the PANSTAMP library which is the core of this project
 * 
 * (c) Adam Loboda 2021
 * 
 * This is ARDUINO PRO MICRO version (ATMEGA32U4)
 *
 */


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

#define F_CPU 8000000UL

#define  CC1101_CRYSTAL_FREQUENCY 26000000ul

/**
 * SPI pins
 */

#define SPI_SS   B,6     // ATMEGA32U4 PB6 = SPI_SS
#define SPI_MOSI B,2     // ATMEGA32U4 PB2 = MOSI
#define SPI_MISO B,3     // ATMEGA32U4 PB3 = MISO
#define SPI_SCK  B,1     // ATMEGA32U4 PB1 = SCK
#define GDO2     B,5     // ATMEGA32U4 PB5 
#define GDO0     D,0     // ATMEGA32U4 PD0 = INT0

#define PORT_SPI_MISO  PINB
#define BIT_SPI_MISO  3

#define PORT_SPI_SS  PORTB
#define BIT_SPI_SS   6

#define PORT_GDO0  PIND
#define BIT_GDO0  0


#define PORT_GDO2  PINB
#define BIT_GDO2  5

/**
 * Buffer and data lengths
 */
#define CC1101_BUFFER_LEN        64
#define CC1101_DATA_LEN          CC1101_BUFFER_LEN - 3

// UART ring buffer
#define BUFFER_SIZE   64

// buffer for RX CC data 
static volatile uint8_t ccreceivebuffer[CC1101_BUFFER_LEN] = "12345678901234567890123456789012345678901234567890123456789012345678901234567890123\x00";

// buffer for TC CC data 
static volatile uint8_t ccsendingbuffer[CC1101_BUFFER_LEN] = "12345678901234567890123456789012345678901234567890123456789012345678901234567890123\x00";

// a flag that a wireless packet has been received
static volatile uint8_t data_ready = 0;

// flag for reception/tranmission identification
static volatile uint8_t data_reception = 0;

// pointer to  CC reception buffer
static volatile uint8_t ccreceivebufferpos = 0;

/**
 * Class: CCPACKET
 * 
 * Description:
 * CC1101 data packet class
 */
typedef struct 
{
  //public:
    /**
     * Data length
     */
    uint8_t length;

    /**
     * Data buffer
     */
    uint8_t data[CC1101_DATA_LEN];

    /**
     * CRC OK flag
     */
    uint8_t crc_ok;

    /**
     * Received Strength Signal Indication
     */
    uint8_t rssi;

    /**
     * Link Quality Index
     */
    uint8_t lqi;
} CCPACKET;


/**
 * Carrier frequencies
 */
enum CFREQ
{
  CFREQ_868 = 0,
  CFREQ_915,
  CFREQ_433,
  CFREQ_315,
  CFREQ_LAST
};

/**
 * RF STATES
 */
enum RFSTATE
{
  RFSTATE_IDLE = 0,
  RFSTATE_RX,
  RFSTATE_TX
};


/**
 * Frequency channels
 */
#define NUMBER_OF_FCHANNELS      10

/**
 * Type of transfers
 */
#define WRITE_BURST              0x40
#define READ_SINGLE              0x80
#define READ_BURST               0xC0

/**
 * Type of register
 */
#define CC1101_CONFIG_REGISTER   READ_SINGLE
#define CC1101_STATUS_REGISTER   READ_BURST

/**
 * PATABLE & FIFO's
 */
#define CC1101_PATABLE           0x3E        // PATABLE address
#define CC1101_TXFIFO            0x3F        // TX FIFO address
#define CC1101_RXFIFO            0x3F        // RX FIFO address

/**
 * Command strobes
 */
#define CC1101_SRES              0x30        // Reset CC1101 chip
#define CC1101_SFSTXON           0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA):
                                             // Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define CC1101_SXOFF             0x32        // Turn off crystal oscillator
#define CC1101_SCAL              0x33        // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without
                                             // setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define CC1101_SRX               0x34        // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
#define CC1101_STX               0x35        // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1.
                                             // If in RX state and CCA is enabled: Only go to TX if channel is clear
#define CC1101_SIDLE             0x36        // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
#define CC1101_SWOR              0x38        // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if
                                             // WORCTRL.RC_PD=0
#define CC1101_SPWD              0x39        // Enter power down mode when CSn goes high
#define CC1101_SFRX              0x3A        // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC1101_SFTX              0x3B        // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC1101_SWORRST           0x3C        // Reset real time clock to Event1 value
#define CC1101_SNOP              0x3D        // No operation. May be used to get access to the chip status uint8_t

/**
 * CC1101 configuration registers
 */
#define CC1101_IOCFG2            0x00        // GDO2 Output Pin Configuration
#define CC1101_IOCFG1            0x01        // GDO1 Output Pin Configuration
#define CC1101_IOCFG0            0x02        // GDO0 Output Pin Configuration
#define CC1101_FIFOTHR           0x03        // RX FIFO and TX FIFO Thresholds
#define CC1101_SYNC1             0x04        // Sync Word, High uint8_t
#define CC1101_SYNC0             0x05        // Sync Word, Low uint8_t
#define CC1101_PKTLEN            0x06        // Packet Length
#define CC1101_PKTCTRL1          0x07        // Packet Automation Control
#define CC1101_PKTCTRL0          0x08        // Packet Automation Control
#define CC1101_ADDR              0x09        // Device Address
#define CC1101_CHANNR            0x0A        // Channel Number
#define CC1101_FSCTRL1           0x0B        // Frequency Synthesizer Control
#define CC1101_FSCTRL0           0x0C        // Frequency Synthesizer Control
#define CC1101_FREQ2             0x0D        // Frequency Control Word, High uint8_t
#define CC1101_FREQ1             0x0E        // Frequency Control Word, Middle uint8_t
#define CC1101_FREQ0             0x0F        // Frequency Control Word, Low uint8_t
#define CC1101_MDMCFG4           0x10        // Modem Configuration
#define CC1101_MDMCFG3           0x11        // Modem Configuration
#define CC1101_MDMCFG2           0x12        // Modem Configuration
#define CC1101_MDMCFG1           0x13        // Modem Configuration
#define CC1101_MDMCFG0           0x14        // Modem Configuration
#define CC1101_DEVIATN           0x15        // Modem Deviation Setting
#define CC1101_MCSM2             0x16        // Main Radio Control State Machine Configuration
#define CC1101_MCSM1             0x17        // Main Radio Control State Machine Configuration
#define CC1101_MCSM0             0x18        // Main Radio Control State Machine Configuration
#define CC1101_FOCCFG            0x19        // Frequency Offset Compensation Configuration
#define CC1101_BSCFG             0x1A        // Bit Synchronization Configuration
#define CC1101_AGCCTRL2          0x1B        // AGC Control
#define CC1101_AGCCTRL1          0x1C        // AGC Control
#define CC1101_AGCCTRL0          0x1D        // AGC Control
#define CC1101_WOREVT1           0x1E        // High uint8_t Event0 Timeout
#define CC1101_WOREVT0           0x1F        // Low uint8_t Event0 Timeout
#define CC1101_WORCTRL           0x20        // Wake On Radio Control
#define CC1101_FREND1            0x21        // Front End RX Configuration
#define CC1101_FREND0            0x22        // Front End TX Configuration
#define CC1101_FSCAL3            0x23        // Frequency Synthesizer Calibration
#define CC1101_FSCAL2            0x24        // Frequency Synthesizer Calibration
#define CC1101_FSCAL1            0x25        // Frequency Synthesizer Calibration
#define CC1101_FSCAL0            0x26        // Frequency Synthesizer Calibration
#define CC1101_RCCTRL1           0x27        // RC Oscillator Configuration
#define CC1101_RCCTRL0           0x28        // RC Oscillator Configuration
#define CC1101_FSTEST            0x29        // Frequency Synthesizer Calibration Control
#define CC1101_PTEST             0x2A        // Production Test
#define CC1101_AGCTEST           0x2B        // AGC Test
#define CC1101_TEST2             0x2C        // Various Test Settings
#define CC1101_TEST1             0x2D        // Various Test Settings
#define CC1101_TEST0             0x2E        // Various Test Settings

/**
 * Status registers
 */
#define CC1101_PARTNUM           0x30        // Chip ID
#define CC1101_VERSION           0x31        // Chip ID
#define CC1101_FREQEST           0x32        // Frequency Offset Estimate from Demodulator
#define CC1101_LQI               0x33        // Demodulator Estimate for Link Quality
#define CC1101_RSSI              0x34        // Received Signal Strength Indication
#define CC1101_MARCSTATE         0x35        // Main Radio Control State Machine State
#define CC1101_WORTIME1          0x36        // High uint8_t of WOR Time
#define CC1101_WORTIME0          0x37        // Low uint8_t of WOR Time
#define CC1101_PKTSTATUS         0x38        // Current GDOx Status and Packet Status
#define CC1101_VCO_VC_DAC        0x39        // Current Setting from PLL Calibration Module
#define CC1101_TXBYTES           0x3A        // Underflow and Number of uint8_ts
#define CC1101_RXBYTES           0x3B        // Overflow and Number of uint8_ts
#define CC1101_RCCTRL1_STATUS    0x3C        // Last RC Oscillator Calibration Result
#define CC1101_RCCTRL0_STATUS    0x3D        // Last RC Oscillator Calibration Result 

/**
 * CC1101 configuration registers - Default values extracted from SmartRF Studio
 *
 * Configuration:
 *
 * Deviation = 20.629883 
 * Base frequency = 867.999939 
 * Carrier frequency = 867.999939 
 * Channel number = 0 
 * Carrier frequency = 867.999939 
 * Modulated = true 
 * Modulation format = GFSK 
 * Manchester enable = false
 * Data whitening = off
 * Sync word qualifier mode = 30/32 sync word bits detected 
 * Preamble count = 4 
 * Channel spacing = 199.951172 
 * Carrier frequency = 867.999939 
 * Data rate = 38.3835 Kbps
 * RX filter BW = 101.562500 
 * Data format = Normal mode 
 * Length config = Variable packet length mode. Packet length configured by the first uint8_t after sync word 
 * CRC enable = true 
 * Packet length = 255 
 * Device address = 1 
 * Address config = Enable address check
 * Append status = Append two status uint8_ts to the payload of the packet. The status uint8_ts contain RSSI and
 * LQI values, as well as CRC OK
 * CRC autoflush = false 
 * PA ramping = false 
 * TX power = 12
 * GDO0 mode = Asserts when sync word has been sent / received, and de-asserts at the end of the packet.
 * In RX, the pin will also de-assert when a packet is discarded due to address or maximum length filtering
 * or when the radio enters RXFIFO_OVERFLOW state. In TX the pin will de-assert if the TX FIFO underflows
 * Settings optimized for low current consumption
 */
//#define CC1101_DEFVAL_IOCFG2     0x29        // GDO2 Output Pin Configuration
#define CC1101_DEFVAL_IOCFG2     0x2E        // GDO2 Output Pin Configuration
#define CC1101_DEFVAL_IOCFG1     0x2E        // GDO1 Output Pin Configuration
#define CC1101_DEFVAL_IOCFG0     0x06        // GDO0 Output Pin Configuration
#define CC1101_DEFVAL_FIFOTHR    0x07        // RX FIFO and TX FIFO Thresholds
#define CC1101_DEFVAL_SYNC1      0xB5        // Synchronization word, high uint8_t
#define CC1101_DEFVAL_SYNC0      0x47        // Synchronization word, low uint8_t
#define CC1101_DEFVAL_PKTLEN     0x3D        // Packet Length
#define CC1101_DEFVAL_PKTCTRL1   0x06        // Packet Automation Control
#define CC1101_DEFVAL_PKTCTRL0   0x05        // Packet Automation Control
#define CC1101_DEFVAL_ADDR       0xFF        // Device Address
#define CC1101_DEFVAL_CHANNR     0x00        // Channel Number
#define CC1101_DEFVAL_FSCTRL1    0x08        // Frequency Synthesizer Control
#define CC1101_DEFVAL_FSCTRL0    0x00        // Frequency Synthesizer Control
// Carrier frequency = 868 MHz
#define CC1101_DEFVAL_FREQ2_868  0x21        // Frequency Control Word, High uint8_t
#define CC1101_DEFVAL_FREQ1_868  0x62        // Frequency Control Word, Middle uint8_t
#define CC1101_DEFVAL_FREQ0_868  0x76        // Frequency Control Word, Low uint8_t
// Carrier frequency = 902 MHz
#define CC1101_DEFVAL_FREQ2_915  0x22        // Frequency Control Word, High uint8_t
#define CC1101_DEFVAL_FREQ1_915  0xB1        // Frequency Control Word, Middle uint8_t
#define CC1101_DEFVAL_FREQ0_915  0x3B        // Frequency Control Word, Low uint8_t
// Carrier frequency = 433 MHz
#define CC1101_DEFVAL_FREQ2_433  0x10        // Frequency Control Word, High uint8_t
#define CC1101_DEFVAL_FREQ1_433  0xA7        // Frequency Control Word, Middle uint8_t
#define CC1101_DEFVAL_FREQ0_433  0x62        // Frequency Control Word, Low uint8_t
// Carrier frequency = 315 MHz
#define CC1101_DEFVAL_FREQ2_315  0x0C        // Frequency Control Word, High uint8_t
#define CC1101_DEFVAL_FREQ1_315  0x1D        // Frequency Control Word, Middle uint8_t
#define CC1101_DEFVAL_FREQ0_315  0x89        // Frequency Control Word, Low uint8_t


#define CC1101_DEFVAL_MDMCFG4    0xCA        // Modem Configuration
#define CC1101_DEFVAL_MDMCFG3    0x83        // Modem Configuration
#define CC1101_DEFVAL_MDMCFG2    0x93        // Modem Configuration
#define CC1101_DEFVAL_MDMCFG1    0x22        // Modem Configuration
#define CC1101_DEFVAL_MDMCFG0    0xF8        // Modem Configuration
#define CC1101_DEFVAL_DEVIATN    0x35        // Modem Deviation Setting
#define CC1101_DEFVAL_MCSM2      0x07        // Main Radio Control State Machine Configuration
//#define CC1101_DEFVAL_MCSM1      0x30        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_MCSM1      0x20        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_MCSM0      0x18        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_FOCCFG     0x16        // Frequency Offset Compensation Configuration
#define CC1101_DEFVAL_BSCFG      0x6C        // Bit Synchronization Configuration
#define CC1101_DEFVAL_AGCCTRL2   0x43        // AGC Control
#define CC1101_DEFVAL_AGCCTRL1   0x40        // AGC Control
#define CC1101_DEFVAL_AGCCTRL0   0x91        // AGC Control
#define CC1101_DEFVAL_WOREVT1    0x87        // High uint8_t Event0 Timeout
#define CC1101_DEFVAL_WOREVT0    0x6B        // Low uint8_t Event0 Timeout
#define CC1101_DEFVAL_WORCTRL    0xFB        // Wake On Radio Control
#define CC1101_DEFVAL_FREND1     0x56        // Front End RX Configuration
#define CC1101_DEFVAL_FREND0     0x10        // Front End TX Configuration
#define CC1101_DEFVAL_FSCAL3     0xE9        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL2     0x2A        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL1     0x00        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL0     0x1F        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_RCCTRL1    0x41        // RC Oscillator Configuration
#define CC1101_DEFVAL_RCCTRL0    0x00        // RC Oscillator Configuration
#define CC1101_DEFVAL_FSTEST     0x59        // Frequency Synthesizer Calibration Control
#define CC1101_DEFVAL_PTEST      0x7F        // Production Test
#define CC1101_DEFVAL_AGCTEST    0x3F        // AGC Test
#define CC1101_DEFVAL_TEST2      0x81        // Various Test Settings
#define CC1101_DEFVAL_TEST1      0x35        // Various Test Settings
#define CC1101_DEFVAL_TEST0      0x09        // Various Test Settings

/**
 * Macros
 */
// Read CC1101 Config register
#define readConfigReg(regAddr)    cc1101_readReg(regAddr, CC1101_CONFIG_REGISTER)
// Read CC1101 Status register
#define readStatusReg(regAddr)    cc1101_readReg(regAddr, CC1101_STATUS_REGISTER)
// Enter Rx state
#define setRxState()              cc1101_cmdStrobe(CC1101_SRX)
// Enter Tx state
#define setTxState()              cc1101_cmdStrobe(CC1101_STX)
// Enter IDLE state
#define setIdleState()            cc1101_cmdStrobe(CC1101_SIDLE)
// Flush Rx FIFO
#define flushRxFifo()             cc1101_cmdStrobe(CC1101_SFRX)
// Flush Tx FIFO
#define flushTxFifo()             cc1101_cmdStrobe(CC1101_SFTX)
// Disable address check
#define disableAddressCheck()     cc1101_writeReg(CC1101_PKTCTRL1, 0x04)
// Enable address check
#define enableAddressCheck()      cc1101_writeReg(CC1101_PKTCTRL1, 0x06)
// Disable CCA
#define disableCCA()              cc1101_writeReg(CC1101_MCSM1, 0)
// Enable CCA
#define enableCCA()               cc1101_writeReg(CC1101_MCSM1, CC1101_DEFVAL_MCSM1)
// Set PATABLE single uint8_t
//  #define setTxPowerAmp(setting)    cc1101_paTableuint8_t = setting
// PATABLE values
#define PA_LowPower               0x60
#define PA_LongDistance           0xC0


// pseudorandom 1024 values to send over radio frequency to generate the noise
// on jammed frequency

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














// ***********************************
// ******* SPI SPECIFIC ROUTINES
// ***********************************

/**
 * init
 * 
 * SPI initialization
 */
void spi_init() 
{
  //digitalWrite(SPI_SS, HIGH);
  // SPI_SS = PORTB6
  PORTB |= _BV(6);  

  //digitalWrite(GD02, LOW);
  // set PB5 as output
  // GD02 = PORTB5
  DDRB |= _BV(5);
  DDRB &= ~_BV(5);
  
  // Configure SPI pins
  //pinMode(SPI_SS, OUTPUT);
  //pinMode(SPI_MOSI, OUTPUT);
  //pinMode(SPI_MISO, INPUT);
  //pinMode(SPI_SCK, OUTPUT);

  // SPI_SS = PORTB6
  // SPI_MOSI = PORTB2
  // SPI_MISO = PORTB3 
  // SPI_SCK = PORTB1 
  DDRB |= _BV(6);
  DDRB |= _BV(2);
  DDRB &= ~_BV(3);
  DDRB |= _BV(1);

  //digitalWrite(SPI_SCK, HIGH);
  //digitalWrite(SPI_MOSI, LOW);
  PORTB |= _BV(1);
  PORTB &= ~_BV(2);

  // SPI speed = clk/4
  SPCR = _BV(SPE) | _BV(MSTR);
  
}

/**
 * send
 * 
 * Send uint8_t via SPI
 * 
 * 'value'  Value to be sent
 * 
 * Return:
 *  Response received from SPI slave
 */
uint8_t spi_send(uint8_t value) 
{
  SPDR = value;                          // Transfer uint8_t via SPI
  wait_Spi();                            // Wait until SPI operation is terminated
  return SPDR;
}

/*
 * Wait until SPI operation is terminated
 */
void wait_Spi(void)
  {
    while(!(SPSR & _BV(SPIF)));
  };





// ********************************************************************
// ******* CC1101 CHIP LOW LEVEL COMMUNICATION ROUTINES
// ********************************************************************

/**
 * select CC1101 by LOW SS signal - PORT B6
 */
void cc1101_Select(void)
{
PORTB &= ~_BV(6);
};

/**
 * deselect CC1101 by HIGH SS signal - PORT B6
 */
void cc1101_Deselect(void)
{
PORTB |= _BV(6);
};

/**
 * Wait until GDO0 line goes high - PORT D0
 */
void wait_GDO0_high(void)
{
while(!((PIND >> 0) & 1));
};

/**
 * Wait until GDO0 line goes low - PORT D0
 */
void wait_GDO0_low(void)
{
while ((PIND >> 0) & 1);
};

/*
  Wait until SPI MISO line goes low - PORT B3
 */
void wait_Miso(void)  
{
while(( PINB >> 3) & 1);
};


/**
 * cc1101_wakeUp
 * 
 * Wake up CC1101 from Power Down state
 */
void cc1101_wakeUp(void)
{
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * cc1101_writeReg
 * 
 * Write single register into the CC1101 IC via SPI
 * 
 * 'regAddr'    Register address
 * 'value'  Value to be writen
 */
void cc1101_writeReg(uint8_t regAddr, uint8_t value) 
{
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  spi_send(regAddr);                    // Send register address
  spi_send(value);                      // Send value
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * cc1101_writeBurstReg
 * 
 * Write multiple registers into the CC1101 IC via SPI
 * 
 * 'regAddr'    Register address
 * 'buffer' Data to be writen
 * 'len'    Data length
 */
void cc1101_writeBurstReg(uint8_t regAddr, uint8_t* buffer, uint8_t len)
{
  uint8_t addr, i;
  
  addr = regAddr | WRITE_BURST;         // Enable burst transfer
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait unt0il MISO goes low
  spi_send(addr);                       // Send register address
  
  for(i=0 ; i<len ; i++)
    spi_send(buffer[i]);                // Send value

  cc1101_Deselect();                    // Deselect CC1101  
}

/**
 * cc1101_cmdStrobe
 * 
 * Send command strobe to the CC1101 IC via SPI
 * 
 * 'cmd'    Command strobe
 */     
void cc1101_cmdStrobe(uint8_t cmd) 
{
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  spi_send(cmd);                        // Send strobe command
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * cc1101_readReg
 * 
 * Read CC1101 register via SPI
 * 
 * 'regAddr'    Register address
 * 'regType'    Type of register: CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
 * 
 * Return:
 *  Data uint8_t returned by the CC1101 IC
 */
uint8_t cc1101_readReg(uint8_t regAddr, uint8_t regType) 
{
  uint8_t addr, val;

  addr = regAddr | regType;
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  spi_send(addr);                       // Send register address
  val = spi_send(0x00);                 // Read result
  cc1101_Deselect();                    // Deselect CC1101

  return val;
}

/**
 * cc1101_readBurstReg
 * 
 * Read burst data from CC1101 via SPI
 * 
 * 'buffer' Buffer where to copy the result to
 * 'regAddr'    Register address
 * 'len'    Data length
 */
void cc1101_readBurstReg(uint8_t * buffer, uint8_t regAddr, uint8_t len) 
{
  uint8_t addr, i;
  
  addr = regAddr | READ_BURST;
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  spi_send(addr);                       // Send register address
  for(i=0 ; i<len ; i++)
    buffer[i] = spi_send(0x00);         // Read result uint8_t by uint8_t
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * cc1101_reset
 * 
 * Reset CC1101
 */
void cc1101_reset(void) 
{
  cc1101_Deselect();                    // Deselect CC1101

  //delayMicroseconds(5);
  // Delay 40 cycles
  // 5us at 8.0 MHz
   asm volatile (
    "    ldi  r18, 13"  "\n"
    "1:  dec  r18"  "\n"
    "    brne 1b" "\n"
    "    nop" "\n"
   );

  cc1101_Select();                      // Select CC1101


  //delayMicroseconds(10);
  // Delay 80 cycles
  // 10us at 8.0 MHz
  asm volatile (
    "    ldi  r18, 26"  "\n"
    "1:  dec  r18"  "\n"
    "    brne 1b" "\n"
    "    rjmp 1f" "\n"
    "1:"  "\n"
   );

  cc1101_Deselect();                    // Deselect CC1101

  //delayMicroseconds(41);
  // Delay 328 cycles
  // 41us at 8.0 MHz
  asm volatile (
    "    ldi  r18, 109" "\n"
    "1:  dec  r18"  "\n"
    "    brne 1b" "\n"
    "    nop" "\n"
   );

  cc1101_Select();                      // Select CC1101

  wait_Miso();                          // Wait until MISO goes low
  spi_send(CC1101_SRES);                // Send cc1101_reset command strobe
  wait_Miso();                          // Wait until MISO goes low

  cc1101_Deselect();                    // Deselect CC1101

  cc1101_setDefaultRegs();                     // Reconfigure CC1101
}

/**
 * cc1101_setDefaultRegs
 * 
 * Configure CC1101 registers
 */
void cc1101_setDefaultRegs(void) 
{
  cc1101_writeReg(CC1101_IOCFG2,  CC1101_DEFVAL_IOCFG2);
  cc1101_writeReg(CC1101_IOCFG1,  CC1101_DEFVAL_IOCFG1);
  cc1101_writeReg(CC1101_IOCFG0,  CC1101_DEFVAL_IOCFG0);
  cc1101_writeReg(CC1101_FIFOTHR,  CC1101_DEFVAL_FIFOTHR);
  cc1101_writeReg(CC1101_PKTLEN,  CC1101_DEFVAL_PKTLEN);
  cc1101_writeReg(CC1101_PKTCTRL1,  CC1101_DEFVAL_PKTCTRL1);
  cc1101_writeReg(CC1101_PKTCTRL0,  CC1101_DEFVAL_PKTCTRL0);

  // Set default synchronization word
  cc1101_setSyncWord(CC1101_DEFVAL_SYNC1, CC1101_DEFVAL_SYNC0);

  // Set default device address
  cc1101_setDevAddress(CC1101_DEFVAL_ADDR);
  // Set default frequency channel
  cc1101_setChannel(CC1101_DEFVAL_CHANNR);
  
  cc1101_writeReg(CC1101_FSCTRL1,  CC1101_DEFVAL_FSCTRL1);
  cc1101_writeReg(CC1101_FSCTRL0,  CC1101_DEFVAL_FSCTRL0);

  // Set default carrier frequency = 868 MHz
  // cc1101_setCarrierFreq(CFREQ_868);
  cc1101_setCarrierFreq(CFREQ_433);

  cc1101_writeReg(CC1101_MDMCFG4,  CC1101_DEFVAL_MDMCFG4);
  cc1101_writeReg(CC1101_MDMCFG3,  CC1101_DEFVAL_MDMCFG3);
  cc1101_writeReg(CC1101_MDMCFG2,  CC1101_DEFVAL_MDMCFG2);
  cc1101_writeReg(CC1101_MDMCFG1,  CC1101_DEFVAL_MDMCFG1);
  cc1101_writeReg(CC1101_MDMCFG0,  CC1101_DEFVAL_MDMCFG0);
  cc1101_writeReg(CC1101_DEVIATN,  CC1101_DEFVAL_DEVIATN);
  cc1101_writeReg(CC1101_MCSM2,  CC1101_DEFVAL_MCSM2);
  cc1101_writeReg(CC1101_MCSM1,  CC1101_DEFVAL_MCSM1);
  cc1101_writeReg(CC1101_MCSM0,  CC1101_DEFVAL_MCSM0);
  cc1101_writeReg(CC1101_FOCCFG,  CC1101_DEFVAL_FOCCFG);
  cc1101_writeReg(CC1101_BSCFG,  CC1101_DEFVAL_BSCFG);
  cc1101_writeReg(CC1101_AGCCTRL2,  CC1101_DEFVAL_AGCCTRL2);
  cc1101_writeReg(CC1101_AGCCTRL1,  CC1101_DEFVAL_AGCCTRL1);
  cc1101_writeReg(CC1101_AGCCTRL0,  CC1101_DEFVAL_AGCCTRL0);
  cc1101_writeReg(CC1101_WOREVT1,  CC1101_DEFVAL_WOREVT1);
  cc1101_writeReg(CC1101_WOREVT0,  CC1101_DEFVAL_WOREVT0);
  cc1101_writeReg(CC1101_WORCTRL,  CC1101_DEFVAL_WORCTRL);
  cc1101_writeReg(CC1101_FREND1,  CC1101_DEFVAL_FREND1);
  cc1101_writeReg(CC1101_FREND0,  CC1101_DEFVAL_FREND0);
  cc1101_writeReg(CC1101_FSCAL3,  CC1101_DEFVAL_FSCAL3);
  cc1101_writeReg(CC1101_FSCAL2,  CC1101_DEFVAL_FSCAL2);
  cc1101_writeReg(CC1101_FSCAL1,  CC1101_DEFVAL_FSCAL1);
  cc1101_writeReg(CC1101_FSCAL0,  CC1101_DEFVAL_FSCAL0);
  cc1101_writeReg(CC1101_RCCTRL1,  CC1101_DEFVAL_RCCTRL1);
  cc1101_writeReg(CC1101_RCCTRL0,  CC1101_DEFVAL_RCCTRL0);
  cc1101_writeReg(CC1101_FSTEST,  CC1101_DEFVAL_FSTEST);
  cc1101_writeReg(CC1101_PTEST,  CC1101_DEFVAL_PTEST);
  cc1101_writeReg(CC1101_AGCTEST,  CC1101_DEFVAL_AGCTEST);
  cc1101_writeReg(CC1101_TEST2,  CC1101_DEFVAL_TEST2);
  cc1101_writeReg(CC1101_TEST1,  CC1101_DEFVAL_TEST1);
  cc1101_writeReg(CC1101_TEST0,  CC1101_DEFVAL_TEST0);
}


void cc1101_setMyRegs(void)
{
// Rf settings for CC1101
// mod GFSK, TX 10dbm, baud 1200, high sensivity, dev 5.2khz, rx bw 58khz
// optimized for packet sending
cc1101_writeReg(CC1101_IOCFG0,0x06); //GDO0 Output Pin Configuration
cc1101_writeReg(CC1101_FIFOTHR,0x47); //RX FIFO and TX FIFO Thresholds
cc1101_writeReg(CC1101_PKTCTRL0,0x05);//Packet Automation Control
cc1101_writeReg(CC1101_FSCTRL1,0x06); //Frequency Synthesizer Control
cc1101_writeReg(CC1101_FREQ2,0x10); //Frequency Control Word, High uint8_t
cc1101_writeReg(CC1101_FREQ1,0xA7); //Frequency Control Word, Middle uint8_t
cc1101_writeReg(CC1101_FREQ0,0x62); //Frequency Control Word, Low uint8_t
cc1101_writeReg(CC1101_MDMCFG4,0xF5); //Modem Configuration
cc1101_writeReg(CC1101_MDMCFG3,0x83); //Modem Configuration
cc1101_writeReg(CC1101_MDMCFG2,0x13); //Modem Configuration
cc1101_writeReg(CC1101_MDMCFG1,0x00); //Modem Configuration
cc1101_writeReg(CC1101_DEVIATN,0x15); //Modem Deviation Setting
cc1101_writeReg(CC1101_MCSM0,0x18); //Main Radio Control State Machine Configuration
cc1101_writeReg(CC1101_FOCCFG,0x16); //Frequency Offset Compensation Configuration
cc1101_writeReg(CC1101_WORCTRL,0xFB); //Wake On Radio Control
cc1101_writeReg(CC1101_FSCAL3,0xE9); //Frequency Synthesizer Calibration
cc1101_writeReg(CC1101_FSCAL2,0x2A); //Frequency Synthesizer Calibration
cc1101_writeReg(CC1101_FSCAL1,0x00); //Frequency Synthesizer Calibration
cc1101_writeReg(CC1101_FSCAL0,0x1F); //Frequency Synthesizer Calibration
cc1101_writeReg(CC1101_TEST2,0x81); //Various Test Settings
cc1101_writeReg(CC1101_TEST1,0x35); //Various Test Settings
cc1101_writeReg(CC1101_TEST0,0x09); //Various Test Settings
}


/**
 * cc1101_init
 * 
 * Initialize CC1101
 */
void cc1101_init(void) 
{
  spi_init();                           // Initialize SPI interface

  cc1101_reset();                              // Reset CC1101

  // Configure PATABLE
  // for 10 mW power use this option
  cc1101_writeReg(CC1101_PATABLE, PA_LongDistance);
  // for 1mW power use this option
  // cc1101_writeReg(CC1101_PATABLE, PA_LowPower);
}

/**
 * cc1101_setSyncWord
 * 
 * Set synchronization word
 * 
 * 'syncH'  Synchronization word - High uint8_t
 * 'syncL'  Synchronization word - Low uint8_t
 * 'save' If TRUE, save parameter in EEPROM
 */
void cc1101_setSyncWord(uint8_t syncH, uint8_t syncL) 
{
    cc1101_writeReg(CC1101_SYNC1, syncH);
    cc1101_writeReg(CC1101_SYNC0, syncL);
}

/**
 * cc1101_setDevAddress
 * 
 * Set device address
 * 
 * 'addr'   Device address
 * 'save' If TRUE, save parameter in EEPROM
 */
void cc1101_setDevAddress(uint8_t addr) 
{
    cc1101_writeReg(CC1101_ADDR, addr);
}


/**
 * cc1101_setChannel
 * 
 * Set frequency channel
 * 
 * 'chnl'   Frequency channel
 * 'save' If TRUE, save parameter in EEPROM
 */
void cc1101_setChannel(uint8_t chnl) 
{
    cc1101_writeReg(CC1101_CHANNR,  chnl);
}

/**
 * cc1101_setCarrierFreq
 * 
 * Set carrier frequency
 * 
 * 'freq'   New carrier frequency
 */
void cc1101_setCarrierFreq(uint8_t freq)
{
  switch(freq)
  {
    case CFREQ_915:
      cc1101_writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_915);
      cc1101_writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_915);
      cc1101_writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_915);
      break;
    case CFREQ_433:
      cc1101_writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_433);
      cc1101_writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_433);
      cc1101_writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_433);
      break;
    case CFREQ_315:
      cc1101_writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_315);
      cc1101_writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_315);
      cc1101_writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_315);
      break;
    default:
      cc1101_writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_868);
      cc1101_writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_868);
      cc1101_writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_868);
      break;
  }
   
  //cc1101_carrierFreq = freq;  
}



void CC1101_setFrequency(uint32_t freq) 
  {
    // We use uint64_t as the <<16 overflows uint32_t
    // however the division with 26000000 allows the final
    // result to be uint32 again
    uint32_t reg_freq = ((uint64_t)freq << 16) / CC1101_CRYSTAL_FREQUENCY;

    // this is split into 3 bytes that are written to 3 different registers on the CC1101
    uint8_t FREQ2 = (reg_freq >> 16) & 0xFF;   // high byte, bits 7..6 are always 0 for this register
    uint8_t FREQ1 = (reg_freq >> 8) & 0xFF;    // middle byte
    uint8_t FREQ0 = reg_freq & 0xFF;         // low byte
    // setIdleState();
    // cc1101_writeReg(CC1101_CHANNR, 0);
    cc1101_writeReg(CC1101_FREQ2, FREQ2);
    cc1101_writeReg(CC1101_FREQ1, FREQ1);
    cc1101_writeReg(CC1101_FREQ0, FREQ0);
}

/**
 * cc1101_setPowerDownState
 * 
 * Put CC1101 into power-down state
 */
void cc1101_setPowerDownState() 
{
  // Comming from RX state, we need to enter the IDLE state first
  cc1101_cmdStrobe(CC1101_SIDLE);
  // Enter Power-down state
  cc1101_cmdStrobe(CC1101_SPWD);
}



/**
 * cc1101_sendData
 * 
 * Send data packet via RF
 * 
 * 'packet' Packet to be transmitted. First uint8_t is the destination address
 *
 *  Return:
 *    True if the transmission succeeds
 *    False otherwise
 */
bool cc1101_sendData(CCPACKET packet)
{
  uint8_t marcState;
  bool res = false;
 
  // Declare to be in Tx state. This will avoid receiving packets whilst
  // transmitting
  //cc1101_rfState = RFSTATE_TX;

  // Enter RX state
  setRxState();

  // Check that the RX state has been entered
  while (((marcState = readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x0D)
  {
    if (marcState == 0x11)        // RX_OVERFLOW
      flushRxFifo();              // flush receive queue
  }

  //delayMicroseconds(500);
  // Delay 4 000 cycles
  // 500us at 8.0 MHz
  asm volatile (
    "    ldi  r18, 6" "\n"
    "    ldi  r19, 49"  "\n"
    "1:  dec  r19"  "\n"
    "    brne 1b" "\n"
    "    dec  r18"  "\n"
    "    brne 1b" "\n"
   );

  // Set data length at the first position of the TX FIFO
  cc1101_writeReg(CC1101_TXFIFO,  packet.length);

  // Write data into the TX FIFO
  cc1101_writeBurstReg(CC1101_TXFIFO, packet.data, packet.length);

  // CCA enabled: will enter TX state only if the channel is clear
  setTxState();

  // Check that TX state is being entered (state = RXTX_SETTLING)
  marcState = readStatusReg(CC1101_MARCSTATE) & 0x1F;
  if((marcState != 0x13) && (marcState != 0x14) && (marcState != 0x15))
  {
    setIdleState();       // Enter IDLE state
    flushTxFifo();        // Flush Tx FIFO
    setRxState();         // Back to RX state

    // Declare to be in Rx state
    //cc1101_rfState = RFSTATE_RX;
    return false;
  }

  // Wait for the sync word to be transmitted
  wait_GDO0_high();

  // Wait until the end of the packet transmission
  wait_GDO0_low();

  // Check that the TX FIFO is empty
  if((readStatusReg(CC1101_TXBYTES) & 0x7F) == 0)
    res = true;

  setIdleState();       // Enter IDLE state
  flushTxFifo();        // Flush Tx FIFO

  // Enter back into RX state
  setRxState();

  // Declare to be in Rx state
  //cc1101_rfState = RFSTATE_RX;

  return res;
}



/**
 * cc1101_receiveData
 * 
 * Read data packet from RX FIFO
 *
 * 'packet' Container for the packet received
 * 
 * Return:
 *  Amount of uint8_ts received
 */
uint8_t cc1101_receiveData(CCPACKET * packet)
{
  uint8_t val;
  uint8_t rxbytes = readStatusReg(CC1101_RXBYTES);

  // Any uint8_t waiting to be read and no overflow?
  if (rxbytes & 0x7F && !(rxbytes & 0x80))
  {
    // Read data length
    packet->length = readConfigReg(CC1101_RXFIFO);
    // If packet is too long
    if (packet->length > CC1101_DATA_LEN)
      packet->length = 0;   // Discard packet
    else
    {
      // Read data packet
      cc1101_readBurstReg(packet->data, CC1101_RXFIFO, packet->length);
      // Read RSSI
      packet->rssi = readConfigReg(CC1101_RXFIFO);
      // Read LQI and CRC_OK
      val = readConfigReg(CC1101_RXFIFO);
      packet->lqi = val & 0x7F;
      //packet->crc_ok = bitRead(val, 7);
      packet->crc_ok = (val>>7);
    }
  }
  else
    packet->length = 0;

  setIdleState();       // Enter IDLE state
  flushRxFifo();        // Flush Rx FIFO
  //cc1101_cmdStrobe(CC1101_SCAL);

  // Back to RX state
  setRxState();

  return packet->length;
}




// ************************************************************************ 
// INTERRUPT ROUTINE FOR CC1101 HANDLING TRANSMITING/RECEIVING PACKETS
// Generated by CC1101 when receives SyncWord (or sends a packet)
// ************************************************************************ 
ISR (INT0_vect)
{
    /* interrupt code here */

    // structure for incoming packet over radio CC1101
    static CCPACKET inpacket;
    
    // first we mark as omething was TRANSMITTED - that is why interrupt happened
        data_ready = true;
        data_reception = false;

    // then we check if there any data receiver - something was RECEIVED - that is why interrupt happened 
    // Any packet waiting to be read?
    if (cc1101_receiveData(&inpacket)>0  && inpacket.crc_ok != 0 )
    {
 
        // copy received packet to buffer to send it over USART
        memcpy(ccreceivebuffer, inpacket.data, inpacket.length); 

        // copy length of the packet to global pointer
        ccreceivebufferpos = inpacket.length; 

        // put NULL at the end of received data
        ccreceivebuffer[ccreceivebufferpos] = 0x00; 

        // some valid data received so we have to set the flag
        data_ready = true;
        data_reception = true;

     // end of IF statement
     }

    // update again INT0 conditions
    EICRA |= (1 << ISC01);    // set INT0 to trigger on ANY logic change
    EIMSK |= (1 << INT0);     // Turns on INT0


}



// ***********************************************************
//        CC1101 communication procedures for main code
// ***********************************************************

/* radio_init()
 * init CC1101 radio with my predefined settings and enable interrupts
 */
void radio_init(void) {
    // init registers with defaults
    cc1101_init();

    // default is 433Mhz
    cc1101_setChannel(CFREQ_433);

    // set my radio settings   
    // mod GFSK, TX 10dbm, baud 1200, high sensivity, dev 5.2khz, rx bw 58khz 
    //cc1101_setMyRegs();

    // send syncword to match transmitter
    cc1101_setSyncWord(0xAF,0xAF);

    disableAddressCheck();

   EICRA |= (1 << ISC01);    // set INT0 to trigger on falling edge 
   EIMSK |= (1 << INT0);     // Turns on INT0 - GCO0 output from CC1101

    __asm__("nop\n\t");
    __asm__("nop\n\t");
    data_ready = false;

}


/* radiosendtxt(char *send_str)
 * send text via CC1101
 */
void radiosendtxt(char *send_str)
 {
    int i;

    // structure for packet to be send over radio through CC1101 module
    CCPACKET outpkt;

    outpkt.length=strlen(send_str);


      for (i=0; i<(strlen(send_str)); i++)   
        { outpkt.data[i]= send_str[i] ; 
        };

    cc1101_sendData(outpkt);
    // wait for INT0 falling edge on GCO0 
   // while (! data_ready);

    // data were sent so we can clear the flag
    data_ready = false;             // clear CC1101 tx/rx flag  
    data_reception = false;         // clear CC1101 type flag

}

// *************************************************************************
//               MAIN FUNCTION OF THE PROGRAM
// *************************************************************************

int main()
{  


uint8_t index2;
uint8_t index;
uint8_t channelnumber;
uint8_t marcState;

    // clear all flags
    data_ready = false;
    data_reception = false;
    ccreceivebufferpos = 0;



    // CC1101 INITIALIZATION
    radio_init();
    
    // setup interrpupts for GCO0 connected to INT0
    //
    DDRD &= ~(1 << DDD0);     // Clear the PD3 pin
    // PD0 (PCINT0 pin) is now an input


    PORTD |= (1 << PORTD0);    // turn On the Pull-up
    // PD0 is now an input with pull-up enabled

    EICRA |= (1 << ISC01);    // set INT0 to trigger on ANY logic change
    EIMSK |= (1 << INT0);     // Turns on INT0

     sei();                    // turn on interrupts



while(1)
 { // neverendingloop for transmitting


   // send 16 noise frames 

       for(index=0; index<16; index++)
        {

        // read next 64 randomized values from PROGMEM and put to CC1101 sending buffer
           for(index2=0; index2<64; index2++)
               {
                ccsendingbuffer[index2] = pgm_read_byte(randomized + (index*16) + index2) ;
               }
           //

       // Set default carrier frequency = 868 MHz or different
       // cc1101_setCarrierFreq(CFREQ_868);
       //  cc1101_setCarrierFreq(CFREQ_433);
       // cc1101_setCarrierFreq(CFREQ_315);
       
        // PUT YOUR OWN FREQUENCY HERE !!!
       // Example for 433.52 MHz jamming 
        CC1101_setFrequency(433520000ul);   
       // CC1101_setFrequency(315000000ul);   


        // this is to send on 16 different channels, may be removed for particular freq jamming 
       // cc1101_setChannel(index);
        cc1101_setChannel(0);
          
        // send these data to radio over CC1101
        radiosendtxt(ccsendingbuffer);

        // clear all flags - especially position in the UART buffer
        data_ready = false;  
  
        // go back to RX mode
        setIdleState();       // Enter IDLE state
        flushRxFifo();        // Flush Rx FIFO  
        setRxState();
              
        }; // end of indexvalue loop
        
   // end of neverending WHILE
  };

                                                               
// end of MAIN function                           
};
