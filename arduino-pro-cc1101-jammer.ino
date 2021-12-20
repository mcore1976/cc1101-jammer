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

// for serial port communication lowest error rates for 8MHZ CPU clock are 38400, 9600, 2400..
#define BAUDRATE 9600       //The baudrate that we want to use
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)    //The formula that does all the required maths


/**
 * SPI pins
 */

#define SPI_SS   B,6     // ATMEGA32U4 PB6 = SPI_SS
#define SPI_MOSI B,2     // ATMEGA32U4 PB2 = MOSI
#define SPI_MISO B,3     // ATMEGA32U4 PB3 = MISO
#define SPI_SCK  B,1     // ATMEGA32U4 PB1 = SCK
#define GDO0     D,0     // ATMEGA32U4 PD0 = INT0

#define PORT_SPI_MISO  PINB
#define BIT_SPI_MISO  3

#define PORT_SPI_SS  PORTB
#define BIT_SPI_SS   6

#define PORT_GDO0  PIND
#define BIT_GDO0  0

/**
 * Buffer and data lengths
 */
#define CC1101_BUFFER_LEN        64
#define CC1101_DATA_LEN          CC1101_BUFFER_LEN - 3

// UART ring buffer
#define BUFFER_SIZE   64

// buffer for RX CC data to further send over serial port
static volatile uint8_t ccreceivebuffer[CC1101_BUFFER_LEN] = "12345678901234567890123456789012345678901234567890123456789012345678901234567890123\x00";

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
    default:
      cc1101_writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_868);
      cc1101_writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_868);
      cc1101_writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_868);
      break;
  }
   
  //cc1101_carrierFreq = freq;  
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


int i;
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

        // send received data over serial to radio over CC1101
        radiosendtxt("Hello World");


        // clear all flags - especially position in the UART buffer
        data_ready = false;  
  
        // go back to RX mode
        setIdleState();       // Enter IDLE state
        flushRxFifo();        // Flush Rx FIFO  
        setRxState();
        
   // end of neverending WHILE
     };

                                                               
// end of MAIN function                           
};
