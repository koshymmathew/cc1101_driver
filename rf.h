/*	
 *	rf.h
 *	
 *	TI CC1101 915MHz RF Driver
 *	
 *  WheelWobbler V0.2
 *  Harvest Robotics
 *
 */

#include <stdio.h>
#include <Energia.h>
#include <SPI.h>
//#include <cc1101_config.h>

#include <stddef.h>
//#include <avr/pgmspace.h>


struct FLASH{
  uint8_t txbuffer[20];
  uint16_t done;
  uint16_t crc;
  uint16_t src;
  uint32_t ID;
  
};

struct WAKENODE{
	char node_num;
	char command;
};

struct SENSOR{
  uint16_t val_0;
  uint16_t val_1;
  uint16_t val_2;
  uint16_t val_3;
  uint16_t val_4;
  uint16_t val_5;
  uint16_t val_6;
};

struct DATETIME{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
};

struct HEADER{
  long ID;
  uint16_t  src;
  uint16_t  flashRequest;
  uint16_t  eraseRequest;
  struct SENSOR sensor;
  struct DATETIME datetime;
};

extern struct HEADER reply;
extern struct HEADER header;
extern struct WAKENODE wake;
extern struct FLASH flash;

// Configuration Registers
#define CC1101_IOCFG2       0x00        // GDO2 output pin configuration
#define CC1101_IOCFG1       0x01        // GDO1 output pin configuration
#define CC1101_IOCFG0       0x02        // GDO0 output pin configuration
#define CC1101_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1        0x04        // Sync word, high byte
#define CC1101_SYNC0        0x05        // Sync word, low byte
#define CC1101_PKTLEN       0x06        // Packet length
#define CC1101_PKTCTRL1     0x07        // Packet automation control
#define CC1101_PKTCTRL0     0x08        // Packet automation control
#define CC1101_ADDR         0x09        // Device address
#define CC1101_CHANNR       0x0A        // Channel number
#define CC1101_FSCTRL1      0x0B        // Frequency synthesizer control
#define CC1101_FSCTRL0      0x0C        // Frequency synthesizer control
#define CC1101_FREQ2        0x0D        // Frequency control word, high byte
#define CC1101_FREQ1        0x0E        // Frequency control word, middle byte
#define CC1101_FREQ0        0x0F        // Frequency control word, low byte
#define CC1101_MDMCFG4      0x10        // Modem configuration
#define CC1101_MDMCFG3      0x11        // Modem configuration
#define CC1101_MDMCFG2      0x12        // Modem configuration
#define CC1101_MDMCFG1      0x13        // Modem configuration
#define CC1101_MDMCFG0      0x14        // Modem configuration
#define CC1101_DEVIATN      0x15        // Modem deviation setting
#define CC1101_MCSM2        0x16        // Main Radio Cntrl State Machine config
#define CC1101_MCSM1        0x17        // Main Radio Cntrl State Machine config
#define CC1101_MCSM0        0x18        // Main Radio Cntrl State Machine config
#define CC1101_FOCCFG       0x19        // Frequency Offset Compensation config
#define CC1101_BSCFG        0x1A        // Bit Synchronization configuration
#define CC1101_AGCCTRL2     0x1B        // AGC control
#define CC1101_AGCCTRL1     0x1C        // AGC control
#define CC1101_AGCCTRL0     0x1D        // AGC control
#define CC1101_WOREVT1      0x1E        // High byte Event 0 timeout
#define CC1101_WOREVT0      0x1F        // Low byte Event 0 timeout
#define CC1101_WORCTRL      0x20        // Wake On Radio control
#define CC1101_FREND1       0x21        // Front end RX configuration
#define CC1101_FREND0       0x22        // Front end TX configuration
#define CC1101_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC1101_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC1101_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC1101_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC1101_RCCTRL1      0x27        // RC oscillator configuration
#define CC1101_RCCTRL0      0x28        // RC oscillator configuration
#define CC1101_FSTEST       0x29        // Frequency synthesizer cal control
#define CC1101_PTEST        0x2A        // Production test
#define CC1101_AGCTEST      0x2B        // AGC test
#define CC1101_TEST2        0x2C        // Various test settings
#define CC1101_TEST1        0x2D        // Various test settings
#define CC1101_TEST0        0x2E        // Various test settings

// Strobe commands
#define CC1101_SRES         0x30        // Reset chip.
#define CC1101_SFSTXON      0x31        // Enable/calibrate freq synthesizer
#define CC1101_SXOFF        0x32        // Turn off crystal oscillator.
#define CC1101_SCAL         0x33        // Calibrate freq synthesizer & disable
#define CC1101_SRX          0x34        // Enable RX.
#define CC1101_STX          0x35        // Enable TX.
#define CC1101_SIDLE        0x36        // Exit RX / TX
#define CC1101_SAFC         0x37        // AFC adjustment of freq synthesizer
#define CC1101_SWOR         0x38        // Start automatic RX polling sequence
#define CC1101_SPWD         0x39        // Enter pwr down mode when CSn goes hi
#define CC1101_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST      0x3C        // Reset real time clock.
#define CC1101_SNOP         0x3D        // No operation.

// Status registers
#define CC1101_PARTNUM      0x30        // Part number
#define CC1101_VERSION      0x31        // Current version number
#define CC1101_FREQEST      0x32        // Frequency offset estimate
#define CC1101_LQI          0x33        // Demodulator estimate for link quality
#define CC1101_RSSI         0x34        // Received signal strength indication
#define CC1101_MARCSTATE    0x35        // Control state machine state
#define CC1101_WORTIME1     0x36        // High byte of WOR timer
#define CC1101_WORTIME0     0x37        // Low byte of WOR timer
#define CC1101_PKTSTATUS    0x38        // Current GDOx status and packet status
#define CC1101_VCO_VC_DAC   0x39        // Current setting from PLL cal module
#define CC1101_TXBYTES      0x3A        // Underflow and # of bytes in TXFIFO
#define CC1101_RXBYTES      0x3B        // Overflow and # of bytes in RXFIFO
#define CC1101_NUM_RXBYTES  0x7F        // Mask "# of bytes" field in _RXBYTES

// Other memory locations
#define CC1101_PATABLE      0x3E
#define CC1101_TXFIFO       0x3F
#define CC1101_RXFIFO       0x3F

// Masks for appended status bytes
#define CC1101_LQI_RX       0x01        // Position of LQI byte
#define CC1101_CRC_OK       0x80        // Mask "CRC_OK" bit within LQI byte

// Definitions to support burst/single access:
#define CC1101_WRITE_BURST  0x40
#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0

// State machine states
#define CC1101_IDLE         1
#define CC1101_XOFF         2
#define CC1101_TX           19
#define CC1101_TX_DONE      20
#define CC1101_TX_UNDERFLOW 22

#define MAX_RETRIES 5
//#define csn_pin 53
//#define PORT_SPI_MISO  PINB
//#define BIT_SPI_MISO  3
//#define PORT_GDO0  PINE
//#define BIT_GDO0  4



void rf_spi_write_reg(char addr, char value);
void rf_spi_write_burst_reg(char addr, char *buffer, char count);
char rf_spi_read_reg(char addr);
void rf_spi_read_burst_reg(char addr, char *buffer, char count);
char rf_spi_read_status(char addr);
void rf_spi_strobe(char strobe);
void rf_spi_powerup_reset(void);

void rf_write_settings_250kbps(void);
void rf_write_settings_1_2kbps(void);
void rf_reconfigure(uint8_t int_on); 

void rf_init(char);
void rf_status(void);
void rf_receive(void);
void rf_send(void);
void rf_set_address(char address); 
void rf_debug(void);
void rf_flush(void);
void rf_sleep(void);


void rf_enable_wor(void);
void write_wor_reg(void);
char rf_wor_tx(char,char);
char rf_xmit(long);
char rf_write(const void* buf, int len);
char rf_read(void* buf,int len); 
char rf_check_receive(void);
char rf_available(void);
char rf_wait(char retries); 
void rf_nak(long myID);
char rf_ack(char response, long myID);
void rf_start_listening(void);
char rf_available(void);
void packet_end_seen_ISR(void);

char rf_receive_packet(void);
char rf_send_packet(char *buffer, char size);
char rf_wait_flash(char retries);