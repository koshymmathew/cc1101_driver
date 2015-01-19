#include "rf.h"
//#include <msp430.h>
//#include <stdio.h>
//#include <stdint.h>
//#include "bitop.h"
//#include "config.h"
//#include "clock.h"
//pinMode(csn_pin,OUTPUT);
#define MAX_RETRIES 5
#define csn_pin 53
#define PORT_SPI_MISO  PINB
#define BIT_SPI_MISO  3
#define PORT_GDO0  PINE
#define BIT_GDO0  4

#define select_rf() digitalWrite(csn_pin,LOW);//clear_bit(OUT(RF_CS_PORT),RF_CS)
#define unselect_rf() digitalWrite(csn_pin,HIGH);
#define wait_miso() ;//while(bitRead(PORT_SPI_MISO, BIT_SPI_MISO))
#define wait_tx_sync() while(bitRead(PORT_GDO0, BIT_GDO0))
#define wait_tx_done() while(!bitRead(PORT_GDO0, BIT_GDO0))
#define wait_rx_sync() while(bitRead(PORT_GDO0, BIT_GDO0))
#define wait_rx_done() while(!bitRead(PORT_GDO0, BIT_GDO0))

#define rf_spi_send_receive SPI.transfer


#define FLASHDONE 77
#define FLASHNOTDONE 88
#define FLASHNUMBER 55
#define ERASENUMBER 66
#define FLASH_SIZE 4000  //(20*FLASH_SIZE) bytes of flash data to send 
#define DEVICEID 0x04


struct HEADER reply;
struct HEADER header;
struct WAKENODE wake;
struct FLASH flash;
volatile unsigned int packet_recv_flag=0; //flag indicating if a packet has been recieved


//struct HEADER test;
//struct HEADER header;

char rf_settings_1_2kbps[0x30] = {
	0x25, // IOCFG2 GDO2 Output Pin Configuration //monitor Event1 on GDO2
	0x2E, // IOCFG1 GDO1 Output Pin Configuration
	0x06, // IOCFG0 GDO0 Output Pin Configuration
	0x47, // FIFOTHR RX FIFO and TX FIFO Thresholds
	0xD3, // SYNC1 Sync Word, High Byte
	0x91, // SYNC0 Sync Word, Low Byte
	0xFF, // PKTLEN Packet Length
	0x44, // PKTCTRL1 Packet Automation Control //default 0x04 // (RX) 0x64 for 24 byte preamble quality estimator(PQI) threshold //0x44 for tx
	0x05, // PKTCTRL0 Packet Automation Control //packet length configured by first byte after sync word
	0x00, // ADDR Device Address
	0x00, // CHANNR Channel Number
	0x06, // FSCTRL1 Frequency Synthesizer Control
	0x00, // FSCTRL0 Frequency Synthesizer Control
	0x23, // FREQ2 Frequency Control Word, High Byte
	0x31, // FREQ1 Frequency Control Word, Middle Byte
	0x3B, // FREQ0 Frequency Control Word, Low Byte
	0xF5, // MDMCFG4 Modem Configuration
	0x83, // MDMCFG3 Modem Configuration
	0x13, // MDMCFG2 Modem Configuration
	0x72, // MDMCFG1 Modem Configuration //0x72 indicates a 24 byte preamble //0x22 is default
	0xF8, // MDMCFG0 Modem Configuration
	0x15, // DEVIATN Modem Deviation Setting
	0x07, // MCSM2 Main Radio Control State Machine Configuration //0x07 default //0x18 for WOR RX_TIME_RSSI=1, RX_TIME_QUAL=1
	0x30, // MCSM1 Main Radio Control State Machine Configuration
	0x18, // MCSM0 Main Radio Control State Machine Configuration
	0x16, // FOCCFG Frequency Offset Compensation Configuration
	0x6C, // BSCFG Bit Synchronization Configuration
	0x03, // AGCCTRL2 AGC Control
	0x40, // AGCCTRL1 AGC Control
	0x91, // AGCCTRL0 AGC Control
	0x87, // WOREVT1 High Byte Event0 Timeout
	0x6B, // WOREVT0 Low Byte Event0 Timeout
	0xFB, // WORCTRL Wake On Radio Control
	0x56, // FREND1 Front End RX Configuration
	0x10, // FREND0 Front End TX Configuration
	0xE9, // FSCAL3 Frequency Synthesizer Calibration
	0x2A, // FSCAL2 Frequency Synthesizer Calibration
	0x00, // FSCAL1 Frequency Synthesizer Calibration
	0x1F, // FSCAL0 Frequency Synthesizer Calibration
	0x41, // RCCTRL1 RC Oscillator Configuration
	0x00, // RCCTRL0 RC Oscillator Configuration
	0x59, // FSTEST Frequency Synthesizer Calibration Control
	0x7F, // PTEST Production Test
	0x3F, // AGCTEST AGC Test
	0x81, // TEST2 Various Test Settings
	0x35, // TEST1 Various Test Settings
	0x09, // TEST0 Various Test Settings
};
char rf_settings_250_kbps[0x30] = {
	0x25,  // IOCFG2        GDO2 Output Pin Configuration //monitor Event1 on GDO2 
	0x2E,  // IOCFG1        GDO1 Output Pin Configuration
	0x06,  // IOCFG0        GDO0 Output Pin Configuration
	0x47,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
	0xD3,  // SYNC1         Sync Word, High Byte
	0x91,  // SYNC0         Sync Word, Low Byte
	0xFF,  // PKTLEN        Packet Length
	0x44,  // PKTCTRL1      Packet Automation Control //default 0x04 // (RX) 0x64 for 24 byte preamble quality estimator(PQI) threshold //0x44 for tx 
	0x05,  // PKTCTRL0      Packet Automation Control //packet length configured by first byte after sync word 
	0x00,  // ADDR          Device Address
	0x00,  // CHANNR        Channel Number
	0x12,  // FSCTRL1       Frequency Synthesizer Control
	0x00,  // FSCTRL0       Frequency Synthesizer Control
	0x23,  // FREQ2         Frequency Control Word, High Byte
	0x31,  // FREQ1         Frequency Control Word, Middle Byte
	0x3B,  // FREQ0         Frequency Control Word, Low Byte
	0x2D,  // MDMCFG4       Modem Configuration
	0x3B,  // MDMCFG3       Modem Configuration
	0x93,  // MDMCFG2       Modem Configuration
	0x72,  // MDMCFG1       Modem Configuration  //0x72 indicates a 24 byte preamble  //0x22 is default  
	0xF8,  // MDMCFG0       Modem Configuration
	0x62,  // DEVIATN       Modem Deviation Setting
	0x07,  // MCSM2         Main Radio Control State Machine Configuration   //0x07 default  //0x18 for WOR   RX_TIME_RSSI=1, RX_TIME_QUAL=1 
	0x30,  // MCSM1         Main Radio Control State Machine Configuration
	0x18,  // MCSM0         Main Radio Control State Machine Configuration
	0x1D,  // FOCCFG        Frequency Offset Compensation Configuration
	0x1C,  // BSCFG         Bit Synchronization Configuration
	0xC7,  // AGCCTRL2      AGC Control
	0x00,  // AGCCTRL1      AGC Control
	0xB0,  // AGCCTRL0      AGC Control
	0x87,  // WOREVT1       High Byte Event0 Timeout
	0x6B,  // WOREVT0       Low Byte Event0 Timeout
	0xFB,  // WORCTRL       Wake On Radio Control
	0xB6,  // FREND1        Front End RX Configuration
	0x10,  // FREND0        Front End TX Configuration
	0xEA,  // FSCAL3        Frequency Synthesizer Calibration
	0x2A,  // FSCAL2        Frequency Synthesizer Calibration
	0x00,  // FSCAL1        Frequency Synthesizer Calibration
	0x1F,  // FSCAL0        Frequency Synthesizer Calibration
	0x41,  // RCCTRL1       RC Oscillator Configuration
	0x00,  // RCCTRL0       RC Oscillator Configuration
	0x59,  // FSTEST        Frequency Synthesizer Calibration Control
	0x7F,  // PTEST         Production Test
	0x3F,  // AGCTEST       AGC Test
	0x81,  // TEST2         Various Test Settings
	0x35,  // TEST1         Various Test Settings
	0x09,  // TEST0         Various Test Settings
};

int i=0;

void rf_write_settings_250kbps(void) {
	for(int i=0; i<0x30; i++) {
		rf_spi_write_reg(i,rf_settings_250_kbps[i]);	
	}
	
	//rf_spi_write_burst_reg(0x00, rf_settings, 0x30); 
}
void rf_write_settings_1_2kbps(void) {
	for(int i=0; i<0x30; i++) {
		rf_spi_write_reg(i,rf_settings_1_2kbps[i]);	
	}
	
	//rf_spi_write_burst_reg(0x00, rf_settings, 0x30); 
}

// PATABLE (10 dBm output power)
char paTable[] = {0x03}; //0x03 for lowest power 
char paTableLen = 1;

void rf_spi_write_reg(char addr, char value) {
	select_rf();
	wait_miso();
	rf_spi_send_receive(addr);
	rf_spi_send_receive(value);
	unselect_rf();
}

void rf_spi_write_burst_reg(char addr, char *buffer, char count) {
	unsigned int i;

	select_rf();
	wait_miso(); 
	rf_spi_send_receive((addr | CC1101_WRITE_BURST)); // Send address

	for (i = 0; i < count; i++) {
		rf_spi_send_receive(buffer[i]);                  // Send data
	}

	unselect_rf();
}

char rf_spi_read_reg(char addr) {
	char byte;

	select_rf();
	wait_miso(); 
	rf_spi_send_receive((addr | CC1101_READ_SINGLE));// Send address
	byte = rf_spi_send_receive(0);                            // Read data
	unselect_rf();

	return byte;
}

void rf_spi_read_burst_reg(char addr, char *buffer, char count) {
	select_rf();
	wait_miso(); 

	rf_spi_send_receive((addr | CC1101_READ_BURST));// Send address

	for( i = 0; i < count; i++) {
		buffer[i] = rf_spi_send_receive(0);
	}
	unselect_rf();
}

char rf_spi_read_status(char addr) {
	char status;

	select_rf();
	wait_miso();
	rf_spi_send_receive((addr | CC1101_READ_BURST));
	status = rf_spi_send_receive(0);
	unselect_rf();

	return status;
}

void rf_spi_strobe(char strobe) {
	select_rf();
	wait_miso();
	rf_spi_send_receive(strobe);
	unselect_rf();
}

void rf_powerup_reset(void) {
	unselect_rf();
	delay(20);
	select_rf();
	delay(20);
	unselect_rf();
	delay(25);

	rf_spi_strobe(CC1101_SRES);
}

void rf_init(char dataRate) {
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
  	SPI.setDataMode(SPI_MODE0);
  	SPI.setClockDivider(SPI_CLOCK_DIV4);
	Serial.println("init RF ");
	rf_powerup_reset();
	if(dataRate==1){
		 rf_write_settings_250kbps();                       
	}
	else{
		//default to the slower speed 
		rf_write_settings_1_2kbps(); 
	}                      
	rf_spi_write_burst_reg(CC1101_PATABLE, paTable, paTableLen);
	rf_set_address(0x09);
	rf_spi_strobe(CC1101_SIDLE);
	Serial.println("done init ...\n ");
	
}

void rf_reconfigure(unsigned char int_on) {
	
	if(int_on) {
		//rf_settings[1] = 0x06;	
	}
	else {
		//rf_settings[1] = 0x2E;	
	}
	
	//rf_write_settings();
	//rf_spi_strobe(CC1101_SIDLE);

	delay(100);	
}

void rf_debug(void) {
	char buffer[0x2F];

	//rf_spi_read_burst_reg(0x00, buffer, 0x2E); // Pull data

	for( i=0x00; i<0x2F; i++) {
		Serial.print(i,HEX);Serial.print("= ");Serial.println(rf_spi_read_reg(i),HEX);	
	}

	for( i=0x30; i<0x3F; i++) {
		//Serial.print(i,HEX);Serial.println(rf_spi_read_reg(i),HEX);		
	}
}

void rf_sleep(void) {
	rf_spi_strobe(CC1101_SPWD);           
}

void rf_set_address(char address) {
	rf_spi_write_reg(0x09,address);	
}


char rf_write(const void* buf, int len)
{
  //uint8_t status=0;

	const unsigned char* current = (const unsigned char*)(buf);

	uint8_t data_len = min(len,32);
	//uint8_t blank_len = dynamic_payloads_enabled ? 0 : 32 - data_len;
	
	//Serial.print("[Writing bytes] \n");
	//Serial.println(data_len);
	
	select_rf(); //******************HAVE TO SEND THE LENGHT FIRST ***********************
	wait_miso();
	rf_spi_send_receive(CC1101_TXFIFO); //CAN BE CHANGED TO _write_reg
	rf_spi_send_receive(data_len);
	unselect_rf();

	select_rf();
	wait_miso(); 
	rf_spi_send_receive((CC1101_TXFIFO | CC1101_WRITE_BURST)); // Send address
		
	for ( i = 0; i < data_len; i++) {
		rf_spi_send_receive(current[i]);                  // Send data
	}

	unselect_rf();
	
	//return status;
  	// make sure we are idle then go to tx mode
	rf_spi_strobe(CC1101_SIDLE);           
	//wait_miso(); // TODO because of the level shifter it dosent see this ???????
	rf_spi_strobe(CC1101_STX);           

	
	// wait for GDO1 to assert -- sync sent
	wait_tx_sync();
	// wait for GDO1 to deassert -- packet finished sending
	wait_tx_done();
	
	Serial.println(" tx done  \n");
	return 1;	// success
}


char rf_read(void* buf,int len) {
	char status[2];
	char pktLen;
	char length=32;

	unsigned char* current =  ( unsigned char*)(buf);
	
	//Serial.println("receiving \n");

	//delay(100 ms);

	//delay( 100 ms); //needs this delay to work ??????????????
	
	if ((rf_spi_read_status(CC1101_RXBYTES) & CC1101_NUM_RXBYTES)) {
		pktLen = rf_spi_read_reg(CC1101_RXFIFO);
		//Serial.print("bytes in rx FIFO ");
		//Serial.println(pktLen);

		if ((pktLen <= length)) {
			//rf_spi_read_burst_reg(CC1101_RXFIFO, buf, pktLen);
			select_rf();
			wait_miso(); 

			rf_spi_send_receive((CC1101_RXFIFO | CC1101_READ_BURST));// Send address
			
			//Serial.print("writing into memory address = %p\n", (void *)(&buf));
			unsigned char spi_test;
			for( i = 0; i < pktLen; i++) {
				
				 spi_test= rf_spi_send_receive(0);
				 //delay(5 ms); //NEEDS THE DELAY HERE OTHERWISE DATA GETS CORRUPTED	
				 //Serial.print("char read from spi is = %u \n",spi_test); //debug
				 *current++ = spi_test;
				}
			
			unselect_rf();
			//rx_packet.length = pktLen;
			rf_spi_read_burst_reg(CC1101_RXFIFO, status, 2);

			//Serial.print("status-0 = %u status-1 = %u \n",status[0],status[1]);
			//rx_packet.rssi = status[0];
			return (char)((status[CC1101_LQI_RX]&CC1101_CRC_OK)!=0);
		} 
		else {
			Serial.println("length is wrong,flushing rx \n");
			//rx_packet.length = pktLen;
			rf_spi_strobe(CC1101_SIDLE); 
			rf_spi_strobe(CC1101_SFRX);
			delay(100);
			return 0;
		}
	}
	else {
		Serial.println("rx failed \n");
		rf_spi_strobe(CC1101_SIDLE);  
		rf_spi_strobe(CC1101_SFRX);
		delay(100);
		return 0;
	}
}

// Get Ack from relay or timeout
char rf_wait(char retries) {
	rf_start_listening();

	//Serial.println("waiting for DATA \n");
	char response=0;
	char reply_success=0;
	char reply_tx_success=0;
	packet_recv_flag =0; //reset the flag because this is set on TX first 
	
	do{
		int i=0;
		while(i<500) {
			if(response=rf_available()) break;
			i = i+1;
			delay(1);
		}
	//Serial.print("retries %u \n", retries);
	retries--;	
	}
	while(retries>0 && !response);
	
	if(response){
		//packet_recv_flag =0; //reset the flag 	
		
		reply_success = rf_read(&reply,sizeof(reply));
		//rf_receive();
		if(reply_success){
			Serial.println("Got a payload, YAY \n");
			Serial.print("ID ");Serial.println(reply.ID,HEX);
			Serial.print("SRC ");Serial.println(reply.src);
			Serial.print("flashRequest ");Serial.println(reply.flashRequest);
			Serial.print("eraseRequest ");Serial.println(reply.eraseRequest);	
			Serial.print("Temperature ");Serial.println(reply.sensor.val_5);
			Serial.print("Humidity = ");Serial.println(reply.sensor.val_0);	
			Serial.print("Soil M= ");Serial.println(reply.sensor.val_1);	
			Serial.print("Region 2 = ");Serial.println(reply.sensor.val_2);	
			Serial.print("Region 3 = ");Serial.println(reply.sensor.val_3);		
			Serial.print("Region 4 = ");Serial.println(reply.sensor.val_4);
			Serial.print("YEAR = ");Serial.println(reply.datetime.year);	
			Serial.print("Month = ");Serial.println(reply.datetime.month);	
			Serial.print("Day = ");Serial.println(reply.datetime.day);	
			Serial.print("Hour = ");Serial.println(reply.datetime.hour);		
			Serial.print("Minutes = ");Serial.println(reply.datetime.min);
			Serial.print("Seconds = ");Serial.println(reply.datetime.sec);			
			Serial.println("\n");	
			
			//reply.src =0; //base 
			
			reply_tx_success = rf_write(&reply,sizeof(reply));	 //if we get a payload send the mesasge back to confirm ACK
			//delay(500 ms); //needs this delay after the write 
			
			if(reply_tx_success)
			{
				Serial.print("REPLY for ACK success ");Serial.println(reply_tx_success);	
				return 1;
			}
			else
			{
				return 0;
			}
		
		}
		else{
			Serial.println(" rf_read() failed \n");
			rf_spi_strobe(CC1101_SIDLE); 
			rf_spi_strobe(CC1101_SFRX);
			delay(100);
			//rf_start_listening();
			return 0;
			
		}
		
	}
	else 
	{
		//rf_nak(myID);
		Serial.println(" there was no RF transmission from node \n");
		return 0;
		
	}
	 
	//return(ackFlag); 
}
// Get Ack from relay or timeout
char rf_wait_flash(char retries) {
	rf_start_listening();

	//Serial.println("waiting for DATA \n");
	char response=0;
	char reply_success=0;
	char reply_tx_success=0;
	packet_recv_flag =0; //reset the flag because this is set on TX first 
	
	do{
		int i=0;
		while(i<500) {
			if(response=rf_available()) break;
			i = i+1;
			delay(1);
		}
	//Serial.print("retries %u \n", retries);
	retries--;	
	}
	while(retries>0 && !response);
	
	if(response){
		//packet_recv_flag =0; //reset the flag 	
		
		reply_success = rf_read(&flash,sizeof(flash));
		//rf_receive();
		if(reply_success){
			//Serial.println("Got a flash payload \n");
			Serial.print("ID ");Serial.println(flash.ID,HEX);
			Serial.print("SRC ");Serial.println(flash.src);
			//Serial.print("FLASH Done  ");Serial.println(flash.done);
			Serial.print("FLASH crc  ");Serial.println(flash.crc);	
	    	//for(int j=0;j<5;j++)
    		//{
		      //Serial.print(flash.txbuffer[j]);
		      //Serial.print(",");
			//}
			//Serial.println("\n");	
			
			//reply.src =0; //base 
			delay(20); //works 50% with this delay
			reply_tx_success = rf_write(&flash,sizeof(flash));	 //if we get a payload send the mesasge back to confirm ACK
			//delay(40); //works 50% with this delay

			
			if(reply_tx_success)
			{
				Serial.print("REPLY for ACK success ");Serial.println(reply_tx_success);	
				return 1;
			}
			else
			{
				return 0;
			}
		
		}
		else{
			Serial.println(" rf_read() failed \n");
			rf_spi_strobe(CC1101_SIDLE); 
			rf_spi_strobe(CC1101_SFRX);
			delay(100);
			//rf_start_listening();
			return 0;
			
		}
		
	}
	else 
	{
		//rf_nak(myID);
		Serial.println(" there was no RF transmission from node \n");
		return 0;
		
	}
	 
	//return(ackFlag); 
}
char rf_check_receive(void){
	
	
	char reply_success=0;
	char reply_tx_success=0;
	//Serial.print("check receiving \n");
	//packet_recv_flag=0; //reset flag
	
	if(rf_available()){
		
		//packet_recv_flag =0; //reset the flag 	
		
		reply_success = rf_read(&reply,sizeof(reply));
		//rf_receive();
		
		if(reply_success){
			Serial.println("Got a payload, YAY \n");
			Serial.print("ID %04X \n");Serial.println(reply.ID);
			Serial.print("SRC %u \n");Serial.println(reply.src);
			Serial.print("flashRequest %u \n");Serial.println(reply.flashRequest);
			Serial.print("eraseRequest %u \n");Serial.println(reply.eraseRequest);	
			Serial.print("temp %u \n");Serial.println(reply.sensor.val_5);
			Serial.print("Region 0 = %u \n");Serial.println(reply.sensor.val_0);	
			Serial.print("Region 1 = %u \n");Serial.println(reply.sensor.val_1);	
			Serial.print("Region 2 = %u \n");Serial.println(reply.sensor.val_2);	
			Serial.print("Region 3 = %u \n");Serial.println(reply.sensor.val_3);		
			Serial.print("Region 4 = %u \n");Serial.println(reply.sensor.val_4);
			Serial.print("\n");	
		
			reply_tx_success = rf_write(&reply,sizeof(reply));	 //if we get a payload send the mesasge back to confirm ACK
		//delay(500 ms); //needs this delay after the write 
		
			if(reply_tx_success)
			{
				Serial.print("REPLY TX success %u\n");Serial.println(reply_tx_success);	
				return 1;
			}
			
			else
			{
				return 0;
			}
		
		}
		else{
			Serial.println(" rf_read() failed \n");
			rf_spi_strobe(CC1101_SIDLE); 
			rf_spi_strobe(CC1101_SFRX);
			delay(100);
			//rf_start_listening();
			return 0;
			
		}
		//return(reply_success);
		
	}//if rf_avaialbale() 
		
}

char rf_xmit(long myID) {
	
	//__disable_interrupt();	
	
	char tx_success;
	//struct  HEADER header;

	Serial.println("Taking measurements...\n");
	
	// init and take temp reading
	//temp_init(ON);
	header.src = DEVICEID;
	header.ID = myID;
	
	header.eraseRequest =45;
	header.flashRequest =56;
	
	header.sensor.val_5 = 1229;//temp_read();
	header.sensor.val_0 = 1230;
	header.sensor.val_1 = 1231;
	header.sensor.val_2 = 1232;
	header.sensor.val_3 = 1233;
	header.sensor.val_4 = 1234;
	
	
	Serial.print("ID %04X \n");Serial.println(reply.ID);
	Serial.print("SRC %u \n");Serial.println(reply.src);
	Serial.print("flashRequest %u \n");Serial.println(reply.flashRequest);
	Serial.print("eraseRequest %u \n");Serial.println(reply.eraseRequest);	
	Serial.print("temp %u \n");Serial.println(reply.sensor.val_5);
	Serial.print("Region 0 = %u \n");Serial.println(reply.sensor.val_0);	
	Serial.print("Region 1 = %u \n");Serial.println(reply.sensor.val_1);	
	Serial.print("Region 2 = %u \n");Serial.println(reply.sensor.val_2);	
	Serial.print("Region 3 = %u \n");Serial.println(reply.sensor.val_3);		
	Serial.print("Region 4 = %u \n");Serial.println(reply.sensor.val_4);

	Serial.println("\n");

	// output readings
	//Serial.print("Raw Temperature: %u\n",header.sensor.val_5);
	
	Serial.print("the size of struct is %u\n");Serial.println(sizeof(header));
	
	tx_success = rf_write(&header,sizeof(header));
	//rf_send();	
	if(tx_success)
	{
		Serial.print("TX success %u\n");Serial.println(tx_success);	
		return 1;
	}
	
	else
	{
		return 0;
	}
	
	
}

void rf_start_listening(void){
	
	rf_spi_strobe(CC1101_SIDLE);           
	
	rf_spi_strobe(CC1101_SRX);           
	
	//delay(100);
	
}

char rf_available(){
	
	if (packet_recv_flag)//packet done 
	{
		//Serial.println("packet available \n");
		packet_recv_flag =0; //reset the flag 

		return 1;
	}
	
	return 0;
}


void write_wor_reg(void){
	
	rf_spi_write_reg(CC1101_WORCTRL,0x78);
	rf_spi_write_reg(CC1101_WOREVT1,0x11);
	rf_spi_write_reg(CC1101_WOREVT0,0x9A);
	rf_spi_write_reg(CC1101_MCSM2,0x18);
	rf_spi_write_reg(CC1101_PKTCTRL1,0x64);
	
}

void rf_enable_wor(void){
	
	rf_spi_strobe(CC1101_SWOR); //start automatic rx polling sequence
	delay(100);
}

char rf_wor_tx(char wake_node_num,char rf_command){
	char tx_success;	
	wake.node_num= wake_node_num;
	wake.command = rf_command;
	
	Serial.print("waking node # ");Serial.print(wake_node_num);Serial.print("with command # "); Serial.println(rf_command);
	tx_success = rf_write(&wake,sizeof(wake));
	delay(100);
	if(tx_success)
	{
		Serial.print("TX success ");Serial.println(tx_success);	
		return 1;
	}
	
	else
	{
		return 0;
	}
	
}

// The ISR assumes the interrupt came from GDO0. GDO0 fires indicating that
// CCxxxx received a packet
void packet_end_seen_ISR(void)
{
    // if GDO fired
  if(!bitRead(PORT_GDO0, BIT_GDO0))
   {
    //Serial.print("Woken UP !! GDO interrupt fired \n"); //end of packet seen by WOR
	
	packet_recv_flag = 1; //packet has been received 
	}
	  //Serial.print("clearing the interrupt flag \n");
	  //clear_bit(IFG(RF_RX_GDO_PORT),RF_RX_GDO);   // P2.6 IFG cleared
	//__bic_status_register_on_exit(LPM0_bits + GIE); //exit at full power.
	
}


