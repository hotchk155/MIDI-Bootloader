#include <system.h>

/*
PIC16F1825
14K FLASH = 14336 bytes (0x3800)
Bootloader page will be at 0x3700 
Bootloader code will begin at 0x3705

SYSEX FILE

0xF0 0x00 0x7F 0x12
[hh ll][hh ll][hh ll][hh ll][hh ll]....
 len    addrh  addrl  data0  data1...
0xF7

*/


// PIC CONFIG BITS
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

// Replacement reset vector to take us into the bootloader
// 0x0000	31B7  	MOVLP 0x37
// 0x0001	2805  	GOTO	0x00000005
// 0x0002	0000 	NOP
// 0x0003	0000 	NOP
#pragma DATA 0x0000, 0x31b7, 0x2805, 0x0000, 0x0000

// Initial data for the location that will store the application reset vector
//  0x3700 	018A 	CLRF PCLATH	(ensure relative jump is within page 0)
//	0x3701 	0000 	NOP			(replaced by original reset vector byte 0)
//  0x3702 	0000 	NOP			(replaced by original reset vector byte 1)
//	0x3703 	CLRF 	PCLATH		(replaced by original reset vector byte 2)
//	0x3704 	CLRF 	PCL			(replaced by original reset vector byte 3)
// 	0x3705 	..bootloader..
#pragma DATA 0x3700, 0x018A, 0x0000, 0x0000, 0x018A, 0x0082

#define BOOTLOADER_ADDR_HI 0x37

typedef unsigned char byte;	

#define P_LED1		lata.2
#define P_LED2		latc.2
#define P_SWITCH portc.3

#define MIDI_SYNCH_TICK     	0xf8
#define MIDI_SYNCH_START    	0xfa
#define MIDI_SYNCH_CONTINUE 	0xfb
#define MIDI_SYNCH_STOP     	0xfc
#define MIDI_SYSEX_BEGIN     	0xf0
#define MIDI_SYSEX_END     		0xf7

#define MY_SYSEX_ID0	0x00
#define MY_SYSEX_ID1	0x7f
#define MY_SYSEX_ID2	0x12

enum {
	SYSEX_IDLE, 
	SYSEX_IGNORE, 
	SYSEX_ID0,
	SYSEX_ID1,
	SYSEX_ID2,
	SYSEX_DATAHI,
	SYSEX_DATALO
};
				
				
enum {
	HEX_LEN,
	HEX_ADDRH,
	HEX_ADDRL,
	HEX_DATA
};


void error() 
{
	for(;;) {
		P_LED1 = 1;
		P_LED2 = 0;
		delay_ms(100);
		P_LED1 = 0;
		P_LED2 = 1;
		delay_ms(100);
	}
}

// Keep data global to avoid leaving anything on stack
byte ch;
byte data;
byte sysex_state; 
byte hex_state;
byte bytes_to_read;
byte addr_h;
byte addr_l;

		
void read_sysex()
{
	P_LED1 = 0;
	P_LED1 = 1;
	sysex_state = SYSEX_IDLE;

	// loop until we've got our sysex 
	for(;;) {
			
		// is a character present at serial port?
		if(pir1.5) {  
		
			// read character, clearing flag
			ch = rcreg; 
			
			// is this a realtime/system message?
			if((ch & 0xf0) == 0xf0)
			{
				switch(ch)
				{
				// REALTIME
				case MIDI_SYNCH_TICK:
				case MIDI_SYNCH_START:
				case MIDI_SYNCH_CONTINUE:
				case MIDI_SYNCH_STOP:
					// ok, we'll let this one go!
					break;	
				// START OF SYSEX
				case MIDI_SYSEX_BEGIN:
					if(sysex_state != SYSEX_IDLE)						
						error(); // not valid here
					sysex_state = SYSEX_ID0; 
					break;
				case MIDI_SYSEX_END:
					switch(sysex_state) {
						case SYSEX_IGNORE:
						case SYSEX_IDLE: 				
							break;			
						case SYSEX_DATAHI:
							if(hex_state == HEX_LEN) {
								return; // the only valid state to end up in!								
							}
							// fall thru
						default:
							error();
					}
				}
			}    
			else if(!!(ch & 0x80)) // channel status
			{
				// status byte invalid in syste
				switch(sysex_state) {
				case SYSEX_IDLE:
				case SYSEX_IGNORE: 
					break;
				default:
					error();
					break;
				}
			}
			else switch(sysex_state) {
			case SYSEX_IDLE: 
			case SYSEX_IGNORE: 
				// ignore data
				break;
			case SYSEX_ID0: // Checking manufacturer id byte #0
				if(ch == MY_SYSEX_ID0)
					sysex_state = SYSEX_ID1;
				else
					sysex_state = SYSEX_IGNORE;
				break;
			case SYSEX_ID1: // Checking manufacturer id byte #1
				if(ch == MY_SYSEX_ID1)
					sysex_state = SYSEX_ID2;
				else
					sysex_state = SYSEX_IGNORE;
				break;
			case SYSEX_ID2: // Checking manufacturer id byte #2
				if(ch == MY_SYSEX_ID2) {
					hex_state = HEX_LEN;
					sysex_state = SYSEX_DATAHI;
					P_LED1 = 1;
					P_LED1 = 1;
				}
				else {
					sysex_state = SYSEX_IGNORE;
				}				
				break;
			case SYSEX_DATAHI: // Reading data. High nybble
				data = ch << 4;
				sysex_state = SYSEX_DATALO;
				break;
			case SYSEX_DATALO: // Reading data. Low nybble, now we can process it
				data |= (ch & 0x0f);
				sysex_state = SYSEX_DATAHI;
				switch(hex_state) {
				case HEX_LEN:	// getting record length
					bytes_to_read = data;
					hex_state = HEX_ADDRH;
					break;	
				case HEX_ADDRH:	// getting high byte of address
					addr_h = data;
					hex_state = HEX_ADDRL;
					break;
				case HEX_ADDRL: // getting low byte of address
					addr_l = data;
					hex_state = HEX_DATA;
					break;
				case HEX_DATA: // getting data bytes
					if(addr_h >= BOOTLOADER_ADDR_HI) {
						// don't overwrite ourself!
						break;
					}
					else if(addr_h == 0x00 && addr_l < 0x04) {
						// the reset vector is to be moved
						// up into the bootloader area
						eeadrh = BOOTLOADER_ADDR_HI;
						eeadr = addr_l;								
					}
					else {
						// otherwise use the requested address
						eeadrh = addr_h;
						eeadr = addr_l;								
					}
					
					// Write data byte into EEPROM
					eecon1.7 = 1; //EEPGD
					eecon1.2 = 1; //WREN
					intcon.7 = 0; //GIE
					eecon2 = 0x55;
					eecon2 = 0xAA; // unlock sequence
					eecon1.1 = 1; // WR
					nop();
					nop();
					nop();
					eecon1.1 = 0; // WR

					// any more bytes to read?
					if(!--bytes_to_read) {
						// go back to read next block
						hex_state = HEX_LEN;
					}
					else {
						// 
						++addr_l;
					}
					break;
				}
			}			
		}
	}
}

void main( )
{	
	osccon = 0b01111010; // 16MHz internal
	
	//          76543210
	txsta =   0b00000000;
	rcsta =   0b10110000;
	baudcon = 0b00001000;
	spbrgh = 0;		
	spbrg = 31;		//31250
	
	trisa =   0b11111011;
	trisc =   0b11111011;

	delay_ms(10);	// short delay to allow input line to settle	
	if(!P_SWITCH) // is the switch pressed?
		read_sysex();
		
	// Jump into the saved app reset vector
	asm
	{
		MOVLP 0x37
		GOTO  0x00
	};		
		
}


