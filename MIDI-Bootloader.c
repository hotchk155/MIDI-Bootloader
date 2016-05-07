/////////////////////////////////////////////////////////////////////////////////////////
//
// SIXTY FOUR PIXELS - MIDI BOOTLOADER FOR PIC16F1825
//
// 2016/hotchk155
//
// SourceBoost C
// Use linker option: -rb 0x1E25
//
// Bootloader code is loaded to high memory (address 0x1e25)
// Block from 0x1e20..0x01e24 contains the saved firmware reset vector
//
// Firmware provided in a MIDI System Exclusive file with 32 words (64 bytes) 
// of program code per sysex buffer. 
// Sysex "manufacturer ID" is 0x00 0x7F 0x12
// 1 byte sequence number 1..127 between ID and program data
// Sequence number increments for each buffer
// Final buffer is marked by zero sequence number
// 
// [0xF0] [0x00] [0x7F] [0x12] [0x01] [data0] ... [data63] [0xF7]
// [0xF0] [0x00] [0x7F] [0x12] [0x02] [data0] ... [data63] [0xF7]
// [0xF0] [0x00] [0x7F] [0x12] [0x03] [data0] ... [data63] [0xF7]
// [0xF0] [0x00] [0x7F] [0x12] [nnnn] [data0] ... [data63] [0xF7]
// [0xF0] [0x00] [0x7F] [0x12] [0x00] [dummy0] ... [dummy63] [0xF7]
//
// VERSION HISTORY
// 1	07MAY16	First version
// 
/////////////////////////////////////////////////////////////////////////////////////////

//
// INCLUDE FILES
//
#include <system.h>
#include <memory.h>

// PIC CONFIG BITS
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

// Replacement reset vector to take us into the bootloader at address 0x1E30. 
//	0x0000	0x319E  MOVLP 0x1E  
//	0x0001	0x2E25  GOTO  0x625 <-- lower 11 bits of address
//	0x0002	0x0000  NOP
//	0x0003	0x0000  NOP
#pragma DATA 0x0000, 0x319E, 0x2E25, 0x0000, 0x0000

// Initial data for the location that will store the application reset vector
//  0x1E20 	018A 	CLRF PCLATH	(ensure relative jump is within page 0)
//	0x1E21 	0000 	NOP			(replaced by original reset vector byte 0)
//  0x1E22 	0000 	NOP			(replaced by original reset vector byte 1)
//	0x1E23 	CLRF 	PCLATH		(replaced by original reset vector byte 2)
//	0x1E24 	CLRF 	PCL			(replaced by original reset vector byte 3)
// 	0x1E25 	..bootloader..
#pragma DATA 0x1E20, 0x018A, 0x0000, 0x0000, 0x018A, 0x0082

//
// CONSTANTS
//
#define P_LED1				lata.2
#define P_LED2				latc.2
#define P_SWITCH 			portc.3

#define MIDI_SYSEX_BEGIN    0xf0
#define MIDI_SYSEX_END     	0xf7
#define MY_SYSEX_ID0		0x00
#define MY_SYSEX_ID1		0x7f
#define MY_SYSEX_ID2		0x12

#define SAVED_RESET_VECTOR 	0x1E20 

//
// TYPE DEFS
//
typedef unsigned char byte;	

// Define the structure of the sysex buffers containing firmware update
struct {
	byte begin;		// sysex start marker
	byte id0;		// manufacturer id 0 (0x00 = extended id)
	byte id1;		// manufacturer id 1
	byte id2;		// manufacturer id 2
	byte seq;		// 1..127 = incrementing sequence number, 0 = end of data
	byte data[64];	// data buffer for program memory (for sysex each 14 bit word is translated to 2 bytes 0..127)
	byte end;		// sysex end marker
} buffer;

//
// GLOBAL DATA
//

// Temporary storage location for the booloader's reset vector
byte bootloader_reset[8];

// Temporary storage location for the firmware's reset vector
byte firmware_reset[8];

// Address register for program memory read and write operations
int addr;

// Working variables
byte i;
byte seq;

////////////////////////////////////////////////////////
// READ A ROW OF FLASH MEMORY 
// The source address is in the addr variable and must
// be on a flash row boundary (32 words)
// The data is read into 64 byte array buffer.data
void read_flash() 
{	
	eecon1.CFGS = 0; 		// not config space
	eecon1.EEPGD = 1; 		// program memory

	eeadrl = (byte)addr;	// store address low byte
	eeadrh = (addr >> 8);	// store address high byte
	
	// Loop through all 32 words -> 64 bytes
	for(i=0; i<64; i+=2) 
	{	
		// read flash
		eecon1.RD = 1; 	
		nop();
		nop();
	
		// store to buffer in RAM
		buffer.data[i] = eedath;
		buffer.data[i+1] = eedatl;
		
		// next word
		++eeadrl;
	}
}

////////////////////////////////////////////////////////
// ERASE AND WRITE A ROW OF FLASH MEMORY
// The target address is in the addr variable and must
// be on a flash row boundary (32 words)
// The data is read from 64 byte array buffer.data
void write_flash()
{
	// ensure address is on 32-byte flash row boundary
	eeadrl = (addr & 0xE0);
	eeadrh = (addr >> 8);
	
	eecon1.CFGS = 0; 	// not config space
	eecon1.EEPGD = 1; 	// program memory
	eecon1.FREE = 1;	// erase operation
	eecon1.WREN = 1;	// enable write
	
	// ERASE ROW OF FLASH
	eecon2 = 0x55;		
	eecon2 = 0xAA;		
	eecon1.WR = 1;	
	nop();
	nop();			
	
	// LOAD THE WRITE LATCHES FOR THE ROW
	eecon1.LWLO = 1;	
	for(i=0; i<64; i+=2) 
	{
		eedatl = buffer.data[i+1];
		eedath = buffer.data[i];
		eecon2 = 0x55;
		eecon2 = 0xAA;		// special unlock sequence
		eecon1.WR = 1;		// start write
		nop();
		nop();				// NOPs needed during erase
		++eeadrl;
	}
	eecon1.LWLO = 0;	// finished with the latches

	// PERFORM THE WRITE TO FLASH
	eeadrl = (addr & 0xE0);
	eeadrh = (addr >> 8);
	eedatl = buffer.data[1];
	eedath = buffer.data[0];	
	eecon2 = 0x55;
	eecon2 = 0xAA;		// special unlock sequence
	eecon1.WR = 1;		// start write
	nop();
	nop();				// NOPs needed during erase
	
	eecon1.WREN = 0;	// disable writes
}

////////////////////////////////////////////////////////
// ERROR INDICATION
void error(byte b) 
{
	P_LED1 = 0;
	for(;;) {
		for(i=0; i<b; ++i) {
			P_LED2 = 1;
			delay_ms(200);
			P_LED2 = 0;
			delay_ms(200);
		}		
		delay_ms(255);
		delay_ms(255);
	}
}

////////////////////////////////////////////////////////
// SUCCESS INDICATION
void success() {
	for(;;) {
		P_LED1 = 1;
		P_LED2 = 0;
		delay_ms(100);
		P_LED1 = 0;
		P_LED2 = 1;
		delay_ms(100);		
	}
}

////////////////////////////////////////////////////////
// READ SYSEX FILE
void read_sysex() 
{
	seq = 1;
	
	// Start by saving the bootloader reset vector. We
	// will replace the firmware reset vector with this
	addr = 0x0000;
	read_flash();
	memcpy(bootloader_reset, &buffer.data, 8); 

	P_LED1 = 1;
	P_LED2 = 1;

	// there is no way back...
	for(;;) {
			
		// read a full sysex buffer worth of characters
		// from the serial port
		for(i=0; i<sizeof(buffer); ++i) {	
			while(!pir1.5);
			((byte*)&buffer)[i] = rcreg;
		}
		P_LED1 = !P_LED1;
		
		// perform basic validation of the buffer structure
		if(	(buffer.begin != MIDI_SYSEX_BEGIN) ||
			(buffer.id0 != MY_SYSEX_ID0) ||
			(buffer.id1 != MY_SYSEX_ID1) ||
			(buffer.id2 != MY_SYSEX_ID2) ||
			(buffer.end != MIDI_SYSEX_END)
		){
			error(2);
		}
		
		// a zero sequence number indicates the end of the
		// firmware upload. We do not return, instead the user
		// must reset the device to restart it with the new firmware
		if(!buffer.seq) {
			success();
		}
		
		// check that the sequence number matches what we're expecting...
		// if not then we know something has gone wrong (we've missed
		// some data!)
		if(buffer.seq != seq) {
			error(3);
		}
		
		// increment the expected sequence number, avoiding zero, which
		// has special meaning of an end of data marker...
		if(++seq > 127) {
			seq = 1;	
		}	
		
		// Translate the data buffer into a format suitable for programming
		// into the flash memory... shuffle each pair of data bytes so that 
		// the two bytes containing 7 bit values are converted to a 14 bit word, i.e.
		// -------+-------+
		// 7654321076543210
		// .AAAAAAA.BBBBBBB <-- from this
		// ..AAAAAAABBBBBBB <-- to this
		for(i=0; i<64; i+=2) {					
			buffer.data[i+1] |= (buffer.data[i]<<7);
			buffer.data[i] >>= 1;
		}
		
		
		// is this code destined for the first row of flash?
		if(addr == 0x0000) { 
			// bytes 0..7 are the reset vector (i.e. jump to the start of program code)
			// We need to replace the firmware reset vector with the bootloader reset vector,
			// but we also need to save the firmware reset vector into the bootloader memory
			// area so that we can execute it from the bootloader when we want to start the
			// firmware application....
			memcpy(firmware_reset, &buffer.data[0], 8);
			memcpy(&buffer.data[0], bootloader_reset, 8);
			write_flash(); 								
			
			// now we save the firmware reset vector
			addr = SAVED_RESET_VECTOR;
			read_flash(); 									
			memcpy(&buffer.data[2], firmware_reset, 8);	
			write_flash();								
			addr = 0x0000;			
		}
		else if(addr >= SAVED_RESET_VECTOR)	{
			error(4); // prevent bootloader being overwritten
		}
		else {
			// write row to flash
			write_flash();
		}
		addr += 32;
	}
}

////////////////////////////////////////////////////////
// BOOTLOADER ENTRY POINT
void main() 
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
	anselc 	= 0b00000000;

	intcon.GIE = 0; // interrupts not used
	
	delay_ms(10);	// short delay to allow input line to settle	
	if(!P_SWITCH) // is the switch pressed?
		read_sysex();
		
	// otherwise jump into the saved app reset vector at 0x1E20
	asm
	{
		MOVLP 0x1E
		GOTO 0x620
	};				
}