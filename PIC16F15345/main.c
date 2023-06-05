/////////////////////////////////////////////////////////////////////////////////////////
// MIDI BOOTLOADER FOR PIC16F15345
// (c) 2023 Sixty Four Pixels Ltd
// 
// VERSION HISTORY
// 1	05JUN23	First version
// 
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// MEMORY MAP
//
// 0000 bootloader reset vector
// 0004 application interrupt vector
// 0005 application code...
// 1D40 saved application reset vector (must be on a 32-byte FLASH page boundary)
// 1D44 bootloader code...
// 1F7F storage area flash (SAF) used for application settings
// 1FFF end of flash
//
// MPLABX Build Options: Project options|Linker
// Memory Model
//      - ROM Ranges: default,-1F7F-1FFF
// Additional options:  
//      Code offset = 1d44
//      -Wl,-pmyReset=0,-preset_vec=1d40h,-ptext=1d44h
//
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// SYSEX FILE FORMAT
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
/////////////////////////////////////////////////////////////////////////////////////////

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINTPLL// Power-up default value for COSC bits (HFINTOSC with 2x PLL, with OSCFRQ = 16 MHz and CDIV = 1:1 (FOSC = 32 MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = OFF      // Master Clear Enable bit (MCLR pin function is port defined function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF        // WDT operating mode (WDT enabled regardless of sleep; SWDTEN ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/Vpp must be used for programming)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//
// MEMORY ORGANISATION
// 
#define SAVED_RESET_VECTOR 	0x1D40 // must be on 32-byte boundary
asm("PSECT myReset,class=CODE,reloc=2,delta=2");
asm("CustomReset:");
asm("ljmp start");

//
// INCLUDE FILES
//
#define _XTAL_FREQ 32000000
#include <xc.h>
#include <string.h>

//
// CONSTANTS
//
#define P_LED1 LATAbits.LATA5
#define P_LED2 LATCbits.LATC0
#define P_KEY1 PORTCbits.RC3
#define P_KEY2 PORTAbits.RA4
#define P_KEY3 PORTCbits.RC6
#define P_KEY4 PORTCbits.RC7
#define P_KEY5 PORTBbits.RB7

#define TRISA_BITS  0b11011111
#define TRISB_BITS  0b11111111
#define TRISC_BITS  0b11101110

#define WPUA_BITS   0b00010000
#define WPUB_BITS   0b10000000
#define WPUC_BITS   0b11001000

#define MIDI_SYSEX_BEGIN    0xf0
#define MIDI_SYSEX_END     	0xf7
#define MIDI_ACTIVE_SENSE   0xfe
#define MY_SYSEX_ID0		0x00
#define MY_SYSEX_ID1		0x7f
#define MY_SYSEX_ID2		0x25

//
// TYPE DEFS
//
typedef uint8_t byte;	

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

// Working variables (global- do not use stack)
byte i;
byte seq;

////////////////////////////////////////////////////////
// READ A ROW OF FLASH MEMORY 
// The source address is in the addr variable and must
// be on a flash row boundary (32 words)
// The data is read into 64 byte array buffer.data
void read_flash() 
{	
    NVMCON1bits.NVMREGS = 0;        // access program flash memory
	NVMADRL = (byte)addr;           // set word address 
	NVMADRH = (byte)(addr >> 8);	
	
	// Loop through all 32 words -> 64 bytes
	for(i=0; i<64; i+=2) 
	{	        
        NVMCON1bits.RD = 1;             // initiate the read
		buffer.data[i] = NVMDATH;       // move the word into memory
		buffer.data[i+1] = NVMDATL;
        ++NVMADRL;                      // advance to the next word address
	}
}

////////////////////////////////////////////////////////
// ERASE AND WRITE A ROW OF FLASH MEMORY
// The target address is in the addr variable and must
// be on a flash row boundary (32 words)
// The data is read from 64 byte array buffer.data
void write_flash()
{
    // NOTE - FLASH updates must run with interrupts disabled
    
    // prepare the row erase
    NVMADRL = (byte)(addr & 0xE0); // set up base address of the row to erase (32-word boundary)
    NVMADRH = (byte)(addr >> 8);
    NVMCON1bits.NVMREGS = 0;        // access program flash memory
    NVMCON1bits.FREE = 1;           // flag this is an erase operation
    NVMCON1bits.WREN = 1;           // enable write
	NVMCON2 = 0x55;                 // special unlock sequence
	NVMCON2 = 0xAA;		    
	NVMCON1bits.WR = 1;                 // kick off the erase

    // LOAD THE WRITE LATCHES
    // Prepare to write new data into the cleared row. The sequence is to 
    // load the 32 word write buffer (latches), then kick off the write	
	NVMCON1bits.LWLO = 1;           // indicate we're setting up the latches
	for(i=0; i<64; i+=2) 
	{
		NVMDATL = buffer.data[i+1]; // set up the word to load into the next word latch
		NVMDATH = buffer.data[i];
		NVMCON2 = 0x55;             // special unlock sequence
		NVMCON2 = 0xAA;		
		NVMCON1bits.WR = 1;         // perform the write to the latches. execution halts till complete
		++NVMADRL;                  // and on to the next address...
	}

	// PERFORM THE WRITE TO FLASH
	NVMCON1bits.LWLO = 0;           // ready to write from latches to FLASH
    NVMADRL = (byte)(addr & 0xE0);        // set the row address
    NVMADRH = (byte)(addr >> 8);
	NVMDATL = buffer.data[1];
	NVMDATH = buffer.data[0];
	NVMCON2 = 0x55;                 // special unlock sequence
	NVMCON2 = 0xAA;		
	NVMCON1bits.WR = 1;             // start write
	
	NVMCON1bits.WREN = 0;	// disable writes
}

////////////////////////////////////////////////////////
// ERROR INDICATION
void error(byte b) 
{
	P_LED1 = 0;
	for(;;) {
		for(i=0; i<b; ++i) {
			P_LED2 = 1;
            __delay_ms(200);        
 			P_LED2 = 0;
            __delay_ms(200);        
		}		
        __delay_ms(500);        
	}
}

////////////////////////////////////////////////////////
// SUCCESS INDICATION
void success() {
	for(;;) {
		P_LED1 = 1;
		P_LED2 = 0;
       __delay_ms(100);        
 		P_LED1 = 0;
		P_LED2 = 1;
       __delay_ms(100);        
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
		for(i=0; i<sizeof(buffer);) {	
            while(!PIR3bits.RC1IF);
			byte ch = RC1REG;
			((byte*)&buffer)[i] = ch;
			if(ch != MIDI_ACTIVE_SENSE) {
				++i;
			}
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
			memcpy(buffer.data, bootloader_reset, 8);
			write_flash(); 								
			
			// now we save the firmware reset vector
			addr = SAVED_RESET_VECTOR;
			read_flash(); 									
			memcpy(buffer.data, firmware_reset, 8);	
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
// MAIN ENTRY POINT
void main() {
    
    // configure GPIO
    TRISA=TRISA_BITS;
    TRISB=TRISB_BITS;
    TRISC=TRISC_BITS;
    LATA=0;
    LATC=0;
    
    // disable analog inputs
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    
    // enable weak pullups
    WPUA=WPUA_BITS;
    WPUB=WPUB_BITS;
    WPUC=WPUC_BITS;
    
    // setup alternative pin functions
    RC4PPS = 0x0F;      // Pin RC4 PPS register set to point to UART TX
    RX1DTPPS = 0x15;    // UART RX PPS register set to point to RC5

    // configure baud rate (31250)
    BAUD1CON = 0b00001000; 
    SP1BRGH = 0;            
    SP1BRGL = 63;           

    // configure serial port for receive
    RC1STA= 0b10110000;
    
    __delay_ms(100);        // allow inputs to settle
    if(!(P_KEY1) && !(P_KEY2)) {
        // key combination pressed to start bootloader
        read_sysex();
    }
    
    // otherwise jump to the application reset vector, running the application
	asm("GOTO 0x1D40");    // address must match SAVED_RESET_VECTOR
}

//
// EOF
//