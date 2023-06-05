/*
 * File:   main.c
 * Author: jason
 *
 * Created on 26 January 2023, 11:02
 */

#define SAVED_RESET_VECTOR 	0x1D40 // must be on 32-byte boundary
// program memory map
//
// 0000 bootloader reset vector
// 0004 application interrupt vector
// 0005 application code
// :
// 1D40 saved application reset vector (must be on a 32-byte FLASH page boundary)
// 1D44 bootloader code
// :
// 1F7F storage area flash (SAF) used for application settings
// :
// 1FFF end of flash
//
// 
// build bootloader with options
//
// Project options|XC8 Linker
// Memory Model
//      - ROM Ranges: default,-1F7F-1FFF
//
// Additional options:  
//      Code offset = 1d44
//      -Wl,-pmyReset=0,-preset_vec=1d40h,-ptext=1d44h

// Bootloader reset vector
// MOVLP 0x1D        001 1100       7 bits->PCLATH
// GOTO 0x553   101 0101 0111      11 bits->PC(10:0) PCLATH<6:3> -> PC<14:11>
// 

//      654 3210       
//      --- -
//      001 1100
//
//       --- -
//      0000 0101 0101 0111   553


//      0000 0101 0101 0111   553
//      0001 1101 0101 0111   1D53
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
// INCLUDE FILES
//
#define _XTAL_FREQ 32000000
#include <xc.h>
#include <string.h>


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


typedef uint8_t byte;	

// Replacement reset vector to take us into the bootloader at address 0x1E30. 
//	0x0000	0x319E  MOVLP 0x1E  
//	0x0001	0x2E25  GOTO  0x625 <-- lower 11 bits of address
//	0x0002	0x0000  NOP
//	0x0003	0x0000  NOP
//#pragma DATA 0x0000, 0x319E, 0x2E25, 0x0000, 0x0000

// Initial data for the location that will store the application reset vector
//  0x1E20 	018A 	CLRF PCLATH	(ensure relative jump is within page 0)
//	0x1E21 	0000 	NOP			(replaced by original reset vector byte 0)
//  0x1E22 	0000 	NOP			(replaced by original reset vector byte 1)
//	0x1E23 	CLRF 	PCLATH		(replaced by original reset vector byte 2)
//	0x1E24 	CLRF 	PCL			(replaced by original reset vector byte 3)
// 	0x1E25 	..bootloader..
//#pragma DATA 0x1E20, 0x018A, 0x0000, 0x0000, 0x018A, 0x0082

asm("PSECT myReset,class=CODE,reloc=2,delta=2");
asm("CustomReset:");
asm("ljmp start");
//asm("GLOBAL _savedVectorReset");

//void __at(0x0000) vectorReset(void)  {
//    asm("GOTO 0x1e30");
//}

 //void __at(0x1E20) savedVectorReset(void)  {
 //   asm("CLRF PCLATH");
 //   asm("NOP");
 //   asm("NOP");
 //   asm("CLRF PCLATH");
 //   asm("CLRF PCL");
//}

//
// CONSTANTS
//

#define MIDI_SYSEX_BEGIN    0xf0
#define MIDI_SYSEX_END     	0xf7
#define MIDI_ACTIVE_SENSE   0xfe
#define MY_SYSEX_ID0		0x00
#define MY_SYSEX_ID1		0x7f



#define MY_SYSEX_ID2		0x25


//
// TYPE DEFS
//


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
#if 0
////////////////////////////////////////////////////////
// SHORT DELAY
// NB: Saves a lot of memory over standard delay_ms()
// library function. Even with multiple calls!
void delay() {
	volatile byte i=0;
	while(--i) {
		volatile byte j=0;
		while(--j) nop();
	}
}
#endif

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
			//while(!pir1.5); 
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


void main() {
    TRISA=TRISA_BITS;
    TRISB=TRISB_BITS;
    TRISC=TRISC_BITS;
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    WPUA=WPUA_BITS;
    WPUB=WPUB_BITS;
    WPUC=WPUC_BITS;
    LATA=0;
    LATC=0;
    // Pin RC4 PPS register set to point to UART TX
    RC4PPS = 0x0F;
    
    // UART RX PPS register set to point to RC5
    RX1DTPPS = 0x15;
    
    //PIE3 = 
    //PIR3bits.RC1IF = 0;     
    //PIE3bits.RC1IE = 1;     
    //PIR3bits.TX1IF = 0;     
    //PIE3bits.TX1IE = 0;     
    
    BAUD1CON = 0b00001000; // synchronous bit polarity 
    //BAUD1CONbits.SCKP = 0;  // synchronous bit polarity 
    //BAUD1CONbits.BRG16 = 1; // enable 16 bit brg
    //BAUD1CONbits.WUE = 0;   // wake up enable off
    //BAUD1CONbits.ABDEN = 0; // auto baud detect
    
    //TX1STA = 0b00000000;
            
    //TX1STAbits.TX9 = 0;     // 8 bit transmission
    //TX1STAbits.SYNC = 0;    // async mode
    //TX1STAbits.SENDB = 0;   // break character
    //TX1STAbits.BRGH = 0;    // high baudrate 
    //TX1STAbits.TX9D = 0;    // bit 9

    RC1STA= 0b10110000;
    
    //TX1STAbits.TXEN = 1;    // transmit enable
    //RC1STAbits.SPEN = 1;    // serial port enable    
    //RC1STAbits.RX9 = 0;     // 8 bit operation
    //RC1STAbits.SREN = 1;    // enable receiver
    //RC1STAbits.CREN = 1;    // continuous receive enable

    SP1BRGH = 0;            // brg high byte
    SP1BRGL = 63;            // brg low byte (31250)	 
    
    //__delay_ms(100);        
    if(!(P_KEY1) && !(P_KEY2)) {
        read_sysex();
    }
	asm("GOTO 0x1D40");    
}