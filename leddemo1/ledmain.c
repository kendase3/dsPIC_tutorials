
#include <p33Fxxxx.h>
#include <pps.h>
#include <libpic30.h>
#include <uart.h>
#include <stdlib.h>
#include <stdio.h>
#include <timer.h>
#include <string.h>

// local includes
#include "../lang.h"

//constants
#define FCY 4E7

//#define startButton PORTAbits.RA4 // also stop button 
//#define tagButton PORTAbits.RA2 // also upload 

//Configuration Bits Setup
_FBS(RBS_NO_RAM & BSS_NO_BOOT_CODE & BSS_NO_FLASH & BWRP_WRPROTECT_OFF);
_FSS(RSS_NO_RAM & SSS_NO_FLASH & SWRP_WRPROTECT_OFF);
_FGS( GSS_OFF & GCP_OFF & GWRP_OFF );
_FOSCSEL(FNOSC_FRCPLL); 				
// we allow multiple changes to PPS ( IOL1WAY_OFF )
_FOSC(FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_OFF  & POSCMD_NONE);	
// Clock Switching Enabled and Fail Safe Clock Monitor is disabled				// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystal
_FWDT(FWDTEN_OFF);	// Watchdog Timer Enabled/disabled by user software
// ^^ as i recall, one needs to additionally tell the software to turn off WDT
// in init below

_FPOR( PWMPIN_ON & ALTI2C_OFF & FPWRT_PWR128 );
_FICD( JTAGEN_OFF & ICS_PGD1 );

// function pre-declarations
void init();
void disableUart();
void initTimer();

void init() {
	//disable watch dog (required in addition to conf bits above) 
	RCONbits.SWDTEN = 0;
	
	//configure clock
	PLLFBD = 150; // m = 217
	CLKDIVbits.PLLPRE = 5; // n1 = 10
	CLKDIVbits.PLLPOST = 0; //n2 = 2
	OSCTUN = 0; // tune FRC

	__builtin_write_OSCCONH(1); //init clock switch to internal
	__builtin_write_OSCCONL(1); //start clock switch process
	while (OSCCONbits.LOCK != 1) {} //wait for clock switching
	TRISA = 0b0000000000011100; // 3 and 4 for input buttons 
	//init ports
	TRISB = 0b0001010000010000;
	disableUart();
	//then handle UART config
	initTimer();
}

void disableUart() {
	U1MODEbits.UARTEN = 0;
	IEC4bits.U1EIE = 0; // disable uart1
	U2MODEbits.UARTEN = 0;
	IEC0bits.U1TXIE = 0; //disable transmit interrupts
	IEC0bits.U1RXIE = 0; //disable receive interrupts
	IEC4bits.U2EIE = 0; //diable uart2 
	IEC1bits.U2TXIE = 0; //disablee transmit interrupts
	IEC1bits.U2RXIE = 0; //disablee receive interrupts
}

void initTimer()
{
   /*Initialize Timer 1 for 1 decisecond interrupt i.e. 10 Hz*/

	T1CONbits.TON = 0; // Disable Timer
	T1CONbits.TCS = 0; // Select internal instruction cycle clock
	T1CONbits.TGATE = 0; // Disable Gated Timer mode
	T1CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
	TMR1 = 0x00; // Clear timer register
	//PR1 = 156; // Load the period value 1000Hz *(40MHz/256)/1000Hz = 156.25*
	//PR1 = 1563; // now 100 Hz 
	PR1 = 15625; // now 10 Hz 
	IPC0bits.T1IP = 5; // Set Timer1 Interrupt Priority Level
	IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
	IEC0bits.T1IE = 1; // Enable Timer1 interrupt
	T1CONbits.TON = 1; // Start Timer
}

int main() {

	init();
	while (1) {
		// do nothing
	} 
}


void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
	
	static unsigned int dsecs = 0;
	TMR1 = 0;
	dsecs++;
	if (dsecs >= 10) {
		// second interrupt code
		dsecs = 0;
		// we alternate on and off every second
		LATAbits.LATA0 = ~LATAbits.LATA0;
	} 
	IFS0bits.T1IF = 0; //clear interrupt flag
}

