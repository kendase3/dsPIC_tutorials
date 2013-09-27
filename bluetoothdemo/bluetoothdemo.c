/*
	the main function and implementation-specific file for MGWD on dsPIC33
*/

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
#define BAUDRATE_PC 9600 
#define BRGVAL_PC ((FCY/BAUDRATE_PC)/16)-1
#define BAUDRATE_BLUETOOTH 9600 
#define BRGVAL_BLUETOOTH ((FCY/BAUDRATE_BLUETOOTH)/16)-1

//custom PPS input/output binders for more meaningful names than pin numbers
#define iPPSOutput2(pin, fn) pin=fn;
//#define PPSOutput2(fn, pin) iPPSOutput(pin, OUT_FN_##fn)
#define PPSOutput2(fn, pin) iPPSOutput2(pin, OUT_FN_##fn)
#define PPSInput2(fn, pin) iPPSInput(IN_FN_##fn, pin)

#define bluetoothInputPin 8 //i.e. RP8 
#define bluetoothOutputPin RPOR3bits.RP7R //i.e. RP7 
#define pcOutputPin RPOR3bits.RP6R //i.e. RP6 //output TO the PC 
#define pcInputPin 10 //i.e. RP10 

#define BUFFER_SIZE 500
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

// globals
volatile char buffer[BUFFER_SIZE];
volatile bool hasStarted = false;
volatile bool isDone = false;
volatile bool bluetoothReady = false;
volatile int bluetoothTimeout = 0;

// function pre-declarations
void init();
void initTimer();
void initUart1();
//void initUart2();
void uart1ToPc();
void uart1ToUpload();

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
	TRISB = 0b0001010100010000;
	//then handle UART config
	initTimer();
	initUart1();
	//initUart2();
}

void initUart1() {
	//PPS stuff: default to pc 
	uart1ToPc();

	U1MODEbits.UARTEN = 0; //disable UART during config, enable after
	U1MODEbits.ABAUD = 0; //disable autobaud, would require sending '55'
	U1MODEbits.URXINV = 0; // IdleState = 1 //TODO: remove?
	U1MODEbits.BRGH = 0; // 16 clocks per bit period; low speed mode
	U1MODEbits.PDSEL = 0; // no parity
	U1MODEbits.STSEL = 0; // one stop bit

	//set baud rate to 9600
	U1BRG = BRGVAL_PC; //40 mhz clock, 4800 baud
	
	U1STAbits.UTXISEL1 = 0;
	U1STAbits.UTXISEL0 = 0; //together determines when to interrupt
							// in this case, when a char is transferred
	U1STAbits.URXISEL = 0; //interrupt when char received
	
	IPC2bits.U1RXIP = 4; //set priority level to mid-range
	IPC3bits.U1TXIP = 4; //same for transmission

	//flag handling
	IFS0bits.U1TXIF = 0; //clear transmit interrupt flag
	IEC0bits.U1TXIE = 1; //enable transmit interrupts
	IFS0bits.U1RXIF = 0; //clear receive interrupt flag
	IEC0bits.U1RXIE = 1; //enable receive interrupts
	IFS4bits.U1EIF = 0; //clear UART1 error flag
	IEC4bits.U1EIE = 1; //season to taste (enable interrupt)

	U1MODEbits.UARTEN = 1; //enable uart
	U1STAbits.UTXEN = 1; //enable tx
}

void uart1ToPc() {
	//PPS stuff
	PPSUnLock;
	PPSOutput2(PPS_U1TX, pcOutputPin);
	PPSOutput2(PPS_SDO1, bluetoothOutputPin);
	// see above 
	PPSInput2(PPS_U1RX, pcInputPin);
	PPSInput2(PPS_SDI1, bluetoothInputPin);
	PPSLock;
}

void uart1ToUpload() {
	PPSUnLock;
	PPSOutput2(PPS_U1TX, bluetoothOutputPin);
	// to stop data from going to other pin, need to reassign it
	// to a dummy function we are not using
	PPSOutput2(PPS_SDO1, pcOutputPin);
	PPSInput2(PPS_U1RX, bluetoothInputPin);
	PPSInput2(PPS_SDI1, pcInputPin);
	PPSLock;
} 
/*
void initUart2() {
	// we receive only from the GPS
	PPSUnLock;
	PPSOutput2(PPS_U2TX, gpsOutputPin);
	PPSInput2(PPS_U2RX, gpsInputPin); //the important one 
	PPSLock;
	System_getSystemSingleton()->uart2Mode = UART_2_GPS; //FIXME: inconsistent as to when this happens 

	U2MODEbits.UARTEN = 0; //disable UART during config, enable after
	U2MODEbits.ABAUD = 0; //disable autobaud, would require sending '55'
	U2MODEbits.URXINV = 0; // IdleState = 1 //TODO: remove?
	U2MODEbits.BRGH = 0; // 16 clocks per bit period; low speed mode
	U2MODEbits.PDSEL = 0; // no parity
	U2MODEbits.STSEL = 0; // one stop bit

	U2BRG = BRGVAL_GPS; 
	
	U2STAbits.UTXISEL1 = 0;
	U2STAbits.UTXISEL0 = 0; //together determines when to interrupt
							// in this case, when a char is transferred
	U2STAbits.URXISEL = 0; //interrupt when char received
	
	IPC7bits.U2RXIP = 4; //set priority level to slightly lower than debug 
	IPC7bits.U2TXIP = 4; //same for transmission

	//flag handling
	IFS1bits.U2TXIF = 0; //clear transmit interrupt flag
	IEC1bits.U2TXIE = 1; //enable transmit interrupts
	IFS1bits.U2RXIF = 0; //clear receive interrupt flag
	IEC1bits.U2RXIE = 1; //enable receive interrupts
	IFS4bits.U2EIF = 0; //clear UART1 error flag
	IEC4bits.U2EIE = 1; //season to taste (enable interrupt)

	U2MODEbits.UARTEN = 1; //enable uart
	U2STAbits.UTXEN = 1; //enable tx
}
*/
/*
void Hardware_uart2ToUpload() {
	PPSUnLock;
	PPSOutput2(PPS_U2TX, bluetoothOutputPin);
	PPSOutput2(PPS_SDO1, gpsOutputPin);
	PPSInput2(PPS_U2RX, bluetoothInputPin); 
	PPSInput2(PPS_SDI1, gpsInputPin); 
	PPSLock;
	U2MODEbits.UARTEN = 0; //disable UART during config, enable after
	// update the baud rate
	U2BRG = BRGVAL_BLUETOOTH;

	U2MODEbits.UARTEN = 1; //disable UART during config, enable after
	U2STAbits.UTXEN = 1; //enable tx
	System_getSystemSingleton()->uart2Mode = UART_2_UPLOAD;  
} 
*/
void initTimer()
{
   /*Initialize Timer 1 for one millisecond Period Interrupts*/
   /* jk now it's...decaseconds? 10 hz */

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

//NOTE: of questionable accuracy
void waitMs(int numMs) {
	unsigned int i, j;
	for (i = 0; i < numMs; i++) {
		// 40 MHz processor, 40k instructions per millisecond
		for (j = 0; j < 40000 / 8; j++) {
		} 
	}
}

int main() {
	init();
	// uart starts on PC
	memset(& buffer, 0, BUFFER_SIZE);
	LATAbits.LATA0 = 0;
	printf("\nready to receive message of form '{message}'\n");
	while (!isDone) { } 
	// if we're done then the message is waiting in the buffer to be sent	
	uart1ToUpload();
	waitMs(100);
	//while (!bluetoothReady) { } 
	char * curChar;
	for (curChar = (char *)buffer; * curChar != '\0'; curChar++) {
		char tmp = * curChar;
		U1TXREG = tmp;				
		waitMs(10);
	}
	// turn on LED to indicate finished
	LATAbits.LATA0 = 1;
	uart1ToPc();
	waitMs(100);
	printf("\nFinished uploading!\n");
	while (1) { } // become unstuck in time
	return 0;
}

void handlePcInterrupt(char curChar) {
	static char * buffItr = NULL;
	if (buffItr == NULL) {
		buffItr = (char *) & buffer;
	}
	if (!hasStarted && curChar == '{') {
		hasStarted = true;
		//printf("received start character!\n");
	}	
	// not else so we include '{'
	if (hasStarted) {
		* buffItr = curChar;
		buffItr += 1;	
	} 
	// note else so we include '}'
	if (hasStarted && curChar == '}') {
		//printf("received end character!\n");
		isDone = true;
	} 
} 

void handleBluetoothInterrupt(char curChar) {
	if (curChar == '{') {
		bluetoothReady = true;
		bluetoothTimeout = 0;
		// debug
		LATAbits.LATA0 = ~LATAbits.LATA0;
	} 
} 

// interrupts
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
	//disallow self-interrupt
	IEC0bits.U1RXIE = 0;
	//U1TXREG = U1RXREG;
	// if we're not done, we are interacting with the PC 
	char curChar = U1RXREG; 
	if (!isDone) {
		//printf("passing '%c'to PC...\n", curChar);
		handlePcInterrupt(curChar);
	} 
	// otherwise, we are interacting with bluetooth 
	else {
		handleBluetoothInterrupt(curChar);
	}  
	//reset interrupt flag
	IFS0bits.U1RXIF = 0; 

	IEC0bits.U1RXIE = 1;
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
	IEC0bits.U1TXIE = 0; //disallow self-interrupt
	//LATBbits.LATB6 = ~LATBbits.LATB6;
	
	//reset flag
	IFS0bits.U1TXIF = 0;

	IEC0bits.U1TXIE = 1; //re-enable interrupt
}

void __attribute__((interrupt, no_auto_psv)) _U1ErrInterrupt(void) {
	//LATBbits.LATB7 = ~LATBbits.LATB7; //turn on 'error' LED
	//printf("UART1 Error!");
	U1STAbits.OERR = 0; //clear overflow error bit
	IFS4bits.U1EIF = 0; //clear error interrupt flag
}
/*
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void) {
	//disallow self-interrupt
	IEC1bits.U2RXIE = 0;
	//printf("quack!");
	//U1TXREG = U2RXREG;
	
	char curChar = U2RXREG;
	static System * sys;
	static bool hasSys = false;
	if (!hasSys) {
		sys = System_getSystemSingleton();	
		hasSys = true;
	}
	if (sys->uart2Mode == UART_2_GPS) {
		System_handleGpsInterrupt(sys, curChar);
	}
	else {
		System_handleStorageInterrupt(curChar);
	}

	//reset interrupt flag
	IFS1bits.U2RXIF = 0;
	IEC1bits.U2RXIE = 1;
}

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void) {

	//reset flag
	IFS1bits.U2TXIF = 0;
}

//error interrupts
void __attribute__((interrupt, no_auto_psv)) _U2ErrInterrupt(void) {
	U2STAbits.OERR = 0; //clear overflow error bit
	IFS4bits.U2EIF = 0; //clear error interrupt flag
}
*/
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
	
	//TEMP DEBUG INTERRUPT
	static unsigned int dsecs = 0;
	static unsigned int seconds = 0;
	TMR1 = 0;
	dsecs++;
	if (dsecs >= 10) {
		LATAbits.LATA0 = ~LATAbits.LATA0;
		seconds += 1;
		// second interrupt code
		if (bluetoothReady) {
			bluetoothTimeout += 1;
		}	
		// if we wander out of bluetooth range
		if (bluetoothTimeout > 5) {
			bluetoothReady = false;
		} 
		dsecs = 0;
	} 
	if (seconds >= 3) {
		seconds = 0;
		if (!isDone) { 
			printf("string: %s\n", (char *)& buffer);
		}
	} 
	IFS0bits.T1IF = 0; //clear interrupt flag
}

