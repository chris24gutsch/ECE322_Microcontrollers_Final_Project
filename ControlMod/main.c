//************************************************************************
// <Final Project Control Module>
// C code to acomplish the following:
//	The main file for the Control Module, handling glucose levels and interacting with Pump module
//
// <Allen Poe & Chris Gutschlag>
// <12/4/19>   <VERSION A>
//
//************************************************************************
// Includes & defines
//************************************************************************
#include <msp430fr6989.h>							// Register Definitions
#include <ControlModule.h>							// Custom header file containing some necessary values
#include <string.h> 								// Standard String Function Library
#include <driverlib.h> 								// Required for the LCD
#include "myGpio.h" 								// Required for the LCD
#include "myClocks.h" 								// Required for the LCD
#include "myLcd.h" 									// Required for the LCD
#define ENABLE_PINS 0xFFFE 							// Required to use inputs and outputs
#define UART_CLK_SEL 0x0080 						// Specifies accurate clock for UART peripheral
#define BR0_FOR_9600 0x34 							// Value required to use 9600 baud
#define BR1_FOR_9600 0x00 							// Value required to use 9600 baud
#define CLK_MOD 0x4911 								// Microcontroller will "clean-up" clock signal

//************************************************************************
// Function Prototypes
//************************************************************************
void ADC_SETUP(void); 								// Used to setup ADC12 peripheral
void select_clock_signals(void); 					// Assigns microcontroller clock signals
void assign_pins_to_uart(void); 					// P4.2 is for TXD, P4.3 is for RXD
void use_9600_baud(void);	 						// UART operates at 9600 bits/second

//************************************************************************
// Main Function
//************************************************************************
static int ADCValue;								// Value read by potentiometer
static int com = 1;									// Bool used to check disconnection
void main(void) {
	WDTCTL = 0x5A80; 								// Stop WDT
	ADC_SETUP();									// Set up the ADC
	ADC12IER0 = ADC12IE0;							// Enable ADC interrupt


	select_clock_signals(); 						// Assigns microcontroller clock signals
	assign_pins_to_uart(); 							// P4.2 is for TXD, P4.3 is for RXD
	use_9600_baud(); 								// UART operates at 9600 bits/second
// Timer 2 ---- Check pot
	TA2CCR0 = 8778; 								// Sets value of Timer_2
	TA2CTL = 0x0114; 								// Setup TA2 to count from zero with ACLK
	TA2CCTL0 = CCIE; 								// Enable interrupt for Timer_2
// Timer B0 ---- 15 seconds
	TB0CTL = 0x01D4; 								// Setup TB0 to count from zero with ACLK
	TB0EX0 = 0x0007;
	TB0CCR0 = 7670;
	TB0CCTL0 = CCIE; 								// Enable interrupt for Timer_3
// Timer 3 ---- Disconnection
	TA3CTL = 0x01D4; 								// Setup TA3 to count from zero with ACLK
	TA3EX0 = 0x0007;
	TA3CCR0 = 11300;
	TA3CCTL0 = CCIE; 								// Enable interrupt for Timer_3

	UCA0IE |= 0x0001; 								// Interrupt when TX stop bit complete

	initGPIO(); 									// Initialize General Purpose Inputs and Outputs for the LCD
	initClocks(); 									// Initialize clocks for the LCD
	myLCD_init(); 									// Initialize Liquid Crystal Display

	P1IFG = 0x00; 									// Ensure no interrupts are pending

	_BIS_SR(GIE);

	ADC12CTL0 = ADC12CTL0 | ADC12ENC; 				// Enable conversion
	ADC12CTL0 = ADC12CTL0 | ADC12SC; 				// Start conversion

	myLCD_showSymbol(LCD_UPDATE, LCD_BATT, 0);
	myLCD_showSymbol(LCD_UPDATE, LCD_BRACKETS, 0);
	UCA0TXBUF = 0x98;								//send start message to pump after emergency stop
	while(1);
}

//************************************************************************
// Function Defintions
//************************************************************************

void ADC_SETUP(void)
{
   #define ADC12_SHT_16 0x0200 						// 16 clock cycles for sample and hold
   #define ADC12_ON 0x0010		 					// Used to turn ADC12 peripheral on
   #define ADC12_SHT_SRC_SEL 0x0200 				// Selects source for sample & hold
   #define ADC12_12BIT 0x0020 						// Selects 12-bits of resolution
   #define ADC12_P92 0x000A 						// Use input P9.2 for analog input
   ADC12CTL0 = ADC12_SHT_16 | ADC12_ON ; 			// Turn on, set sample & hold time
   ADC12CTL1 = ADC12_SHT_SRC_SEL; 					// Specify sample & hold clock source
   ADC12CTL2 = ADC12_12BIT; 						// 12-bit conversion results
   ADC12MCTL0 = ADC12_P92;
}

void select_clock_signals(void)
{
	CSCTL0 = 0xA500; 								// "Password" to access clock calibration registers
	CSCTL1 = 0x0046; 								// Specifies frequency of main clock
	CSCTL2 = 0x0133; 								// Assigns additional clock signals
	CSCTL3 = 0x0000; 								// Use clocks at intended frequency,
													//do not slow them down
}

void assign_pins_to_uart(void)
{
	P4SEL1 = 0x00; 									// 0000 0000
	P4SEL0 = BIT3 | BIT2; 							// 0000 1100
													// ^^
													// ||
													// |+---- 01 assigns P4.2 to UART Transmit (TXD)
													// |
													// +----- 01 assigns P4.3 to UART Receive (RXD)
}

void use_9600_baud(void)
{
	UCA0CTLW0 = UCSWRST; 							// Put UART into SoftWare ReSeT
	UCA0CTLW0 = UCA0CTLW0 | UART_CLK_SEL; 			// Specifies clock source for UART
	UCA0BR0 = BR0_FOR_9600; 						// Specifies bit rate (baud) of 9600
	UCA0BR1 = BR1_FOR_9600; 						// Specifies bit rate (baud) of 9600
	UCA0MCTLW = CLK_MOD;							// "Cleans" clock signal
	UCA0CTLW0 = UCA0CTLW0 & (~UCSWRST); 			// Takes UART out of SoftWare ReSeT
}


//************************************************************************
// Interrupt Service Routines
//************************************************************************
#pragma vector=USCI_A0_VECTOR
__interrupt void UART_ISR(void)
{

	myLCD_showSymbol(LCD_CLEAR, LCD_TMR, 0);			//reset timer symbol
	myLCD_showSymbol(LCD_UPDATE, LCD_ANT, 0);
	if(UCA0RXBUF == 0x80){ 							// Pump Resevoir Low, alert with R on LCD
		myLCD_showSymbol(LCD_UPDATE, LCD_REC, 0);
		myLCD_showChar('0', 1);
	}
	else if ((UCA0RXBUF == 0x81)) {					// Reservoir level 1
		myLCD_showSymbol(LCD_CLEAR, LCD_REC, 0);
		myLCD_showChar('1', 1);
	}
	else if (UCA0RXBUF == 0x82) {					// Reservoir level 2
		myLCD_showSymbol(LCD_CLEAR, LCD_REC, 0);
		myLCD_showChar('2', 1);
	}
	else if (UCA0RXBUF == 0x83) {					// Reservoir level 3
			myLCD_showSymbol(LCD_CLEAR, LCD_REC, 0);
			myLCD_showChar('3', 1);
	}
	else if (UCA0RXBUF == 0x84) {					// Reservoir level 4
			myLCD_showSymbol(LCD_CLEAR, LCD_REC, 0);
			myLCD_showChar('4', 1);
	}
	else if (UCA0RXBUF == 0x85) {					// Reservoir level 5
			myLCD_showSymbol(LCD_CLEAR, LCD_REC, 0);
			myLCD_showChar('5', 1);
	}
	else if (UCA0RXBUF == 0x86) {					// Reservoir level 6
			myLCD_showSymbol(LCD_CLEAR, LCD_REC, 0);
			myLCD_showChar('6', 1);
	}
	else if (UCA0RXBUF == 0x87) {					// Reservoir level 7
			myLCD_showSymbol(LCD_CLEAR, LCD_REC, 0);
			myLCD_showChar('7', 1);
	}
	else if (UCA0RXBUF == 0x88) {					// Reservoir level 8
			myLCD_showSymbol(LCD_CLEAR, LCD_REC, 0);
			myLCD_showChar('8', 1);
	}
	else if (UCA0RXBUF == 0x90)	{					//Pump battery empty, alert on LCD
		P9OUT = 0x80 ^ P9OUT;						// Turn on GREEN LED
		myLCD_showSymbol(LCD_CLEAR, LCD_B1, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B2, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B3, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B4, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B5, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B6, 0);
	}
	else if (UCA0RXBUF == 0x91) {					//Pump battery level 1
		myLCD_showSymbol(LCD_UPDATE, LCD_B1, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B2, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B3, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B4, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B5, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B6, 0);
		P9OUT = P9OUT & ~BIT7; 						// Turn OFF P9.7 (Green LED)
	}
	else if (UCA0RXBUF == 0x92) {					//Pump battery level 2
		myLCD_showSymbol(LCD_UPDATE, LCD_B1, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B2, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B3, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B4, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B5, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B6, 0);
		P9OUT = P9OUT & ~BIT7; 						// Turn OFF P9.7 (Green LED)
	}
	else if (UCA0RXBUF == 0x93) {					//Pump battery level 3
		myLCD_showSymbol(LCD_UPDATE, LCD_B1, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B2, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B3, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B4, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B5, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B6, 0);
		P9OUT = P9OUT & ~BIT7; 						// Turn OFF P9.7 (Green LED)
	}
	else if (UCA0RXBUF == 0x94) {					//Pump battery level 4
		myLCD_showSymbol(LCD_UPDATE, LCD_B1, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B2, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B3, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B4, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B5, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B6, 0);
		P9OUT = P9OUT & ~BIT7; 						// Turn OFF P9.7 (Green LED)
	}
	else if (UCA0RXBUF == 0x95) {					//Pump battery level 5
		myLCD_showSymbol(LCD_UPDATE, LCD_B1, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B2, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B3, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B4, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B5, 0);
		myLCD_showSymbol(LCD_CLEAR, LCD_B6, 0);
		P9OUT = P9OUT & ~BIT7; 						// Turn OFF P9.7 (Green LED)

	}
	else if (UCA0RXBUF == 0x96) {					//Pump battery level 6
		myLCD_showSymbol(LCD_UPDATE, LCD_B1, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B2, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B3, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B4, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B5, 0);
		myLCD_showSymbol(LCD_UPDATE, LCD_B6, 0);
		P9OUT = P9OUT & ~BIT7; 						// Turn OFF P9.7 (Green LED)
	}
	else if (UCA0RXBUF == 0x99) {					//Error transmitting glucose level, show TX
		com = 1;									// set com to 1
	}
	//Using pump's buttons for these, my controller's buttons are broken
	else if (UCA0RXBUF == 0x97) {					//Emergency stop
		UCA0TXBUF = 0x00;							//send stop message to pump
		int i;
		for (i=0;i<2000;i++);
		myLCD_showChar('S', 1);
		myLCD_showChar('T', 2);
		myLCD_showChar('O', 3);
		myLCD_showChar('P', 4);
		myLCD_showChar(' ', 5);
		myLCD_showChar(' ', 6);

		while(1);									//stops all operations
	}
	UCA0IFG = UCA0IFG & (~UCTXCPTIFG); 				// Clear TX ComPleTe Interrupt FlaG
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
	#define ADC12_P84 0x0007 						// Use input P8.4 for analog input
 	#define ADC12_P92 0x000A 						// Use input P9.2 for analog input
	static unsigned char input = 92; 				// input = 84 if P8.4 sampled
	//--------------------BATTERY LEVELS------------------------------------
	if (input == 92) {
		if (ADC12MEM0 < ADCBATLEVEL1) { 											// If input < 2.2V
			myLCD_showSymbol(LCD_CLEAR, LCD_A1DP, 0);
			myLCD_showSymbol(LCD_CLEAR, LCD_A2DP, 0);
			myLCD_showSymbol(LCD_CLEAR, LCD_A3DP, 0);
			myLCD_showSymbol(LCD_CLEAR, LCD_A4DP, 0);
			myLCD_showSymbol(LCD_CLEAR, LCD_A5DP, 0);
			P1OUT = BIT0; 															// Set P1.0 HI (Turn Red LED ON)
		} else if (ADC12MEM0 >= ADCBATLEVEL1 && ADC12MEM0 < ADCBATLEVEL2) {			// Control Mod battery level 1
			myLCD_showSymbol(LCD_UPDATE, LCD_A1DP, 0);
			P1OUT = 0x00; 															// Turn OFF P1.0 (Red LED)
		} else if (ADC12MEM0 >= ADCBATLEVEL2 && ADC12MEM0 < ADCBATLEVEL3) {			// Control Mod battery level 2
			myLCD_showSymbol(LCD_UPDATE, LCD_A1DP, 0);
			myLCD_showSymbol(LCD_UPDATE, LCD_A2DP, 0);
			P1OUT = 0x00; 															// Turn OFF P1.0 (Red LED)
		} else if (ADC12MEM0 >= ADCBATLEVEL3 && ADC12MEM0 < ADCBATLEVEL4) {			// Control Mod battery level 3
			myLCD_showSymbol(LCD_UPDATE, LCD_A1DP, 0);
			myLCD_showSymbol(LCD_UPDATE, LCD_A2DP, 0);
			myLCD_showSymbol(LCD_UPDATE, LCD_A3DP, 0);
			P1OUT = 0x00; 															// Turn OFF P1.0 (Red LED)
		} else if (ADC12MEM0 >= ADCBATLEVEL4 && ADC12MEM0 < ADCBATLEVEL5) {			// Control Mod battery level 4
			myLCD_showSymbol(LCD_UPDATE, LCD_A1DP, 0);
			myLCD_showSymbol(LCD_UPDATE, LCD_A2DP, 0);
			myLCD_showSymbol(LCD_UPDATE, LCD_A3DP, 0);
			myLCD_showSymbol(LCD_UPDATE, LCD_A4DP, 0);
			P1OUT = 0x00;															// Turn off Red LED
		} else {																	// Max control mod battery level
			myLCD_showSymbol(LCD_UPDATE, LCD_A1DP, 0);
			myLCD_showSymbol(LCD_UPDATE, LCD_A2DP, 0);
			myLCD_showSymbol(LCD_UPDATE, LCD_A3DP, 0);
			myLCD_showSymbol(LCD_UPDATE, LCD_A4DP, 0);
			myLCD_showSymbol(LCD_UPDATE, LCD_A5DP, 0);
			P1OUT = 0x00;															// Turn off Red LED
		}
		input = 84;
	    ADC12CTL0 = ADC12CTL0 & (~ADC12ENC); 										// Need to disable peripheral
	    ADC12MCTL0 = ADC12_P84; 													// to change to input P8.4
    //--------------------DOSAGE LEVELS-------------------------------------
	} else {
		ADCValue = ADC12MEM0;									//save ADC value
		input = 92; 											// Next sample from P8.4
		ADC12CTL0 = ADC12CTL0 & (~ADC12ENC); 					// Need to disable peripheral
		ADC12MCTL0 = ADC12_P92;								 	// to change to input P9.2
	}
	ADC12CTL0 = ADC12CTL0 | ADC12ENC;							// Re-enable conversion
	ADC12CTL0 = ADC12CTL0 | ADC12SC; 							// Start next conversion
}


#pragma vector=TIMER2_A0_VECTOR
//************************************************************************
__interrupt void Timer2_ISR (void)
//************************************************************************
{
	//If dosage level is too low, show alert heart
	if (ADCValue <= 0x2A5) {									//If glucose levels are below 75mL, show heart
		myLCD_showSymbol(LCD_UPDATE, LCD_HRT, 0);
	} else {
		myLCD_showSymbol(LCD_CLEAR, LCD_HRT, 0);				//glucose levels are fine, clear heart
	}

	float ADCmessage = ADCValue / 4085.0 * 150.0 +50.0;			//convert ADC to be used for message
	myLCD_displayNumber(ADCmessage);							//show the number
}

#pragma vector=TIMER3_A0_VECTOR
//************************************************************************
__interrupt void Timer3_ISR (void)
//************************************************************************
{
	if (com == 1) {											//check if communication bool has been set to true
		com = 0;											//reset flag for communication
		myLCD_showSymbol(LCD_CLEAR, LCD_TMR, 0);			//reset timer symbol
	}
	else {
		myLCD_showSymbol(LCD_UPDATE, LCD_TMR, 0);			//if communication has been disconnected for 30s,
															//show timer symbol
	}
}

#pragma vector=TIMER0_B0_VECTOR
//************************************************************************
__interrupt void Timer0_ISR (void)
//************************************************************************
{
	UCA0TXBUF = ADCValue / 4085.0 * 150.0 +50.0;			//Send ADC value every 15 seconds
}
