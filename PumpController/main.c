//************************************************************************
// <Final Project>
//      Pump Controller
//
// <DESCRIPTION>
//      The main file for the pump controller.
// <NAME> Chris Gutschlag and Allen Poe
// <DATE> 12/4/2019
//************************************************************************

#include <msp430.h> 
#include <driverlib.h>
#include <stdbool.h>
#include "myClocks.h"
#include "myGpio.h"
#include "myLcd.h"
#include "PumpController.h"

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer

	ADC_SETUP(); // Sets up ADC peripheral

	ADC12IER0 = ADC12IE0; // Enable ADC interrupt

	/*-------UART Setup--------*/
	select_clock_signals(); // Assigns microcontroller clock signals
	assign_pins_to_uart(); // P4.2 is for TXD, P4.3 is for RXD
	use_9600_baud(); // UART operates at 9600 bits/second

	UCA0IE |= 0x0001; // Enable UART RXD interrupt
	/*-------------------------*/
	initGPIO();  // Initializes General Purpose
	             // Inputs and Outputs for LCD

	initClocks(); // Initialize clocks for LCD

	myLCD_init(); // Prepares LCD to receive commands

	// Enable button interrupts
    P1IE  = BIT1 | BIT2;
    P1IES = BIT1 | BIT2;
    P1IFG = 0x00;

	_BIS_SR(GIE); // Activate interrupts
	
	ADC12CTL0 |= ADC12ENC | ADC12SC; // Enable and start conversion

	while(1);
}

/*-------------Functions-------------*/
void runPump(unsigned char RunCycles) {
    _BIC_SR(GIE); // Disable interrupts

    //Convert 75 through 200 to 1 through 125
    RunCycles -= 75;

    //Hacky get around for if the initial value is 75
    if (RunCycles == 0) {RunCycles = 1;}

    // Make sure RunCycles isn't above 125
    if(RunCycles > MAXDOSECYCLES) return; // Above max cycles so do nothing

    RunCycles *= 2; // Multiply by 2 to get the desired pump run time

    // Set timer up to have a base 20ms period and 10% duty cycle
    TA0CCR0 = PERIOD20MS;
    TA0CCR1 = P20MS_DC10;
    TA0CTL  = 0x0204; // Setup TA0 to count from zero with SMCLK

    P1OUT |= 0x01; // Set P1.0 HI (RED LED)

    TA0CTL |= 0x0010; // Start TA0 in UP mode

    unsigned char i = 1; // Counting variable
    while(i <= RunCycles) { // Run the base timer for RunCycles number of cycles
        if(TA0CCTL1 & 0x0001) { // Counted up to TA0CCR1?
            TA0CCTL1 &= ~0x0001; // Reset CCTL1 flag
            P1OUT &= ~0x01;      // Turn off P1.0
        }
        if(TA0CCTL0 & 0x0001) { // Counted up to TA0CCR0?
            TA0CCTL0 &= ~0x0001; // Reset CCTL0 flag
            P1OUT |= 0x01;      // Turn off P1.0
            i++;
        }
    }

    P1OUT &= ~0x01;    // Turn off P1.0
    TA0CTL &= ~0x0010; // Turn off TA0

    _BIS_SR(GIE); // Re-enable interrupts
}

void changeADC12Pin(unsigned char Pin) {
    ADC12CTL0  &= (~ADC12ENC); // Need to disable peripheral
    ADC12MCTL0  = Pin;         // Change to Pin value
}

void ADC_SETUP(void) {
    ADC12CTL0  = ADC12_SHT_16 | ADC12_ON ; // Turn on, set sample & hold time
    ADC12CTL1  = ADC12_SHT_SRC_SEL;        // Specify sample & hold clock source
    ADC12CTL2  = ADC12_12BIT;              // 12-bit conversion results
    ADC12MCTL0 = ADC12_P84;                // Start with P8.4 as analog input
}

//*********************************************************************************
//* Select Clock Signals *
//*********************************************************************************
void select_clock_signals(void)
{
    CSCTL0 = 0xA500; // "Password" to access clock calibration registers
    CSCTL1 = 0x0046; // Specifies frequency of main clock
    CSCTL2 = 0x0133; // Assigns additional clock signals
    CSCTL3 = 0x0000; // Use clocks at intended frequency, do not slow them down
}

//*********************************************************************************
//* Used to Give UART Control of Appropriate Pins *
//*********************************************************************************
void assign_pins_to_uart(void)
{
    P4SEL1 = 0x00;          // 0000 0000
    P4SEL0 = BIT3 | BIT2;   // 0000 1100
                            //      ^^
                            //      |+---- 01 assigns P4.2 to UART Transmit (TXD)
                            //      |
                            //      +----- 01 assigns P4.3 to UART Receive (RXD)
}

//*********************************************************************************
//* Specify UART Baud Rate *
//*********************************************************************************
void use_9600_baud(void)
{
    UCA0CTLW0 = UCSWRST; // Put UART into SoftWare ReSeT
    UCA0CTLW0 = UCA0CTLW0 | UART_CLK_SEL; // Specifies clock source for UART
    UCA0BR0   = BR0_FOR_9600; // Specifies bit rate (baud) of 9600
    UCA0BR1   = BR1_FOR_9600; // Specifies bit rate (baud) of 9600
    UCA0MCTLW = CLK_MOD; // "Cleans" clock signal
    UCA0CTLW0 = UCA0CTLW0 & (~UCSWRST); // Takes UART out of SoftWare ReSeT
}

/*-----------------------------------*/

// ADC12 interrupt
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    static unsigned char input = 84; // input = 84 if P8.4 sampled
                                     //       = 92 if P9.2 sampled

    if(input == 84) { // If sample was from P8.4 (Insulin Tank Level)
        // Begin checking which level the tank is currently at

        if (ADC12MEM0 < ADCLEVEL1) {
            //LEVEL0
            UCA0TXBUF = LEVEL0; // Send the UART message LEVEL0
            myLCD_showChar('0', 1);
        } else if ((ADC12MEM0 >= ADCLEVEL1) && (ADC12MEM0 < ADCLEVEL2)) {
            //LEVEL1
            UCA0TXBUF = LEVEL1; // Send the UART message LEVEL1
            myLCD_showChar('1', 1);
        } else if ((ADC12MEM0 >= ADCLEVEL2) && (ADC12MEM0 < ADCLEVEL3)) {
            //LEVEL2
            UCA0TXBUF = LEVEL2; // Send the UART message LEVEL2
            myLCD_showChar('2', 1);
        } else if ((ADC12MEM0 >= ADCLEVEL3) && (ADC12MEM0 < ADCLEVEL4)) {
            //LEVEL3
            UCA0TXBUF = LEVEL3; // Send the UART message LEVEL3
            myLCD_showChar('3', 1);
        } else if ((ADC12MEM0 >= ADCLEVEL4) && (ADC12MEM0 < ADCLEVEL5)) {
            //LEVEL4
            UCA0TXBUF = LEVEL4; // Send the UART message LEVEL4
            myLCD_showChar('4', 1);
        } else if ((ADC12MEM0 >= ADCLEVEL5) && (ADC12MEM0 < ADCLEVEL6)) {
            //LEVEL5
            UCA0TXBUF = LEVEL5; // Send the UART message LEVEL5
            myLCD_showChar('5', 1);
        } else if ((ADC12MEM0 >= ADCLEVEL6) && (ADC12MEM0 < ADCLEVEL7)) {
            //LEVEL6
            UCA0TXBUF = LEVEL6; // Send the UART message LEVEL6
            myLCD_showChar('6', 1);
        } else if ((ADC12MEM0 >= ADCLEVEL7) && (ADC12MEM0 < ADCLEVEL8)) {
            //LEVEL7
            UCA0TXBUF = LEVEL7; // Send the UART message LEVEL7
            myLCD_showChar('7', 1);
        } else {
            //LEVEL8
            UCA0TXBUF = LEVEL8; // Send the UART message LEVEL8
            myLCD_showChar('8', 1);
        }

        input = 92; // Next sample from P9.2
        changeADC12Pin(ADC12_P92); // Change to P9.2
    }
    else { // Else, sample was from P9.2 (Battery Level)
        // Begin checking which level the battery is currently at
        if (ADC12MEM0 < ADCBATLEVEL1) {
            //LEVEL0
            UCA0TXBUF = BATLEVEL0; // Send the UART message BATLEVEL0
            myLCD_showChar('0', 2);
        } else if ((ADC12MEM0 >= ADCBATLEVEL1) && (ADC12MEM0 < ADCBATLEVEL2)) {
            //LEVEL1
            UCA0TXBUF = BATLEVEL1; // Send the UART message BATLEVEL1
            myLCD_showChar('1', 2);
        } else if ((ADC12MEM0 >= ADCBATLEVEL2) && (ADC12MEM0 < ADCBATLEVEL3)) {
            //LEVEL2
            UCA0TXBUF = BATLEVEL2; // Send the UART message BATLEVEL2
            myLCD_showChar('2', 2);
        } else if ((ADC12MEM0 >= ADCBATLEVEL3) && (ADC12MEM0 < ADCBATLEVEL4)) {
            //LEVEL3
            UCA0TXBUF = BATLEVEL3; // Send the UART message BATLEVEL3
            myLCD_showChar('3', 2);
        } else if ((ADC12MEM0 >= ADCBATLEVEL4) && (ADC12MEM0 < ADCBATLEVEL5)) {
            //LEVEL4
            UCA0TXBUF = BATLEVEL4; // Send the UART message BATLEVEL4
            myLCD_showChar('4', 2);
        } else if ((ADC12MEM0 >= ADCBATLEVEL5) && (ADC12MEM0 < ADCBATLEVEL6)) {
            //LEVEL5
            UCA0TXBUF = BATLEVEL5; // Send the UART message BATLEVEL5
            myLCD_showChar('5', 2);
        } else {
            UCA0TXBUF = BATLEVEL6; // Send the UART message BATLEVEL6
            myLCD_showChar('6', 2);
        }

        input = 84; // Next sample from P8.4
        changeADC12Pin(ADC12_P84); // Change to P8.4
    }
    ADC12CTL0 |= ADC12ENC | ADC12SC; // Re-enable conversion and start next conversion
}

// RX UART interrupt
#pragma vector=USCI_A0_VECTOR
__interrupt void UART_ISR(void)
{
    if((UCA0RXBUF >= 0x32) && (UCA0RXBUF <= 0xC8)) { // Check to see if the message is >=50 and <=200
        myLCD_displayNumber(UCA0RXBUF);
        P9OUT |= BIT7; // Light up green LED to show received signal
        unsigned char tempStorage = UCA0RXBUF; // Make a temp variable to hold UCA0RXBUF
        if ((UCA0RXBUF >= 0x32) && (UCA0RXBUF < 0x4B)) { // Set value to 75 if it is between 50 and 74
            tempStorage = 0x4B;
        }

        UCA0TXBUF = MSGCONFIRM; // Send a confirm message to the controller
        runPump(tempStorage); // Run the pump based on the glucose level
        P9OUT &= ~BIT7; // Turn OFF the green LED

    } else if(UCA0RXBUF == ESTOP) { // Emergency stop
        while(UCA0RXBUF != WAKEUP) { // Stay here until wake up signal is given by the controller
            myLCD_showChar('E', 1);
            myLCD_showChar(' ', 2);
            myLCD_showChar('S', 3);
            myLCD_showChar('T', 4);
            myLCD_showChar('O', 5);
            myLCD_showChar('P', 6);
        }
        // Clear screen when re-entering normal operation
        myLCD_showChar(' ', 1);
        myLCD_showChar(' ', 2);
        myLCD_showChar(' ', 3);
        myLCD_showChar(' ', 4);
        myLCD_showChar(' ', 5);
        myLCD_showChar(' ', 6);
    }
    else { // If no, turn off the red LED
        myLCD_showChar('U', 1);
        myLCD_showChar('A', 2);
        myLCD_showChar('R', 3);
        myLCD_showChar('T', 4);
        myLCD_showChar('E', 5);
        myLCD_showChar('R', 6);
    }

    UCA0IFG = UCA0IFG & (~UCRXIFG); // Clear RX Interrupt FlaG
}

// Interrupt for buttons P1.1 and P1.2
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void) {

    /*----debounce----*/
    unsigned char k;
    unsigned char debounce = 0;
    unsigned int i;
    for(k=0;k<15;k++) {
        for(i=0; i<100;i++);
        if(!(P1IN & 0x02) | !(P1IN & 0x04)) debounce++;
    }
    /*-----------------*/

    if(debounce > 4) {
        switch(P1IV) {
        case 4: //P1.1 interrupt
            // Our controller microcontroller's buttons are broken and we don't have time to debug.
            //  Pump buttons are now emulating the controller's button by sending this as the E-STOP
            //  command and before the controller E-STOPS, send the pump 0x00 so that we don't have
            //  to change existing code.
            UCA0TXBUF = 0x97;

            // Delay because it wont send without a delay
            int i;
            for(i = 0; i <= 2000; i++);

            break;
        case 6: //P1.2 interrupt
            // Manually inject for 1000ms
            runPump(100);
            break;

        }
    }

    if(P1IV); // reset pending interrupt

}
