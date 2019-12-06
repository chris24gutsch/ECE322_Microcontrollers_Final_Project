/*
 * PumpController.h
 *
 *  Created on: Dec 4, 2019
 *      Author: Chris Gutschlag
 */

#ifndef PUMPCONTROLLER_H_
#define PUMPCONTROLLER_H_

/*----------UART Constants----------*/
#define UART_CLK_SEL 0x0080 // Specifies accurate clock for UART peripheral
#define BR0_FOR_9600 0x34 // Value required to use 9600 baud
#define BR1_FOR_9600 0x00 // Value required to use 9600 baud
#define CLK_MOD 0x4911 // Microcontroller will "clean-up" clock signal
/*----------------------------------*/

/*----------ADC12 Constants----------*/
#define ADC12_SHT_16 0x0200 // 16 clock cycles for sample and hold
#define ADC12_ON 0x0010 // Used to turn ADC12 peripheral on
#define ADC12_SHT_SRC_SEL 0x0200 // Selects source for sample & hold
#define ADC12_12BIT 0x0020 // Selects 12-bits of resolution

#define ADC12_P84 0x0007 // P8.4 for analog input
#define ADC12_P92 0x000A // P9.2 for analog input
/*-----------------------------------*/

/*----------Pump Constants----------*/
#define PERIOD20MS    40000 // 20ms period due to SMCLK being changed to 2MHz by initClocks()
#define P20MS_DC10    4000  // 10% duty cycle for PERIOD20MS
#define MAXDOSECYCLES 125   // Max cycles the timer controlling the pump can run
/*----------------------------------*/

/*----------Insulin Reservoir Level Constants----------*/
#define LEVEL0 0x80 // <0.4125V (Empty)
#define LEVEL1 0x81 // >=0.4125V
#define LEVEL2 0x82 // >=0.825V
#define LEVEL3 0x83 // >=1.2375V
#define LEVEL4 0x84 // >=1.65V
#define LEVEL5 0x85 // >=2.0625V
#define LEVEL6 0x86 // >=2.475V
#define LEVEL7 0x87 // >=2.8875V
#define LEVEL8 0x88 // >=3.20V (Full)

#define ADCLEVEL1 0x200 // 0.4125V
#define ADCLEVEL2 0x400 // 0.825V
#define ADCLEVEL3 0x600 // 1.2375V
#define ADCLEVEL4 0x800 // 1.65V
#define ADCLEVEL5 0x9FF // 2.0625V
#define ADCLEVEL6 0xBFF // 2.475V
#define ADCLEVEL7 0xDFF // 2.8875V
#define ADCLEVEL8 0xF83 // 3.20V (Full)
/*-----------------------------------------------------*/

/*----------Voltage Level Constants----------*/
#define BATLEVEL0 0x90 // Vbat< 2.20V
#define BATLEVEL1 0x91 // Vbat>=2.20V
#define BATLEVEL2 0x92 // Vbat>=2.42V
#define BATLEVEL3 0x93 // Vbat>=2.64V
#define BATLEVEL4 0x94 // Vbat>=2.86V
#define BATLEVEL5 0x95 // Vbat>=3.08V
#define BATLEVEL6 0x96 // Vbat>=3.20V

#define ADCBATLEVEL1 0xAAA // 2.20V
#define ADCBATLEVEL2 0xBBB // 2.42V
#define ADCBATLEVEL3 0xCCC // 2.64V
#define ADCBATLEVEL4 0xDDD // 2.86V
#define ADCBATLEVEL5 0xEEE // 3.08V
#define ADCBATLEVEL6 0xF83 // 3.20V
/*-------------------------------------------*/

#define ESTOP      0x00 // Emergency stop code
#define WAKEUP     0x98 // Wake up signal to wake up from E-Stop
#define MSGCONFIRM 0x99 // Message to transmit back to controller after successfully receiving a message

/*
 * Name:
 *      runPump
 *
 * Description:
 *      Runs the pump with a period of 40ms at 10% duty cycle
 *       for the given number of cycles, RunCycles
 *
 * Parameters:
 *      RunCycles - Number of cycles to run the pump
 */
void runPump(unsigned char RunCycles);

/*
 * Name:
 *      changeADC12Pin
 *
 * Description:
 *      Disable ADC12 peripheral on ADC12CTL0 and change to Pin
 *
 * Parameters:
 *      RunCycles - Number of cycles to run the pump
 */
void changeADC12Pin(unsigned char Pin);

/*
 * Name:
 *      ADC_SETUP
 *
 * Description:
 *      Setup the ADC12
 */
void ADC_SETUP(void);

/*
 * Name:
 *      select_clock_signals
 *
 * Description:
 *      Select the clock to be used for UART
 */
void select_clock_signals(void);

/*
 * Name:
 *      assign_pins_to_uart
 *
 * Description:
 *      Assign the pins that will be used for the UART
 *      TXD - P4.2
 *      RXD - P4.3
 */
void assign_pins_to_uart(void);

/*
 * Name:
 *      use_9600_baud
 *
 * Description:
 *      Set the baud rate to be 9600
 */
void use_9600_baud(void);
#endif /* PUMPCONTROLLER_H_ */
