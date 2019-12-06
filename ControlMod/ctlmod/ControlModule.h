/*
 * ControlModule.h
 *
 *  Created on: Dec 4, 2019
 *      Author: Allen Poe
 */

#ifndef CONTROLMODULE_H_
#define CONTROLMODULE_H_

/*----------ADC12 Constants----------*/
#define ADC12_SHT_16 0x0200 // 16 clock cycles for sample and hold
#define ADC12_ON 0x0010 // Used to turn ADC12 peripheral on
#define ADC12_SHT_SRC_SEL 0x0200 // Selects source for sample & hold
#define ADC12_12BIT 0x0020 // Selects 12-bits of resolution

#define ADC12_P84 0x0007 // P8.4 for analog input
#define ADC12_P92 0x000A // P9.2 for analog input
/*-----------------------------------*/

/*----------Pump Constants----------*/
#define PERIOD40MS    40000 // 40ms period
#define P40MS_DC10    4000  // 10% duty cycle for PERIOD40MS
#define MAXDOSECYCLES 125   // Max cycles the timer controlling the pump can run
/*----------------------------------*/

/*----------Insulin Reservoir Level Constants----------*/
#define LEVEL0 0x80 // Empty
#define LEVEL1 0x81
#define LEVEL2 0x82
#define LEVEL3 0x83
#define LEVEL4 0x84
#define LEVEL5 0x85
#define LEVEL6 0x86
#define LEVEL7 0x87
#define LEVEL8 0x88 // Full
/*-----------------------------------------------------*/

/*----------Voltage Level Constants----------*/
#define BATLEVEL0 0x80 // Vbat< 2.20V
#define BATLEVEL1 0x81 // Vbat>=2.20V
#define BATLEVEL2 0x82 // Vbat>=2.42V
#define BATLEVEL3 0x83 // Vbat>=2.64V
#define BATLEVEL4 0x84 // Vbat>=2.86V
#define BATLEVEL5 0x85 // Vbat>=3.08V
#define BATLEVEL6 0x86 // Vbat>=3.20V

#define ADCBATLEVEL1 0xAAA // Vbat>=2.20V
#define ADCBATLEVEL2 0xBBB // Vbat>=2.42V
#define ADCBATLEVEL3 0xCCC // Vbat>=2.64V
#define ADCBATLEVEL4 0xDDD // Vbat>=2.86V
#define ADCBATLEVEL5 0xEEE // Vbat>=3.08V
#define ADCBATLEVEL6 0xF83 // Vbat>=3.20V
/*-------------------------------------------*/

#endif /* CONTROLMODULE_H_ */
