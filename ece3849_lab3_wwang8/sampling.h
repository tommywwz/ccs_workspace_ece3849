/*
 * sampling.h
 *
 *  Created on: Apr 10, 2021
 *      Author: Weizhe Wang
 */

#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <stdint.h>

#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates

//#define SINGLE_SAMPLE_ISR 1
//#define DMA_ONE_MSPS 1
#define DMA_TWO_MSPS 1

#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))

extern volatile int32_t gADCBufferIndex; // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
extern volatile uint32_t gADCErrors; // number of missed ADC deadlines

void ADCInit(void);

void ADC_ISR(void);

int32_t getADCBufferIndex(void);


#endif /* SAMPLING_H_ */
