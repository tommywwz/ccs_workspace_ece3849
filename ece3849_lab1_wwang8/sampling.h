/*
 * sampling.h
 *
 *  Created on: Apr 10, 2021
 *      Author: Weizhe Wang
 */

#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <stdint.h>

#define ADC_SAMPLING_RATE 1000000   // [samples/sec] desired ADC sampling rate
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates


void ADCInit(void);


#endif /* SAMPLING_H_ */
