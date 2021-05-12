/*
 * sampling.c
 *
 *  Created on: Apr 10, 2021
 *      Author: Weizhe Wang
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "sampling.h"
#include "driverlib/udma.h"
#include <xdc/cfg/global.h> //needed for gate object

volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1; // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile uint32_t gADCErrors = 0; // number of missed ADC deadlines

#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required
tDMAControlTable gDMAControlTable[64]; // uDMA control table (global)

volatile bool gDMAPrimary = true; // is DMA occurring in the primary channel?

void ADCInit(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; //round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCSequenceDisable(ADC0_BASE, 0); // choose ADC0 sequence 0; disable before configuring
    ADCSequenceDisable(ADC1_BASE, 0); // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0); // specify the "Always" trigger
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
#ifdef SINGLE_SAMPLE_ISR
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);   // in the 0th step, sample channel 3 (AIN3)
#endif

#ifdef DMA_ONE_MSPS     // configure as FIFO, interrupting every 1 usec
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADC_CTL_CH3 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 2, ADC_CTL_CH3 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADC_CTL_CH3 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 4, ADC_CTL_CH3 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 5, ADC_CTL_CH3 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 6, ADC_CTL_CH3 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 7, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
#endif

#ifdef DMA_TWO_MSPS     // configure as FIFO, interrupting every 2 usec
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADC_CTL_CH3 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 2, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADC_CTL_CH3 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 4, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 5, ADC_CTL_CH3 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 6, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 7, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
#endif
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH13);                             // Joystick HOR(X)
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END);  // Joystick VER(Y)
                                                                                         // enable interrupt, and make it the end of sequence
    ADCSequenceEnable(ADC0_BASE, 0); // enable the sequence. it is now sampling
    ADCSequenceEnable(ADC1_BASE, 0);
#ifdef SINGLE_SAMPLE_ISR
    ADCIntEnable(ADC1_BASE, 0); // enable sequence 0 interrupt in the ADC1 peripheral
#endif
//    IntPrioritySet(INT_ADC1SS0, 0); // set ADC1 sequence 0 interrupt priority
//    IntEnable(INT_ADC1SS0); // enable ADC1 sequence 0 interrupt in int. controller

#if defined DMA_ONE_MSPS || DMA_TWO_MSPS
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);
    uDMAChannelAssign(UDMA_CH24_ADC1_0); // assign DMA channel 24 to ADC1 sequence 0
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);
    // primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
     UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
     UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
     (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);
    // alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
     UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
     UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
     (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);

    ADCSequenceDMAEnable(ADC1_BASE, 0); // enable DMA for ADC1 sequence 0
    ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0); // enable ADC1 sequence 0 DMA interrupt
#endif
}

#ifdef SINGLE_SAMPLE_ISR
void ADC_ISR(void) {

    ADC1_ISC_R = ADC_ISC_IN0; // clear ADC1 sequence0 interrupt flag in the ADCISC register
    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
        gADCErrors++; // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0; // clear overflow condition
    }
    gADCBuffer[
               gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
               ] = ADC1_SSFIFO0_R; // read sample from the ADC1 sequence 0 FIFO
}
#endif

#if defined DMA_ONE_MSPS || DMA_TWO_MSPS
void ADC_ISR(void) {

    ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS0); // clear the ADC1 sequence 0 DMA interrupt flag
    // Check the primary DMA channel for end of transfer, and restart if needed.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) ==
    UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2); // restart the primary channel (same as setup)
        gDMAPrimary = false; // DMA is currently occurring in the alternate buffer
    }
    // Check the alternate DMA channel for end of transfer, and restart if needed.
    // Also set the gDMAPrimary global.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) ==
    UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2); // restart the alternate channel (same as setup)
        gDMAPrimary = true; // DMA is currently occurring in the primary buffer
    }
    // The DMA channel may be disabled if the CPU is paused by the debugger.
    if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10)) {
    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10); // re-enable the DMA channel
    }
}
#endif


int32_t getADCBufferIndex(void)
{
    int32_t index;
    if (gDMAPrimary) { // DMA is currently in the primary channel
        index = ADC_BUFFER_SIZE/2 - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
    }
    else { // DMA is currently in the alternate channel
        index = ADC_BUFFER_SIZE - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
    }
    return index;
}
