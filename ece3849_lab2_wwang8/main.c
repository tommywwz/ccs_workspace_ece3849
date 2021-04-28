/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "Crystalfontz128x128_ST7735.h"
#include "inc/tm4c1294ncpdt.h"

#include "buttons.h"
#include "sampling.h"

#define PWM_FREQUENCY 20000  // PWM frequency = 20 kHz
#define ADC_OFFSET 2048         // ADC value when oscope reading = 0V
#define GRID_SPACING 20     // space for gird drawing
#define VIN_RANGE 3.3f       // range of ADC
#define PIXELS_PER_DIV 20    // LCD pixels per voltage division
#define ADC_BITS 12          // number of bits in the ADC sample

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
uint16_t WaveBuffer [LCD_HORIZONTAL_MAX];
uint16_t processedBuffer [LCD_HORIZONTAL_MAX];
float fScale;

void signal_init(void);
int RisingTrigger(void);
int FallingTrigger(void);
void loadBuffer(uint16_t* locBuffer, int trigger);

//debug
uint32_t task1cnt = 0;
uint32_t task2cnt = 0;
uint32_t task3cnt = 0;

/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    // initialize PWM
    signal_init();
    // initialize ADCs and Buttons
    ADCInit();
    ButtonInit();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void waveform_task(UArg arg1, UArg arg2) // highest priority
{
    IntMasterEnable();
    int trigger;
    const float fVoltsPerDiv = 2.0f;

    while (true) {
        Semaphore_pend(sema0, BIOS_WAIT_FOREVER);
        task1cnt ++;

        trigger = RisingTrigger();
        fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv);
        loadBuffer(WaveBuffer, trigger);

        Semaphore_post(sema1); // trigger processing_task
    }
}

void processing_task(UArg arg1, UArg arg2) // lowest priority
{
    uint32_t i;
    while (true) {
        Semaphore_pend(sema1, BIOS_WAIT_FOREVER); // pending on trigger from waveform task
        task2cnt ++;

        i = 0;
        while (i < LCD_HORIZONTAL_MAX) {
            processedBuffer[i] = WaveBuffer[i];
            i ++;
        }

        Semaphore_post(screenupdate); // trigger display task
        Semaphore_post(sema0); // trigger waveform task
    }
}

void display_task(UArg arg1, UArg arg2) // low priority
{
    int x, y,nxt_x,nxt_y;

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    while(true) {
        Semaphore_pend(screenupdate, BIOS_WAIT_FOREVER); // pending on trigger from processing task

        task3cnt ++;

        // draw grid
        GrContextForegroundSet(&sContext, ClrDarkBlue);
        uint16_t gridoffset = 0;
        while (GRID_SPACING*gridoffset <= LCD_VERTICAL_MAX/2) {
            GrLineDrawH (&sContext, 0, LCD_HORIZONTAL_MAX-1, LCD_VERTICAL_MAX/2 - GRID_SPACING*gridoffset - 1);
            GrLineDrawH (&sContext, 0, LCD_HORIZONTAL_MAX-1, LCD_VERTICAL_MAX/2 + GRID_SPACING*gridoffset);
            GrLineDrawV (&sContext, LCD_HORIZONTAL_MAX/2 - GRID_SPACING*gridoffset - 1, 0, LCD_VERTICAL_MAX-1);
            GrLineDrawV (&sContext, LCD_HORIZONTAL_MAX/2 + GRID_SPACING*gridoffset, 0, LCD_VERTICAL_MAX-1);
            gridoffset++;
        }

        // draw wave
        x = 0;
        GrContextForegroundSet(&sContext, ClrYellow); // yellow text
        while (x < LCD_HORIZONTAL_MAX-1) {
            nxt_x = x + 1;
            y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(*(processedBuffer + x)/*sample*/) - ADC_OFFSET));
            nxt_y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(*(processedBuffer + nxt_x)/*sample*/) - ADC_OFFSET));
            GrLineDraw (&sContext, x, y, nxt_x, nxt_y);
            x++;
        }

        GrFlush(&sContext); // flush the frame buffer to the LCD
    }
}


int RisingTrigger(void) // search for rising edge trigger
{
    // Step 1
    int32_t locBufferIndex = gADCBufferIndex;
    int x = locBufferIndex - LCD_HORIZONTAL_MAX/2/* half screen width; don’t use a magic number */;
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--) {
        if (gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET &&
                gADCBuffer[ADC_BUFFER_WRAP(x-1)]/* next older sample */ < ADC_OFFSET)
            break;
    }
    // Step 3
    if (x == x_stop) // for loop ran to the end
        x = gADCBufferIndex - LCD_HORIZONTAL_MAX/2; // reset x back to how it was initialized
    return x;
}

//int FallingTrigger(void) // search for falling edge trigger
//{
//    // Step 1
//    int32_t locBufferIndex = gADCBufferIndex;
//    int x = locBufferIndex - LCD_HORIZONTAL_MAX/2/* half screen width; don’t use a magic number */;
//    // Step 2
//    int x_stop = x - ADC_BUFFER_SIZE/2;
//    for (; x > x_stop; x--) {
//        if (gADCBuffer[ADC_BUFFER_WRAP(x)] <= ADC_OFFSET &&
//                gADCBuffer[ADC_BUFFER_WRAP(x-1)]/* next older sample */ > ADC_OFFSET)
//            break;
//    }
//    // Step 3
//    if (x == x_stop) // for loop ran to the end
//        x = gADCBufferIndex - LCD_HORIZONTAL_MAX/2; // reset x back to how it was initialized
//    return x;
//}

void loadBuffer(uint16_t* locBuffer, int trigger) {
    // Step 4
    int i = 0; // local buffer index
    int x = trigger - LCD_HORIZONTAL_MAX/2; //set beginning of index to the half screen behind the trigger
    while (i < LCD_HORIZONTAL_MAX){
        *(locBuffer + i) = gADCBuffer[ADC_BUFFER_WRAP(x + i)];
        i++;
    }
}


void signal_init(void) {
    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
    // configure M0PWM3, at GPIO PF3, BoosterPack 1 header C1 pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}
