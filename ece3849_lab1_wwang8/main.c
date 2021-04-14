/**
 * main.c
 *
 * ECE 3849 Lab 0 Starter Project
 * Gene Bogdanov    10/18/2017
 *
 * This version is using the new hardware for B2017: the EK-TM4C1294XL LaunchPad with BOOSTXL-EDUMKII BoosterPack.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "buttons.h"
#include "sampling.h"
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

#define PWM_FREQUENCY 20000  // PWM frequency = 20 kHz
#define ADC_OFFSET 2048         // ADC value when oscope reading = 0V
#define VIN_RANGE 3.3f       // range of ADC
#define PIXELS_PER_DIV 20    // LCD pixels per voltage division
#define ADC_BITS 12          // number of bits in the ADC sample


uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 0; // time in hundredths of a second

uint32_t dec2bin (volatile uint32_t button_dec); // function that converts decimal to binary
void loadBuffer(uint16_t* locBuffer, int trigger);
int RisingTrigger(void);
void signal_init(void);                          // initial the PWM to generate wave


int main(void)
{
    IntMasterDisable();

    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    //uint32_t time;  // local copy of gTime
//    uint32_t locTime;
//    uint32_t mm;
//    uint32_t ss;
//    uint32_t ms;
//    uint32_t gButton_b;
    float fVoltsPerDiv = 1;
    float fScale;
    int x, y,nxt_x,nxt_y;
    int trigger;
    uint16_t locBuffer [LCD_HORIZONTAL_MAX];
    uint16_t gridspace = 20;

    char str_tscale[10];
    char str_vscale[10];
//    char str[50];   // string buffer
//    char str_bitmap[50];

    // full-screen rectangle
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    // initialize PWM
    signal_init();

    // initialize ADCs and Buttons
    ADCInit();
    ButtonInit();
    IntMasterEnable();

    while (true) {
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black
//        time = gTime; // read shared global only once
//        locTime = gTime;
//        mm = locTime / 6000;
//        ss = (locTime - 6000 * mm) / 100;
//        ms = locTime - 6000 * mm - 100 * ss;
//        gButton_b = dec2bin (gButtons); // function that converts decimal to binary\
//        snprintf(str, sizeof(str), "Time = %02u:%02u:%02u", mm, ss, ms); // convert time to string
//        snprintf(str_bitmap, sizeof(str_bitmap), "Input: %09u", gButton_b);
//        GrStringDraw(&sContext, str_bitmap, /*length*/ -1, /*x*/ 0, /*y*/ 8, /*opaque*/ false);

        fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv);
        trigger = RisingTrigger();
        loadBuffer(locBuffer, trigger);

        x = 0;

        GrContextForegroundSet(&sContext, ClrDarkBlue);

        // draw grid
        uint16_t gridoffset = 0;
        while (gridspace*gridoffset <= LCD_VERTICAL_MAX/2) {
            GrLineDrawH (&sContext, 0, LCD_HORIZONTAL_MAX-1, LCD_VERTICAL_MAX/2 - gridspace*gridoffset - 1);
            GrLineDrawH (&sContext, 0, LCD_HORIZONTAL_MAX-1, LCD_VERTICAL_MAX/2 + gridspace*gridoffset);
            GrLineDrawV (&sContext, LCD_HORIZONTAL_MAX/2 - gridspace*gridoffset - 1, 0, LCD_VERTICAL_MAX-1);
            GrLineDrawV (&sContext, LCD_HORIZONTAL_MAX/2 + gridspace*gridoffset, 0, LCD_VERTICAL_MAX-1);
            gridoffset++;
        }



        // draw wave
        GrContextForegroundSet(&sContext, ClrYellow); // yellow text
        while (x < LCD_HORIZONTAL_MAX-1) {
            nxt_x = x + 1;
            y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(*(locBuffer + x)/*sample*/) - ADC_OFFSET));
            nxt_y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(*(locBuffer + nxt_x)/*sample*/) - ADC_OFFSET));
            GrLineDraw (&sContext, x, y, nxt_x, nxt_y);
            x++;
        }

        //draw screen elements
        GrContextForegroundSet(&sContext, ClrWhite); // white text
        snprintf(str_tscale, sizeof(str_tscale), "%u us", 20);
        snprintf(str_vscale, sizeof(str_vscale), "%u mV", 1000);
        GrStringDraw(&sContext, str_tscale, /*length*/ -1, /*x*/ 10, /*y*/ 0, /*opaque*/ false);
        GrStringDraw(&sContext, str_vscale, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 - sizeof(str_vscale), /*y*/ 0, /*opaque*/ false);
        // draw trigger shape
        GrLineDrawH(&sContext, LCD_HORIZONTAL_MAX-20, LCD_HORIZONTAL_MAX-15, 6);
        GrLineDrawH(&sContext, LCD_HORIZONTAL_MAX-15, LCD_HORIZONTAL_MAX-10, 0);
        GrLineDrawV(&sContext, LCD_HORIZONTAL_MAX-15, 6, 0);
        // draw trigger arrow
        GrLineDraw (&sContext, LCD_HORIZONTAL_MAX-17, 3, LCD_HORIZONTAL_MAX-15, 1);
        GrLineDraw (&sContext, LCD_HORIZONTAL_MAX-13, 3, LCD_HORIZONTAL_MAX-15, 1);

        GrFlush(&sContext); // flush the frame buffer to the LCD
    }
}

uint32_t dec2bin (volatile uint32_t button_dec) {
    uint32_t button_bin = 0;
    uint32_t rem, temp = 1;

    while (button_dec != 0)
    {
        rem = button_dec % 2; // get reminder
        button_dec = button_dec / 2;
        button_bin = button_bin + rem*temp; // add number to each bit by binary
        temp = temp * 10;
    }
    return button_bin;
}


void loadBuffer(uint16_t* locBuffer, int trigger) {
    // Step 4
    int i = 0; // local buffer index
    int x = trigger - LCD_HORIZONTAL_MAX/2; //set beginning of index to the half screen behind the trigger
    while (i < LCD_HORIZONTAL_MAX){
        *(locBuffer + i) = gADCBuffer[ADC_BUFFER_WRAP(x + i)];
        i++;
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
