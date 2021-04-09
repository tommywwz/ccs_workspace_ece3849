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
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

#define PWM_FREQUENCY 20000  // PWM frequency = 20 kHz

uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 0; // time in hundredths of a second

uint32_t dec2bin (volatile uint32_t button_dec); // function that converts decimal to binary
void signal_init();

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

    // initialize PWM
    signal_init();

    //uint32_t time;  // local copy of gTime
    uint32_t mm;
    uint32_t ss;
    uint32_t ms;
    uint32_t gButton_b;

    char str[50];   // string buffer
    char str_bitmap[50];
    // full-screen rectangle
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    ButtonInit();
    IntMasterEnable();

    while (true) {
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black
        //time = gTime; // read shared global only once
        mm = gTime / 6000;
        ss = (gTime - 6000 * mm) / 100;
        ms = gTime - 6000 * mm - 100 * ss;
        gButton_b = dec2bin (gButtons); // function that converts decimal to binary
        snprintf(str, sizeof(str), "Time = %02u:%02u:%02u", mm, ss, ms); // convert time to string
        snprintf(str_bitmap, sizeof(str_bitmap), "Input: %09u", gButton_b);
        GrContextForegroundSet(&sContext, ClrYellow); // yellow text
        GrStringDraw(&sContext, str, /*length*/ -1, /*x*/ 0, /*y*/ 0, /*opaque*/ false);
        GrStringDraw(&sContext, str_bitmap, /*length*/ -1, /*x*/ 0, /*y*/ 8, /*opaque*/ false);
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

void signal_init() {
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
