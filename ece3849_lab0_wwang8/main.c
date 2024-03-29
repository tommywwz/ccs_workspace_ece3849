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


uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 0; // time in hundredths of a second

uint32_t dec2bin (volatile uint32_t button_dec); // function that converts decimal to binary


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

