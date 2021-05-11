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

#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
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
#define FFT_OFFSET 128      // an offset for FFT display

#define PI 3.14159265358979f
#define NFFT 1024   //FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(NFFT-1))

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
uint16_t WaveBuffer [NFFT];
uint16_t processedBuffer [NFFT];
volatile bool trigger_mode = 1;    // trigger mode, 1: trigger at up, 0: trigger at down
volatile bool spectrum_mode = 0;   // spectrum mode, 1: enable spectrum mode, 0: disable spectrum mode

void signal_init(void);
int FindTrigger(bool trig_mode);
void loadBuffer(uint16_t* locBuffer, int trigger);
void loadFFTBuffer(uint16_t* fftBuffer, int trigger);
int32_t cpu_unload_count(void);
int32_t cpu_load_count(void);


////debug
//uint32_t task1cnt = 0;
//uint32_t task2cnt = 0;
//uint32_t task3cnt = 0;
//uint32_t clkcnt = 0;

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

    while (true) {
        Semaphore_pend(sema0, BIOS_WAIT_FOREVER);
//        task1cnt ++; //debug

        if (spectrum_mode) {
            int32_t FFTBufferIndex = gADCBufferIndex;
            FFTBufferIndex = FFTBufferIndex - NFFT + 1; // offset the index for FFT buffer start point
            loadFFTBuffer(WaveBuffer, FFTBufferIndex);

        } else {
            bool l_trigger_mode = trigger_mode;  // a local copy of global trigger selection
            int trigger = FindTrigger(l_trigger_mode);
            loadBuffer(WaveBuffer, trigger);
        }

        Semaphore_post(sema1); // trigger processing_task
    }
}

void processing_task(UArg arg1, UArg arg2) // lowest priority
{
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg; // Kiss FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT]; // complex waveform and spectrum buffers
    int i;
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT

    static float w[NFFT]; // window function
    for (i = 0; i < NFFT; i++) {
     // Blackman window
     w[i] = 0.42f
     - 0.5f * cosf(2*PI*i/(NFFT-1))
     + 0.08f * cosf(4*PI*i/(NFFT-1));
    }

    while (true) {
        Semaphore_pend(sema1, BIOS_WAIT_FOREVER); // pending on trigger from waveform task
//        task2cnt ++; //debug

        if (spectrum_mode) {
            for (i = 0; i < NFFT; i++) { // generate an input waveform
             in[i].r = WaveBuffer[i] * w[i]; // real part of waveform
             in[i].i = 0; // imaginary part of waveform
            }
            kiss_fft(cfg, in, out); // compute FFT
            // convert first 128 bins of out[] to dB for display
            i = 0;
            while (i < LCD_HORIZONTAL_MAX) {
                processedBuffer[i] = FFT_OFFSET - 10 * log10f(out[i].r * out[i].r + out[i].i * out[i].i);
                i ++;
            }
        }
        else {
            i = 0;
            while (i < LCD_HORIZONTAL_MAX) {
                processedBuffer[i] = WaveBuffer[i];
                i ++;
            }
        }

        Semaphore_post(screenupdate); // trigger display task
        Semaphore_post(sema0); // trigger waveform task
    }
}

void display_task(UArg arg1, UArg arg2) // low priority
{
    int x, y,nxt_x,nxt_y;
    bool l_trigger_mode; // a local copy of trigger mode selector
    const float fVoltsPerDiv = 1.0f;
    char str_tscale [10];        // string for displaying time scale
    char VoltageScaleStr [10];   // string for displaying voltage scale
    char str_FFTfreq [10];        // string for displaying time scale
    char str_FFTdB [10];   // string for displaying voltage scale
    float fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    while(true) {
        Semaphore_pend(screenupdate, BIOS_WAIT_FOREVER); // pending on trigger from processing task
//        task3cnt ++; // debug

        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black

        if (spectrum_mode) {
            // draw FFT grid
            GrContextForegroundSet(&sContext, ClrDarkBlue);
            GrLineDrawH (&sContext, 0, LCD_HORIZONTAL_MAX-1, GRID_SPACING - 1);
            uint16_t gridoffset = 0;
            while (GRID_SPACING*gridoffset <= LCD_VERTICAL_MAX) {
                GrLineDrawH (&sContext, 0, LCD_HORIZONTAL_MAX-1, GRID_SPACING*gridoffset);
                GrLineDrawV (&sContext, GRID_SPACING*gridoffset, 0, LCD_VERTICAL_MAX-1);
                gridoffset++;
            }

            // draw wave
            x = 0;
            GrContextForegroundSet(&sContext, ClrYellow); // yellow text
            while (x < LCD_HORIZONTAL_MAX-1) {
                nxt_x = x + 1;
                y = *(processedBuffer + x);
                nxt_y = *(processedBuffer + nxt_x);
                GrLineDraw (&sContext, x, y, nxt_x, nxt_y);
                x++;
            }

            //draw screen elements
            GrContextForegroundSet(&sContext, ClrWhite); // white text
            snprintf(str_FFTfreq, sizeof(str_FFTfreq), "%u kHz", 20);
            GrStringDraw(&sContext, str_FFTfreq, /*length*/ -1, /*x*/ 0, /*y*/ 0, /*opaque*/ false);
            snprintf(str_FFTdB, sizeof(str_FFTdB), "%u dB", 20);
            GrStringDraw(&sContext, str_FFTdB, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 - sizeof(str_FFTdB), /*y*/ 0, /*opaque*/ false);

            GrFlush(&sContext); // flush the frame buffer to the LCD
        } else {
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

            //draw screen elements
            GrContextForegroundSet(&sContext, ClrWhite); // white text
            snprintf(str_tscale, sizeof(str_tscale), "%u us", 20);
            GrStringDraw(&sContext, str_tscale, /*length*/ -1, /*x*/ 10, /*y*/ 0, /*opaque*/ false);
            snprintf(VoltageScaleStr, sizeof(VoltageScaleStr), "%uV", 1);
            GrStringDraw(&sContext, VoltageScaleStr, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 - sizeof(VoltageScaleStr), /*y*/ 0, /*opaque*/ false);

            // draw trigger
            l_trigger_mode = trigger_mode;  // save a local copy to prevent shared data issue
            if (l_trigger_mode) {
                // draw rising trigger shape
                GrLineDrawH(&sContext, LCD_HORIZONTAL_MAX-20, LCD_HORIZONTAL_MAX-15, 6);
                GrLineDrawH(&sContext, LCD_HORIZONTAL_MAX-15, LCD_HORIZONTAL_MAX-10, 0);
                GrLineDrawV(&sContext, LCD_HORIZONTAL_MAX-15, 6, 0);
                // draw rising trigger arrow
                GrLineDraw (&sContext, LCD_HORIZONTAL_MAX-17, 3, LCD_HORIZONTAL_MAX-15, 1);
                GrLineDraw (&sContext, LCD_HORIZONTAL_MAX-13, 3, LCD_HORIZONTAL_MAX-15, 1);
            } else {
                // draw falling trigger shape
                GrLineDrawH(&sContext, LCD_HORIZONTAL_MAX-15, LCD_HORIZONTAL_MAX-10, 6);
                GrLineDrawH(&sContext, LCD_HORIZONTAL_MAX-20, LCD_HORIZONTAL_MAX-15, 0);
                GrLineDrawV(&sContext, LCD_HORIZONTAL_MAX-15, 6, 0);
                // draw falling trigger arrow
                GrLineDraw (&sContext, LCD_HORIZONTAL_MAX-17, 2, LCD_HORIZONTAL_MAX-15, 4);
                GrLineDraw (&sContext, LCD_HORIZONTAL_MAX-13, 2, LCD_HORIZONTAL_MAX-15, 4);
            }

            GrFlush(&sContext); // flush the frame buffer to the LCD
        }
    }
}

void button_clock(void) // clock function
{
//    clkcnt++; //debug
    Semaphore_post(sample_btn);
}


void button_task(UArg arg1, UArg arg2) // high priority
{

    while(true) {
        Semaphore_pend(sample_btn, BIOS_WAIT_FOREVER);

        TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
        // read hardware button state
        uint32_t gpio_buttons = ~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0); // EK-TM4C1294XL buttons in positions 0 and 1
        gpio_buttons = gpio_buttons | ((~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & GPIO_PIN_1) << 1); // load S1 to bitmap
        gpio_buttons = gpio_buttons | ((~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & GPIO_PIN_6) >> 3); // load S2 to bitmap
        gpio_buttons = gpio_buttons | (~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & GPIO_PIN_4);        // load Joystick select to bitmap

        uint32_t old_buttons = gButtons;    // save previous button state
        ButtonDebounce(gpio_buttons);       // Run the button debouncer. The result is in gButtons.
        ButtonReadJoystick();               // Convert joystick state to button presses. The result is in gButtons.
        uint32_t presses = ~old_buttons & gButtons;   // detect button presses (transitions from not pressed to pressed)
        presses |= ButtonAutoRepeat();      // autorepeat presses if a button is held long enough

        char loc_button = (char)gButtons;
        Mailbox_post(button_mailbox, &loc_button, BIOS_WAIT_FOREVER);
    }
}

void userinput_task (UArg arg1, UArg arg2) // medium priority
{
    char button = 0;
    char button_old, pressed;

    while(true) {
        button_old = button;
        Mailbox_pend(button_mailbox, &button, BIOS_WAIT_FOREVER);

        pressed = ~button & button_old; // detect the button from pressed to non pressed

        if (pressed & 1)
            trigger_mode = !trigger_mode; // if sw 1 pressed, flip the trigger mode

        if (pressed & 2) {
            spectrum_mode = !spectrum_mode; // if sw 2 pressed, change display mode
        }
    }

}


int FindTrigger (bool trig_mode) // search for rising edge trigger
{
    // Step 1
    int32_t locBufferIndex = gADCBufferIndex;
    int x = locBufferIndex - LCD_HORIZONTAL_MAX/2/* half screen width; don’t use a magic number */;
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    if (trig_mode) {
        for (; x > x_stop; x--) {
            if (gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET &&
                    gADCBuffer[ADC_BUFFER_WRAP(x-1)]/* next older sample */ < ADC_OFFSET)
                break;
        }
    }
    else {
        for (; x > x_stop; x--) {
            if (gADCBuffer[ADC_BUFFER_WRAP(x)] <= ADC_OFFSET &&
                    gADCBuffer[ADC_BUFFER_WRAP(x-1)]/* next older sample */ > ADC_OFFSET)
                break;
        }
    }

    // Step 3
    if (x == x_stop) // for loop ran to the end
        x = gADCBufferIndex - LCD_HORIZONTAL_MAX/2; // reset x back to how it was initialized
    return x;
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

void loadFFTBuffer(uint16_t* fftBuffer, int trigger) {

    int i = 0; // local buffer index
    while (i < NFFT){
        *(fftBuffer + i) = gADCBuffer[ADC_BUFFER_WRAP(trigger + i)];
        i++;
    }
}


int32_t cpu_unload_count(void) {
    int32_t CPU_unload = 0;
    TimerEnable(TIMER3_BASE, TIMER_A);
    while (TIMER3_CTL_R & TIMER_CTL_TAEN) {
        CPU_unload++; // iterate if time is not up
    }
    return CPU_unload;
}

int32_t cpu_load_count(void) {
    int32_t CPU_load = 0;
    TimerEnable(TIMER3_BASE, TIMER_A);
    while (TIMER3_CTL_R & TIMER_CTL_TAEN) {
        CPU_load++; // iterate if time is not up
    }
    return CPU_load;
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
