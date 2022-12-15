#include "lab04.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "math.h"

#include "types.h"
#include "lcd.h"
#include "led.h"
#include "dac.h"

// signal parameter
#define f 10
#define sr 300
#define V_min 1
#define V_max 3
#define V_offset 2
#define PI 3.1415926535

float V_out;
float t;

/*
 * Timer code
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

void timer_initialize()
{
     __builtin_write_OSCCONL(OSCCONL | 2);//enable LPOSC, enable macro to a 32kHz
    // Disable the Timers

    T3CONbits.TON = 0;//Disable Timer 3
    // Set Prescaler

    T3CONbits.TCKPS = 0b00;// 1/1
    // Set Clock Source

    T3CONbits.TCS = 0;
    // Set Gated Timer Mode -> don't use gating

    CLEARBIT(T3CONbits.TGATE);
    // T1: Set External Clock Input Synchronization -> no sync
    // Load Timer Periods
    PR3 =(1.0/sr)*12800;//(1.0/sr) is period time in second. PR3 is ticking.
    // Reset Timer Values
    TMR3 = 0x00;
    
    // Set Interrupt Priority
    IPC2bits.T3IP = 0x01;
    // Clear Interrupt Flags
    IFS0bits.T3IF = 0;
    // Enable Interrupts
    IEC0bits.T3IE = 1;
    // Enable the Timers
    T3CONbits.TON = 1;
}


void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T3Interrupt(void)
{ // invoked every ??
    IFS0bits.T3IF = 0; // clear the interrupt flag
    TOGGLELED(LED1_PORT);
    t=(float)(t+(1.0/sr));//here use 1.0/sr to make sure the type is float
    V_out=sin(2*PI*f*t)+V_offset;
    dac_convert_milli_volt(V_out*1000);//here the parameter is milli volt
}
/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab04: Wave");
    lcd_locate(0, 1);
    lcd_printf("Group: Group7");

    
    
    while(TRUE) {
    }
}
