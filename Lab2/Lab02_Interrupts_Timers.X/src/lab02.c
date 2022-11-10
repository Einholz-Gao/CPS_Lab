#include "lab02.h"

#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

#define FCY_EXT 32768
int T1_counter,T2_counter;
void initialize_timer()
{
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);//enable LPOSC, enable macro to a 32kHz
    // Disable the Timers
    T1CONbits.TON = 0; //Disable Timer 1
    T2CONbits.TON = 0;//Disable Timer 2
    T3CONbits.TON = 0;//Disable Timer 3
    // Set Prescaler
    T1CONbits.TCKPS = 0b40;
    T2CONbits.TCKPS = 0b40;
    T3CONbits.TCKPS = 0b00;
    // Set Clock Source
    T1CONbits.TCS = 1;
    T2CONbits.TCS = 0;
    T2CONbits.TCS = 0;
    // Set Gated Timer Mode -> don't use gating
    CLEARBIT(T1CONbits.TGATE);
    CLEARBIT(T2CONbits.TGATE);
    CLEARBIT(T3CONbits.TGATE);
    // T1: Set External Clock Input Synchronization -> no sync
    T1CONbits.TSYNC = 0;
    // Load Timer Periods
    PR1 = 128;// 1 second starting from 0
    PR2 = 100;// 2 milliseconds
    PR3 = 100000000000000000;
    // Reset Timer Values
    TMR1 = 0x00;
    TMR2 = 0x00;
    // Set Interrupt Priority
    IPC0bits.T1IP = 0x01;
    IPC0bits.T2IP = 0x01;
    // Clear Interrupt Flags
    IFS0bits.T1IF = 0;
    IFS0bits.T2IF = 0;
    // Enable Interrupts
    IEC0bits.T1IE = 1;
    IEC0bits.T2IE = 1;
    // Enable the Timers
    T1CONbits.TON = 1;
    T2CONbits.TON = 1;
    
}

void timer_loop()
{
    // print assignment information
    lcd_printf("Lab02: Int & Timer");
    lcd_locate(0, 1);
    lcd_printf("Group: Group7");
    TMR3 = 0x00;
    while(TRUE)
    {
        lcd_locate(0, 2);
        lcd_printf("%2d:%2d:%3d",T1_counter%60,T1_counter,T2_counter*2);
      
        if (TMR3 == 2000){
            TOGGLELED(LED3_PORT);
            TMR3 = 0x00;
            }
    }
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void)
{ // invoked every ??
    T1_counter++; // Increment a global counter
    IFS0bits.T1IF = 0; // clear the interrupt flag
    TOGGLELED(LED2_PORT);
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void)
{ // invoked every ??
    T2_counter++; // Increment a global counter
    IFS0bits.T2IF = 0; // clear the interrupt flag
    TOGGLELED(LED1_PORT);
}
