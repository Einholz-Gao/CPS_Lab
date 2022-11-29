#include "lab03.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * DAC code
 */

#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13
    
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

#define DAC_SDI_AD2CFG AD2PCFGLbits.PCFG10
#define DAC_SCK_AD2CFG AD2PCFGLbits.PCFG11
#define DAC_LDAC_AD2CFG AD2PCFGLbits.PCFG13
unsigned int T2_counter;


void dac_initialize()
{
    SETBIT(DAC_SDI_AD1CFG); // set Pin to Digital
    SETBIT(DAC_SCK_AD1CFG); // set Pin to Digital
    SETBIT(DAC_LDAC_AD1CFG); // set Pin to Digital
    
    SETBIT(DAC_SDI_AD2CFG); // set Pin to Digital
    SETBIT(DAC_SCK_AD2CFG); // set Pin to Digital
    SETBIT(DAC_LDAC_AD2CFG); // set Pin to Digital
    // set AN10, AN11 AN13 to digital mode
    // this means AN10 will become RB10, AN11->RB11, AN13->RB13
    // see datasheet 11.3
    
    CLEARBIT(DAC_CS_TRIS); // set Pin to Output
    CLEARBIT(DAC_SDI_TRIS); // set Pin to Output
    CLEARBIT(DAC_SCK_TRIS); // set Pin to Output
    CLEARBIT(DAC_LDAC_TRIS); // set Pin to Output
    // set RD8, RB10, RB11, RB13 as output pins
    
    SETBIT(DAC_CS_PORT);
    SETBIT(DAC_SCK_PORT);
    SETBIT(DAC_SDI_PORT);
    SETBIT(DAC_LDAC_PORT);
    // set default state: CS=1, SCK=1, SDI=??, LDAC=1
    
}

/*
 * Timer code
 */

#define FCY_EXT   32768UL

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

void timer_initialize()
{
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);
    // configure timer
    T1CONbits.TON = 0; //Disable Timer 1
    T2CONbits.TON = 0;//Disable Timer 2
    T3CONbits.TON = 0;//Disable Timer 3
    // Set Prescaler
    T1CONbits.TCKPS = 0b11;
    T2CONbits.TCKPS = 0b11;// 1/256
    T3CONbits.TCKPS = 0b00;// 1/1
    // Set Clock Source
    T1CONbits.TCS = 1;
    T2CONbits.TCS = 0;
    T3CONbits.TCS = 0;
    // Set Gated Timer Mode -> don't use gating
    CLEARBIT(T1CONbits.TGATE);
    CLEARBIT(T2CONbits.TGATE);
    CLEARBIT(T3CONbits.TGATE);
    // T1: Set External Clock Input Synchronization -> no sync
    T1CONbits.TSYNC = 0;
    // Load Timer Periods
    PR1 = 128;// 1 second starting from 0
    PR2 = 50;// 1 milliseconds
    PR3 = 65535;//max value for PR3 is 16bit
    // Reset Timer Values
    TMR1 = 0x00;
    TMR2 = 0x00;
    
    // Set Interrupt Priority
    IPC0bits.T1IP = 0x01;
    IPC1bits.T2IP = 0x01;
    // Clear Interrupt Flags
    IFS0bits.T1IF = 0;
    IFS0bits.T2IF = 0;
    // Enable Interrupts
    IEC0bits.T1IE = 1;
    IEC0bits.T2IE = 1;
    // Enable the Timers
    T1CONbits.TON = 1;
    T2CONbits.TON = 1;
    T3CONbits.TON = 1;
    
}


void Voltage1(){
        CLEARBIT(DAC_SCK_PORT);
        Nop();
        CLEARBIT(DAC_CS_PORT);
        Nop();
        
        int i;
        for(i=0; i<16;i++){
            if(i==0||i==2||i==3||i==4){
            SETBIT(DAC_SDI_PORT);// set to 1
            Nop();
            SETBIT(DAC_SCK_PORT);
            Nop();
            CLEARBIT(DAC_SCK_PORT);
            Nop();
            }
            else{
            CLEARBIT(DAC_SDI_PORT);// set to 1
            Nop();
            SETBIT(DAC_SCK_PORT);
            Nop();
            CLEARBIT(DAC_SCK_PORT);
            Nop();}
        
        }
       
        CLEARBIT(DAC_SCK_PORT);
        Nop();
        //we can use for-loop here, instead of so many lines
        SETBIT(DAC_CS_PORT);
        Nop();
        CLEARBIT(DAC_LDAC_PORT);
        Nop();
        SETBIT(DAC_LDAC_PORT);
        Nop();}

void Voltage2(){
        CLEARBIT(DAC_SCK_PORT);
        Nop();
        CLEARBIT(DAC_CS_PORT);
        Nop();
        
        int i;
        for(i=0; i<16;i++){
            if(i==0||i==3||i==4||i==6){
            SETBIT(DAC_SDI_PORT);// set to 1
            Nop();
            SETBIT(DAC_SCK_PORT);
            Nop();
            CLEARBIT(DAC_SCK_PORT);
            Nop();
            }
            else{
            CLEARBIT(DAC_SDI_PORT);// set to 1
            Nop();
            SETBIT(DAC_SCK_PORT);
            Nop();
            CLEARBIT(DAC_SCK_PORT);
            Nop();}
        
        }
       
        CLEARBIT(DAC_SCK_PORT);
        Nop();
        //we can use for-loop here, instead of so many lines
        SETBIT(DAC_CS_PORT);
        Nop();
        CLEARBIT(DAC_LDAC_PORT);
        Nop();
        SETBIT(DAC_LDAC_PORT);
        Nop();}

void Voltage3(){
        CLEARBIT(DAC_SCK_PORT);
        Nop();
        CLEARBIT(DAC_CS_PORT);
        Nop();
        
        int i;
        for(i=0; i<16;i++){
            if(i==0||i==3||i==4||i==5||i==6){
            SETBIT(DAC_SDI_PORT);// set to 1
            Nop();
            SETBIT(DAC_SCK_PORT);
            Nop();
            CLEARBIT(DAC_SCK_PORT);
            Nop();
            }
            else{
            CLEARBIT(DAC_SDI_PORT);// set to 1
            Nop();
            SETBIT(DAC_SCK_PORT);
            Nop();
            CLEARBIT(DAC_SCK_PORT);
            Nop();}
        
        }
       
        CLEARBIT(DAC_SCK_PORT);
        Nop();
        //we can use for-loop here, instead of so many lines
        SETBIT(DAC_CS_PORT);
        Nop();
        CLEARBIT(DAC_LDAC_PORT);
        Nop();
        SETBIT(DAC_LDAC_PORT);
        Nop();}
// interrupt service routine?
void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void)
{ // invoked every ??
    T2_counter++; // Increment a global counter
    IFS0bits.T2IF = 0; // clear the interrupt flag
    TOGGLELED(LED1_PORT);
}
/*
 * main loop
 */


void main_loop()
{
        // Enable the Timers
    T1CONbits.TON = 1;
    T2CONbits.TON = 1;
    T3CONbits.TON = 1;
    // print assignment information
    lcd_printf("Lab03: DAC");
    lcd_locate(0, 1);
    lcd_printf("Group: GroupName");
    int time0,time;
    time0=T2_counter;
    Voltage1();
    while(TRUE)
    {
        time=T2_counter-time0;
        if(time==500){
        Voltage2();
        }
        else if(time==2500){
        Voltage3();
        }
        else if(time==3500){
        Voltage1();
        }
    
        // main loop code
    }
}
