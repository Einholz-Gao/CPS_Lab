#include "lab06.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include <stdint.h>

#include <math.h>

#include "types.h"
#include "lcd.h"
#include "led.h"


/*
 * Parameter
 */
unsigned int T2_counter;


/*
 * Common Definitions
 */
#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03
#define a1 1.0
#define a2 -0.6796
#define b1 0.1602
#define b2 0.1602
#define r 50
#define x_center 325
#define y_center 260
#define d_speed 15.7


#define kp 1
#define kd 0.1



/*
 * Global Variables
 */
uint16_t rx_pre,ry_pre,rx_cur,ry_cur,rx_ref,ry_ref;
float deg=0;

//Circle
uint16_t circle(){
    deg+=d_speed;
    rx_ref = r * cos(deg)*180/3.14+x_center;
    ry_ref = r * cos(deg)*180/3.14+y_center;
}

/*
 * Timer Code
 */
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
    T3CONbits.TON = 0;//Disable Timer 2
    // Set Prescaler
    T3CONbits.TCKPS = 0b11;// 1/256
    // Set Clock Source
    T3CONbits.TCS = 0;
    // Set Gated Timer Mode -> don't use gating
    CLEARBIT(T3CONbits.TGATE);
    // Load Timer Periods
    PR3 = 500;// 10 milliseconds
    // Reset Timer Values
    TMR3 = 0x00;
    // Set Interrupt Priority
    IPC2bits.T3IP = 0x01;
    // Clear Interrupt Flags
    IFS0bits.T3IF = 0;
    // Enable Interrupts
    IEC0bits.T3IE = 1;
 
    
    CLEARBIT(T2CONbits.TON); // Disable Timer
    CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
    TMR2 = 0x00; // Clear timer register
    T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
    CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
    PR2 = 4000; // Set timer period 20ms:
  // Set timer period 20ms:
  // 4000= 20*10^-3 * 12.8*10^6 * 1/64
       // Enable the Timers
    T3CONbits.TON = 1; 

}

//100Hz


/*
 * Servo Code
 */
void init_servo(char num_servo) { // here the number is X(OC8) or Y(OC7)
  if(num_servo == 'X'){
    // setup OC8
    CLEARBIT(TRISDbits.TRISD7); /* Set OC8 as output */
    // OC8R = 1000;
    // /* Set the initial duty cycle to 5ms*/
    // OC8RS = 1000;
    // /* Load OCRS: next pwm duty cycle */
    
    /* Set OC8: PWM, no fault check, Timer2 */
    // SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
  }
  if(num_servo == 'Y'){
    // setup OC7
    CLEARBIT(TRISDbits.TRISD6); /* Set OC7 as output */
    // OC7R = 1000;
    // /* Set the initial duty cycle to 5ms*/
    // OC7RS = 1000;
    // /* Load OCRS: next pwm duty cycle */
    /* Set OC7: PWM, no fault check, Timer2 */
    // SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
  }
  
}

void set_servo(char num_servo, float duty_ms){
    if(num_servo == 'X'){
/*      if(duty_ms==0.9){
        OC8R = 3820;
        OC8RS = 3820; 
        OC8CON = 0x0006;
      }
      else if(duty_ms==1.5){
        OC8R = 3700;
        OC8RS = 3700; 
        OC8CON = 0x0006;
      }
      else if(duty_ms==2.1){
        OC8R = 3580;
        OC8RS = 3580; 
        OC8CON = 0x0006;
      }
    */
    OC8R = 4000-(20*duty_ms);
    OC8RS = 4000-(20*duty_ms); 
    OC8CON = 0x0006;
        //
    SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
    /* Set the initial duty cycle to 5ms(duty=1000)*/
    // 0 degree: 0.9ms  OC8R = 20ms*0.9 = 180
    //90 degree: 1.5ms  OC8R = 20ms*1.5 = 300
    //180 degree: 2.1ms  OC8R = 20ms*2.1 = 420
  }
  if(num_servo == 'Y'){
      if(duty_ms==0.9){
        OC7R = 3820;
        OC7RS = 3820; /* Load OCRS: next pwm duty cycle */
        OC7CON = 0x0006;
      }
      else if(duty_ms==1.5){
        OC7R = 3700;
        OC7RS = 3700; /* Load OCRS: next pwm duty cycle */
        OC7CON = 0x0006;
      }
      else if(duty_ms==2.1){
        OC7R = 3580;
        OC7RS = 3580; /* Load OCRS: next pwm duty cycle */
        OC7CON = 0x0006;
      }
      SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
    /* Set the initial duty cycle to 5ms*/
  }
}


/*
 * Touch screen code
 */
void init_touch(){
    //disable ADC
    CLEARBIT(AD1CON1bits.ADON);
    //initialize PIN
    CLEARBIT(TRISEbits.TRISE1);
    CLEARBIT(TRISEbits.TRISE2);
    CLEARBIT(TRISEbits.TRISE3);//set TRISE RE1,2,3 to output
    //CLEARBIT( AD1PCFGLbits.PCFG15); //set AD1 AN15 input pin as analog-X
    //Configure AD1CON1
    CLEARBIT(AD1CON1bits.AD12B); //set 10b Operation Mode
    AD1CON1bits.FORM = 0; //set integer output
    AD1CON1bits.SSRC = 0x7; //set automatic conversion
    //Configure AD1CON2
    AD1CON2 = 0; //not using scanning sampling
    //Configure AD1CON3
    CLEARBIT(AD1CON3bits.ADRC); //internal clock source
    AD1CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
    AD1CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
    //Leave AD1CON4 at its default value
    //enable ADC
    SETBIT(AD1CON1bits.ADON);
    
        //disable ADC
    CLEARBIT(AD2CON1bits.ADON);
    //initialize PIN
    CLEARBIT(TRISEbits.TRISE1);
    CLEARBIT(TRISEbits.TRISE2);
    CLEARBIT(TRISEbits.TRISE3);//set TRISE RE1,2,3 to output
    //CLEARBIT( AD2PCFGLbits.PCFG9); //set AD2 AN9 input pin as analog-Y
    //Configure AD2CON1
    CLEARBIT(AD2CON1bits.AD12B); //set 10b Operation Mode
    AD2CON1bits.FORM = 0; //set integer output
    AD2CON1bits.SSRC = 0x7; //set automatic conversion
    //Configure AD2CON2
    AD2CON2 = 0; //not using scanning sampling
    //Configure AD2CON3
    CLEARBIT(AD2CON3bits.ADRC); //internal clock source
    AD2CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
    AD2CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
    //Leave AD2CON4 at its default value
    //enable ADC
    SETBIT(AD2CON1bits.ADON);
}

void dim_touch(char dim){//set the dimension
    
    //connected to ADC-X
    if(dim=='X'){
        Nop();
        CLEARBIT(PORTEbits.RE1);
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        AD1CHS0bits.CH0SA = 0x0F; //set ADC to Sample AN15 pin
    }
    
    //connected to ADC-Y
    if(dim=='Y'){
        Nop();
        SETBIT(PORTEbits.RE1);
        Nop();
        CLEARBIT(PORTEbits.RE2);
        Nop();
        CLEARBIT(PORTEbits.RE3);
        AD2CHS0bits.CH0SA = 0x09; //set ADC to Sample AN9 pin
    }
    __delay_ms(10);
}

uint16_t read_touch_x(){//read the position of the ball
    
    SETBIT(AD1CON1bits.SAMP); //start to sample
    while(!AD1CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return ADC1BUF0;
}
uint16_t read_touch_y(){//read the position of the ball
    
    SETBIT(AD2CON1bits.SAMP); //start to sample
    while(!AD2CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD2CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return ADC2BUF0;
}


/*
 * PD Controller
 */
uint16_t PD(uint16_t input, uint16_t ref){
    uint16_t output = kp*(ref-input)+kd*((ref-input)/0.02);
    return output;
}

/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */
uint16_t butter_x(){
    uint16_t output;
    output = b1*rx_cur+b2*rx_pre-a2*output;
    rx_pre = output;
    return output;
}

uint16_t butter_y(){
    uint16_t output;
    output = b1*ry_cur+b2*ry_pre-a2*output;
    ry_pre = output;
    return output;
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void)
{ // invoked every ??
    T2_counter++; // Increment a global counter
    IFS0bits.T2IF = 0; // clear the interrupt flag
    circle();
    dim_touch('X');
    rx_cur=read_touch_x();
    dim_touch('Y');
    ry_cur=read_touch_y();
    lcd_locate(0, 3);
    lcd_printf("X: %u   Y: %u   ",rx_cur,ry_cur);
}

/*
 * main loop
 */
void main_loop()
{
    // print assignment information
    lcd_printf("Lab06: Amazing Ball");
    lcd_locate(0, 1);
    lcd_printf("Group: Group7");
    lcd_locate(0, 2);
    
    
    
    while(TRUE) {
     lcd_locate(0, 4);
     lcd_printf("X_ref: %u   Y_ref: %u   ",rx_ref,ry_ref);
    
        
    }
}
