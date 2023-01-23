#include "lab05.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * PWM code
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000
void delay(){
    uint16_t c,d;
   
   for (c = 1; c <=1700; c++)
       for (d = 1; d <= 5000; d++)
       {};
}// definition of delay function
void initialize_timer()
{
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

    __builtin_write_OSCCONL(OSCCONL | 2);//enable LPOSC, enable macro to a 32kHz
    // Disable the Timers
    T1CONbits.TON = 0; //Disable Timer 1
    // Set Prescaler
    T1CONbits.TCKPS = 0b11;//1/256
    // Set Clock Source
    T1CONbits.TCS = 1;
    // Set Gated Timer Mode -> don't use gating
    CLEARBIT(T1CONbits.TGATE);

    // T1: Set External Clock Input Synchronization -> no sync
    T1CONbits.TSYNC = 0;
    // Load Timer Periods
    PR1 = 640;// 5 second starting from 0
    // Reset Timer Values
    TMR1 = 0x00;

}

/*
 * touch screen code
 */
void init_touch(){
    //disable ADC
    CLEARBIT(AD1CON1bits.ADON);
    //initialize PIN
    CLEARBIT(TRISEbits.TRISE1);
    CLEARBIT(TRISEbits.TRISE2);
    CLEARBIT(TRISEbits.TRISE3);//set TRISE RE1,2,3 to output
    CLEARBIT( AD1PCFGLbits.PCFG15); //set AD1 AN15 input pin as analog-X
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
    CLEARBIT( AD2PCFGLbits.PCFG9); //set AD2 AN9 input pin as analog-Y
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
        CLEARBIT(PORTEbits.RE1);
        SETBIT(PORTEbits.RE2);
        SETBIT(PORTEbits.RE3);
        AD1CHS0bits.CH0SA = 0x0F; //set ADC to Sample AN15 pin
    }
    
    //connected to ADC-Y
    if(dim=='Y'){
        SETBIT(PORTEbits.RE1);
        CLEARBIT(PORTEbits.RE2);
        CLEARBIT(PORTEbits.RE3);
        AD2CHS0bits.CH0SA = 0x09; //set ADC to Sample AN9 pin
    }
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
 * servos initialize code
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
      if(duty_ms==0.9){
        OC8R = 3820;
        OC8RS = 3820; /* Load OCRS: next pwm duty cycle */
        OC8CON = 0x0006;
      }
      else if(duty_ms==1.5){
        OC8R = 3700;
        OC8RS = 3700; /* Load OCRS: next pwm duty cycle */
        OC8CON = 0x0006;
      }
      else if(duty_ms==2.1){
        OC8R = 3580;
        OC8RS = 3580; /* Load OCRS: next pwm duty cycle */
        OC8CON = 0x0006;
      }
    SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
    /* Set the initial duty cycle to 5ms(duty=1000)*/
    // 0 degree: 0.9ms
    //90 degree: 1.5ms
    //180 degree: 2.1ms
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
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab05: Touchscreen &\r\n");
    lcd_printf("       Servos");
    lcd_locate(0, 2);
    lcd_printf("Group: Group7");
    initialize_timer();
    // initialize touchscreen
    init_servo('X');
    init_servo('Y');
    // initialize servos

    init_touch();
    uint16_t rx,ry;
    while(TRUE) {

    // Enable the Timers
    set_servo('X', 0.9);
    set_servo('Y', 0.9);
    dim_touch('X');
    rx=read_touch_x();
    dim_touch('Y');
    ry=read_touch_y();
    lcd_locate(0, 3);
    lcd_printf("X: %i Y: %i",rx,ry);
    delay();
    set_servo('X', 0.9);
    set_servo('Y', 2.1);
    dim_touch('X');
    rx=read_touch_x();
    dim_touch('Y');
    ry=read_touch_y();
    lcd_locate(0, 3);
    lcd_printf("X: %i Y: %i",rx,ry);
    delay(); 
    set_servo('X', 2.1);
    set_servo('Y', 2.1);
    dim_touch('X');
    rx=read_touch_x();
    dim_touch('Y');
    ry=read_touch_y();
    lcd_locate(0, 3);
    lcd_printf("X: %i Y: %i",rx,ry);
    delay(); 
    set_servo('X', 2.1);
    set_servo('Y', 0.9);
    dim_touch('X');
    rx=read_touch_x();
    dim_touch('Y');
    ry=read_touch_y();
    lcd_locate(0, 3);
    lcd_printf("X: %i Y: %i",rx,ry);
    delay(); 
        

    }
}
