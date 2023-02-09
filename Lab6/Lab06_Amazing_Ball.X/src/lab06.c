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
unsigned int T3_counter;


/*
 * Common Definitions
 */
#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define a1 1.0      //butter
#define a2 -0.6796  //butter    
#define b1 0.1602   //butter    
#define b2 0.1602   //butter    

#define r 100
#define x_center 382
#define y_center 393
#define d_speed 2*3.14*0.3


#define kp_x 0.0015
#define kd_x 0.012

#define kp_y 0.001
#define kd_y 0.012

#define X_LEVELED_US 1495
#define Y_LEVELED_US 1390
#define PWM_MAX_US 2100
#define PWM_MIN_US 900


/*
 * Global Variables
 */
uint16_t rx_pre,ry_pre,rx_cur,ry_cur,rx_ref ,ry_ref ;
float deg=0;
float t=0;

//Circle
void circle(){
    t+=0.02; // 1/50
    deg= d_speed*t;
    rx_ref = r * cos(deg)+x_center;
    ry_ref = r * sin(deg)+y_center;
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
    
    //timer for interrupt
    // Disable the Timers
    T3CONbits.TON = 0;//Disable Timer 3
    // Set Prescaler
    T3CONbits.TCKPS = 0b11;// 1/256
    // Set Clock Source
    T3CONbits.TCS = 0;
    // Set Gated Timer Mode -> don't use gating
    CLEARBIT(T3CONbits.TGATE);
    CLEARBIT(T3CONbits.TGATE);
    // Load Timer Periods
    PR3 = 50;// 1 milliseconds
    
    // Reset Timer Values
    TMR3 = 0x00;
    // Set Interrupt Priority
    IPC2bits.T3IP = 0x01;
    // Clear Interrupt Flags
    IFS0bits.T3IF = 0;

    // Enable Interrupts
    IEC0bits.T3IE = 1;

 
    // timer for...
    CLEARBIT(T2CONbits.TON); // Disable Timer
    CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
    TMR2 = 0x00; // Clear timer register
    T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
        IPC1bits.T2IP = 0x01;
        IFS0bits.T2IF = 0;
        IEC0bits.T2IE = 1;
    PR2 = 4000; // Set timer period 20ms:
  // Set timer period 20ms:
  // 4000= 20*10^-3 * 12.8*10^6 * 1/64
       // Enable the Timers
    T3CONbits.TON = 1; 
    T2CONbits.TON = 1;
    T3_counter = 0;

}

//100Hz


/*
 * Servo Code
 */
void init_servo(char num_servo) { // here the number is X(OC8) or Y(OC7)
  if(num_servo == 'X'){
    // setup OC8
    OC8R = 3820;// 0 degree
    OC8CON = 0x0006;
    
    CLEARBIT(TRISDbits.TRISD7); /* Set OC8 as output */
    SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
  }
  if(num_servo == 'Y'){
    // setup OC7
    OC7R = 3820;
    OC7CON = 0x0006;
    
    CLEARBIT(TRISDbits.TRISD6); /* Set OC7 as output */
    SETBIT(T2CONbits.TON); /* Turn Timer 2 on */

  }
  
}

void set_servo(char num_servo, float duty_us){
    if(num_servo == 'X'){

    float OC_val = 4000-(0.2*duty_us);
    OC8RS = OC_val; 
    /* Set the initial duty cycle to 5ms(duty=1000)*/
    // 0 degree: 0.9ms
    //90 degree: 1.5ms
    //180 degree: 2.1ms
  }
  if(num_servo == 'Y'){
    float OC_val = 4000-(0.2*duty_us);
    OC7RS = OC_val; 
      
   // uint16_t OC_val = 4000-(20*duty_ms);
    //OC7R = OC_val;
    //OC7RS = OC_val; 
    //OC7CON = 0x0006;
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
float error_pre_x=0;
float PD_x(float input, float ref){
    float error_cur = ref-input;
    float output = kp_x*error_cur+kd_x*(error_cur-error_pre_x);
    error_pre_x = error_cur;
    if(output<-1) output=-1;
    if(output>1) output=1;
    return output;
}

float error_pre_y=0;
float PD_y(float input, float ref){
    float error_cur = ref-input;
    float output = kp_y*error_cur+kd_y*(error_cur-error_pre_y);
    error_pre_y = error_cur;
    if(output<-1) output=-1;
    if(output>1) output=1;
    return output;
}



/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */


float butter_x(){
    static float x1=0,y1=0;
    float y;
    y = b1*rx_cur+b2*x1-a2*y1;
    x1 = rx_cur;
    y1 = y;
    return y;
}

float butter_y(){
    static float x1=0,y1=0;
    float y;
    y = b1*ry_cur+b2*x1-a2*y1;
    x1 = ry_cur;
    y1 = y;
    return y;
}

int DDL=0;
int DDL_counter=0;

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T3Interrupt(void)
{ // invoked every ??
    T3_counter++; // Increment a global counter
    IFS0bits.T3IF = 0; // clear the interrupt flag
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void)
{ // invoked every ??
     // Increment a global counter
    IFS0bits.T2IF = 0; // clear the interrupt flag
    TOGGLELED(LED3_PORT);
    if(DDL=1){
        DDL_counter++;
    }
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
    initialize_timer();
    // initialize touchscreen
    init_servo('X');
    init_servo('Y');
    // initialize servos
    init_touch();
    // initialize touch screen
    
 
    int flag=0;
    int time_flag=0;
    int flag_DDL = 0;
    
    while(TRUE) {
    DDL=1; 
    TMR2 = 0x00;
    // 100Hz
    if(T3_counter%10!=0){ //!!! to set the frequency: do not use ==, but use module %
        //if(flag==0){
            dim_touch('X');//10ms
            rx_cur=read_touch_x();
            //flag=1;
        //}
        //else{
            dim_touch('Y');//10ms
            ry_cur=read_touch_y();
            //flag=0;
        //}

    lcd_locate(0, 4);
    lcd_printf("X: %u  Y: %u  ",rx_cur,ry_cur);
    T3_counter=0;
    time_flag++;
    }

    //50Hz
    
    if(time_flag==2){
     circle();
     float pid_out_x,pid_out_y;
     uint16_t x_filter = butter_x(rx_cur);
     uint16_t y_filter = butter_y(ry_cur);
     pid_out_x = PD_x(x_filter, rx_ref);
     pid_out_y = PD_y(y_filter, ry_ref);
     float x_servo_us = X_LEVELED_US + pid_out_x*(PWM_MAX_US-PWM_MIN_US)/2.0f;
     float y_servo_us = Y_LEVELED_US + pid_out_y*(PWM_MAX_US-PWM_MIN_US)/2.0f;
     set_servo('X',x_servo_us);
     set_servo('Y',y_servo_us);
     time_flag=0;
     flag_DDL++;
     //5Hz
     if(flag_DDL==10){
        lcd_locate(0, 6);
        lcd_printf("Counter: %i",DDL_counter);
        TOGGLELED(LED2_PORT);
        flag_DDL = 0;
     }
    }
        lcd_locate(0, 5);
        lcd_printf("     DDL: %i",DDL);
     DDL=0;
    
    }
}
