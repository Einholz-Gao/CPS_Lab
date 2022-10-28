#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL 
#include <libpic30.h>

#include "lcd.h"
#include "led.h"


/* Configuration of the Chip */
// Initial Oscillator Source Selection = Primary (XT, HS, EC) Oscillator with PLL
#pragma config FNOSC = PRIPLL
// Primary Oscillator Mode Select = XT Crystal Oscillator mode
#pragma config POSCMD = XT
// Watchdog Timer Enable = Watchdog Timer enabled/disabled by user software
// (LPRC can be disabled by clearing the SWDTEN bit in the RCON register)
#pragma config FWDTEN = OFF

void delay(){
    uint16_t c,d;
   
   for (c = 1; c <= 500; c++)
       for (d = 1; d <= 500; d++)
       {};
}
int main(){
    uint8_t i=0;
    uint8_t flag;
    lcd_initialize();
    led_init();
    lcd_clear();
    
    //Init LCD and LEDs
while(1){
    // Clear the Screen and reset the cursor
    
    lcd_locate(0, 0);
    lcd_printf("Dorothea");
    lcd_locate(0, 1);
    lcd_printf("Yichao");
    lcd_locate(0, 2);
    lcd_printf("Giuila");
    
    lcd_locate(0, 3);
    lcd_printf("Counter:%d",i);
    
    if (i==4){
        flag=0;
    }
    else if(i==0)
        flag=1;
    
    if(flag==0) i--;
    else if(flag==1) i++;
    
    
    if(i==0) {
        SETLED(LED1_PORT); 
        delay();
        CLEARLED(LED1_PORT);
    }
    else if(i==1){
        SETLED(LED2_PORT); 
        delay();
        CLEARLED(LED2_PORT);
    }
    else if(i==2) {
        SETLED(LED3_PORT); 
        delay();
        CLEARLED(LED3_PORT);
    }
    else if(i==3){
        SETLED(LED4_PORT); 
        delay();
        CLEARLED(LED4_PORT);
    }
    else {
        SETLED(LED5_PORT); 
        delay();
        CLEARLED(LED5_PORT);
    }
    
    
 
        
} 
   
    
    // Stop

        ;
}

