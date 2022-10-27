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

void delay(uint16_t j){
    uint16_t x;
    for(x=0;x<j;x++)
        Nop();
}
int main(){
    uint8_t i=0;
    lcd_initialize();
    led_init();
    //Init LCD and LEDs
while(1){
    // Clear the Screen and reset the cursor
    lcd_clear();
    lcd_locate(0, 0);
    lcd_printf("Dorothea");
    lcd_locate(0, 1);
    lcd_printf("Yichao");
    lcd_locate(0, 2);
    lcd_printf("Guila");
    
    lcd_locate(0, 3);
    lcd_printf("Counter:%*d",i);
    delay(1000);
    i++;
        
} 
   
    
    // Stop

        ;
}

