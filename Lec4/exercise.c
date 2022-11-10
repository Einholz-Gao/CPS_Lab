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

int main(){
    //Init LCD and LEDs
	lcd_initialize();
    led_init();

    // Clear the Screen and reset the cursor
    lcd_clear();

    // Print "Problem session" in the top row of the screen

    // (1) ________ insert code
    // (2) ________ insert code
    // (3) ________ insert code

    // Start Counter Loop
    uint32_t counter = 0;

	while(1){

        // Turn LED1 on when the most significant bit of 'counter' is 1
        // and turn LED5 on when bit 1 bit is set to 1. Otherwise,
        // turn them off

        // (4) ________ insert code
        // (5) ________ insert code
        Nop();

        // The loop should be executed with approx. a frequency of 5Hz  (5 times per second: use delay function about 200ms)
        // (6) ________ insert code
        // (7) ________ insert code
        // (8) ________ insert code

        // Assuming you implemented the code above correctly, will you
        // ever see LED1 turn on when you look for one complete week on our
        // embedded board? Prove your result mathematically.

	}
}

/*
1) lcd_locate(0, 1);
2) printf("Problem session\n");
3) lcd_printf("Problem session\n");
4) lcd_locate(0, 0);
5) LED1_PORT = ((counter << 31) & 0x01);
6) LED1_PORT = ((counter >> 31) & 0x01);
7) LED1_PORT = ((counter >> 15) & 0x01);
8) LED1_PORT = ((counter << 15) & 0x01);
9) LED5_PORT = (((counter << 32) >> 31) & 0x01);  "and" means just clean the other except right first bit
.
10) LED5_PORT = (((counter << 30) >> 31) | 0x01);// "or" x make sence
11) LED5_PORT = ((counter >> 1) | 0x01);
12) LED5_PORT = (((counter << 16) >> 17) & 0x01);
13) __delay_ms(200);
14) __delay_ms(5);
*/

