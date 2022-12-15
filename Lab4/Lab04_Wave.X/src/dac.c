#include "dac.h"
#include "types.h"

// tristate register
#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13

// port register
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

// analog to digital converter 1 port configuration register
#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

// analog to digital converter 2 port configuration register
#define DAC_SDI_AD2CFG AD2PCFGLbits.PCFG10
#define DAC_SCK_AD2CFG AD2PCFGLbits.PCFG11
#define DAC_LDAC_AD2CFG AD2PCFGLbits.PCFG13

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

void dac_convert_milli_volt(uint16_t milliVolt)
{
    CLEARBIT(DAC_SCK_PORT);
    Nop();
    CLEARBIT(DAC_CS_PORT);
    Nop();
    
    milliVolt = (1<<12)|milliVolt;
    milliVolt = (1<<15)|milliVolt;
    int i;
    for(i=0;i<16;i++){
        DAC_SDI_PORT = milliVolt>>(15-i)&1;
        Nop();
        SETBIT(DAC_SCK_PORT);
        Nop();
        CLEARBIT(DAC_SCK_PORT);
        Nop();
    }
    CLEARBIT(DAC_SCK_PORT);
    Nop();
    //we can use for-loop here, instead of so many lines
    SETBIT(DAC_CS_PORT);
    Nop();
    CLEARBIT(DAC_LDAC_PORT);
    Nop();
    SETBIT(DAC_LDAC_PORT);
    Nop();
}
