#include "spi.h"

#include <xc.h>

unsigned int spi_write(unsigned int data) {
    while (SPI1STATbits.SPITBF == 1);

    SPI1BUF = data;
    
    while (SPI1STATbits.SPIRBF == 0);

    return SPI1BUF; // read to prevent buffer overrun
}

void init_spi() {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

    SPI1CON1bits.MSTEN = 1; // master mode 
    SPI1CON1bits.MODE16 = 0; // 8-bit mode 
 
    // since for the SPI we wait actively for the readings, we set the maximum 
    // frequency possible 
    SPI1CON1bits.PPRE = 3; // 1:1 primary prescaler
    SPI1CON1bits.SPRE = 6; // 2:1 secondary prescaler
    
    TRISAbits.TRISA1 = 1; // RA1-RPI17 MISO
    TRISFbits.TRISF12 = 0; // RF12-RP108 SCK
    TRISFbits.TRISF13 = 0; // RF13-RP109 MOSI
    
    TRISBbits.TRISB3 = 0; // Accelerometer chip select
    TRISBbits.TRISB4 = 0; // Gyroscope chip select
    TRISDbits.TRISD6 = 0; // Magnetometer chip select

    RPINR20bits.SDI1R = 0b0010001; // MISO (SDI1) - RPI17
    RPOR12bits.RP109R = 0b000101;// MOSI (SDO1) - RF13;
    RPOR11bits.RP108R = 0b000110; // SCK1;
        
    SPI1STATbits.SPIEN = 1; // enable SPI
    SPI1CON1bits.CKP = 1; // Set clock idle state to high
}
