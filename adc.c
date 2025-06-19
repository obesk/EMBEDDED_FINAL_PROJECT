#include "adc.h"

#include <xc.h>

void init_adc(void){
	// AN11 to read the battery voltage
    ANSELBbits.ANSB11 = 1; 
    TRISBbits.TRISB11 = 1;

	// AN14 to read the IR sensor
    ANSELBbits.ANSB14 = 1;
    TRISBbits.TRISB14 = 1;

    AD1CON1bits.ADON = 0; // Turn off the ADC
    AD1CON1bits.FORM = 0b00; // Data Output Format integer

    AD1CON3bits.ADCS = 8; // Set the TAD
    AD1CON1bits.ASAM = 1; // Selecting automatic mode starting
    AD1CON3bits.SAMC = 16; // Set the automatic end
    AD1CON1bits.SSRC = 7; // conversion starts after time specified by SAMC

    AD1CON1bits.AD12B = 0; // Selecting 10-bit mode

    AD1CON2bits.CSCNA = 1; // activate scan mode
    AD1CON2bits.CHPS = 0; // 1 channel mode
    AD1CON1bits.SIMSAM = 0; // Enable sequential scanning
    AD1CON2bits.SMPI = 1;

    AD1CSSL = 0;
    AD1CSSLbits.CSS14 = 1; // Select AN14 - IR sensor
    AD1CSSLbits.CSS11 = 1; // Select AN11 - Battery
}
