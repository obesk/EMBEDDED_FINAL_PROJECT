#include "adc.h"

#include <xc.h>

//VBAT is connected to AN11 that is the 3rd channel of the ADC 1
//BAT-VSENSE is 1/3 of the VBAT, we have to multiply the readed value by 3

void init_adc(void){

    ANSELBbits.ANSB11 = 1; // setting the pin to analog mode
    TRISBbits.TRISB11 = 1; // setting the pin as input

    ANSELBbits.ANSB14 = 1;
    TRISBbits.TRISB14 = 1;

    AD1CON1bits.ADON = 0; // Turn off the ADC
    AD1CON1bits.FORM = 0b00; // Data Output Format integer

    AD1CON3bits.ADCS = 8; // Set the Tad
    AD1CON1bits.ASAM = 1; // Selecting automatic mode starting
    AD1CON3bits.SAMC = 16; // Set the automatic end
    AD1CON1bits.SSRC = 7; // conversion starts after time specified by SAMC

    AD1CON1bits.AD12B = 0; // Selecting 10-bit mode

    AD1CON2bits.VCFG = 0b00;// Voltage reference vdd

    AD1CON2bits.CSCNA = 1; // activate scan mode
    AD1CON2bits.CHPS = 1; // 2 channel mode
    AD1CON1bits.SIMSAM = 1; // Enable sequential scanning
    AD1CON2bits.SMPI = 1; // 1 interrupt eachc scanning cycle

    // Selecting the channels for automatic scanning
    AD1CSSL = 0;
    AD1CSSLbits.CSS14 = 1; // Select AN14 - IR sensor
    AD1CSSLbits.CSS11 = 1; // Select AN11 - Battery
    
    AD1CON1bits.ADON = 1; // Turn on the ADC
    AD1CON1bits.SAMP = 1; // Starting manually the first sampling 
}