#include "adc.h"

#include <xc.h>


//VBAT is connected to AN11 that is the 3rd channel of the ADC 1
//BAT-VSENSE is 1/3 of the VBAT, we have to multiply the readed value by 3

void init_adc(void){
    AD1CON1bits.ADON = 0; // Turn off the ADC

    AD1CON1bits.AD12B = 0; // Selecting 10-bit mode
    AD1CON2bits.VCFG = 0b00;// Voltage reference vdd
    AD1CON3bits.ADCS = 8; // Set the Tad
    AD1CON3bits.SAMC = 16; // Set the automatic end

    AD1CON1bits.ADDMABM = 0; // DMA on
    AD1CSSLbits.CSS15 = 1; // Select AN15 - IR sensor
    // AD1CSSLbits.CSS11 = 1; // Select AN11 - Battery
    AD1CON1bits.FORM = 0b00; // Data Output Format integer

    AD1CON1bits.ASAM = 0; // Selecting automatic mode starting
    AD1CON1bits.SSRC = 7; // conversion starts after time specified by SAMC

    AD1CON2bits.CHPS = 0b00; // 1 channel mode
    AD1CON2bits.CSCNA = 1; // Scan ch0
    AD1CHS0bits.CH0SA = 15; // Choosing AN15 
    // AD1CHS0bits.CH0SA = 11; // Choosing AN11
    AD1CHS0bits.CH0NA = 0; // Choosing VREFL
}