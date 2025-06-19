#include "uart.h"
#include <xc.h>

int UART_INTERRUPT_TX_MANUAL_TRIG = 1; 

void init_uart() {
    RPINR18bits.U1RXR = 0b1001011; // mapping pin RD11(RPI75) to UART RX
    RPOR0bits.RP64R = 0b000001; // mapping pin RD0(RP64) to UART TX

    U1BRG = 467; // baud rate to 9600 -> 72 000 000 / (16 * 9600) - 1

    U1STAbits.URXISEL = 0; // set to interrupt on char received
    U1MODEbits.UARTEN = 1; // enable UART
    U1STAbits.UTXEN = 1; // enable UART transmission
    
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 1;

    IFS0bits.U1RXIF = 0; // interrupt flag set to 0
    IEC0bits.U1RXIE = 1; // enabled interrupt on receive
    IEC0bits.U1TXIE = 1; // enabled interrupt on transmission

}

void print_to_buff(const char * str, volatile struct circular_buffer *buff) {
    if(!str) {
        return; 
    }
    
    for (int i = 0; str[i] != '\0'; ++i) {
        const int new_write_index = (buff->write + 1) % buff->len;

        if(new_write_index == buff->read) {
            break;
        }

        buff->buff[buff->write] = str[i];
        buff->write = new_write_index;
    }

    // if the last UART transfer didn't send any data we retrigger the interrupt manually to send the new data
    if(UART_INTERRUPT_TX_MANUAL_TRIG){
        UART_INTERRUPT_TX_MANUAL_TRIG = 0;
        IFS0bits.U1TXIF = 1;
    }
}
