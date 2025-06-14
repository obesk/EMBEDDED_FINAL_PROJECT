#ifndef UART_H
#define	UART_H

#include <xc.h>


// flag used to manually trigger the UART interrupt on new data
extern int UART_INTERRUPT_TX_MANUAL_TRIG; 

struct circular_buffer {
    char *buff;
    int read;
    int write;

    int len;
};

void init_uart();
void print_to_buff(const char * str, struct circular_buffer *buff);

#endif	/* UART_H */
