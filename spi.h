#ifndef SPI_H
#define	SPI_H

#define CS_ACC PORTBbits.RB3
#define CS_GYR PORTBbits.RB4
#define CS_MAG PORTDbits.RD6

unsigned int spi_write(unsigned int data);
void init_spi();

#endif	/* SPI_H */

