#include "adc.h"
#include "citoa.h"
#include "parser.h"
#include "pwm.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"

#include <math.h>
#include <string.h>
#include <xc.h>

#define MAIN_HZ 500
// 1 Hz, 500ms high, 500ms low
#define CLOCK_LD1_TOGGLE 250
// 1Hz
#define CLOCK_BATT_PRINT 500
// 10 Hz
#define CLOCK_IR_PRINT 50
// 10 Hz
#define CLOCK_ACC_READ 50

#define COUNT_T3_CALLS 500

/* Since we use a 10 bit UART transmission we use 10 bits for a byte of data.
Given the baud rate at 9600, and with the main executed at 500 Hz, we have
19.2 bit/s which leaves 2 byte/s */
#define INPUT_BUFF_LEN 2

// Considerations on fixed messages:
//  - $MEMRG, 1* or $MEMRG, 0* -> same number of bytes; cannot occur on the
//     same cycle
//  - $MACK, 1* or $MACK, 0* -> same number of bytes;
//     NB: this message is printed both for the $PCSTP,* message and for the
//     $PCSTT,* one;
//     we can have at maximum one command inputted per main due to the 2 byte
//     restriction
//     ***********************************************************
//     Considerations on MBATT message:
//  - $MBATT,* -> 8 bytes
//  - vbatt is always positive and 2 decimals are printed (x.xx) -> 4 bytes
// Cosiderations on MDIST message:
//  - $MDIST,* -> 8 bytes
//  - dist is always positive and an integer of maximum 3 digits -> 3 bytes
//  Considerations on MACC message:
//  - $MACC,,,* -> 9 bytes
//  - x values can assume either positive and negative values,
//  	and at maximum requiring 4 bytes fot the absolute values -> 5 bytes (4
//     + 1 for the sign)
//  - same reasoning can be done for y and z
//  NB: for each message, we have to add 1 byte to consider the \0 (end of
//     string)
//  Considering the worst case scenario, all the messages listed above can
//     occur at on the same cycle. In this case, the buffer should be able to
//     contain all of them. Its size should then be given by:
//  - $MEMRG,1* or $MEMRG,0* -> (9 + 1) = 10 bytes
//  - $MACK,1* or $MACK,0* -> (8 + 1) = 9 bytes
//  - $MBATT,* + vbatt -> (8 + 4 + 1) = 13 bytes
//  - $MDIST,* + dist -> (8 + 3 + 1) = 12 bytes
//  - $MACC,,,* + x + y + z -> (9 + 5 * 3 + 1) = 25 bytes
//  TOTAL : 69 bytes

#define OUTPUT_BUFF_LEN 69

// this is sized to contain the largest message, which is MACC
#define OUTPUT_STR_LEN 25

#define OBSTACLE_THRESHOLD_CM 30

#define N_ADC_READINGS 10

enum RobotState { WAIT, MOVING, EMERGENCY };

volatile struct circular_buffer UART_input_buff = {
	.len = INPUT_BUFF_LEN,
};

volatile struct circular_buffer UART_output_buff = {
	.len = OUTPUT_BUFF_LEN,
};
struct AccReading {
	int x;
	int y;
	int z;
};

struct ADCReading {
	float distance;
	float vbatt;
};

struct ADCReadings {
	int w;
	struct ADCReading readings[N_ADC_READINGS];
};

enum Axis {
	X_AXIS = 0,
	Y_AXIS,
	Z_AXIS,
};

void timers_init() {
	IFS0bits.T2IF = 0;
	IEC0bits.T2IE = 1; // enabling the timer 2 interrupt
	IFS0bits.T3IF = 0;
	IEC0bits.T3IE = 1; // enabling the timer 3 interrupt
}

void button_init(void) {
	RPINR0bits.INT1R = 0x58; // remapping the interrupt 1 to the T2 button pin
	IFS1bits.INT1IF = 0;	 // resetting flag of interrupt 1
	IEC1bits.INT1IE = 1;	 // enabling interrupt 1
	// Resetting the flag after the enable prevents the interrupt to be
	// triggered at startup
	IFS1bits.INT1IF = 0; // resetting flag of interrupt 1
}

void activate_accelerometer() {
	// Selecting the accelerometer and disabling magnetometer and gyroscope
	CS_MAG = 1;
	CS_GYR = 1;

	CS_ACC = 0;
	spi_write(0x11);
	spi_write(0x00); // changing the accelerometer to active state
	CS_ACC = 1;
	tmr_wait_ms(TIMER4,
				3); // waiting for the accelerometer to go into active state
}

int read_acc_axis(enum Axis axis) {
	CS_ACC = 0;
	// Startin from a specific register, I read twice
	spi_write((0x02 + axis * 2) |
			  0x80); // 2 registers for each axis -> starting form 0x02 for x,
					 // 0x04 for y and 0x06 for z
	uint8_t axis_lsb = spi_write(0x00);
	uint8_t axis_msb = spi_write(0x00);
	CS_ACC = 1;

	// Combining readings to reconstruct the real value of accelerations
	int axis_value = (((axis_msb << 4) | (axis_lsb & 0x0F)));
	if (axis_value & 0x800) {
		axis_value |= 0xF000;
	}

	return axis_value;
}

struct ADCReading read_adc(void) {
	struct ADCReading reading;

	// we activate the ADC only for the reading phase to keep the acquisition
	// frequency at 500Hz
	AD1CON1bits.ADON = 1;

	// the active wait time is negligible since the ADC frequency is orders of
	// magnitude bigger
	while (!AD1CON1bits.DONE) {
		;
	}

	int adc0_raw_reading = ADC1BUF0;
	double v_adc = (adc0_raw_reading / 1023.0) * 3.3; // assuming Vref+ = 3.3
	reading.vbatt = v_adc * 3; // multplying since the voltage is partitioned

	int adc2_raw_reading = ADC1BUF1;
	double v_adc_ir =
		(adc2_raw_reading / 1023.0) * 3.3; // assuming Vref+ = 3.3 V
	reading.distance = (2.34 - 4.74 * v_adc_ir + 4.06 * pow(v_adc_ir, 2) -
						1.6 * pow(v_adc_ir, 3) + 0.24 * pow(v_adc_ir, 4)) *
					   100;
	AD1CON1bits.ADON = 0;

	return reading;
}

volatile enum RobotState robot_state = WAIT;

// this variable is used to keep track of how many times the timer3 interrupt is
// called this is necessary because the timer is used to count 5 seconds to go
// back to wait state from emergency state
volatile int count_t3_calls = 0;

// TODO: interrupt triggering at the start
int main(void) {
	INTCON2bits.GIE = 0;
	TRISA = TRISG = ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG =
		0x0000;

	// activating the side light pins
	TRISBbits.TRISB8 = 0;
	TRISFbits.TRISF1 = 0;

	// enabling IR on buggy's MikroBUS 2
	TRISBbits.TRISB9 = 0;
	LATBbits.LATB9 = 1;

	struct AccReading acc_reading = {0};

	char output_str[OUTPUT_STR_LEN];
	char input_buff[INPUT_BUFF_LEN];
	char output_buff[OUTPUT_BUFF_LEN];

	UART_input_buff.buff = input_buff;
	UART_output_buff.buff = output_buff;

	timers_init();
	pwm_init();
	button_init();
	init_uart();
	init_spi();
	init_adc();

	pwm_set_velocities(100, 0); // initializing motor velocities when running

	struct ADCReadings ADC_readings;

	// filling the ADC readings s.t. we do not print an errouneous average for
	// the first real measurements
	for (int i = 0; i < N_ADC_READINGS; ++i) {
		ADC_readings.readings[i] = read_adc();
	}

	int count_ld1_toggle = 0;
	int count_acc_read = 0;
	int count_ir_print = 0;
	int count_batt_print = 0;

	parser_state pstate = {.state = STATE_DOLLAR};

	enum RobotState robot_state_prev = robot_state;
	int distance = OBSTACLE_THRESHOLD_CM;

	activate_accelerometer();

	INTCON2bits.GIE = 1;

	tmr_setup_period(TIMER1, 1000 / MAIN_HZ); // 100 Hz frequency

	char *cursor;
	// Messages are printed using citoa instead of sprintf since the latter is
	// very inefficient and casuses the deadlines to be missed
	while (1) {
		if (++count_acc_read >= CLOCK_ACC_READ) {
			count_acc_read = 0;

			acc_reading.x = read_acc_axis(X_AXIS);
			acc_reading.y = read_acc_axis(Y_AXIS);
			acc_reading.z = read_acc_axis(Z_AXIS);

			cursor = output_str;

			cursor = stpcpy(cursor, "$MACC,");
			cursor = citoa(acc_reading.x, cursor, 10);
			cursor = stpcpy(cursor, ",");
			cursor = citoa(acc_reading.y, cursor, 10);
			cursor = stpcpy(cursor, ",");
			cursor = citoa(acc_reading.z, cursor, 10);
			cursor = strcpy(cursor, "*");
			print_to_buff(output_str, &UART_output_buff);
		}

		if (++count_ld1_toggle >= CLOCK_LD1_TOGGLE) {
			count_ld1_toggle = 0;
			LATAbits.LATA0 = !LATAbits.LATA0;
			if (robot_state == EMERGENCY) {
				LATBbits.LATB8 = !LATBbits.LATB8;
				LATFbits.LATF1 = !LATFbits.LATF1;
			} else {
				LATBbits.LATB8 = 0;
				LATFbits.LATF1 = 0;
			}
		}

		// each time the obstacle is too close we reset the timer which will put
		// the robot back in wait state
		if (distance < OBSTACLE_THRESHOLD_CM) {
			if (robot_state != EMERGENCY) {
				print_to_buff("$MEMRG,1*", &UART_output_buff);
			}
			robot_state = EMERGENCY;
			count_t3_calls = 0;
			tmr_setup_period(TIMER3, 10);
		}

		if (robot_state != robot_state_prev) {
			if (robot_state == MOVING) {
				pwm_start();
			} else {
				pwm_stop();
			}
			robot_state_prev = robot_state;
		}

		ADC_readings.readings[ADC_readings.w] = read_adc();
		ADC_readings.w = (ADC_readings.w + 1) % N_ADC_READINGS;

		if (++count_ir_print >= CLOCK_IR_PRINT) {
			count_ir_print = 0;

			double dist = 0;
			for (int i = 0; i < N_ADC_READINGS; ++i) {
				dist += ADC_readings.readings[i].distance;
			}

			distance = dist / N_ADC_READINGS;
			int idist = (int)distance;
			cursor = output_str;
			cursor = stpcpy(cursor, "$MDIST,");
			cursor = citoa(idist, cursor, 10);
			cursor = stpcpy(cursor, "*");
			print_to_buff(output_str, &UART_output_buff);
		}

		if (++count_batt_print >= CLOCK_BATT_PRINT) {
			count_batt_print = 0;

			double v_batt = 0;
			for (int i = 0; i < N_ADC_READINGS; ++i) {
				v_batt += ADC_readings.readings[i].vbatt;
			}

			v_batt /= N_ADC_READINGS;

			cursor = output_str;
			cursor = stpcpy(cursor, "$MBATT,");
			cursor = citoa((int)v_batt, cursor, 10);
			cursor = stpcpy(cursor, ".");
			cursor = citoa((int)(v_batt * 100) % 100, cursor, 10);
			cursor = stpcpy(cursor, "*");
			print_to_buff(output_str, &UART_output_buff);
		}

		while (UART_input_buff.read != UART_input_buff.write) {
			const int status =
				parse_byte(&pstate, UART_input_buff.buff[UART_input_buff.read]);
			if (status == NEW_MESSAGE) {
				if (strcmp(pstate.msg_type, "PCREF") == 0) {
					const int speed = extract_integer(pstate.msg_payload);
					const int next_arg = next_value(pstate.msg_payload, 1);
					const int yaw =
						extract_integer(&pstate.msg_payload[next_arg]);
					pwm_set_velocities(speed, yaw);
				} else if (strcmp(pstate.msg_type, "PCSTP") == 0) {
					if (robot_state != EMERGENCY) {
						print_to_buff("$MACK,1*", &UART_output_buff);
						robot_state = WAIT;
					} else {
						print_to_buff("$MACK,0*", &UART_output_buff);
					}
				} else if (strcmp(pstate.msg_type, "PCSTT") == 0) {
					if (robot_state != EMERGENCY) {
						print_to_buff("$MACK,1*", &UART_output_buff);
						robot_state = MOVING;
					} else {
						print_to_buff("$MACK,0*", &UART_output_buff);
					}
				}
			}
			UART_input_buff.read = (UART_input_buff.read + 1) % INPUT_BUFF_LEN;
		}
		tmr_wait_period(TIMER1);
	}
	return 0;
}

// triggered when the RE8 button
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(void) {
	IFS1bits.INT1IF = 0;
	// using a timer to debounce the button click
	tmr_setup_period(TIMER2, 10);
}

// this is triggered when RE8 is released for at least 10ms
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
	IFS0bits.T2IF = 0;
	T2CONbits.TON = 0;

	if (robot_state == EMERGENCY) {
		return;
	}

	if (robot_state == WAIT) {
		robot_state = MOVING;
	} else if (robot_state == MOVING) {
		robot_state = WAIT;
	}
}

// used to manage the emergency state
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
	IFS0bits.T3IF = 0;

	if (++count_t3_calls > COUNT_T3_CALLS) {
		print_to_buff("$MEMRG,0*", &UART_output_buff);
		count_t3_calls = 0;
		T3CONbits.TON = 0;
		robot_state = WAIT;
	}
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void) {
	IFS0bits.U1TXIF = 0; // clear TX interrupt flag

	if (UART_output_buff.read == UART_output_buff.write) {
		UART_INTERRUPT_TX_MANUAL_TRIG = 1;
	}

	while (!U1STAbits.UTXBF &&
		   UART_output_buff.read != UART_output_buff.write) {
		U1TXREG = UART_output_buff.buff[UART_output_buff.read];
		UART_output_buff.read = (UART_output_buff.read + 1) % OUTPUT_BUFF_LEN;
	}
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
	IFS0bits.U1RXIF = 0; // resetting the interrupt flag to 0

	while (U1STAbits.URXDA) {
		const char read_char = U1RXREG;

		const int new_write_index =
			(UART_input_buff.write + 1) % INPUT_BUFF_LEN;
		if (new_write_index != UART_input_buff.read) {
			UART_input_buff.buff[UART_input_buff.write] = read_char;
			UART_input_buff.write = new_write_index;
		}
	}
}
